function esp32_mic_serial()
% Live waveform + FFT + scrolling spectrogram of ESP32 INMP441 (I2S) audio.
% Reads the COM port directly (no Python, no UDP). Requires MATLAB R2019b+
% for serialport; clim needs R2022a+ (falls back to caxis), turbo needs R2020b+ (falls back to jet).
% Close the Python program first: only one program can open the COM port.

% ---------- Settings ----------
COM_PORT      = "COM5";         % change to your port
BAUD          = 921600;         % must match Serial.begin() in the firmware
FS            = 16000;          % must match firmware SAMPLE_RATE
BLOCK         = 1024;           % must match firmware BLOCK_SIZE
FULL_SCALE    = 32768;          % int16 full scale
FRAME_BYTES   = 2 + 4 + 2*BLOCK + 2;   % header + seq + samples + crc = 2056

FMAX          = 8000;           % top of FFT / spectrogram axis (Hz) = Nyquist at 16 kHz

% FFT panel
WINDOW_BLOCKS = 4;              % 4096 samples -> 3.9 Hz bins at 16 kHz
SHOW_DBFS     = false;          % FFT panel in dBFS (true) or linear amplitude (false)

% Spectrogram panel
NWIN          = 1024;           % samples per FFT column (15.6 Hz bins, 64 ms)
HOP           = 512;            % step between columns (32 ms)
DISPLAY_SEC   = 5;              % history shown
CLIM_DBFS     = [-100 -20];     % color range

% ---------- CRC16-CCITT table (poly 0x1021, init 0xFFFF, same as firmware/Python) ----------
crcTable = zeros(1,256,'uint16');
for i = 0:255
    c = uint16(bitshift(i,8));
    for k = 1:8
        if bitand(c, uint16(32768))
            c = bitxor(bitshift(c,1), uint16(4129));   % 0x1021
        else
            c = bitshift(c,1);
        end
    end
    crcTable(i+1) = c;
end

% ---------- Input: serial port ----------
sp = serialport(COM_PORT, BAUD, 'Timeout', 1);
flush(sp);
cleanupObj = onCleanup(@() delete(sp)); %#ok<NASGU>

% ---------- FFT panel constants ----------
N     = BLOCK * WINDOW_BLOCKS;
wF    = 0.5 - 0.5*cos(2*pi*(0:N-1)/(N-1));      % Hann (same as np.hanning)
sumWF = sum(wF);
halfF = floor(N/2) + 1;
freqF = (0:halfF-1) * (FS/N);
y      = nan(1,N);
filled = 0;

% ---------- Spectrogram constants ----------
wS     = 0.5 - 0.5*cos(2*pi*(0:NWIN-1)/(NWIN-1));
sumWS  = sum(wS);
halfS  = floor(NWIN/2) + 1;
freqS  = (0:halfS-1) * (FS/NWIN);
nCols  = round(DISPLAY_SEC * FS / HOP);
tAxisS = (-nCols+1:0) * (HOP/FS);
S      = -120 * ones(halfS, nCols);
newCols = zeros(halfS,0);
pend    = [];

% ---------- Figure ----------
fig = figure('Name','ESP32 INMP441 (serial)','NumberTitle','off','Position',[100 100 1200 800]);
tl  = tiledlayout(fig, 2, 2, 'TileSpacing', 'compact');

ax1 = nexttile(tl, 1);
tAxis = (0:N-1)/FS;
lnT = plot(ax1, tAxis, y, 'Color', [0 0.6 0.5], 'LineWidth', 1);
grid(ax1,'on'); xlabel(ax1,'Time (s)'); ylabel(ax1,'Amplitude (normalized)');
ylim(ax1,[-1 1]); xlim(ax1,[0 tAxis(end)]);
title(ax1,'Time Domain');

ax2 = nexttile(tl, 2);
lnF = plot(ax2, freqF, nan(1,halfF), 'Color', [1 0.53 0], 'LineWidth', 1);
hold(ax2,'on');
pk  = plot(ax2, nan, nan, 'ro', 'MarkerFaceColor','r');
grid(ax2,'on'); xlabel(ax2,'Frequency (Hz)'); xlim(ax2,[0 min(FMAX, FS/2)]);
if SHOW_DBFS, ylabel(ax2,'Amplitude (dBFS)'); else, ylabel(ax2,'Amplitude'); end
title(ax2,'Frequency Domain (FFT)');

ax3 = nexttile(tl, 3, [1 2]);
img = imagesc(ax3, tAxisS, freqS, S);
axis(ax3,'xy');
try, clim(ax3, CLIM_DBFS); catch, caxis(ax3, CLIM_DBFS); end %#ok<NOCOM>
try, colormap(ax3, turbo); catch, colormap(ax3, jet); end
ylim(ax3, [0 min(FMAX, FS/2)]);
cb = colorbar(ax3); cb.Label.String = 'dBFS';
xlabel(ax3,'Time (s)'); ylabel(ax3,'Frequency (Hz)');
title(ax3,'Spectrogram');

% ---------- Main loop ----------
buf = uint8([]);
lastSeq = [];
good = 0; lost = 0; crcErr = 0;
tLast = tic;

while ishandle(fig)
    % ---- read whatever the serial port has ----
    n = sp.NumBytesAvailable;
    if n > 0
        data = read(sp, n, "uint8");
        buf  = [buf, data(:).']; %#ok<AGROW>
    else
        pause(0.001);
    end

    % ---- parse frames ----
    while numel(buf) >= FRAME_BYTES
        k = find(buf(1:end-1)==0xCD & buf(2:end)==0xAB, 1);
        if isempty(k), buf = buf(end); break; end
        if k > 1, buf(1:k-1) = []; end
        if numel(buf) < FRAME_BYTES, break; end

        frame = buf(1:FRAME_BYTES);

        % CRC check over everything except the last 2 bytes
        crc = uint16(65535);                       % 0xFFFF
        for b = frame(1:end-2)
            idx = double(bitxor(bitshift(crc,-8), uint16(b))) + 1;
            crc = bitxor(bitshift(crc,8), crcTable(idx));
        end
        rxCrc = typecast(frame(end-1:end), 'uint16');
        if crc ~= rxCrc
            crcErr = crcErr + 1;
            buf(1) = [];                           % false header, slide by one byte
            continue;
        end

        seq = typecast(frame(3:6), 'uint32');
        if ~isempty(lastSeq) && seq ~= lastSeq + 1
            lost = lost + double(seq - lastSeq - 1);
        end
        lastSeq = seq;
        good = good + 1;

        % signed int16 samples -> normalized -1 ... +1
        counts = double(typecast(frame(7:6+2*BLOCK), 'int16'));
        amp    = counts(:).' / FULL_SCALE;
        buf(1:FRAME_BYTES) = [];

        % --- time/FFT window ---
        y = [y(BLOCK+1:end), amp];
        filled = min(filled + BLOCK, N);

        % --- spectrogram ---
        pend = [pend, amp]; %#ok<AGROW>
        while numel(pend) >= NWIN
            seg = pend(1:NWIN);
            seg = seg - mean(seg);
            X   = fft(seg .* wS);
            mag = 2 * abs(X(1:halfS)).' / sumWS;          % one-sided peak amplitude
            newCols(:,end+1) = 20*log10(max(mag, 1e-9)); %#ok<AGROW>
            pend(1:HOP) = [];
        end
    end

    % ---------- Update plots (~20 fps) ----------
    if toc(tLast) > 0.05
        updated = false;

        if filled == N
            v    = y;
            spec = fft((v - mean(v)) .* wF);
            mag  = 2 * abs(spec(1:halfF)) / sumWF;   % one-sided peak amplitude

            [~, i] = max(mag(2:end));
            i = i + 1;
            peakF = freqF(i);
            pp    = max(v) - min(v);

            if SHOW_DBFS
                yPlot = 20*log10(max(mag, 1e-9));
            else
                yPlot = mag;
            end

            set(lnT, 'YData', v);
            set(lnF, 'YData', yPlot);
            set(pk,  'XData', peakF, 'YData', yPlot(i));
            title(ax1, sprintf('Time Domain   |   p-p = %.3f FS', pp));
            title(ax2, sprintf('FFT   |   peak = %.1f Hz', peakF));
            updated = true;
        end

        if ~isempty(newCols)
            nNew = min(size(newCols,2), nCols);
            S = [S(:, nNew+1:end), newCols(:, end-nNew+1:end)];
            newCols = zeros(halfS,0);
            set(img, 'CData', S);
            title(ax3, sprintf('Spectrogram   |   frames: %d   lost: %d   CRC errors: %d', ...
                               good, lost, crcErr));
            updated = true;
        end

        if updated
            drawnow limitrate;
            tLast = tic;
        end
    end
end
end
