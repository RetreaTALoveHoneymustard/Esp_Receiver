function esp32_live_serial()
% Live time-domain + FFT + scrolling spectrogram of ESP32 ADC frames.
% Reads the COM port directly (no Python, no UDP). Requires MATLAB R2019b+
% for serialport. clim (R2022a+) and turbo (R2020b+) fall back to caxis/jet.
% Close the Python program first: only one program can open the COM port.
%
% Save this file as esp32_live_serial.m and run:  esp32_live_serial

% ---------- Settings ----------
COM_PORT      = "COM5";         % change to your port
BAUD          = 1000000;        % must match Serial.begin() in the firmware
FS            = 40000;          % must match firmware TARGET_RATE
BLOCK         = 1024;           % must match firmware BLOCK_SIZE
VREF          = 3.3;
ADC_MAX       = 4095;
FRAME_BYTES   = 2 + 4 + 2*BLOCK + 2;   % 2056 for BLOCK = 1024

FMAX          = 8000;           % top of FFT / spectrogram axis (Hz)

% FFT panel
WINDOW_BLOCKS = 4;              % 4096 samples -> 9.8 Hz bins at 40 kHz
SHOW_DBV      = false;          % FFT panel in dBV (true) or volts (false)

% Spectrogram panel
NWIN          = 2048;           % samples per FFT column
HOP           = 1024;           % step between columns
DISPLAY_SEC   = 5;              % history shown
CLIM_DBV      = [-100 -20];     % color range

% ---------- Keeping up with the stream ----------
CHECK_CRC          = true;      % false = rely on the sequence counter only (fastest)
PLOT_PERIOD        = 0.20;      % seconds between redraws (0.20 = 5 fps)
TITLE_PERIOD       = 1.0;       % seconds between title updates (titles are slow)
USE_SOFTWARE_OPENGL = true;     % true = draw on the CPU; avoids GPU/driver stalls
MAX_BACKLOG_FRAMES = 8;         % skip drawing while more than this many frames
                                % are waiting, so parsing can catch up first

% ---------- CRC16-CCITT table (poly 0x1021, init 0xFFFF, same as Python) ----------
% Plain doubles are used so the CRC loop stays fast.
crcTable = zeros(1,256);
for i = 0:255
    c = i * 256;
    for k = 1:8
        if c >= 32768
            c = bitxor(mod(c*2, 65536), 4129);   % 0x1021
        else
            c = c * 2;
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
wF    = 0.5 - 0.5*cos(2*pi*(0:N-1)/(N-1));
halfF = floor(N/2) + 1;
freqF = (0:halfF-1) * (FS/N);
y      = nan(1,N);
filled = 0;

% ---------- Spectrogram constants ----------
wS     = 0.5 - 0.5*cos(2*pi*(0:NWIN-1)/(NWIN-1));
halfS  = floor(NWIN/2) + 1;
freqS  = (0:halfS-1) * (FS/NWIN);
nCols  = round(DISPLAY_SEC * FS / HOP);
tAxisS = (-nCols+1:0) * (HOP/FS);
S      = -120 * ones(halfS, nCols);
newCols = zeros(halfS,0);
pend    = [];

% ---------- Figure ----------
% Must be set before the figure is created. Wrapped in try/catch because newer
% MATLAB releases changed or removed the opengl command.
if USE_SOFTWARE_OPENGL
    try, opengl software; catch, end
end
fig = figure('Name','ESP32 ADC (serial)','NumberTitle','off','Position',[100 100 1200 800]);
tl  = tiledlayout(fig, 2, 2, 'TileSpacing', 'compact');

ax1 = nexttile(tl, 1);
tAxis = (0:N-1)/FS;
lnT = plot(ax1, tAxis, y, 'Color', [0 0.6 0.5], 'LineWidth', 1);
grid(ax1,'on'); xlabel(ax1,'Time (s)'); ylabel(ax1,'Voltage (V)');
ylim(ax1,[0 VREF]); xlim(ax1,[0 tAxis(end)]);
title(ax1,'Time Domain');

ax2 = nexttile(tl, 2);
lnF = plot(ax2, freqF, nan(1,halfF), 'Color', [1 0.53 0], 'LineWidth', 1);
hold(ax2,'on');
pk  = plot(ax2, nan, nan, 'ro', 'MarkerFaceColor','r');
grid(ax2,'on'); xlabel(ax2,'Frequency (Hz)'); xlim(ax2,[0 min(FMAX, FS/2)]);
if SHOW_DBV, ylabel(ax2,'Amplitude (dBV)'); else, ylabel(ax2,'Amplitude (V)'); end
title(ax2,'Frequency Domain (FFT)');

ax3 = nexttile(tl, 3, [1 2]);
img = imagesc(ax3, tAxisS, freqS, S);
axis(ax3,'xy');
try, clim(ax3, CLIM_DBV); catch, caxis(ax3, CLIM_DBV); end %#ok<CAXIS>
try, colormap(ax3, turbo); catch, colormap(ax3, jet); end
ylim(ax3, [0 min(FMAX, FS/2)]);
cb = colorbar(ax3); cb.Label.String = 'dBV';
xlabel(ax3,'Time (s)'); ylabel(ax3,'Frequency (Hz)');
title(ax3,'Spectrogram');

% ---------- Main loop ----------
buf = uint8([]);
lastSeq = [];
good = 0; lost = 0; crcErr = 0;
tStart = [];                    % starts when the first good frame arrives
tLast = tic;
tTitle = tic;

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
        if CHECK_CRC
            crc   = crc16_ccitt(double(frame(1:end-2)), crcTable);
            rxCrc = double(typecast(frame(end-1:end), 'uint16'));
            if crc ~= rxCrc
                crcErr = crcErr + 1;
                buf(1) = [];                       % false header, slide by one byte
                continue;
            end
        end

        seq = typecast(frame(3:6), 'uint32');
        if ~isempty(lastSeq) && seq ~= lastSeq + 1
            lost = lost + double(seq - lastSeq - 1);
        end
        lastSeq = seq;
        good = good + 1;
        if isempty(tStart), tStart = tic; end

        counts = double(typecast(frame(7:6+2*BLOCK), 'uint16'));
        volts  = counts(:).' * (VREF / ADC_MAX);
        buf(1:FRAME_BYTES) = [];

        % --- time/FFT window ---
        y = [y(BLOCK+1:end), volts];
        filled = min(filled + BLOCK, N);

        % --- spectrogram ---
        pend = [pend, volts]; %#ok<AGROW>
        while numel(pend) >= NWIN
            seg = pend(1:NWIN);
            seg = seg - mean(seg);
            X   = fft(seg .* wS);
            mag = abs(X(1:halfS)).' / (NWIN*0.5);
            newCols(:,end+1) = 20*log10(max(mag/sqrt(2), 1e-12)); %#ok<AGROW>
            pend(1:HOP) = [];
        end
    end

    % ---------- Update plots ----------
    % Skipped while a backlog is waiting, so parsing always wins over drawing.
    backlog = numel(buf) >= MAX_BACKLOG_FRAMES * FRAME_BYTES;
    if ~backlog && toc(tLast) > PLOT_PERIOD
        try
            updated = false;
            doTitle = toc(tTitle) > TITLE_PERIOD;   % title() is slow: do it ~1x per second

            if filled == N
                v    = y;
                spec = fft((v - mean(v)) .* wF);
                mag  = abs(spec(1:halfF)) / (N*0.5);

                [~, i] = max(mag(2:end));
                i = i + 1;
                peakF = freqF(i);
                vpp   = max(v) - min(v);

                if SHOW_DBV
                    yPlot = 20*log10(max(mag/sqrt(2), 1e-12));
                else
                    yPlot = mag;
                end

                set(lnT, 'YData', v);
                set(lnF, 'YData', yPlot);
                set(pk,  'XData', peakF, 'YData', yPlot(i));
                if doTitle
                    title(ax1, sprintf('Time Domain   |   Vpp = %.3f V', vpp));
                    title(ax2, sprintf('FFT   |   peak = %.3f Hz', peakF));
                end
                updated = true;
            end

            if ~isempty(newCols)
                nNew = min(size(newCols,2), nCols);
                S = [S(:, nNew+1:end), newCols(:, end-nNew+1:end)];
                newCols = zeros(halfS,0);
                set(img, 'CData', S);

                updated = true;
            end

            if doTitle
                % Informational Fs estimate (long-term average, PC timing only).
                % The axes always use FS above.
                if good > 200
                    fsEst = (good-1) * BLOCK / toc(tStart);
                    fsTxt = sprintf('%.0f Hz', fsEst);
                else
                    fsTxt = '...';
                end
                title(ax3, sprintf(['Spectrogram   |   frames: %d   lost: %d   ' ...
                                    'CRC errors: %d   |   Fs est: %s'], ...
                                   good, lost, crcErr, fsTxt));
                tTitle = tic;
                updated = true;
            end

            if updated
                drawnow limitrate;
            end
            tLast = tic;
        catch ME
            if ~ishandle(fig), break; end   % window was closed mid-update
            rethrow(ME);
        end
    end
end
end

% ===================== Helpers =====================
function crc = crc16_ccitt(bytes, tbl)
% Table-driven CRC16-CCITT (poly 0x1021, init 0xFFFF).
% bytes: row vector of doubles (0..255), tbl: 1x256 table of doubles.
crc = 65535;
for b = bytes
    idx = bitxor(bitshift(crc, -8), b) + 1;
    crc = bitxor(bitand(bitshift(crc, 8), 65535), tbl(idx));
end
end
