function esp32_live_and_spectrogram()
% Live time-domain + FFT + scrolling spectrogram of ESP32 ADC frames (one window).
% Input: verified frames forwarded by PythonReceiveSignal.py over UDP (127.0.0.1:5005),
% received through MATLAB's Python interface (no toolbox needed).
% Start this script FIRST, then start the Python program.

% ---------- Settings ----------
UDP_PORT      = 5005;           % must match udp_addr port in Python
FS            = 40000;          % must match firmware TARGET_RATE
BLOCK         = 1024;           % must match firmware BLOCK_SIZE
VREF          = 3.3;
ADC_MAX       = 4095;
FRAME_BYTES   = 2 + 4 + 2*BLOCK + 2;   % 2056 for BLOCK = 1024

FMAX          = 8000;           % top of FFT / spectrogram frequency axis (Hz)

% FFT panel (whole window, like Python --window-blocks)
WINDOW_BLOCKS = 4;              % 4096 samples -> 9.8 Hz bins at 40 kHz
SHOW_DBV      = false;          % FFT panel in dBV (true) or volts (false)

% Spectrogram panel
NWIN          = 2048;           % samples per FFT column (19.5 Hz bins at 40 kHz)
HOP           = 1024;           % step between columns (25.6 ms)
DISPLAY_SEC   = 5;              % history shown
CLIM_DBV      = [-100 -20];     % color range

% ---------- Input: UDP socket through MATLAB's Python interface ----------
socketMod = py.importlib.import_module('socket');
sock = socketMod.socket(socketMod.AF_INET, socketMod.SOCK_DGRAM);
sock.setsockopt(socketMod.SOL_SOCKET, socketMod.SO_RCVBUF, int32(4*1024*1024));
sock.bind(py.tuple({'127.0.0.1', int32(UDP_PORT)}));
sock.settimeout(0.002);                          % recvfrom waits at most 2 ms
cleanupObj = onCleanup(@() sock.close()); %#ok<NASGU>

% ---------- FFT panel constants ----------
N     = BLOCK * WINDOW_BLOCKS;
wF    = 0.5 - 0.5*cos(2*pi*(0:N-1)/(N-1));      % = np.hanning(N)
halfF = floor(N/2) + 1;
freqF = (0:halfF-1) * (FS/N);
y      = nan(1,N);                               % scrolling time window
filled = 0;

% ---------- Spectrogram constants ----------
wS     = 0.5 - 0.5*cos(2*pi*(0:NWIN-1)/(NWIN-1));
halfS  = floor(NWIN/2) + 1;
freqS  = (0:halfS-1) * (FS/NWIN);
nCols  = round(DISPLAY_SEC * FS / HOP);
tAxisS = (-nCols+1:0) * (HOP/FS);
S      = -120 * ones(halfS, nCols);
newCols = zeros(halfS,0);
pend    = [];                                    % samples waiting for the STFT

% ---------- Figure: [time | fft] on top, spectrogram below ----------
fig = figure('Name','ESP32 ADC','NumberTitle','off','Position',[100 100 1200 800]);
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

ax3 = nexttile(tl, 3, [1 2]);                    % spans the whole bottom row
img = imagesc(ax3, tAxisS, freqS, S);
axis(ax3,'xy'); clim(ax3, CLIM_DBV); colormap(ax3, turbo);
ylim(ax3, [0 min(FMAX, FS/2)]);
cb = colorbar(ax3); cb.Label.String = 'dBV';
xlabel(ax3,'Time (s)'); ylabel(ax3,'Frequency (Hz)');
title(ax3,'Spectrogram');

% ---------- Main loop ----------
buf = uint8([]);
lastSeq = [];
good = 0; lost = 0;
tLast = tic;

while ishandle(fig)
    % ---- read up to 20 datagrams per pass ----
    gotAny = false;
    for j = 1:20
        try
            r = sock.recvfrom(int32(4096));      % returns (bytes, address)
        catch ME
            if ~contains(ME.message, 'timed out')
                rethrow(ME);                     % a real error, not just "nothing waiting"
            end
            break;
        end
        data = uint8(r{1});
        buf  = [buf, data(:).']; %#ok<AGROW>
        gotAny = true;
    end
    if ~gotAny, pause(0.001); end

    % ---- parse frames (CRC already verified by Python) ----
    while numel(buf) >= FRAME_BYTES
        k = find(buf(1:end-1)==0xCD & buf(2:end)==0xAB, 1);
        if isempty(k), buf = buf(end); break; end
        if k > 1, buf(1:k-1) = []; end
        if numel(buf) < FRAME_BYTES, break; end

        frame = buf(1:FRAME_BYTES);
        seq = typecast(frame(3:6), 'uint32');
        if ~isempty(lastSeq) && seq ~= lastSeq + 1
            lost = lost + double(seq - lastSeq - 1);
        end
        lastSeq = seq;
        good = good + 1;

        counts = double(typecast(frame(7:6+2*BLOCK), 'uint16'));
        volts  = counts(:).' * (VREF / ADC_MAX);
        buf(1:FRAME_BYTES) = [];

        % --- feed the time/FFT window ---
        y = [y(BLOCK+1:end), volts];
        filled = min(filled + BLOCK, N);

        % --- feed the spectrogram (one column per HOP samples) ---
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

    % ---------- Update plots (~20 fps) ----------
    if toc(tLast) > 0.05
        updated = false;

        % Time domain + FFT (only once the window is full)
        if filled == N
            v    = y;
            spec = fft((v - mean(v)) .* wF);
            mag  = abs(spec(1:halfF)) / (N*0.5);

            [~, i] = max(mag(2:end));            % skip DC bin
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
            title(ax1, sprintf('Time Domain   |   Vpp = %.3f V', vpp));
            title(ax2, sprintf('FFT   |   peak = %.3f Hz', peakF));
            updated = true;
        end

        % Spectrogram
        if ~isempty(newCols)
            nNew = min(size(newCols,2), nCols);
            S = [S(:, nNew+1:end), newCols(:, end-nNew+1:end)];
            newCols = zeros(halfS,0);
            set(img, 'CData', S);
            title(ax3, sprintf('Spectrogram   |   frames: %d   lost: %d', good, lost));
            updated = true;
        end

        if updated
            drawnow limitrate;
            tLast = tic;
        end
    end
end
end
