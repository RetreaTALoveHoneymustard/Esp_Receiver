import sys
import time
import argparse
import struct
import numpy as np
import collections
import serial
from serial.tools import list_ports
import pyqtgraph as pg
from PyQt6 import QtWidgets, QtCore

# ===================== Fixed by firmware (must match .ino) =====================
BLOCK_SIZE = 256          # samples per frame -- must equal firmware BLOCK_SIZE
VREF = 3.3                # ADC reference voltage (ADC_11db covers ~0-3.3V)
ADC_MAX_COUNTS = 4095     # 12-bit resolution

FRAME_FORMAT = f"<HI{BLOCK_SIZE}HH"
FRAME_SIZE = struct.calcsize(FRAME_FORMAT)

# How many frames to average over when estimating the true sample rate from
# inter-frame timing. Larger = more stable estimate, slower to update.
FS_ESTIMATE_WINDOW_FRAMES = 50

# ===================== Rendering options =====================
# Antialiasing is purely cosmetic -- it smooths how PyQtGraph draws line
# edges to pixels, it does NOT change, filter, or interpolate the underlying
# sample data in any way. Safe to toggle off again if it costs too much
# performance on your machine.
pg.setConfigOptions(antialias=True)
# If the plot feels laggy with antialiasing on, uncomment the next line to
# move antialiasing work onto the GPU instead of the CPU (requires a working
# OpenGL context):
# pg.setConfigOptions(antialias=True, useOpenGL=True)

# ===================== CRC16-CCITT (must match firmware) =====================
def crc16_ccitt(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc

def counts_to_volts(counts):
    return np.asarray(counts, dtype=np.float64) * (VREF / ADC_MAX_COUNTS) #calculate to find exact volts with unpack data and ensure data type into float 64bits for easily calculation.

def linear_volts_to_dbv(amplitude):
    """
    Convert a linear peak-amplitude FFT magnitude array (volts) to dBV,
    matching the convention used by Rigol's Math1 FFT (20*log10(Vrms/1V)).
    A tiny floor is applied before the log to avoid log(0) on empty bins.
    """
    vrms = np.asarray(amplitude, dtype=np.float64) / np.sqrt(2.0)
    vrms = np.maximum(vrms, 1e-12)
    return 20.0 * np.log10(vrms)

# ===================== Serial Receiver Thread =====================
class SerialThread(QtCore.QThread):
    new_data_signal = QtCore.pyqtSignal(list)
    status_signal = QtCore.pyqtSignal(str)
    stats_signal = QtCore.pyqtSignal(int, int, int)  # valid_frames, crc_errors, dropped_frames
    fs_estimate_signal = QtCore.pyqtSignal(float)     # measured sample rate, Hz
    #Why use QThread instead of main Qwidget cause large amount of datas that changing overtime will broke the qt.
    def __init__(self, port, baud_rate):
        super().__init__()
        self.port = port
        self.baud_rate = baud_rate
        self.running = True
        #set

    def run(self):
        try:
            ser = serial.Serial(self.port, self.baud_rate, timeout=1.0)
            ser.reset_input_buffer()
            self.status_signal.emit(f"Connected to {self.port}")
        except Exception as e:
            self.status_signal.emit(f"Connection Error: {e}")
            return

        buffer = bytearray()
        valid_frames = 0
        crc_errors = 0
        dropped_frames = 0
        last_seq = None

        # --- actual sample-rate estimation state ---
        fs_window_start_time = None
        fs_window_frame_count = 0

        while self.running:
            try:
                chunk = ser.read(ser.in_waiting or 1)
                if chunk:
                    buffer.extend(chunk)

                while len(buffer) >= FRAME_SIZE:
                    header_idx = buffer.find(b"\xcd\xab")
                    if header_idx == -1:
                        buffer = buffer[-1:]
                        break
                    elif header_idx > 0:
                        del buffer[:header_idx]

                    if len(buffer) < FRAME_SIZE:
                        break

                    frame_data = bytes(buffer[:FRAME_SIZE])
                    received_crc = struct.unpack("<H", frame_data[-2:])[0]
                    calculated_crc = crc16_ccitt(frame_data[:-2])

                    if received_crc != calculated_crc:
                        crc_errors += 1
                        del buffer[0]
                        continue

                    unpacked = struct.unpack(FRAME_FORMAT, frame_data)
                    seq_id = unpacked[1]
                    samples = list(unpacked[2:-1])

                    del buffer[:FRAME_SIZE]
                    valid_frames += 1

                    if last_seq is not None:
                        gap = (seq_id - last_seq - 1) & 0xFFFFFFFF
                        if gap:
                            dropped_frames += gap
                    last_seq = seq_id

                    # --- measure actual sample rate from real inter-frame timing,
                    #     independent of whatever --fs the user typed in ---
                    if fs_window_start_time is None:
                        fs_window_start_time = time.monotonic()
                        fs_window_frame_count = 0
                    fs_window_frame_count += 1
                    if fs_window_frame_count >= FS_ESTIMATE_WINDOW_FRAMES:
                        elapsed = time.monotonic() - fs_window_start_time
                        if elapsed > 0:
                            measured_fs = (fs_window_frame_count * BLOCK_SIZE) / elapsed
                            self.fs_estimate_signal.emit(measured_fs)
                        fs_window_start_time = time.monotonic()
                        fs_window_frame_count = 0

                    self.new_data_signal.emit(samples)
                    self.stats_signal.emit(valid_frames, crc_errors, dropped_frames)

            except Exception as e:
                self.status_signal.emit(f"Read Error: {e}")
                break

        ser.close()
        self.status_signal.emit("Serial Connection Closed")

    def stop(self):
        self.running = False
        self.wait()

# ===================== Real-Time Plot GUI =====================
class ADCVisualizer(QtWidgets.QMainWindow):
    def __init__(self, port, baud_rate, sample_rate, window_blocks=8,
                 configured_freq=None, configured_vpp=None):
        super().__init__()
        self.configured_sample_rate = sample_rate   # what the user passed via --fs
        self.measured_sample_rate = None             # auto-detected from frame timing
        self.window_samples = BLOCK_SIZE * window_blocks

        # Configured (ground-truth) signal parameters, supplied via CLI so the
        # measured values can be checked against what was actually set on the
        # function generator.
        self.configured_freq = configured_freq
        self.configured_vpp = configured_vpp

        # Stashed connection settings so the Start button can reopen the port
        # after Stop has closed it.
        self.port = port
        self.baud_rate = baud_rate

        # FFT display unit: start in linear volts; user can toggle to dBV
        # to directly compare against a scope's Math1 FFT (e.g. Rigol).
        self.show_dbv = False
        self.last_fft_mag = None  # cached linear-volts magnitude array, so the
                                   # dBV toggle can redraw instantly without new data

        self.setWindowTitle(f"ESP32 ADC Stream  |  Fs(configured)={sample_rate} Hz  |  {port}")
        self.resize(1100, 940)

        self.data_buffer = collections.deque(maxlen=self.window_samples)

        central = QtWidgets.QWidget()
        layout = QtWidgets.QVBoxLayout(central)
        self.setCentralWidget(central)

        # --- Sample-rate confirmation banner ---
        self.fs_label = QtWidgets.QLabel(
            f"Fs configured: {sample_rate} Hz   |   Fs measured: -- (collecting...)"
        )
        self.fs_label.setStyleSheet("font-weight: bold; padding: 4px;")
        layout.addWidget(self.fs_label, stretch=0)

        # --- Start / Stop acquisition controls ---
        control_row = QtWidgets.QHBoxLayout()
        self.stop_button = QtWidgets.QPushButton("Stop Acquisition")
        self.stop_button.clicked.connect(self.on_stop_clicked)
        control_row.addWidget(self.stop_button)

        self.start_button = QtWidgets.QPushButton("Start Acquisition")
        self.start_button.clicked.connect(self.on_start_clicked)
        self.start_button.setEnabled(False)
        control_row.addWidget(self.start_button)

        control_row.addStretch(1)
        layout.addLayout(control_row)

        glw = pg.GraphicsLayoutWidget()
        layout.addWidget(glw, stretch=3)

        # --- Time-domain plot ---
        self.time_plot = glw.addPlot(row=0, col=0, title="Time Domain (Voltage vs Time)")
        self.time_plot.setLabel('left', 'Voltage', units='V')
        self.time_plot.setLabel('bottom', 'Time', units='s')
        self.time_plot.showGrid(x=True, y=True, alpha=0.3)
        self.time_plot.setYRange(0, VREF)
        self.time_curve = self.time_plot.plot(pen=pg.mkPen(color='#00FFCC', width=1.5), antialias=True)

        glw.nextRow()

        # --- Frequency-domain plot ---
        self.freq_plot = glw.addPlot(row=1, col=0, title="Frequency Domain (FFT)")
        self.freq_plot.setLabel('left', 'Amplitude', units='V')
        self.freq_plot.setLabel('bottom', 'Frequency', units='Hz')
        self.freq_plot.showGrid(x=True, y=True, alpha=0.3)
        self.freq_plot.setXRange(0, sample_rate / 2)
        self.freq_curve = self.freq_plot.plot(pen=pg.mkPen(color='#FF8800', width=1.5), antialias=True)
        self.peak_marker = self.freq_plot.plot(
            [], [], pen=None, symbol='o', symbolBrush='r', symbolSize=8
        )

        # --- dBV toggle, to align with scope FFT (e.g. Rigol Math1) ---
        dbv_row = QtWidgets.QHBoxLayout()
        self.dbv_checkbox = QtWidgets.QCheckBox("Show FFT in dBV (matches scope Math1 convention)")
        self.dbv_checkbox.stateChanged.connect(self._on_dbv_toggle)
        dbv_row.addWidget(self.dbv_checkbox)
        dbv_row.addStretch(1)
        layout.addLayout(dbv_row)

        # --- Configured vs Measured comparison table ---
        table_label = QtWidgets.QLabel("Configured vs Measured Comparison")
        table_label.setStyleSheet("font-weight: bold; padding-top: 6px;")
        layout.addWidget(table_label)

        self.table = QtWidgets.QTableWidget(2, 4)
        self.table.setHorizontalHeaderLabels(["Parameter", "Configured", "Measured", "Error"])
        self.table.setVerticalHeaderLabels(["Frequency (Hz)", "Amplitude Vpp (V)"])
        self.table.horizontalHeader().setSectionResizeMode(QtWidgets.QHeaderView.ResizeMode.Stretch)
        self.table.setEditTriggers(QtWidgets.QAbstractItemView.EditTrigger.NoEditTriggers)
        self.table.setMaximumHeight(110)
        for r in range(2):
            for c in range(4):
                self.table.setItem(r, c, QtWidgets.QTableWidgetItem("--"))
        # Pre-fill "Configured" column from CLI args, if provided
        self.table.item(0, 1).setText(f"{configured_freq:.3f}" if configured_freq is not None else "N/A")
        self.table.item(1, 1).setText(f"{configured_vpp:.3f}" if configured_vpp is not None else "N/A")
        layout.addWidget(self.table, stretch=0)

        # --- Data-integrity confirmation statement ---
        self.integrity_label = QtWidgets.QLabel("Awaiting data...")
        self.integrity_label.setStyleSheet("font-weight: bold; padding: 4px;")
        layout.addWidget(self.integrity_label, stretch=0)

        self.statusBar().showMessage("Initializing...")

        self.frames_valid = 0
        self.crc_errors = 0
        self.dropped_frames = 0

        self.serial_thread = None
        self.start_serial_thread()

    def start_serial_thread(self):
        """(Re)create and start the serial reader thread, wiring up its signals."""
        self.serial_thread = SerialThread(self.port, self.baud_rate)
        self.serial_thread.new_data_signal.connect(self.update_plot)
        self.serial_thread.status_signal.connect(self.update_status)
        self.serial_thread.stats_signal.connect(self.update_stats)
        self.serial_thread.fs_estimate_signal.connect(self.update_fs_estimate)
        self.serial_thread.start()

        self.stop_button.setEnabled(True)
        self.start_button.setEnabled(False)

    @QtCore.pyqtSlot()
    def on_stop_clicked(self):
        """Stop acquisition: closes the serial port and halts the reader thread."""
        if self.serial_thread is not None and self.serial_thread.isRunning():
            self.serial_thread.stop()
        self.stop_button.setEnabled(False)
        self.start_button.setEnabled(True)
        self.statusBar().showMessage("Acquisition stopped by user.")

    @QtCore.pyqtSlot()
    def on_start_clicked(self):
        """Resume acquisition by reopening the serial port and restarting the thread."""
        self.start_serial_thread()
        self.statusBar().showMessage("Acquisition resumed.")

    @property
    def effective_sample_rate(self):
        """
        Use the measured sample rate once we have one -- it reflects what the
        ESP32 is actually doing, not what --fs assumed. Falls back to the
        configured value until enough frames have arrived to estimate it.
        """
        return self.measured_sample_rate or self.configured_sample_rate

    @QtCore.pyqtSlot(float)
    def update_fs_estimate(self, measured_fs):
        self.measured_sample_rate = measured_fs

        mismatch_pct = abs(measured_fs - self.configured_sample_rate) / self.configured_sample_rate * 100.0
        text = (
            f"Fs configured: {self.configured_sample_rate} Hz   |   "
            f"Fs measured: {measured_fs:.1f} Hz"
        )
        if mismatch_pct > 5.0:
            text += f"   \u26a0 MISMATCH ({mismatch_pct:.0f}% off -- FFT axis uses the measured value)"
            self.fs_label.setStyleSheet("font-weight: bold; padding: 4px; color: #ff4444;")
        else:
            text += "   \u2713 matches configured rate"
            self.fs_label.setStyleSheet("font-weight: bold; padding: 4px; color: #00cc44;")
        self.fs_label.setText(text)

        # Keep the FFT x-axis range sane for the corrected rate
        self.freq_plot.setXRange(0, self.effective_sample_rate / 2)

    def _on_dbv_toggle(self, state):
        self.show_dbv = bool(state)
        if self.show_dbv:
            self.freq_plot.setLabel('left', 'Amplitude', units='dBV')
        else:
            self.freq_plot.setLabel('left', 'Amplitude', units='V')
        # Redraw immediately from the cached magnitude array, no need to wait
        # for the next frame.
        if self.last_fft_mag is not None:
            self._redraw_fft(self.last_freqs, self.last_fft_mag)

    def _redraw_fft(self, freqs, mag):
        if self.show_dbv:
            y = linear_volts_to_dbv(mag)
        else:
            y = mag
        self.freq_curve.setData(freqs, y)

        if len(mag) > 2:
            peak_idx = np.argmax(mag[1:]) + 1
            peak_freq = freqs[peak_idx]
            peak_y = y[peak_idx]
            self.peak_marker.setData([peak_freq], [peak_y])

    @QtCore.pyqtSlot(list)
    def update_plot(self, new_samples):
        self.data_buffer.extend(new_samples)
        counts = np.array(self.data_buffer)
        volts = counts_to_volts(counts)
        n = len(volts)

        fs = self.effective_sample_rate

        # --- Time domain ---
        time_x = np.arange(n) / fs
        self.time_curve.setData(time_x, volts)

        # --- Frequency domain (FFT with Hann window) ---
        if n >= 16:
            window = np.hanning(n)
            windowed = (volts - np.mean(volts)) * window
            spectrum = np.fft.rfft(windowed)
            freqs = np.fft.rfftfreq(n, d=1.0 / fs)

            # Amplitude-correct: Hann window coherent gain is 0.5
            mag = np.abs(spectrum) / (n * 0.5)

            # cache for instant redraw when the dBV toggle changes
            self.last_freqs = freqs
            self.last_fft_mag = mag

            self._redraw_fft(freqs, mag)

            # Peak-frequency detection, skipping DC bin (always computed in
            # linear volts/Hz regardless of display unit, for the status bar
            # and comparison table)
            if len(mag) > 2:
                peak_idx = np.argmax(mag[1:]) + 1
                peak_freq = freqs[peak_idx]
                peak_amp = mag[peak_idx]

                vpp = volts.max() - volts.min()
                self.last_measured = (peak_freq, peak_amp, vpp)
                self._update_comparison_table(peak_freq, vpp)

    def _update_comparison_table(self, meas_freq, meas_vpp):
        # Frequency row
        self.table.item(0, 2).setText(f"{meas_freq:.3f}")
        if self.configured_freq:
            err = (meas_freq - self.configured_freq) / self.configured_freq * 100.0
            self.table.item(0, 3).setText(f"{err:+.2f}%")
        else:
            self.table.item(0, 3).setText("N/A")

        # Amplitude row
        self.table.item(1, 2).setText(f"{meas_vpp:.3f}")
        if self.configured_vpp:
            err = (meas_vpp - self.configured_vpp) / self.configured_vpp * 100.0
            self.table.item(1, 3).setText(f"{err:+.2f}%")
        else:
            self.table.item(1, 3).setText("N/A")

    @QtCore.pyqtSlot(int, int, int)
    def update_stats(self, valid, crc_err, dropped):
        self.frames_valid = valid
        self.crc_errors = crc_err
        self.dropped_frames = dropped
        self._refresh_status()
        self._refresh_integrity_label()

    @QtCore.pyqtSlot(str)
    def update_status(self, message):
        self._base_status = message
        self._refresh_status()

    def _refresh_status(self):
        # Kept minimal on purpose: the FFT-bin "peak amplitude" used to be
        # shown here too, but it only matches Vpp/2 for a clean single-tone
        # signal -- with real harmonic content it reads much lower and was
        # confusing next to the (more trustworthy) table's Vpp measurement.
        # Frequency and amplitude ground-truth now live only in the fs_label
        # banner and the comparison table, respectively.
        base = getattr(self, "_base_status", "")
        msg = f"{base}  |  frames OK: {self.frames_valid}  CRC errors: {self.crc_errors}  dropped: {self.dropped_frames}"
        self.statusBar().showMessage(msg)

    def _refresh_integrity_label(self):
        if self.crc_errors == 0 and self.dropped_frames == 0:
            self.integrity_label.setText(
                f"\u2713 Data integrity OK — {self.frames_valid} frames received, "
                f"0 CRC errors, 0 dropped frames. No samples or data blocks were lost."
            )
            self.integrity_label.setStyleSheet("font-weight: bold; padding: 4px; color: #00cc44;")
        else:
            self.integrity_label.setText(
                f"\u2717 Data loss detected — {self.crc_errors} CRC error(s), "
                f"{self.dropped_frames} dropped frame(s) out of {self.frames_valid + self.dropped_frames} expected."
            )
            self.integrity_label.setStyleSheet("font-weight: bold; padding: 4px; color: #ff4444;")

    def closeEvent(self, event):
        if self.serial_thread is not None and self.serial_thread.isRunning():
            self.serial_thread.stop()
        event.accept()

# ===================== Entry Point =====================
def main():
    parser = argparse.ArgumentParser(description="ESP32 ADC serial receiver / visualizer")
    parser.add_argument("--port", default="COM9", help="Serial port (default: COM9)")
    parser.add_argument("--baud", type=int, default=2000000, help="Baud rate (default: 2000000)")
    parser.add_argument("--fs", type=int, default=200,
                         help="Sample rate in Hz -- MUST MATCH firmware SAMPLE_RATE (default: 200). "
                              "The GUI will auto-detect the true rate from frame timing and flag any mismatch.")
    parser.add_argument("--window-blocks", type=int, default=8,
                         help="How many 256-sample frames to keep on screen (default: 8)")
    parser.add_argument("--sig-freq", type=float, default=None,
                         help="Configured/expected input signal frequency in Hz, for the comparison table")
    parser.add_argument("--sig-vpp", type=float, default=None,
                         help="Configured/expected input signal peak-to-peak amplitude in V, for the comparison table")
    args = parser.parse_args()

    app = QtWidgets.QApplication(sys.argv)

    port = args.port
    if not port:
        ports = list_ports.comports()
        if ports:
            port = ports[0].device

    gui = ADCVisualizer(port=port, baud_rate=args.baud, sample_rate=args.fs,
                         window_blocks=args.window_blocks,
                         configured_freq=args.sig_freq, configured_vpp=args.sig_vpp)
    gui.show()
    sys.exit(app.exec())

if __name__ == "__main__":
    main()
