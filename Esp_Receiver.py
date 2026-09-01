import sys
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
    return np.asarray(counts, dtype=np.float64) * (VREF / ADC_MAX_COUNTS)

# ===================== Serial Receiver Thread =====================
class SerialThread(QtCore.QThread):
    new_data_signal = QtCore.pyqtSignal(list)
    status_signal = QtCore.pyqtSignal(str)
    stats_signal = QtCore.pyqtSignal(int, int, int)  # valid_frames, crc_errors, dropped_frames

    def __init__(self, port, baud_rate):
        super().__init__()
        self.port = port
        self.baud_rate = baud_rate
        self.running = True

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
    def __init__(self, port, baud_rate, sample_rate, window_blocks=8):
        super().__init__()
        self.sample_rate = sample_rate
        self.window_samples = BLOCK_SIZE * window_blocks

        self.setWindowTitle(f"ESP32 ADC Stream  |  Fs={sample_rate} Hz  |  {port}")
        self.resize(1100, 750)

        self.data_buffer = collections.deque(maxlen=self.window_samples)

        central = QtWidgets.QWidget()
        layout = QtWidgets.QVBoxLayout(central)
        self.setCentralWidget(central)

        glw = pg.GraphicsLayoutWidget()
        layout.addWidget(glw)

        # --- Time-domain plot ---
        self.time_plot = glw.addPlot(row=0, col=0, title="Time Domain (Voltage vs Time)")
        self.time_plot.setLabel('left', 'Voltage', units='V')
        self.time_plot.setLabel('bottom', 'Time', units='s')
        self.time_plot.showGrid(x=True, y=True, alpha=0.3)
        self.time_plot.setYRange(0, VREF)
        self.time_curve = self.time_plot.plot(pen=pg.mkPen(color='#00FFCC', width=1.5))

        glw.nextRow()

        # --- Frequency-domain plot ---
        self.freq_plot = glw.addPlot(row=1, col=0, title="Frequency Domain (FFT)")
        self.freq_plot.setLabel('left', 'Amplitude', units='V')
        self.freq_plot.setLabel('bottom', 'Frequency', units='Hz')
        self.freq_plot.showGrid(x=True, y=True, alpha=0.3)
        self.freq_plot.setXRange(0, sample_rate / 2)
        self.freq_curve = self.freq_plot.plot(pen=pg.mkPen(color='#FF8800', width=1.5))
        self.peak_marker = self.freq_plot.plot(
            [], [], pen=None, symbol='o', symbolBrush='r', symbolSize=8
        )

        self.statusBar().showMessage("Initializing...")

        self.frames_valid = 0
        self.crc_errors = 0
        self.dropped_frames = 0

        self.serial_thread = SerialThread(port, baud_rate)
        self.serial_thread.new_data_signal.connect(self.update_plot)
        self.serial_thread.status_signal.connect(self.update_status)
        self.serial_thread.stats_signal.connect(self.update_stats)
        self.serial_thread.start()

    @QtCore.pyqtSlot(list)
    def update_plot(self, new_samples):
        self.data_buffer.extend(new_samples)
        counts = np.array(self.data_buffer)
        volts = counts_to_volts(counts)
        n = len(volts)

        # --- Time domain ---
        time_x = np.arange(n) / self.sample_rate
        self.time_curve.setData(time_x, volts)

        # --- Frequency domain (FFT with Hann window) ---
        if n >= 16:
            window = np.hanning(n)
            windowed = (volts - np.mean(volts)) * window
            spectrum = np.fft.rfft(windowed)
            freqs = np.fft.rfftfreq(n, d=1.0 / self.sample_rate)

            # Amplitude-correct: Hann window coherent gain is 0.5
            mag = np.abs(spectrum) / (n * 0.5)

            self.freq_curve.setData(freqs, mag)

            # Peak-frequency detection, skipping DC bin
            if len(mag) > 2:
                peak_idx = np.argmax(mag[1:]) + 1
                peak_freq = freqs[peak_idx]
                peak_amp = mag[peak_idx]
                self.peak_marker.setData([peak_freq], [peak_amp])

                vpp = volts.max() - volts.min()
                self.last_measured = (peak_freq, vpp / 2.0, vpp)

    @QtCore.pyqtSlot(int, int, int)
    def update_stats(self, valid, crc_err, dropped):
        self.frames_valid = valid
        self.crc_errors = crc_err
        self.dropped_frames = dropped
        self._refresh_status()

    @QtCore.pyqtSlot(str)
    def update_status(self, message):
        self._base_status = message
        self._refresh_status()

    def _refresh_status(self):
        base = getattr(self, "_base_status", "")
        measured = getattr(self, "last_measured", None)
        msg = f"{base}  |  frames OK: {self.frames_valid}  CRC errors: {self.crc_errors}  dropped: {self.dropped_frames}"
        if measured:
            f_meas, amp_meas, vpp = measured
            msg += f"  |  measured f: {f_meas:.2f} Hz  amplitude: {amp_meas:.3f} V (Vpp {vpp:.3f} V)"
        self.statusBar().showMessage(msg)

    def closeEvent(self, event):
        self.serial_thread.stop()
        event.accept()

# ===================== Entry Point =====================
def main():
    parser = argparse.ArgumentParser(description="ESP32 ADC serial receiver / visualizer")
    parser.add_argument("--port", default="COM5", help="Serial port (default: COM5)")
    parser.add_argument("--baud", type=int, default=2000000, help="Baud rate (default: 2000000)")
    parser.add_argument("--fs", type=int, default=200,
                         help="Sample rate in Hz -- MUST MATCH firmware SAMPLE_RATE (default: 200)")
    parser.add_argument("--window-blocks", type=int, default=8,
                         help="How many 256-sample frames to keep on screen (default: 8)")
    args = parser.parse_args()

    app = QtWidgets.QApplication(sys.argv)

    port = args.port
    if not port:
        ports = list_ports.comports()
        if ports:
            port = ports[0].device

    gui = ADCVisualizer(port=port, baud_rate=args.baud, sample_rate=args.fs,
                         window_blocks=args.window_blocks)
    gui.show()
    sys.exit(app.exec())

if __name__ == "__main__":
    main()
