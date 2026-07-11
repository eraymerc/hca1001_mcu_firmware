"""HCA1001 ADC Monitor - dark-mode PyQt5 GUI.

Connects to the STM32F429I-DISC1 over its USB CDC virtual COM port (no UART
wiring involved -- see USB_DEVICE/App/usbd_cdc_if.c on the firmware side),
streams Voltage / Current / Encoder / HCA-error samples, plots them live,
and can save the captured data as CSV and the plots as PNG. Each signal has
its own button to open a live FFT window.

Run:
    pip install -r requirements.txt
    python main.py
"""

import sys
import time
from collections import deque

import numpy as np
import pyqtgraph as pg
import pyqtgraph.exporters
from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtWidgets import (
    QApplication,
    QCheckBox,
    QComboBox,
    QFileDialog,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QMainWindow,
    QMessageBox,
    QPushButton,
    QVBoxLayout,
    QWidget,
)
from serial.tools import list_ports

from dark_theme import apply_dark_theme
from export_worker import CsvExportWorker
from fft_window import FFTWindow
from protocol import SIGNAL_LABELS, SIGNAL_NAMES, STREAM_RATE_HZ
from serial_worker import SerialWorker

LIVE_WINDOW_SECONDS = 5
LIVE_SAMPLES = int(LIVE_WINDOW_SECONDS * STREAM_RATE_HZ)
MAX_SESSION_SAMPLES = 2_000_000  # ~33 min at 1kHz, bounds RAM use for CSV export


class SignalRow(QWidget):
    """One monitored signal: checkbox, live plot, FFT button."""

    def __init__(self, name: str, label: str, open_fft_cb, parent=None):
        super().__init__(parent)
        self._name = name

        outer = QHBoxLayout(self)
        outer.setContentsMargins(0, 0, 0, 0)

        side = QVBoxLayout()
        self.checkbox = QCheckBox(label)
        self.checkbox.setChecked(True)
        self.checkbox.setToolTip("Include in CSV / PNG export")
        side.addWidget(self.checkbox)

        fft_btn = QPushButton("FFT")
        fft_btn.setFixedWidth(60)
        fft_btn.clicked.connect(lambda: open_fft_cb(self._name, label))
        side.addWidget(fft_btn)
        side.addStretch(1)

        side_widget = QWidget()
        side_widget.setLayout(side)
        side_widget.setFixedWidth(110)
        outer.addWidget(side_widget)

        self.plot = pg.PlotWidget()
        self.plot.showGrid(x=True, y=True, alpha=0.3)
        self.plot.setLabel("left", label)
        self.plot.setLabel("bottom", "Time", units="s")
        self.curve = self.plot.plot(pen=pg.mkPen("#00d0ff", width=1.2))
        outer.addWidget(self.plot, stretch=1)


class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("HCA1001 ADC Monitor (USB)")
        self.resize(1150, 800)

        self.worker: SerialWorker | None = None
        self._csv_worker: CsvExportWorker | None = None
        self._fft_windows = []
        self._session_capped = False
        self._t0_ms = None

        self.live_t = deque(maxlen=LIVE_SAMPLES)
        self.live_buffers = {name: deque(maxlen=LIVE_SAMPLES) for name in SIGNAL_NAMES}
        # Full-session record for CSV export: list of (seq, timestamp_ms, v, i, enc, err)
        self.session_records = []

        self._build_ui()
        self._refresh_ports()

        self._ping_timer = QTimer(self)
        self._ping_timer.setSingleShot(True)
        self._ping_timer.timeout.connect(self._on_ping_timeout)
        self._ping_pending = False

    # ------------------------------------------------------------------ UI
    def _build_ui(self):
        central = QWidget()
        root = QVBoxLayout(central)
        self.setCentralWidget(central)

        # --- connection bar ---
        conn_box = QGroupBox("Connection")
        conn_layout = QHBoxLayout(conn_box)

        conn_layout.addWidget(QLabel("COM Port:"))
        self.port_combo = QComboBox()
        self.port_combo.setMinimumWidth(220)
        conn_layout.addWidget(self.port_combo)

        refresh_btn = QPushButton("Refresh")
        refresh_btn.clicked.connect(self._refresh_ports)
        conn_layout.addWidget(refresh_btn)

        self.connect_btn = QPushButton("Connect")
        self.connect_btn.clicked.connect(self._on_connect_clicked)
        conn_layout.addWidget(self.connect_btn)

        self.ping_btn = QPushButton("Test Connection")
        self.ping_btn.setEnabled(False)
        self.ping_btn.clicked.connect(self._on_ping_clicked)
        conn_layout.addWidget(self.ping_btn)

        self.stream_btn = QPushButton("Start Streaming")
        self.stream_btn.setEnabled(False)
        self.stream_btn.setCheckable(True)
        self.stream_btn.clicked.connect(self._on_stream_toggled)
        conn_layout.addWidget(self.stream_btn)

        conn_layout.addStretch(1)
        self.status_label = QLabel("Disconnected")
        self.status_label.setObjectName("statusLabel")
        conn_layout.addWidget(self.status_label)

        root.addWidget(conn_box)

        # --- signal rows ---
        self.rows = {}
        signals_box = QGroupBox("Monitored Signals")
        signals_layout = QVBoxLayout(signals_box)
        for name in SIGNAL_NAMES:
            row = SignalRow(name, SIGNAL_LABELS[name], self._open_fft_window)
            self.rows[name] = row
            signals_layout.addWidget(row, stretch=1)
        root.addWidget(signals_box, stretch=1)

        # --- export bar ---
        export_box = QGroupBox("Export")
        export_layout = QHBoxLayout(export_box)
        self.save_csv_btn = QPushButton("Save Selected as CSV")
        self.save_csv_btn.clicked.connect(self._save_csv)
        export_layout.addWidget(self.save_csv_btn)

        save_png_btn = QPushButton("Save Selected as PNG")
        save_png_btn.clicked.connect(self._save_png)
        export_layout.addWidget(save_png_btn)
        export_layout.addStretch(1)

        self.samples_label = QLabel("0 samples captured")
        export_layout.addWidget(self.samples_label)

        root.addWidget(export_box)

        self.statusBar().showMessage("Select a COM port and click Connect.")

    # ---------------------------------------------------------- Connection
    def _refresh_ports(self):
        self.port_combo.clear()
        for p in list_ports.comports():
            self.port_combo.addItem(f"{p.device}  ({p.description})", p.device)

    def _on_connect_clicked(self):
        if self.worker is not None:
            self._disconnect()
            return

        if self.port_combo.count() == 0:
            QMessageBox.warning(self, "No port selected", "No COM ports found. Click Refresh.")
            return

        port_name = self.port_combo.currentData()
        self.worker = SerialWorker(port_name)
        self.worker.connected.connect(self._on_connected)
        self.worker.disconnected.connect(self._on_disconnected)
        self.worker.frames_received.connect(self._on_frames)
        self.worker.error.connect(self._on_error)
        self.worker.ping_ok.connect(self._on_ping_ok)
        self.worker.start()
        self.connect_btn.setEnabled(False)

    def _disconnect(self):
        if self.worker is not None:
            self.worker.stop()
            self.worker.wait(2000)
            self.worker = None
        self._on_disconnected()

    def _on_connected(self, port_name):
        self.status_label.setText(f"Connected: {port_name}")
        self.status_label.setObjectName("statusLabelOk")
        self.status_label.setStyle(self.status_label.style())
        self.connect_btn.setText("Disconnect")
        self.connect_btn.setEnabled(True)
        self.ping_btn.setEnabled(True)
        self.stream_btn.setEnabled(True)
        self.statusBar().showMessage(f"Connected to {port_name}.")

    def _on_disconnected(self):
        self.status_label.setText("Disconnected")
        self.status_label.setObjectName("statusLabel")
        self.status_label.setStyle(self.status_label.style())
        self.connect_btn.setText("Connect")
        self.connect_btn.setEnabled(True)
        self.ping_btn.setEnabled(False)
        self.stream_btn.setEnabled(False)
        self.stream_btn.setChecked(False)
        self.stream_btn.setText("Start Streaming")
        self.worker = None

    def _on_error(self, message):
        self.status_label.setText("Error")
        self.status_label.setObjectName("statusLabelError")
        self.status_label.setStyle(self.status_label.style())
        self.statusBar().showMessage(message)
        QMessageBox.critical(self, "Connection error", message)
        self._disconnect()

    def _on_ping_clicked(self):
        if self.worker is None:
            return
        self._ping_pending = True
        self.worker.ping()
        self._ping_timer.start(1000)
        self.statusBar().showMessage("Pinging device...")

    def _on_ping_ok(self, ok):
        if not self._ping_pending:
            return
        self._ping_pending = False
        self._ping_timer.stop()
        self.statusBar().showMessage("Device identified: HCA1001_ADC_STREAM_V1")

    def _on_ping_timeout(self):
        if self._ping_pending:
            self._ping_pending = False
            self.statusBar().showMessage(
                "No reply from device. Wrong COM port, or firmware not flashed."
            )

    def _on_stream_toggled(self, checked):
        if self.worker is None:
            return
        if checked:
            self.worker.start_streaming()
            self.stream_btn.setText("Stop Streaming")
        else:
            self.worker.stop_streaming()
            self.stream_btn.setText("Start Streaming")

    # -------------------------------------------------------------- Data
    def _on_frames(self, frames):
        if not frames:
            return
        if self._t0_ms is None:
            self._t0_ms = frames[0].timestamp_ms

        for f in frames:
            t = (f.timestamp_ms - self._t0_ms) / 1000.0
            self.live_t.append(t)
            self.live_buffers["voltage"].append(f.voltage)
            self.live_buffers["current"].append(f.current)
            self.live_buffers["encoder"].append(f.encoder)
            self.live_buffers["error"].append(f.error)

            if not self._session_capped:
                self.session_records.append(
                    (f.seq, f.timestamp_ms, f.voltage, f.current, f.encoder, f.error)
                )
                if len(self.session_records) >= MAX_SESSION_SAMPLES:
                    self._session_capped = True
                    self.statusBar().showMessage(
                        "Session buffer full (2,000,000 samples) - save and reconnect to continue capturing."
                    )

        t_arr = np.fromiter(self.live_t, dtype=np.float64)
        for name, row in self.rows.items():
            y_arr = np.fromiter(self.live_buffers[name], dtype=np.float64)
            row.curve.setData(t_arr, y_arr)

        self.samples_label.setText(f"{len(self.session_records):,} samples captured")

    def _get_live_buffer(self, name: str, n: int):
        buf = self.live_buffers.get(name)
        if buf is None or len(buf) < n:
            return np.fromiter(buf, dtype=np.float64) if buf else None
        return np.fromiter(buf, dtype=np.float64)[-n:]

    # ---------------------------------------------------------------- FFT
    def _open_fft_window(self, name, label):
        win = FFTWindow(name, label, self._get_live_buffer, parent=self)
        win.destroyed.connect(lambda: self._fft_windows.remove(win) if win in self._fft_windows else None)
        self._fft_windows.append(win)
        win.show()

    # ------------------------------------------------------------- Export
    def _selected_signals(self):
        return [name for name, row in self.rows.items() if row.checkbox.isChecked()]

    def _save_csv(self):
        if self._csv_worker is not None:
            QMessageBox.information(self, "Export in progress", "A CSV export is already running.")
            return

        selected = self._selected_signals()
        if not selected:
            QMessageBox.information(self, "Nothing selected", "Check at least one signal to export.")
            return
        if not self.session_records:
            QMessageBox.information(self, "No data", "No samples captured yet.")
            return

        path, _ = QFileDialog.getSaveFileName(self, "Save CSV", "hca1001_adc_log.csv", "CSV Files (*.csv)")
        if not path:
            return

        col_index = {"voltage": 2, "current": 3, "encoder": 4, "error": 5}
        header = ["seq", "timestamp_ms"] + [SIGNAL_LABELS[n] for n in selected]
        col_indices = [col_index[n] for n in selected]

        # Snapshot now (GUI thread) so the export thread never iterates a list
        # that _on_frames() is concurrently appending to.
        snapshot = list(self.session_records)

        self.save_csv_btn.setEnabled(False)
        self.statusBar().showMessage(f"Saving {len(snapshot):,} rows to {path} ...")

        self._csv_worker = CsvExportWorker(path, header, snapshot, col_indices)
        self._csv_worker.finished_ok.connect(self._on_csv_saved)
        self._csv_worker.failed.connect(self._on_csv_failed)
        self._csv_worker.start()

    def _on_csv_saved(self, path, row_count):
        self.save_csv_btn.setEnabled(True)
        self._csv_worker = None
        self.statusBar().showMessage(f"Saved {row_count:,} rows to {path}")

    def _on_csv_failed(self, message):
        self.save_csv_btn.setEnabled(True)
        self._csv_worker = None
        QMessageBox.critical(self, "Save failed", message)

    def _save_png(self):
        selected = self._selected_signals()
        if not selected:
            QMessageBox.information(self, "Nothing selected", "Check at least one signal to export.")
            return

        directory = QFileDialog.getExistingDirectory(self, "Choose folder for PNG export")
        if not directory:
            return

        stamp = time.strftime("%Y%m%d_%H%M%S")
        saved = []
        for name in selected:
            row = self.rows[name]
            exporter = pg.exporters.ImageExporter(row.plot.plotItem)
            out_path = f"{directory}/{name}_{stamp}.png"
            exporter.export(out_path)
            saved.append(out_path)

        self.statusBar().showMessage(f"Saved {len(saved)} PNG file(s) to {directory}")

    def closeEvent(self, event):
        if self.worker is not None:
            self._disconnect()
        if self._csv_worker is not None:
            self._csv_worker.wait(3000)
        super().closeEvent(event)


def main():
    app = QApplication(sys.argv)
    apply_dark_theme(app)
    window = MainWindow()
    window.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
