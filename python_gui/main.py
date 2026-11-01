"""HCA1001 ADC Monitor - dark-mode PyQt5 GUI.

Connects to the STM32G474RE over the ST-LINK's virtual COM port (LPUART1,
8,000,000 baud, see Core/Src/main.c) -- a single USB cable, no external
UART adapter needed. Streams the single ADC1 Voltage reading plus the
HCA reference and error signals at the full 40kHz loop rate, plots them live, and can save the captured data as CSV
and the plots as PNG. Each signal has its own button to open a live FFT
window.

Run:
    pip install -r requirements.txt
    python main.py
"""

import sys
import time
from collections import deque
from itertools import islice

import numpy as np
import pyqtgraph as pg
import pyqtgraph.exporters
from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtWidgets import (
    QApplication,
    QCheckBox,
    QComboBox,
    QDoubleSpinBox,
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

from calibration_dialog import CalibrationDialog
from coefficients_window import CoefficientsWindow
from dark_theme import apply_dark_theme
from export_worker import CsvExportWorker
from fft_window import FFTWindow
from protocol import SIGNAL_LABELS, SIGNAL_NAMES, STREAM_RATE_HZ, V_PEAK_NOM
from recorder import ARMING, DONE, FAILED, Recorder
from serial_worker import SerialWorker
from tuner_window import TunerWindow

DEFAULT_LIVE_WINDOW_SECONDS = 0.1  # five 50Hz periods
MIN_LIVE_WINDOW_SECONDS = 0.005
MAX_LIVE_WINDOW_SECONDS = 5.0  # 200,000 samples a row at 40kHz
# Capture buffer is sized for the largest window any row could ask for;
# each row's spinbox just controls how much of that trailing history it
# displays, so changing a spinbox never needs to resize a deque.
MAX_LIVE_SAMPLES = int(MAX_LIVE_WINDOW_SECONDS * STREAM_RATE_HZ)
MAX_SESSION_SAMPLES = 2_000_000  # 50s at 40kHz, bounds RAM use for CSV export
# Record button: capture length range. The top end matches the session buffer.
DEFAULT_RECORD_SECONDS = 0.1
MIN_RECORD_SECONDS = 0.005
MAX_RECORD_SECONDS = MAX_SESSION_SAMPLES / STREAM_RATE_HZ
# Pixel size of the PNG drawn for each recorded signal.
RECORD_PNG_SIZE = (1600, 500)

# Display-only ideal low-pass filter (connection bar, top right). The cutoff
# tops out at the stream's Nyquist frequency, where the filter is a no-op.
DEFAULT_LPF_CUTOFF_HZ = 3000.0
MIN_LPF_CUTOFF_HZ = 10.0
MAX_LPF_CUTOFF_HZ = STREAM_RATE_HZ / 2
# Mirrored samples added on each side before the FFT. The FFT treats its input
# as periodic, so an unpadded window would wrap its last sample onto its first
# and ring (Gibbs) at both edges; mirroring keeps the wrap far from the data.
LPF_PAD_SECONDS = 0.02  # one 50Hz period

# Extra history the trigger search may look back through beyond the widest
# row's window -- a few periods of the slowest fundamental it will see.
TRIGGER_LOOKBACK_SECONDS = 0.1

# GUI redraw cadence. Deliberately decoupled from the 40kHz data rate: the
# worker thread just buffers parsed frames (see serial_worker.drain_frames),
# and this timer pulls+redraws on its own schedule. If a redraw ever takes
# longer than this interval, the next pull just picks up a bigger batch -
# there is no per-sample signal backlog that can pile up.
GUI_UPDATE_INTERVAL_MS = 50  # 20 Hz

# How much stream history each rate/loss estimate is computed over. Long enough
# that HAL_GetTick's 1ms resolution contributes well under a percent of error.
RATE_WINDOW_MS = 2000
# Report the link as healthy only within this much of the nominal rate. The FFT
# scales frequency by the assumed rate, so a 20% shortfall reads a 50Hz
# fundamental as 62.5Hz -- worth flagging loudly rather than leaving to be
# discovered in the spectrum.
RATE_TOLERANCE = 0.02


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
        fft_btn.setFixedWidth(90)
        fft_btn.clicked.connect(lambda: open_fft_cb(self._name, label))
        side.addWidget(fft_btn)

        self.window_spin = QDoubleSpinBox()
        self.window_spin.setDecimals(3)
        self.window_spin.setSingleStep(0.01)
        self.window_spin.setRange(MIN_LIVE_WINDOW_SECONDS, MAX_LIVE_WINDOW_SECONDS)
        self.window_spin.setValue(DEFAULT_LIVE_WINDOW_SECONDS)
        self.window_spin.setSuffix(" s")
        self.window_spin.setFixedWidth(90)
        self.window_spin.setToolTip("Plotted time window")
        side.addWidget(self.window_spin)
        side.addStretch(1)

        side_widget = QWidget()
        side_widget.setLayout(side)
        side_widget.setFixedWidth(110)
        outer.addWidget(side_widget)

        plot_col = QVBoxLayout()
        plot_col.setContentsMargins(0, 0, 0, 0)

        self.stats_label = QLabel("Mean: -   RMS: -   Pk-Pk: -")
        self.stats_label.setObjectName("statsLabel")
        plot_col.addWidget(self.stats_label)

        self.plot = pg.PlotWidget()
        self.plot.showGrid(x=True, y=True, alpha=0.3)
        self.plot.setLabel("left", label)
        self.plot.setLabel("bottom", "Time", units="s")
        self.curve = self.plot.plot(pen=pg.mkPen("#00d0ff", width=1.2))
        # Drawn over the raw trace, like a scope's math channel; hidden while
        # the Ideal LPF checkbox is off.
        self.lpf_curve = self.plot.plot(pen=pg.mkPen("#ff9f1c", width=1.8))
        self.lpf_curve.setVisible(False)

        self.trigger_line = pg.InfiniteLine(angle=90, pen=pg.mkPen("#ffd400", width=1.5))
        self.trigger_line.setVisible(False)
        self.plot.addItem(self.trigger_line)

        plot_col.addWidget(self.plot, stretch=1)

        plot_widget = QWidget()
        plot_widget.setLayout(plot_col)
        outer.addWidget(plot_widget, stretch=1)

    def window_samples(self):
        return max(1, int(round(self.window_spin.value() * STREAM_RATE_HZ)))

    def set_stats(self, y: np.ndarray, y_lpf: np.ndarray | None = None,
                  extra: str = "", extra_lpf: str = ""):
        text = self._format_stats(y) + extra
        if y_lpf is not None:
            text += "      LPF  " + self._format_stats(y_lpf) + extra_lpf
        self.stats_label.setText(text)

    @staticmethod
    def _format_stats(y: np.ndarray) -> str:
        # True RMS: sqrt of the mean square of the samples themselves, so any
        # DC offset counts towards it (an AC-coupled reading would subtract
        # the mean first).
        rms = float(np.sqrt(np.mean(np.square(y, dtype=np.float64))))
        return f"Mean: {y.mean():.4g}   RMS: {rms:.4g}   Pk-Pk: {np.ptp(y):.4g}"

    def set_trigger_marker(self, x: float | None):
        if x is None:
            self.trigger_line.setVisible(False)
        else:
            self.trigger_line.setPos(x)
            self.trigger_line.setVisible(True)


class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("HCA1001 ADC Monitor (STM32G474RE, ST-LINK VCP)")
        self.resize(1150, 600)

        self.worker: SerialWorker | None = None
        self._csv_worker: CsvExportWorker | None = None
        self._rec_csv_worker: CsvExportWorker | None = None
        self._recorder: Recorder | None = None
        self._rec_target = None  # (directory, stamp, selected signals) of the running capture
        self._fft_windows = []
        self._coeff_window: CoefficientsWindow | None = None
        self._cal_dialog: CalibrationDialog | None = None
        self._tuner_window: TunerWindow | None = None
        self._session_capped = False
        self._seq0 = None

        # Rolling stream-integrity estimate, see _update_rate_stats
        self._rate_window_start_ms = None
        self._rate_window_first_seq = None
        self._rate_window_received = 0

        self.live_t = deque(maxlen=MAX_LIVE_SAMPLES)
        self.live_buffers = {name: deque(maxlen=MAX_LIVE_SAMPLES) for name in SIGNAL_NAMES}
        # Full-session record for CSV export: list of (seq, timestamp_ms, voltage, reference, error)
        self.session_records = []

        self._build_ui()
        self._refresh_ports()

        self._ping_timer = QTimer(self)
        self._ping_timer.setSingleShot(True)
        self._ping_timer.timeout.connect(self._on_ping_timeout)
        self._ping_pending = False

        self._gui_update_timer = QTimer(self)
        self._gui_update_timer.timeout.connect(self._pull_and_update)

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

        # Not gated on a connection: the editor is also useful offline for
        # preparing a coefficient set and saving it to CSV.
        self.coeff_btn = QPushButton("Coefficients")
        self.coeff_btn.setToolTip("Read, edit and send the HCA Kp/Ki gains per harmonic")
        self.coeff_btn.clicked.connect(self._open_coefficients_window)
        conn_layout.addWidget(self.coeff_btn)

        # Gated on a connection, unlike the coefficient editor: everything this
        # does happens on the device, against the live output.
        self.calibrate_btn = QPushButton("Calibrate")
        self.calibrate_btn.setEnabled(False)
        self.calibrate_btn.setToolTip(
            "Solve the voltage sensor's gain and offset on the device, against a\n"
            "known DC bus voltage and the modulator's own commanded duty"
        )
        self.calibrate_btn.clicked.connect(self._open_calibration_dialog)
        conn_layout.addWidget(self.calibrate_btn)

        self.tuner_btn = QPushButton("Tuner")
        self.tuner_btn.setEnabled(False)
        self.tuner_btn.setToolTip(
            "Grey Wolf search over the HCA integral gains, scored on the live\n"
            "system with the ITAE of the streamed error"
        )
        self.tuner_btn.clicked.connect(self._open_tuner_window)
        conn_layout.addWidget(self.tuner_btn)

        conn_layout.addStretch(1)
        self.rate_label = QLabel("")
        self.rate_label.setObjectName("statsLabel")
        self.rate_label.setToolTip(
            "Measured from the seq and timestamp_ms fields of the arriving frames.\n\n"
            "'sent' is how fast the device pushed frames: seq advances only on a\n"
            "successful push, so a device-side FIFO overflow shows up here as a\n"
            "rate below nominal, with no gaps.\n\n"
            "'lost' is the fraction of pushed frames that never arrived -- gaps in\n"
            "seq, i.e. bytes dropped between the device and this program.\n\n"
            "Either one skews the FFT: it assumes samples are uniformly spaced at\n"
            "the nominal rate."
        )
        conn_layout.addWidget(self.rate_label)

        self.status_label = QLabel("Disconnected")
        self.status_label.setObjectName("statusLabel")
        conn_layout.addWidget(self.status_label)

        lpf_tip = (
            "Ideal (brick-wall) low-pass on the plotted signals: every FFT bin\n"
            "above the cutoff is zeroed, everything below passes untouched.\n"
            "Drawn in orange over the raw trace, with its own stats. Display\n"
            "only -- CSV, Record and the FFT windows keep the raw samples."
        )
        self.lpf_enable = QCheckBox("Ideal LPF")
        self.lpf_enable.setToolTip(lpf_tip)
        conn_layout.addWidget(self.lpf_enable)

        self.lpf_cutoff = QDoubleSpinBox()
        self.lpf_cutoff.setDecimals(2)
        self.lpf_cutoff.setRange(MIN_LPF_CUTOFF_HZ / 1000, MAX_LPF_CUTOFF_HZ / 1000)
        self.lpf_cutoff.setSingleStep(0.5)
        self.lpf_cutoff.setValue(DEFAULT_LPF_CUTOFF_HZ / 1000)
        self.lpf_cutoff.setSuffix(" kHz")
        self.lpf_cutoff.setToolTip(lpf_tip)
        conn_layout.addWidget(self.lpf_cutoff)

        root.addWidget(conn_box)

        # --- trigger bar ---
        trig_box = QGroupBox("Trigger")
        trig_layout = QHBoxLayout(trig_box)

        self.trigger_enable = QCheckBox("Enable")
        trig_layout.addWidget(self.trigger_enable)

        trig_layout.addWidget(QLabel("Source:"))
        self.trigger_source = QComboBox()
        for name in SIGNAL_NAMES:
            self.trigger_source.addItem(SIGNAL_LABELS[name], name)
        trig_layout.addWidget(self.trigger_source)

        trig_layout.addWidget(QLabel("Edge:"))
        self.trigger_edge = QComboBox()
        self.trigger_edge.addItems(["Rising", "Falling"])
        trig_layout.addWidget(self.trigger_edge)

        trig_layout.addWidget(QLabel("Level:"))
        self.trigger_level = QDoubleSpinBox()
        self.trigger_level.setDecimals(4)
        self.trigger_level.setRange(-1_000_000.0, 1_000_000.0)
        self.trigger_level.setSingleStep(0.1)
        self.trigger_level.setValue(0.0)
        trig_layout.addWidget(self.trigger_level)

        trig_layout.addStretch(1)
        self.trigger_status_label = QLabel("")
        self.trigger_status_label.setObjectName("statsLabel")
        trig_layout.addWidget(self.trigger_status_label)

        root.addWidget(trig_box)

        # --- reference bar ---
        # The device owns the valid range (it depends on how the firmware was
        # built, open loop or closed), so the spin box stays wide open and the
        # label reports what the device says it applied.
        ref_box = QGroupBox("Reference")
        ref_layout = QHBoxLayout(ref_box)

        ref_layout.addWidget(QLabel("Multiplier:"))
        self.ref_spin = QDoubleSpinBox()
        self.ref_spin.setDecimals(4)
        self.ref_spin.setRange(0.0, 2.0)
        self.ref_spin.setSingleStep(0.01)
        self.ref_spin.setValue(0.85)
        self.ref_spin.setToolTip(
            "Scales the sine reference before the modulator.\n\n"
            "Open-loop builds accept values above the modulation index\n"
            "(overmodulation); closed-loop builds cap at it so the controller\n"
            "keeps headroom to correct with. The device clamps and echoes back\n"
            "what it actually applied."
        )
        ref_layout.addWidget(self.ref_spin)

        self.ref_apply_btn = QPushButton("Apply")
        self.ref_apply_btn.setEnabled(False)
        self.ref_apply_btn.clicked.connect(self._on_ref_apply_clicked)
        ref_layout.addWidget(self.ref_apply_btn)

        self.ref_read_btn = QPushButton("Read")
        self.ref_read_btn.setEnabled(False)
        self.ref_read_btn.clicked.connect(self._on_ref_read_clicked)
        ref_layout.addWidget(self.ref_read_btn)

        ref_layout.addStretch(1)
        self.ref_status_label = QLabel("")
        self.ref_status_label.setObjectName("statsLabel")
        ref_layout.addWidget(self.ref_status_label)

        root.addWidget(ref_box)

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

        export_layout.addSpacing(24)
        export_layout.addWidget(QLabel("Record:"))
        self.record_spin = QDoubleSpinBox()
        self.record_spin.setDecimals(3)
        self.record_spin.setSingleStep(0.02)
        self.record_spin.setRange(MIN_RECORD_SECONDS, MAX_RECORD_SECONDS)
        self.record_spin.setValue(DEFAULT_RECORD_SECONDS)
        self.record_spin.setSuffix(" s")
        self.record_spin.setFixedWidth(90)
        self.record_spin.setToolTip("Length of the capture, starting when Record is pressed")
        export_layout.addWidget(self.record_spin)

        self.record_align = QCheckBox("Start at ref. zero crossing")
        self.record_align.setToolTip(
            "Wait for the HCA reference's rising zero crossing before capturing,\n"
            "so every capture starts at 0 degrees of the fundamental"
        )
        export_layout.addWidget(self.record_align)

        self.record_btn = QPushButton("Record")
        self.record_btn.setToolTip(
            "Capture the selected signals for the set time, then save them as\n"
            "one CSV plus one PNG per signal. A gap in the stream restarts the\n"
            "capture, so a saved recording never has missing samples."
        )
        self.record_btn.clicked.connect(self._on_record_clicked)
        export_layout.addWidget(self.record_btn)

        self.record_status = QLabel("")
        self.record_status.setObjectName("statsLabel")
        export_layout.addWidget(self.record_status)
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
        self.ref_apply_btn.setEnabled(True)
        self.ref_read_btn.setEnabled(True)
        self.calibrate_btn.setEnabled(True)
        self.tuner_btn.setEnabled(True)
        self.statusBar().showMessage(f"Connected to {port_name}.")
        self._gui_update_timer.start(GUI_UPDATE_INTERVAL_MS)
        # Show what the device is running rather than whatever the box was left at.
        self.worker.request_reference_multiplier()

    def _on_disconnected(self):
        self._gui_update_timer.stop()
        self.status_label.setText("Disconnected")
        self.status_label.setObjectName("statusLabel")
        self.status_label.setStyle(self.status_label.style())
        self.connect_btn.setText("Connect")
        self.connect_btn.setEnabled(True)
        self.ping_btn.setEnabled(False)
        self.stream_btn.setEnabled(False)
        self.stream_btn.setChecked(False)
        self.stream_btn.setText("Start Streaming")
        self.ref_apply_btn.setEnabled(False)
        self.ref_read_btn.setEnabled(False)
        self.calibrate_btn.setEnabled(False)
        self.ref_status_label.setText("")
        self.tuner_btn.setEnabled(False)
        self._cancel_recording("Recording cancelled: disconnected.")
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
            self._cancel_recording("Recording cancelled: streaming stopped.")

    # -------------------------------------------------------------- Data
    def _pull_and_update(self):
        if self.worker is None:
            return
        frames = self.worker.drain_frames()
        if frames:
            self._on_frames(frames)
            # The tuner scores each candidate from these same samples, so it
            # needs them at the GUI's update rate, not the plot's.
            if self._tuner_window is not None:
                self._tuner_window.on_frames(frames)

        # Drained unconditionally so the queue cannot grow while the
        # coefficients window is closed.
        coeff_frames = self.worker.drain_coeff_frames()
        if coeff_frames:
            if self._coeff_window is not None:
                self._coeff_window.on_coeff_frames(coeff_frames)
            if self._tuner_window is not None:
                self._tuner_window.on_coeff_frames(coeff_frames)

        for frame in self.worker.drain_ref_frames():
            self._on_ref_frame(frame)

        cal_frames = self.worker.drain_cal_frames()
        if cal_frames and self._cal_dialog is not None:
            for frame in cal_frames:
                self._cal_dialog.on_cal_frame(frame)

    def _on_frames(self, frames):
        if not frames:
            return
        if self._seq0 is None:
            self._seq0 = frames[0].seq

        self._update_rate_stats(frames)
        if self._recorder is not None:
            self._feed_recorder(frames)

        for f in frames:
            # From the sample index, not timestamp_ms: the device stamps each
            # 8-sample batch with a 1ms tick, so the five batches of every
            # millisecond share one stamp and would be drawn on top of each
            # other -- the trace came out as 1ms-wide steps.
            t = (f.seq - self._seq0) / STREAM_RATE_HZ
            self.live_t.append(t)
            self.live_buffers["voltage"].append(f.voltage)
            self.live_buffers["reference"].append(f.reference)
            self.live_buffers["error"].append(f.error)

            if not self._session_capped:
                self.session_records.append(
                    (f.seq, f.timestamp_ms, f.voltage, f.reference, f.error)
                )
                if len(self.session_records) >= MAX_SESSION_SAMPLES:
                    self._session_capped = True
                    self.statusBar().showMessage(
                        "Session buffer full (2,000,000 samples) - save and reconnect to continue capturing."
                    )

        # Convert only the trailing history this redraw can use, not the whole
        # deque: at 40kHz that is up to 200,000 samples per signal, 20 times a
        # second. Every array below has the same length, so indices into one
        # are indices into all of them.
        max_window = max(row.window_samples() for row in self.rows.values())
        if self.trigger_enable.isChecked():
            max_window += int(TRIGGER_LOOKBACK_SECONDS * STREAM_RATE_HZ)
        t_arr = self._tail(self.live_t, max_window)

        # Filtered over the whole tail, not just the plotted window, so the
        # filter's edge effects mostly fall outside what is drawn.
        lpf_hz = self.lpf_cutoff.value() * 1000 if self.lpf_enable.isChecked() else None
        y_arrs = {name: self._tail(self.live_buffers[name], len(t_arr)) for name in self.rows}
        lpf_arrs = (
            {name: self._ideal_lowpass(y, lpf_hz) for name, y in y_arrs.items()}
            if lpf_hz is not None else {}
        )

        # Trigger: find the most recent edge crossing that still leaves a
        # full window's worth of samples after it, so every row -- each of
        # which may have a different window length -- can be sliced from
        # the exact same point in time and stay aligned with one another.
        trig_idx = None
        if self.trigger_enable.isChecked() and len(t_arr) > 1:
            source_name = self.trigger_source.currentData()
            source_y = y_arrs[source_name]
            row_window = max(row.window_samples() for row in self.rows.values())
            trig_idx = self._find_trigger_index(
                source_y, self.trigger_edge.currentText(), self.trigger_level.value(), row_window
            )
            self.trigger_status_label.setText("Triggered" if trig_idx is not None else "Waiting for trigger...")
        else:
            self.trigger_status_label.setText("")

        for name, row in self.rows.items():
            y_arr = y_arrs[name]
            n = min(len(t_arr), row.window_samples())

            if trig_idx is not None:
                end = min(trig_idx + n, len(y_arr))
                window = slice(trig_idx, end)
                t_window = t_arr[window] - t_arr[trig_idx]
            else:
                window = slice(-n, None) if n < len(y_arr) else slice(None)
                t_window = t_arr[window]
            y_window = y_arr[window]
            lpf_window = lpf_arrs[name][window] if lpf_arrs else None

            # The error row also reports Erms, rebuilt from the voltage and
            # reference streams over this row's own window.
            erms, erms_lpf = "", ""
            if name == "error":
                erms = self._format_erms(y_arrs["voltage"][window], y_arrs["reference"][window])
                if lpf_arrs:
                    erms_lpf = self._format_erms(
                        lpf_arrs["voltage"][window], lpf_arrs["reference"][window]
                    )

            row.curve.setData(t_window, y_window)
            row.lpf_curve.setVisible(lpf_window is not None)
            if lpf_window is not None:
                row.lpf_curve.setData(t_window, lpf_window)
            row.set_trigger_marker(0.0 if trig_idx is not None else None)

            if y_window.size:
                row.set_stats(y_window, lpf_window, erms, erms_lpf)

        self.samples_label.setText(f"{len(self.session_records):,} samples captured")

    @staticmethod
    def _tail(buf, n: int) -> np.ndarray:
        """The last n items of a deque as a float64 array, in O(n) rather than
        O(len(buf)) -- a deque cannot be sliced, but it can be walked backward."""
        n = min(n, len(buf))
        return np.fromiter(islice(reversed(buf), n), dtype=np.float64, count=n)[::-1]

    @staticmethod
    def _format_erms(voltage: np.ndarray, reference: np.ndarray) -> str:
        """Erms as the paper's eq. (10) defines it: RMS(vref - vout) / RMS(vref).

        DC excluded, unlike the RMS stats: the mean of each signal is removed
        first, so the sensor's DC offset does not swamp the AC tracking error.
        Same quantity as the error stream scaled to volts, but taken from the
        voltage stream's finer 1/16V resolution rather than the error's
        1/2048 p.u. (0.1V).
        """
        if voltage.size == 0:
            return ""
        vref = reference * V_PEAK_NOM
        erms = float(np.std(vref - voltage))
        vref_rms = float(np.std(vref))
        pct = f"{100.0 * erms / vref_rms:.3g} %" if vref_rms > 0 else "-"
        return f"   Erms: {erms:.4g} V   Erms: {pct}"

    @staticmethod
    def _ideal_lowpass(y: np.ndarray, cutoff_hz: float) -> np.ndarray:
        """Zero every spectral component of y above cutoff_hz."""
        if y.size < 2:
            return y
        pad = min(int(LPF_PAD_SECONDS * STREAM_RATE_HZ), y.size - 1)
        padded = np.pad(y, pad, mode="reflect")
        spectrum = np.fft.rfft(padded)
        spectrum[np.fft.rfftfreq(padded.size, 1.0 / STREAM_RATE_HZ) > cutoff_hz] = 0
        return np.fft.irfft(spectrum, n=padded.size)[pad:pad + y.size]

    @staticmethod
    def _find_trigger_index(y: np.ndarray, edge: str, level: float, min_post: int):
        """Search backward for the most recent edge crossing that still
        leaves at least `min_post` samples after it. Returns None if no
        qualifying crossing exists yet (e.g. just connected, or the signal
        never crosses `level`)."""
        n = len(y)
        search_end = n - min_post
        if search_end < 1:
            return None
        if edge == "Rising":
            for i in range(search_end - 1, 0, -1):
                if y[i - 1] < level <= y[i]:
                    return i
        else:
            for i in range(search_end - 1, 0, -1):
                if y[i - 1] > level >= y[i]:
                    return i
        return None

    def _update_rate_stats(self, frames):
        """Estimate the true stream rate and the in-transit loss from the frames
        themselves, rather than trusting STREAM_RATE_HZ.

        PushStreamFrame advances stream_seq for every batch, including one it
        drops because its FIFO is full, so both failure modes -- the device
        falling behind and bytes lost on the wire -- leave holes in seq. Comparing
        the seq span against the timestamp span gives the device's true sample
        rate, and comparing the received count against that span gives the loss.
        """
        last = frames[-1]
        if self._rate_window_start_ms is None:
            self._rate_window_start_ms = frames[0].timestamp_ms
            self._rate_window_first_seq = frames[0].seq
            self._rate_window_received = 0

        self._rate_window_received += len(frames)
        span_ms = last.timestamp_ms - self._rate_window_start_ms
        if span_ms < RATE_WINDOW_MS:
            return

        span_s = span_ms / 1000.0
        pushed = last.seq - self._rate_window_first_seq + 1
        received = self._rate_window_received
        device_hz = pushed / span_s
        host_hz = received / span_s
        lost = 1.0 - (received / pushed) if pushed > 0 else 0.0

        self._rate_window_start_ms = None  # start the next window fresh

        text = f"{host_hz:.0f} Hz in / {device_hz:.0f} Hz sent"
        if lost > 0.001:
            text += f" / {lost * 100:.1f}% lost"
        healthy = abs(host_hz - STREAM_RATE_HZ) <= RATE_TOLERANCE * STREAM_RATE_HZ
        self.rate_label.setText(text)
        self.rate_label.setObjectName("statsLabel" if healthy else "statusLabelError")
        self.rate_label.setStyle(self.rate_label.style())

        if not healthy:
            culprit = (
                "frames are being lost (seq gaps) - the device FIFO overflowed, "
                "the ST-LINK VCP dropped bytes, or the host fell behind reading"
                if lost > 0.01
                else "the device is sampling off nominal (no seq gaps) - check "
                "the ADC trigger and timer clock"
            )
            self.statusBar().showMessage(
                f"Stream is {host_hz:.0f} Hz, not the {STREAM_RATE_HZ:.0f} Hz the FFT "
                f"assumes: {culprit}. Spectrum frequencies read "
                f"{STREAM_RATE_HZ / host_hz:.2f}x high."
            )

    def _get_live_buffer(self, name: str, n: int):
        buf = self.live_buffers.get(name)
        if not buf:
            return None
        return self._tail(buf, n)

    # -------------------------------------------------- Reference multiplier
    def _on_ref_apply_clicked(self):
        if self.worker is None:
            return
        self.worker.set_reference_multiplier(self.ref_spin.value())
        self.statusBar().showMessage(
            f"Sent reference multiplier {self.ref_spin.value():.4f}, waiting for the device echo..."
        )

    def _on_ref_read_clicked(self):
        if self.worker is None:
            return
        self.worker.request_reference_multiplier()

    def _on_ref_frame(self, frame):
        """The device reports the value it actually runs, which is the request
        clamped to what this firmware build allows."""
        loop = "open loop" if frame.open_loop else "closed loop"
        self.ref_status_label.setText(
            f"device: {frame.value:.4f}   ({loop}, max {frame.limit:.4f})"
        )
        requested = self.ref_spin.value()
        self.ref_spin.blockSignals(True)
        self.ref_spin.setValue(frame.value)
        self.ref_spin.blockSignals(False)
        if abs(requested - frame.value) > 1e-6:
            self.statusBar().showMessage(
                f"Device clamped {requested:.4f} to {frame.value:.4f} "
                f"({loop} limit is {frame.limit:.4f})."
            )
        else:
            self.statusBar().showMessage(f"Reference multiplier is {frame.value:.4f}.")

    # -------------------------------------------------------------- Tuner
    def _open_tuner_window(self):
        if self._tuner_window is None:
            self._tuner_window = TunerWindow(lambda: self.worker, parent=self)
            self._tuner_window.setAttribute(Qt.WA_DeleteOnClose)
            self._tuner_window.destroyed.connect(self._on_tuner_window_closed)
        self._tuner_window.show()
        self._tuner_window.raise_()
        self._tuner_window.activateWindow()

    def _on_tuner_window_closed(self):
        self._tuner_window = None

    # -------------------------------------------------------- Calibration
    def _open_calibration_dialog(self):
        if self._cal_dialog is None:
            self._cal_dialog = CalibrationDialog(lambda: self.worker, parent=self)
            self._cal_dialog.destroyed.connect(self._on_cal_dialog_closed)
            self._cal_dialog.setAttribute(Qt.WA_DeleteOnClose)
        self._cal_dialog.show()
        self._cal_dialog.raise_()
        self._cal_dialog.activateWindow()
        # Open on what the device is actually running rather than a blank panel.
        if self.worker is not None:
            self.worker.request_calibration()

    def _on_cal_dialog_closed(self):
        self._cal_dialog = None

    # ------------------------------------------------------- Coefficients
    def _open_coefficients_window(self):
        if self._coeff_window is None:
            # Hands over a getter, not the worker itself: self.worker is
            # replaced on every reconnect, and the window outlives that.
            self._coeff_window = CoefficientsWindow(lambda: self.worker, parent=self)
            self._coeff_window.destroyed.connect(self._on_coeff_window_closed)
        self._coeff_window.show()
        self._coeff_window.raise_()
        self._coeff_window.activateWindow()

    def _on_coeff_window_closed(self):
        self._coeff_window = None

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

        col_index = {"voltage": 2, "reference": 3, "error": 4}
        header = ["seq", "timestamp_ms"] + [SIGNAL_LABELS[n] for n in selected]
        col_indices = [0, 1] + [col_index[n] for n in selected]

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

    # ------------------------------------------------------------ Record
    def _on_record_clicked(self):
        if self._recorder is not None:
            self._cancel_recording("Recording cancelled.")
            return
        if self._rec_csv_worker is not None:
            QMessageBox.information(self, "Save in progress", "The previous recording is still being saved.")
            return
        if self.worker is None or not self.stream_btn.isChecked():
            QMessageBox.information(self, "Not streaming", "Start streaming before recording.")
            return
        selected = self._selected_signals()
        if not selected:
            QMessageBox.information(self, "Nothing selected", "Check at least one signal to record.")
            return

        directory = QFileDialog.getExistingDirectory(self, "Choose folder for the recording")
        if not directory:
            return

        n_samples = max(1, int(round(self.record_spin.value() * STREAM_RATE_HZ)))
        self._recorder = Recorder(n_samples, self.record_align.isChecked())
        self._rec_target = (directory, time.strftime("%Y%m%d_%H%M%S"), selected)
        self.record_btn.setText("Cancel")
        self._show_record_progress()

    def _feed_recorder(self, frames):
        rec = self._recorder
        rec.feed(frames)
        if rec.state == DONE:
            self._recorder = None
            self.record_btn.setText("Record")
            self._save_recording(rec)
        elif rec.state == FAILED:
            self._cancel_recording(f"Recording failed: {rec.message}")
        else:
            self._show_record_progress()

    def _show_record_progress(self):
        rec = self._recorder
        if rec.state == ARMING:
            text = "Waiting for zero crossing..."
        else:
            text = f"Recording {rec.progress * 100:.0f}%"
        if rec.restarts:
            text += f"  ({rec.message})"
        self.record_status.setText(text)

    def _cancel_recording(self, message):
        if self._recorder is None:
            return
        self._recorder = None
        self._rec_target = None
        self.record_btn.setText("Record")
        self.record_status.setText("")
        self.statusBar().showMessage(message)

    def _save_recording(self, rec):
        directory, stamp, selected = self._rec_target
        self._rec_target = None
        base = f"{directory}/rec_{stamp}"
        records = rec.records

        # PNGs are drawn on the GUI thread (pyqtgraph needs it) and are quick;
        # the CSV can run to two million rows, so it goes to a worker thread.
        t = np.fromiter((r[2] for r in records), dtype=np.float64, count=len(records))
        col_index = {"voltage": 3, "reference": 4, "error": 5}
        for name in selected:
            y = np.fromiter((r[col_index[name]] for r in records), dtype=np.float64, count=len(records))
            self._export_record_png(f"{base}_{name}.png", SIGNAL_LABELS[name], t, y)

        header = ["seq", "timestamp_ms", "t_s"] + [SIGNAL_LABELS[n] for n in selected]
        col_indices = [0, 1, 2] + [col_index[n] for n in selected]
        self._rec_csv_worker = CsvExportWorker(f"{base}.csv", header, records, col_indices)
        self._rec_csv_worker.finished_ok.connect(
            lambda path, rows: self._on_recording_saved(path, rows, len(selected), rec.restarts)
        )
        self._rec_csv_worker.failed.connect(self._on_recording_save_failed)
        self.record_status.setText("Saving...")
        self._rec_csv_worker.start()

    @staticmethod
    def _export_record_png(path, label, t, y):
        """Draw one recorded signal on an off-screen plot styled like the live
        rows and export it. Peak downsampling keeps a long capture's envelope
        intact without handing the renderer millions of points."""
        plot = pg.PlotWidget()
        plot.showGrid(x=True, y=True, alpha=0.3)
        plot.setLabel("left", label)
        plot.setLabel("bottom", "Time", units="s")
        plot.setDownsampling(auto=True, mode="peak")
        plot.plot(t, y, pen=pg.mkPen("#00d0ff", width=1.2))
        # A never-shown widget is never laid out, and the exporter would scale
        # up its 640x480 default -- fonts and all. Showing it off-screen lays
        # it out at the real size without a window flashing up.
        plot.setAttribute(Qt.WA_DontShowOnScreen)
        plot.resize(*RECORD_PNG_SIZE)
        plot.show()
        QApplication.processEvents()
        exporter = pg.exporters.ImageExporter(plot.plotItem)
        exporter.export(path)
        plot.close()
        plot.deleteLater()

    def _on_recording_saved(self, path, row_count, png_count, restarts):
        self._rec_csv_worker = None
        self.record_status.setText("")
        note = f" ({restarts} restart(s) after lost samples)" if restarts else ""
        self.statusBar().showMessage(
            f"Recorded {row_count:,} samples to {path} + {png_count} PNG{note}"
        )

    def _on_recording_save_failed(self, message):
        self._rec_csv_worker = None
        self.record_status.setText("")
        QMessageBox.critical(self, "Save failed", message)

    def closeEvent(self, event):
        if self.worker is not None:
            self._disconnect()
        for worker in (self._csv_worker, self._rec_csv_worker):
            if worker is not None:
                worker.wait(3000)
        super().closeEvent(event)


def main():
    app = QApplication(sys.argv)
    apply_dark_theme(app)
    window = MainWindow()
    window.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
