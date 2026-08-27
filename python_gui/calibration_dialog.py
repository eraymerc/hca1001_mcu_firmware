"""Sensor calibration: solve the voltage sensor's gain and offset on the device.

The firmware calibrates against its own modulator (see the "Sensor calibration"
block in Core/Src/main.c): over a whole number of fundamental cycles it
correlates both the duty it commanded and the uncalibrated sensor reading
against the control loop's sine, which gives the fundamental component of each.
The commanded duty becomes volts only once the DC bus voltage is known -- and
the firmware cannot measure that, which is why this dialog asks the operator
for it. A wrong Vdc scales the resulting gain wrong by exactly that factor.

Preconditions the device cannot check for you:
  * the bridge is switching and the sensor sits on its output,
  * Vdc is the real bus voltage under load,
  * closed loop, the controller has settled -- the calibration is only as good
    as the loop's tracking at the moment it runs.

Nothing typed here is authoritative: the device applies the result itself and
reports back what it derived and the measurements behind it, so a run against a
dead output shows up as a refusal rather than a silently skewed sensor.
"""

from PyQt5.QtCore import QTimer
from PyQt5.QtWidgets import (
    QDialog,
    QDoubleSpinBox,
    QFormLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QMessageBox,
    QPushButton,
    QVBoxLayout,
)

from protocol import (
    CAL_STATUS_OK,
    CAL_STATUS_RESTORED,
    CAL_STATUS_TEXT,
)

# The run takes ~0.6s on the device (settle + measure window). Give it room for
# the reply to make it back before calling it unanswered.
REPLY_TIMEOUT_MS = 4000

# Matches CAL_VDC_MIN/CAL_VDC_MAX in Core/Src/main.c. The device range-checks
# too -- this only spares the round trip.
VDC_MIN = 10.0
VDC_MAX = 1000.0


class CalibrationDialog(QDialog):
    """Runs the device-side sensor calibration and shows what it derived.

    `get_worker_fn` is a callable rather than a stored SerialWorker for the same
    reason as in CoefficientsWindow: the worker is recreated on every reconnect.
    """

    def __init__(self, get_worker_fn, parent=None):
        super().__init__(parent)
        self._get_worker = get_worker_fn

        self.setWindowTitle("Voltage Sensor Calibration")
        self.setMinimumWidth(520)

        root = QVBoxLayout(self)

        intro = QLabel(
            "The device solves its own sensor gain and offset by comparing what "
            "the modulator commanded against what the sensor read. It needs the "
            "DC bus voltage to turn duty into volts -- enter the real bus voltage "
            "under load.\n\n"
            "The bridge must be switching with the sensor on its output while "
            "this runs (~1 second)."
        )
        intro.setWordWrap(True)
        root.addWidget(intro)

        form = QFormLayout()
        self.vdc_spin = QDoubleSpinBox()
        self.vdc_spin.setDecimals(2)
        self.vdc_spin.setRange(VDC_MIN, VDC_MAX)
        self.vdc_spin.setSingleStep(1.0)
        self.vdc_spin.setValue(311.0)
        self.vdc_spin.setSuffix(" V")
        self.vdc_spin.setToolTip("DC bus voltage feeding the bridge, measured under load")
        form.addRow("Vdc:", self.vdc_spin)
        root.addLayout(form)

        btn_row = QHBoxLayout()
        self.calibrate_btn = QPushButton("Calibrate")
        self.calibrate_btn.clicked.connect(self._on_calibrate)
        btn_row.addWidget(self.calibrate_btn)

        self.read_btn = QPushButton("Read from MCU")
        self.read_btn.setToolTip("Report the calibration currently in force")
        self.read_btn.clicked.connect(self._on_read)
        btn_row.addWidget(self.read_btn)

        self.restore_btn = QPushButton("Restore Defaults")
        self.restore_btn.setToolTip("Put the build-time calibration back on the device")
        self.restore_btn.clicked.connect(self._on_restore)
        btn_row.addWidget(self.restore_btn)

        btn_row.addStretch(1)
        close_btn = QPushButton("Close")
        close_btn.clicked.connect(self.close)
        btn_row.addWidget(close_btn)
        root.addLayout(btn_row)

        result_box = QGroupBox("On MCU")
        result_form = QFormLayout(result_box)
        self.gain_label = QLabel("-")
        self.offset_label = QLabel("-")
        self.measured_label = QLabel("-")
        self.expected_label = QLabel("-")
        result_form.addRow("Sensor gain:", self.gain_label)
        result_form.addRow("Sensor offset:", self.offset_label)
        result_form.addRow("Measured (uncalibrated):", self.measured_label)
        result_form.addRow("Commanded:", self.expected_label)
        root.addWidget(result_box)

        self.status_label = QLabel("")
        self.status_label.setWordWrap(True)
        root.addWidget(self.status_label)

        self._reply_timer = QTimer(self)
        self._reply_timer.setSingleShot(True)
        self._reply_timer.timeout.connect(self._on_reply_timeout)

    # ------------------------------------------------------------- actions
    def _worker_or_warn(self):
        worker = self._get_worker()
        if worker is None:
            QMessageBox.warning(
                self, "Not connected", "Connect to the device first (main window)."
            )
        return worker

    def _on_calibrate(self):
        worker = self._worker_or_warn()
        if worker is None:
            return
        vdc = self.vdc_spin.value()
        if QMessageBox.question(
            self,
            "Run calibration?",
            f"This measures the live output for about a second and overwrites the "
            f"sensor calibration on the device.\n\n"
            f"Vdc = {vdc:.2f} V. A wrong Vdc skews the sensor by the same factor.\n\n"
            f"Continue?",
        ) != QMessageBox.Yes:
            return

        worker.calibrate(vdc)
        self._begin_wait("Calibrating... (measuring for about a second)")

    def _on_read(self):
        worker = self._worker_or_warn()
        if worker is None:
            return
        worker.request_calibration()
        self._begin_wait("Reading calibration from the device...")

    def _on_restore(self):
        worker = self._worker_or_warn()
        if worker is None:
            return
        worker.restore_default_calibration()
        self._begin_wait("Restoring the build-time calibration...")

    def _begin_wait(self, message):
        self.status_label.setText(message)
        self.calibrate_btn.setEnabled(False)
        self._reply_timer.start(REPLY_TIMEOUT_MS)

    def _on_reply_timeout(self):
        self.calibrate_btn.setEnabled(True)
        self.status_label.setText(
            "No reply from the device. Either the firmware with calibration "
            "support is not flashed, or the control ISR is not running."
        )

    # -------------------------------------------------------------- replies
    def on_cal_frame(self, frame):
        """One HcaCalFrame_t from the device: the calibration in force, plus the
        measurements it came from when a run actually happened."""
        self._reply_timer.stop()
        self.calibrate_btn.setEnabled(True)

        self.gain_label.setText(f"{frame.gain:.6g}")
        self.offset_label.setText(f"{frame.offset:.6g} V")

        if frame.status in (CAL_STATUS_OK, CAL_STATUS_RESTORED):
            # Only these two changed anything; the rest report a refusal and
            # leave the coefficients above as they were.
            if frame.status == CAL_STATUS_OK:
                self.measured_label.setText(
                    f"peak {frame.raw_peak:.4g} V, DC {frame.raw_dc:.4g} V"
                )
                self.expected_label.setText(
                    f"peak {frame.expected_peak:.4g} V  (Vdc {frame.vdc:.2f} V)"
                )
        else:
            self.measured_label.setText(
                f"peak {frame.raw_peak:.4g} V, DC {frame.raw_dc:.4g} V"
            )
            self.expected_label.setText(f"peak {frame.expected_peak:.4g} V")

        self.status_label.setText(
            CAL_STATUS_TEXT.get(frame.status, f"Unknown status {frame.status}.")
        )
