"""HCA coefficient editor: read, edit, send, and file-persist the per-harmonic
complex PI gains running on the MCU.

One row per harmonic order, 1..MAX_HARMONIC_ORDER. A row is only pushed to the
device when its "Send" box is ticked.

This edits gains, it never creates channels. Which harmonic orders have a
control channel is fixed at boot by the HCA_Add_Channel calls in main(), so the
40kHz control ISR's per-sample workload never changes underneath it; sending to
an order with no channel is ignored by the firmware. Click "Read from MCU"
first and the rows the device actually runs are the ones that fill in -- the
rest are marked "no channel on device" and need a firmware change, not a
different value here.

Nothing typed here is authoritative. The device echoes each channel back after
applying it (see ApplyCoeffCommand in Core/Src/main.c), and that echo is what
fills the "On MCU" column, so a value that did not take effect is visible
rather than silent.
"""

import csv

from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtWidgets import (
    QAbstractItemView,
    QCheckBox,
    QFileDialog,
    QHBoxLayout,
    QHeaderView,
    QLabel,
    QMainWindow,
    QMessageBox,
    QPushButton,
    QTableWidget,
    QTableWidgetItem,
    QVBoxLayout,
    QWidget,
)

from protocol import MAX_HARMONIC_ORDER

COL_SEND, COL_ORDER, COL_KP_RE, COL_KP_IM, COL_KI_RE, COL_KI_IM, COL_MCU = range(7)
HEADERS = ("Send", "Harmonic", "Kp real", "Kp imag", "Ki real", "Ki imag", "On MCU")
EDITABLE_COLS = (COL_KP_RE, COL_KP_IM, COL_KI_RE, COL_KI_IM)

CSV_HEADER = ["order", "kp_real", "kp_imag", "ki_real", "ki_imag", "enabled"]

# How long to wait for the device's echo before calling a send/read unanswered.
REPLY_TIMEOUT_MS = 1500


class CoefficientsWindow(QMainWindow):
    """Editor for the HCA channel gains.

    `get_worker_fn` is a callable rather than a stored SerialWorker because the
    worker is recreated on every reconnect; resolving it per action means this
    window survives a disconnect/reconnect cycle without being rebuilt.
    """

    def __init__(self, get_worker_fn, parent=None):
        super().__init__(parent)
        self._get_worker = get_worker_fn
        self._replies_since_request = 0
        self._pending_read = False
        self._reported_orders = set()   # orders the device has confirmed it runs

        self.setWindowTitle("HCA Coefficients (Kp / Ki per harmonic)")
        self.resize(880, 640)

        central = QWidget()
        layout = QVBoxLayout(central)
        self.setCentralWidget(central)

        # --- device actions ---
        dev_bar = QHBoxLayout()
        self.read_btn = QPushButton("Read from MCU")
        self.read_btn.setToolTip("Ask the device which channels exist and what gains they run")
        self.read_btn.clicked.connect(self._read_from_mcu)
        dev_bar.addWidget(self.read_btn)

        self.send_btn = QPushButton("Send Checked to MCU")
        self.send_btn.setToolTip(
            "Push every ticked row. Orders the device has no channel for are\n"
            "ignored by the firmware -- channels are fixed at boot in main()."
        )
        self.send_btn.clicked.connect(self._send_checked)
        dev_bar.addWidget(self.send_btn)

        self.send_row_btn = QPushButton("Send Selected Row")
        self.send_row_btn.clicked.connect(self._send_selected_row)
        dev_bar.addWidget(self.send_row_btn)

        self.reset_btn = QPushButton("Reset Integrators")
        self.reset_btn.setToolTip(
            "Zero every channel's PI integrator and the shared disperser window\n"
            "on the device. Gains are left alone. The controller is live, so this\n"
            "puts a transient on the output."
        )
        self.reset_btn.clicked.connect(self._reset_integrators)
        dev_bar.addWidget(self.reset_btn)

        dev_bar.addSpacing(16)
        self.count_label = QLabel("")
        self.count_label.setObjectName("statsLabel")
        self.count_label.setToolTip(
            "Rows ticked for sending. Only orders that already have a channel on\n"
            "the device can be updated -- read from the MCU to see which those are."
        )
        dev_bar.addWidget(self.count_label)
        dev_bar.addStretch(1)
        layout.addLayout(dev_bar)

        # --- file actions ---
        file_bar = QHBoxLayout()
        save_btn = QPushButton("Save CSV...")
        save_btn.clicked.connect(self._save_csv)
        file_bar.addWidget(save_btn)

        load_btn = QPushButton("Load CSV...")
        load_btn.clicked.connect(self._load_csv)
        file_bar.addWidget(load_btn)

        file_bar.addSpacing(16)
        check_all_btn = QPushButton("Check All")
        check_all_btn.clicked.connect(lambda: self._set_all_checked(True))
        file_bar.addWidget(check_all_btn)

        uncheck_all_btn = QPushButton("Uncheck All")
        uncheck_all_btn.clicked.connect(lambda: self._set_all_checked(False))
        file_bar.addWidget(uncheck_all_btn)

        clear_btn = QPushButton("Zero Gains")
        clear_btn.setToolTip("Set every editable cell back to 0 (does not touch the device)")
        clear_btn.clicked.connect(self._zero_gains)
        file_bar.addWidget(clear_btn)

        file_bar.addStretch(1)
        layout.addLayout(file_bar)

        # --- table ---
        self.table = QTableWidget(MAX_HARMONIC_ORDER, len(HEADERS))
        self.table.setHorizontalHeaderLabels(HEADERS)
        self.table.setSelectionBehavior(QAbstractItemView.SelectRows)
        self.table.verticalHeader().setVisible(False)
        header = self.table.horizontalHeader()
        header.setSectionResizeMode(QHeaderView.Stretch)
        header.setSectionResizeMode(COL_SEND, QHeaderView.ResizeToContents)
        header.setSectionResizeMode(COL_ORDER, QHeaderView.ResizeToContents)

        self._send_boxes = []
        for row in range(MAX_HARMONIC_ORDER):
            order = row + 1

            box = QCheckBox()
            box.toggled.connect(self._update_count_label)
            holder = QWidget()  # wrapper so the box centres in its cell
            holder_layout = QHBoxLayout(holder)
            holder_layout.setContentsMargins(0, 0, 0, 0)
            holder_layout.addWidget(box, alignment=Qt.AlignCenter)
            self.table.setCellWidget(row, COL_SEND, holder)
            self._send_boxes.append(box)

            order_item = QTableWidgetItem(f"H{order}")
            order_item.setFlags(order_item.flags() & ~Qt.ItemIsEditable)
            self.table.setItem(row, COL_ORDER, order_item)

            for col in EDITABLE_COLS:
                self.table.setItem(row, col, QTableWidgetItem("0"))

            mcu_item = QTableWidgetItem("-")
            mcu_item.setFlags(mcu_item.flags() & ~Qt.ItemIsEditable)
            self.table.setItem(row, COL_MCU, mcu_item)

        layout.addWidget(self.table, stretch=1)

        self.status_label = QLabel("Not read from device yet.")
        self.status_label.setObjectName("statsLabel")
        layout.addWidget(self.status_label)

        self._reply_timer = QTimer(self)
        self._reply_timer.setSingleShot(True)
        self._reply_timer.timeout.connect(self._on_reply_timeout)

        self._update_count_label()

    # ------------------------------------------------------------- helpers
    def _row_values(self, row):
        """(kp, ki) as complex numbers. Raises ValueError on an unparseable cell."""
        vals = []
        for col in EDITABLE_COLS:
            text = self.table.item(row, col).text().strip()
            try:
                vals.append(float(text) if text else 0.0)
            except ValueError:
                raise ValueError(f"H{row + 1}, {HEADERS[col]}: '{text}' is not a number")
        return complex(vals[0], vals[1]), complex(vals[2], vals[3])

    def _checked_rows(self):
        return [r for r, box in enumerate(self._send_boxes) if box.isChecked()]

    def _set_all_checked(self, checked):
        for box in self._send_boxes:
            box.setChecked(checked)

    def _zero_gains(self):
        for row in range(MAX_HARMONIC_ORDER):
            for col in EDITABLE_COLS:
                self.table.item(row, col).setText("0")

    def _update_count_label(self):
        self.count_label.setText(f"Checked: {len(self._checked_rows())} / {MAX_HARMONIC_ORDER}")

    def _worker_or_warn(self):
        worker = self._get_worker()
        if worker is None:
            QMessageBox.warning(
                self, "Not connected", "Connect to the device first (main window)."
            )
        return worker

    def _arm_reply_timer(self):
        self._replies_since_request = 0
        self._reply_timer.start(REPLY_TIMEOUT_MS)

    def _on_reply_timeout(self):
        """Every reply the device was going to send has arrived by now."""
        if self._replies_since_request == 0:
            self.status_label.setText(
                "No reply from the device. Either the firmware with coefficient "
                "support is not flashed, or none of the orders sent has a channel."
            )
            self._pending_read = False
            return

        if self._pending_read:
            self._pending_read = False
            self._mark_absent_orders()

    def _mark_absent_orders(self):
        """Flag every order the device did not report. Those have no channel --
        the firmware ignores writes to them, so make that visible instead of
        letting a send quietly do nothing."""
        missing = 0
        for row in range(MAX_HARMONIC_ORDER):
            if (row + 1) in self._reported_orders:
                continue
            self.table.item(row, COL_MCU).setText("no channel on device")
            self._send_boxes[row].setChecked(False)
            missing += 1
        self._update_count_label()
        self.status_label.setText(
            f"Device runs {len(self._reported_orders)} channel(s): "
            f"{', '.join('H%d' % o for o in sorted(self._reported_orders)) or 'none'}. "
            f"The other {missing} order(s) have no channel -- add them with "
            f"HCA_Add_Channel() in main() and reflash to make them settable."
        )

    # ------------------------------------------------------------- device
    def _read_from_mcu(self):
        worker = self._worker_or_warn()
        if worker is None:
            return
        self._reported_orders.clear()
        worker.request_coefficients()
        self._pending_read = True
        self._arm_reply_timer()
        self.status_label.setText("Reading coefficients from device...")

    def _reset_integrators(self):
        worker = self._worker_or_warn()
        if worker is None:
            return
        # Confirmed because it perturbs a running controller, not just the GUI.
        if QMessageBox.question(
            self,
            "Reset integrators?",
            "Zero every channel's integrator and the disperser window on the "
            "device?\n\nGains are not affected. The controller is running, so "
            "expect a transient on the output.",
            QMessageBox.Yes | QMessageBox.No,
            QMessageBox.No,
        ) != QMessageBox.Yes:
            return

        worker.reset_integrators()
        self._pending_read = False
        self._arm_reply_timer()
        self.status_label.setText("Integrators reset; waiting for the device to confirm...")

    def _send_rows(self, rows):
        worker = self._worker_or_warn()
        if worker is None:
            return
        try:
            payloads = [(r + 1, *self._row_values(r)) for r in rows]
        except ValueError as exc:
            QMessageBox.warning(self, "Invalid value", str(exc))
            return

        for order, kp, ki in payloads:
            worker.set_coefficient(order, kp, ki)
        self._pending_read = False
        self._arm_reply_timer()
        self.status_label.setText(
            f"Sent {len(payloads)} channel(s); waiting for the device to echo them back..."
        )

    def _send_checked(self):
        rows = self._checked_rows()
        if not rows:
            QMessageBox.information(
                self, "Nothing to send", "Tick the Send box on the rows you want to push."
            )
            return
        self._send_rows(rows)

    def _send_selected_row(self):
        rows = sorted({idx.row() for idx in self.table.selectedIndexes()})
        if not rows:
            QMessageBox.information(self, "No row selected", "Select a row in the table first.")
            return
        self._send_rows(rows)

    def on_coeff_frames(self, frames):
        """Fold echoes from the device into the table. Called by MainWindow on
        its GUI timer; safe to call with frames that arrived unsolicited."""
        if not frames:
            return
        self._replies_since_request += len(frames)

        for f in frames:
            row = f.order - 1
            if not (0 <= row < MAX_HARMONIC_ORDER):
                continue  # an order outside what this table addresses
            self._reported_orders.add(f.order)
            self.table.item(row, COL_MCU).setText(
                f"kp {f.kp_real:.4g}{f.kp_imag:+.4g}j   ki {f.ki_real:.4g}{f.ki_imag:+.4g}j"
            )
            # Mirror into the editable cells so the table shows what is actually
            # running; the user's typed value only differs until it is sent.
            for col, value in zip(EDITABLE_COLS, (f.kp_real, f.kp_imag, f.ki_real, f.ki_imag)):
                self.table.item(row, col).setText(f"{value:.6g}")
            self._send_boxes[row].setChecked(True)

        last = frames[-1]
        self.status_label.setText(
            f"Device reports {last.count} active channel(s); "
            f"updated {len(frames)} row(s) from its echo."
        )

    # --------------------------------------------------------------- files
    def _save_csv(self):
        path, _ = QFileDialog.getSaveFileName(
            self, "Save coefficients", "hca_coefficients.csv", "CSV files (*.csv)"
        )
        if not path:
            return
        try:
            rows = [(r + 1, *self._row_values(r)) for r in range(MAX_HARMONIC_ORDER)]
        except ValueError as exc:
            QMessageBox.warning(self, "Invalid value", str(exc))
            return

        try:
            with open(path, "w", newline="") as fh:
                writer = csv.writer(fh)
                writer.writerow(CSV_HEADER)
                for order, kp, ki in rows:
                    writer.writerow(
                        [
                            order,
                            f"{kp.real:.9g}",
                            f"{kp.imag:.9g}",
                            f"{ki.real:.9g}",
                            f"{ki.imag:.9g}",
                            int(self._send_boxes[order - 1].isChecked()),
                        ]
                    )
        except OSError as exc:
            QMessageBox.critical(self, "Save failed", str(exc))
            return
        self.status_label.setText(f"Saved {len(rows)} rows to {path}")

    def _load_csv(self):
        path, _ = QFileDialog.getOpenFileName(
            self, "Load coefficients", "", "CSV files (*.csv);;All files (*)"
        )
        if not path:
            return
        try:
            with open(path, newline="") as fh:
                rows = list(csv.DictReader(fh))
        except OSError as exc:
            QMessageBox.critical(self, "Load failed", str(exc))
            return

        if not rows or "order" not in (rows[0].keys() if rows else {}):
            QMessageBox.warning(
                self,
                "Unrecognised file",
                "Expected a header row starting with 'order' "
                f"(columns: {', '.join(CSV_HEADER)}).",
            )
            return

        applied, skipped = 0, []
        for raw in rows:
            try:
                order = int(float(raw["order"]))
                values = [
                    float(raw.get(key) or 0.0)
                    for key in ("kp_real", "kp_imag", "ki_real", "ki_imag")
                ]
            except (TypeError, ValueError):
                skipped.append(str(raw.get("order")))
                continue
            if not (1 <= order <= MAX_HARMONIC_ORDER):
                skipped.append(str(order))
                continue

            row = order - 1
            for col, value in zip(EDITABLE_COLS, values):
                self.table.item(row, col).setText(f"{value:.6g}")
            # A file without an 'enabled' column is taken as "every listed row
            # is wanted", which is how a hand-written CSV reads.
            enabled = raw.get("enabled")
            self._send_boxes[row].setChecked(
                True if enabled is None else str(enabled).strip() not in ("0", "", "false", "False")
            )
            applied += 1

        self._update_count_label()
        msg = f"Loaded {applied} row(s) from {path}. Nothing sent to the device yet."
        if skipped:
            msg += f" Skipped orders outside 1..{MAX_HARMONIC_ORDER}: {', '.join(skipped)}."
        self.status_label.setText(msg)
