"""Grey Wolf tuner: searches the HCA integral gains against the live hardware.

Every candidate the optimizer proposes is evaluated on the real system, not a
model. One evaluation is:

    push Ki for every channel  ->  optionally reset the integrators
    ->  wait out the settle time  ->  measure the stream for Tm
    ->  score it with the chosen objective in cost_function.py

Two objectives, and each brings its own measurement protocol (see
cost_function.py): THD wants a settled steady state and no reset, ITAE wants
the transient that follows one. Switching the objective switches both, so the
measurement always matches what is being minimised.

Kp is held at whatever the device reports; only Ki_real and Ki_imag move, two
dimensions per channel (16 for the eight channels the firmware boots with).
SET_COEFF carries Kp and Ki together, which is why the channel list has to be
read from the device before a run can start -- the tuner sends each channel's
own Kp straight back unchanged.

Because the search runs on live power hardware, three things bound it: the box
bounds on every dimension, an abort threshold that cuts an evaluation short the
moment the error signal blows up, and the fact that the pack is seeded with the
gains already running, so the optimizer starts from a point known to work.
Stopping -- by button, by closing the window, or by disconnecting -- puts the
gains the run started from back on the device.
"""

from __future__ import annotations

import numpy as np
import pyqtgraph as pg
from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtWidgets import (
    QCheckBox,
    QComboBox,
    QDoubleSpinBox,
    QFormLayout,
    QGroupBox,
    QHBoxLayout,
    QHeaderView,
    QLabel,
    QMainWindow,
    QMessageBox,
    QPushButton,
    QSpinBox,
    QTableWidget,
    QTableWidgetItem,
    QVBoxLayout,
    QWidget,
)

from cost_function import itae, thd
from gwo import GreyWolfOptimizer
from protocol import STREAM_RATE_HZ

COL_ORDER, COL_KP, COL_KI_RE, COL_KI_IM, COL_BEST_RE, COL_BEST_IM = range(6)
HEADERS = ("Harmonic", "Kp (held)", "Ki real (start)", "Ki imag (start)",
           "Ki real (best)", "Ki imag (best)")

# How long to wait for the channel report before calling a read unanswered.
READ_TIMEOUT_MS = 1500

# State machine tick. Fast enough that the phase boundaries are not the
# dominant timing error, cheap enough to leave the GUI responsive.
TICK_MS = 20

# Phases of one evaluation.
IDLE, APPLYING, MEASURING, DONE = range(4)


class TunerWindow(QMainWindow):
    """Grey Wolf search over the per-channel complex integral gains.

    `get_worker_fn` is a callable rather than a stored SerialWorker for the same
    reason as in CoefficientsWindow: the worker is recreated on every reconnect.
    """

    def __init__(self, get_worker_fn, parent=None):
        super().__init__(parent)
        self._get_worker = get_worker_fn

        self.setWindowTitle("Grey Wolf Tuner (HCA integral gains)")
        self.resize(1000, 720)

        # Channel table as reported by the device: order -> (kp, ki)
        self._channels = []          # list of (order, kp_complex, ki_complex)
        self._initial_ki = []        # the Ki the run started from, for restore
        self._pending_read = False

        # Run state
        self._opt: GreyWolfOptimizer | None = None
        self._phase = IDLE
        self._candidate = None
        self._repeat_costs = []      # scores of the candidate being repeated
        self._samples_t = []
        self._samples_e = []
        self._samples_v = []
        self._phase_deadline = 0.0   # monotonic seconds, guards a dead stream
        self._aborted_evals = 0
        self._confirming = False     # re-measuring the winner, see _finish
        self._confirmed_score = None
        self._confirmed_settling = None

        self._build_ui()

        self._read_timer = QTimer(self)
        self._read_timer.setSingleShot(True)
        self._read_timer.timeout.connect(self._on_read_timeout)

        self._tick_timer = QTimer(self)
        self._tick_timer.timeout.connect(self._tick)

    # ------------------------------------------------------------------ UI
    def _build_ui(self):
        central = QWidget()
        root = QVBoxLayout(central)
        self.setCentralWidget(central)

        intro = QLabel(
            "Searches each channel's Ki (real and imaginary) with a Grey Wolf pack, "
            "scoring every candidate on the live system. Kp is held at the values on "
            "the device. The objective picks what is measured and how: THD of the "
            "output voltage from a settled loop, or ITAE of the error across the "
            "transient after an integrator reset.\n"
            "Streaming must be running, and the pack is seeded with the gains the "
            "device is already using, so the first iteration always contains a "
            "known-good candidate."
        )
        intro.setWordWrap(True)
        root.addWidget(intro)

        # --- settings ---
        settings_box = QGroupBox("Search")
        settings_layout = QHBoxLayout(settings_box)

        left_form = QFormLayout()
        self.objective_combo = QComboBox()
        self.objective_combo.addItem("THD of voltage", "thd")
        self.objective_combo.addItem("ITAE of error", "itae")
        self.objective_combo.setToolTip(
            "THD: steady-state distortion of the output voltage, harmonics 2..49\n"
            "over the fundamental. Needs a settled loop, so no integrator reset\n"
            "and a longer settle.\n\n"
            "ITAE: integral of t*|e(t)|, a settling measure. Needs the transient,\n"
            "so the integrators are reset as the candidate is applied and t=0 is\n"
            "that instant."
        )
        self.objective_combo.currentIndexChanged.connect(self._on_objective_changed)
        left_form.addRow("Objective:", self.objective_combo)

        self.wolves_spin = QSpinBox()
        self.wolves_spin.setRange(4, 40)
        self.wolves_spin.setValue(10)
        self.wolves_spin.setToolTip("Pack size. One evaluation per wolf per iteration.")
        left_form.addRow("Wolves:", self.wolves_spin)

        self.iterations_spin = QSpinBox()
        self.iterations_spin.setRange(1, 200)
        self.iterations_spin.setValue(20)
        left_form.addRow("Iterations:", self.iterations_spin)

        self.measure_spin = QDoubleSpinBox()
        self.measure_spin.setDecimals(2)
        self.measure_spin.setRange(0.02, 30.0)
        self.measure_spin.setSingleStep(0.01)
        self.measure_spin.setValue(0.06)
        self.measure_spin.setSuffix(" s")
        self.measure_spin.setToolTip(
            "Measurement window, taken at the END of the settle -- the last of\n"
            "the run, once the loop has converged. 20ms is one fundamental cycle\n"
            "at 50Hz, so the 60ms default scores the last three.\n\n"
            "THD keeps only whole cycles, newest first: 60ms scores the last\n"
            "three, 0.4s the last 20. A few cycles are exact but barely\n"
            "averaged, so their spread across repeats is wide -- raise Repeats,\n"
            "or the window, if the search starts chasing that spread.\n"
            "ITAE integrates it whole, measured from the integrator reset, so it\n"
            "should cover the transient you want settled."
        )
        left_form.addRow("Tm:", self.measure_spin)

        self.delay_spin = QSpinBox()
        self.delay_spin.setRange(0, 60000)
        self.delay_spin.setSingleStep(500)
        self.delay_spin.setValue(15000)
        self.delay_spin.setSuffix(" ms")
        self.delay_spin.setToolTip(
            "Dead time after the gains are sent, before measuring starts.\n\n"
            "For THD this is the settling time, and it dominates the run: the HCA\n"
            "keeps pulling harmonics down for tens of seconds, so a candidate\n"
            "measured early is scored on a transient several points worse than\n"
            "what it actually converges to. Measure at the END of the settle and\n"
            "the score is the steady state you care about.\n"
            "For ITAE it is only the command round trip, so t=0 lands on the\n"
            "reset rather than before it."
        )
        left_form.addRow("Settle:", self.delay_spin)

        self.repeats_spin = QSpinBox()
        self.repeats_spin.setRange(1, 10)
        self.repeats_spin.setValue(3)
        self.repeats_spin.setToolTip(
            "Evaluations per candidate, averaged into one score.\n\n"
            "Measured on this rig, J repeats to about 1.3% sd (3.5-4.4% peak\n"
            "spread) at identical gains -- the same order as the improvement a\n"
            "short run finds. Averaging n repeats divides that by sqrt(n) and\n"
            "multiplies the run time by n. 1 makes the search chase noise."
        )
        left_form.addRow("Repeats:", self.repeats_spin)
        settings_layout.addLayout(left_form)

        right_form = QFormLayout()
        self.ki_re_min = self._bound_spin(-4.0)
        self.ki_re_max = self._bound_spin(4.0)
        self.ki_im_min = self._bound_spin(-4.0)
        self.ki_im_max = self._bound_spin(4.0)
        row_re = QHBoxLayout()
        row_re.addWidget(self.ki_re_min)
        row_re.addWidget(QLabel("to"))
        row_re.addWidget(self.ki_re_max)
        right_form.addRow("Ki real bounds:", row_re)
        row_im = QHBoxLayout()
        row_im.addWidget(self.ki_im_min)
        row_im.addWidget(QLabel("to"))
        row_im.addWidget(self.ki_im_max)
        right_form.addRow("Ki imag bounds:", row_im)

        self.abort_spin = QDoubleSpinBox()
        self.abort_spin.setDecimals(2)
        self.abort_spin.setRange(0.5, 50.0)
        self.abort_spin.setValue(4.0)
        self.abort_spin.setToolTip(
            "Cut an evaluation short and score it worst-possible if |e| exceeds\n"
            "this (per unit). Keeps a divergent candidate on the hardware for\n"
            "milliseconds rather than the whole window."
        )
        right_form.addRow("Abort |e| above:", self.abort_spin)

        self.seed_spin = QSpinBox()
        self.seed_spin.setRange(0, 999999)
        self.seed_spin.setValue(1)
        self.seed_spin.setToolTip("RNG seed, so a run can be repeated exactly.")
        right_form.addRow("Random seed:", self.seed_spin)

        self.reset_check = QCheckBox("Reset integrators at each apply")
        self.reset_check.setChecked(True)
        self.reset_check.setToolTip(
            "ITAE weights error by time from t=0, so t=0 has to be the start of a\n"
            "transient. Unticking this leaves the cost measuring steady state only."
        )
        right_form.addRow(self.reset_check)
        settings_layout.addLayout(right_form)

        settings_layout.addStretch(1)
        root.addWidget(settings_box)

        # --- controls ---
        ctrl = QHBoxLayout()
        self.read_btn = QPushButton("Read Channels")
        self.read_btn.setToolTip("Ask the device which channels it runs, and their gains")
        self.read_btn.clicked.connect(self._read_channels)
        ctrl.addWidget(self.read_btn)

        self.start_btn = QPushButton("Start Tuning")
        self.start_btn.setEnabled(False)
        self.start_btn.clicked.connect(self._start)
        ctrl.addWidget(self.start_btn)

        self.stop_btn = QPushButton("Stop")
        self.stop_btn.setEnabled(False)
        self.stop_btn.setToolTip("Stop the run and put the starting gains back")
        self.stop_btn.clicked.connect(self._stop_clicked)
        ctrl.addWidget(self.stop_btn)

        self.apply_best_btn = QPushButton("Apply Best")
        self.apply_best_btn.setEnabled(False)
        self.apply_best_btn.clicked.connect(self._apply_best)
        ctrl.addWidget(self.apply_best_btn)

        self.restore_btn = QPushButton("Restore Start")
        self.restore_btn.setEnabled(False)
        self.restore_btn.clicked.connect(self._restore_initial)
        ctrl.addWidget(self.restore_btn)

        ctrl.addStretch(1)
        root.addLayout(ctrl)

        # --- channels ---
        self.table = QTableWidget(0, len(HEADERS))
        self.table.setHorizontalHeaderLabels(HEADERS)
        self.table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        self.table.verticalHeader().setVisible(False)
        self.table.setEditTriggers(QTableWidget.NoEditTriggers)
        self.table.setMaximumHeight(260)
        root.addWidget(self.table)

        # --- convergence ---
        self.plot = pg.PlotWidget()
        self.plot.showGrid(x=True, y=True, alpha=0.3)
        self.plot.setLabel("left", "Best J (ITAE)")
        self.plot.setLabel("bottom", "Iteration")
        self.curve = self.plot.plot(pen=pg.mkPen("#00d0ff", width=1.5),
                                    symbol="o", symbolSize=5)
        root.addWidget(self.plot, stretch=1)

        self.status_label = QLabel("Read the channels from the device to begin.")
        self.status_label.setWordWrap(True)
        root.addWidget(self.status_label)

        # The combo starts on THD, so put the protocol controls where that
        # objective needs them rather than leaving the widget defaults.
        self._on_objective_changed()

    def _objective(self):
        return self.objective_combo.currentData()

    def _on_objective_changed(self):
        """Each objective needs its own measurement protocol, so the controls
        that define it follow the selection. Both stay editable afterwards."""
        if self._objective() == "thd":
            # Steady state: settle long, measure at the end, and let the long
            # window do the averaging that repeats would otherwise pay for.
            self.reset_check.setChecked(False)
            self.delay_spin.setValue(15000)
            self.measure_spin.setValue(0.06)
            self.repeats_spin.setValue(1)
            self.plot.setLabel("left", "Best THD")
        else:
            self.reset_check.setChecked(True)
            self.delay_spin.setValue(60)
            self.measure_spin.setValue(0.4)
            self.repeats_spin.setValue(3)
            self.plot.setLabel("left", "Best J (ITAE)")

    @staticmethod
    def _bound_spin(value):
        spin = QDoubleSpinBox()
        spin.setDecimals(4)
        spin.setRange(-100.0, 100.0)
        spin.setSingleStep(0.1)
        spin.setValue(value)
        spin.setFixedWidth(90)
        return spin

    # ------------------------------------------------------------- channels
    def _worker_or_warn(self):
        worker = self._get_worker()
        if worker is None:
            QMessageBox.warning(
                self, "Not connected", "Connect to the device first (main window)."
            )
        return worker

    def _read_channels(self):
        worker = self._worker_or_warn()
        if worker is None:
            return
        self._channels = []
        self._pending_read = True
        worker.request_coefficients()
        self._read_timer.start(READ_TIMEOUT_MS)
        self.status_label.setText("Reading channels from the device...")

    def _on_read_timeout(self):
        self._pending_read = False
        if self._channels:
            self._populate_table()
            self.start_btn.setEnabled(True)
            self.status_label.setText(
                f"{len(self._channels)} channels: {2*len(self._channels)} dimensions "
                f"to search (Ki real and imag per channel)."
            )
        else:
            self.status_label.setText(
                "No reply from the device. Is it connected and the firmware flashed?"
            )

    def on_coeff_frames(self, frames):
        """Channel reports from the device. During a run these are just the
        echoes of what the tuner sent, so they are only collected while a read
        is outstanding."""
        if not self._pending_read:
            return
        for f in frames:
            self._channels.append(
                (f.order, complex(f.kp_real, f.kp_imag), complex(f.ki_real, f.ki_imag))
            )
        self._channels.sort(key=lambda c: c[0])

    def _populate_table(self):
        self.table.setRowCount(len(self._channels))
        for row, (order, kp, ki) in enumerate(self._channels):
            self._set_cell(row, COL_ORDER, str(order))
            self._set_cell(row, COL_KP, f"{kp.real:.4g} {kp.imag:+.4g}j")
            self._set_cell(row, COL_KI_RE, f"{ki.real:.6g}")
            self._set_cell(row, COL_KI_IM, f"{ki.imag:.6g}")
            self._set_cell(row, COL_BEST_RE, "-")
            self._set_cell(row, COL_BEST_IM, "-")

    def _set_cell(self, row, col, text):
        item = QTableWidgetItem(text)
        item.setTextAlignment(Qt.AlignCenter)
        self.table.setItem(row, col, item)

    # ------------------------------------------------------------------ run
    def _start(self):
        worker = self._worker_or_warn()
        if worker is None or not self._channels:
            return

        lower, upper = [], []
        for _ in self._channels:
            lower.extend([self.ki_re_min.value(), self.ki_im_min.value()])
            upper.extend([self.ki_re_max.value(), self.ki_im_max.value()])
        if any(u < l for l, u in zip(lower, upper)):
            QMessageBox.warning(self, "Bounds", "A lower bound is above its upper bound.")
            return

        seed = []
        for _, _, ki in self._channels:
            seed.extend([ki.real, ki.imag])

        n_wolves = self.wolves_spin.value()
        n_iter = self.iterations_spin.value()
        seconds = (n_wolves * n_iter * self.repeats_spin.value()
                   * (self.measure_spin.value() + self.delay_spin.value() / 1000.0))
        if QMessageBox.question(
            self,
            "Start tuning?",
            f"{n_wolves} wolves x {n_iter} iterations = {n_wolves*n_iter} evaluations "
            f"on the live system, roughly {seconds/60.0:.1f} minutes.\n\n"
            f"Each one applies untested gains for about "
            f"{self.measure_spin.value():.2f}s. Stop puts the current gains back.\n\n"
            f"Continue?",
        ) != QMessageBox.Yes:
            return

        self._initial_ki = [ki for _, _, ki in self._channels]
        self._opt = GreyWolfOptimizer(
            lower, upper,
            n_wolves=n_wolves,
            n_iterations=n_iter,
            seed_position=seed,
            rng_seed=self.seed_spin.value(),
        )
        self._aborted_evals = 0
        self._confirming = False
        self._confirmed_score = None
        self._confirmed_settling = None
        self.curve.setData([], [])

        self._set_running(True)
        self._next_candidate()
        self._tick_timer.start(TICK_MS)

    def _set_running(self, running):
        self.start_btn.setEnabled(not running)
        self.read_btn.setEnabled(not running)
        self.stop_btn.setEnabled(running)
        for w in (self.wolves_spin, self.iterations_spin, self.measure_spin,
                  self.delay_spin, self.seed_spin, self.reset_check,
                  self.repeats_spin, self.objective_combo,
                  self.ki_re_min, self.ki_re_max, self.ki_im_min, self.ki_im_max):
            w.setEnabled(not running)

    def _next_candidate(self):
        """Hand the next position to the hardware, or wrap the run up."""
        position = self._opt.ask() if self._opt is not None else None
        if position is None:
            self._finish()
            return
        self._candidate = position
        self._repeat_costs = []
        self._apply_candidate(position)

    def _apply_candidate(self, position):
        """Push one candidate and start its clock.

        The integrator reset goes out last, so t=0 is as close as the link
        allows to the moment the new gains start acting on a cleared
        controller -- which is the transient ITAE is integrating.
        """
        worker = self._get_worker()
        if worker is None:
            self._abort_run("Disconnected mid-run. Nothing was restored.")
            return

        for i, (order, kp, _) in enumerate(self._channels):
            worker.set_coefficient(order, kp, complex(position[2*i], position[2*i + 1]))
        if self.reset_check.isChecked():
            worker.reset_integrators()

        self._samples_t = []   # seconds, from the sample index (see on_frames)
        self._samples_e = []
        self._samples_v = []
        self._phase = APPLYING
        self._phase_deadline = self._now() + self.delay_spin.value() / 1000.0

    @staticmethod
    def _now():
        import time
        return time.monotonic()

    def on_frames(self, frames):
        """Stream samples forwarded by the main window.

        Samples are only kept during the measurement phase, but the divergence
        check runs through the settle as well -- with a 15s settle that is most
        of the evaluation, and a candidate that blows up must not be left on the
        hardware for it."""
        if self._phase not in (APPLYING, MEASURING):
            return

        abort_level = self.abort_spin.value()
        if self._phase == APPLYING:
            for f in frames:
                if abs(f.error) > abort_level:
                    self._aborted_evals += 1
                    self._score(float("inf"), aborted=True)
                    return
            return
        for f in frames:
            # Time comes from the sample index, not the device timestamp: seq
            # counts samples at exactly STREAM_RATE_HZ and survives a dropped
            # frame as a gap, whereas timestamp_ms can jump (a frame left in
            # the device FIFO from an earlier streaming session carries an old
            # tick). ITAE weights by t, so a jump would wreck the score.
            self._samples_t.append(f.seq / STREAM_RATE_HZ)
            self._samples_e.append(f.error)
            self._samples_v.append(f.voltage)
            if abs(f.error) > abort_level:
                # Diverging. Score it worst-possible and move on rather than
                # holding the candidate on the hardware for the full window.
                self._aborted_evals += 1
                self._score(float("inf"), aborted=True)
                return

    def _tick(self):
        """Phase clock. Sample counting drives the end of the measurement
        window; this only handles the timed phases and the stalled-stream
        escape hatch."""
        if self._phase == APPLYING:
            if self._now() >= self._phase_deadline:
                self._phase = MEASURING
                # Generous: the window plus the time it takes to notice a dead
                # stream. Hitting it means frames stopped arriving.
                self._phase_deadline = self._now() + self.measure_spin.value() + 1.5
            return

        if self._phase == MEASURING:
            span = 0.0
            if len(self._samples_t) >= 2:
                span = self._samples_t[-1] - self._samples_t[0]
            if span >= self.measure_spin.value():
                self._score(self._cost())
                return
            if self._now() >= self._phase_deadline:
                self._abort_run(
                    "No samples arriving. Start streaming in the main window, "
                    "then run again. The starting gains have been put back."
                )
            return

    def _cost(self):
        """Score the window just collected with the selected objective."""
        if self._objective() == "thd":
            # Samples arrive in batches, so the collected window overshoots Tm
            # by up to one drain. Cut it back to the requested tail first --
            # "the last 30ms of the run" has to mean the last 30ms.
            wanted = int(self.measure_spin.value() * STREAM_RATE_HZ)
            return thd(np.asarray(self._samples_v[-wanted:]), STREAM_RATE_HZ)
        return itae(np.asarray(self._samples_t), np.asarray(self._samples_e))

    def _settling_check(self):
        """THD over the first and last third of the window just measured.

        If the second number is clearly below the first, the loop was still
        converging when the window opened and the settle is too short -- the
        score is then a transient, not the steady state being searched for.
        """
        v = np.asarray(self._samples_v)
        third = v.size // 3
        if self._objective() != "thd" or third < int(STREAM_RATE_HZ // 25):
            return None
        return thd(v[:third], STREAM_RATE_HZ), thd(v[-third:], STREAM_RATE_HZ)

    def _score(self, cost, aborted=False):
        """One measurement is in. Repeat the candidate if more are asked for,
        otherwise hand the mean to the optimizer and move on."""
        self._repeat_costs.append(cost)
        repeats = self.repeats_spin.value()

        if self._confirming:
            # Not a search evaluation: the winner is being re-measured, and
            # nothing here goes back into the optimizer.
            if not aborted and len(self._repeat_costs) < repeats:
                self._apply_candidate(self._candidate)
                return
            self._confirmed_score = (float("inf") if aborted
                                     else float(np.mean(self._repeat_costs)))
            self._confirmed_settling = self._settling_check()
            self._confirming = False
            self._finish()
            return

        # An abort ends the repeats early: a candidate that diverged once is
        # already disqualified, and re-applying it only puts it back on the
        # hardware.
        if not aborted and len(self._repeat_costs) < repeats:
            self._update_progress(cost, aborted, repeating=True)
            self._apply_candidate(self._candidate)
            return

        mean_cost = float(np.mean(self._repeat_costs)) if not aborted else float("inf")
        self._opt.tell(mean_cost)
        self._update_progress(mean_cost, aborted)

        if self._opt.finished:
            self._finish()
        else:
            self._next_candidate()

    def _update_progress(self, cost, aborted, repeating=False):
        opt = self._opt
        if self._confirming:
            label, fmt = self._score_format()
            self.status_label.setText(
                f"Confirming the best candidate: measurement "
                f"{len(self._repeat_costs)}/{self.repeats_spin.value()}, "
                f"{label} {fmt(cost)}"
            )
            return
        if opt.history:
            xs = list(range(1, len(opt.history) + 1))
            self.curve.setData(xs, opt.history)
            self._show_best(opt.alpha_position)

        label, fmt = self._score_format()
        note = "  (aborted: |e| over the limit)" if aborted else ""
        if repeating:
            note += (f"  (repeat {len(self._repeat_costs)}"
                     f"/{self.repeats_spin.value()})")
        best = "-" if not np.isfinite(opt.alpha_score) else fmt(opt.alpha_score)
        self.status_label.setText(
            f"Candidate {opt.evaluations_done + 1}/{opt.total_evaluations}   "
            f"iteration {opt.iteration + 1}/{opt.n_iterations}   "
            f"last {label} {fmt(cost)}{note}   best {label} {best}"
        )

    def _score_format(self):
        """How to name and print a score. THD reads as a percentage, ITAE as
        the bare integral."""
        if self._objective() == "thd":
            return "THD", lambda v: ("-" if not np.isfinite(v) else f"{100.0*v:.3f}%")
        return "J", lambda v: ("-" if not np.isfinite(v) else f"{v:.6g}")

    def _show_best(self, position):
        for row in range(len(self._channels)):
            self._set_cell(row, COL_BEST_RE, f"{position[2*row]:.6g}")
            self._set_cell(row, COL_BEST_IM, f"{position[2*row + 1]:.6g}")

    def _finish(self):
        """Run complete: re-measure the winner, then leave it on the device.

        The best score seen during a search is the minimum of many noisy
        measurements, so it is biased low -- on this rig THD repeats to a few
        tenths of a point, which is the same size as the improvement a search
        finds. Re-measuring the winner separately is what turns that into a
        number worth quoting.
        """
        opt = self._opt
        if opt is None or not np.isfinite(opt.alpha_score):
            self._tick_timer.stop()
            self._phase = DONE
            self._set_running(False)
            self.status_label.setText(
                "Every candidate diverged or went unmeasured. Starting gains put back."
            )
            self._restore_initial()
            return

        if self._confirmed_score is None:
            self._confirming = True
            self._candidate = opt.alpha_position
            self._repeat_costs = []
            label, _ = self._score_format()
            self.status_label.setText(
                f"Search done. Re-measuring the best candidate "
                f"({self.repeats_spin.value()}x) to confirm its {label}..."
            )
            self._apply_candidate(self._candidate)
            return  # the tick timer keeps running through the confirmation

        self._tick_timer.stop()
        self._phase = DONE
        self._set_running(False)

        self._push_ki(opt.alpha_position)
        self._show_best(opt.alpha_position)
        self.apply_best_btn.setEnabled(True)
        self.restore_btn.setEnabled(True)
        label, fmt = self._score_format()
        aborted = f", {self._aborted_evals} aborted" if self._aborted_evals else ""
        settling = ""
        if self._confirmed_settling is not None:
            first, last = self._confirmed_settling
            settling = (f"  Window start {100.0*first:.2f}% -> end "
                        f"{100.0*last:.2f}%"
                        + ("; still falling, increase the settle."
                           if last < first * 0.9 else "; settled."))
        self.status_label.setText(
            f"Done: {label} {fmt(self._confirmed_score)} re-measured "
            f"({fmt(opt.alpha_score)} during the search) after "
            f"{opt.total_evaluations} candidates{aborted}.{settling} The best "
            f"gains are now on the device -- 'Restore Start' puts the originals back."
        )

    def _abort_run(self, message):
        self._tick_timer.stop()
        self._phase = IDLE
        self._set_running(False)
        self._restore_initial()
        self.status_label.setText(message)

    def _stop_clicked(self):
        self._tick_timer.stop()
        self._phase = IDLE
        self._set_running(False)
        self._restore_initial()
        opt = self._opt
        if opt is not None and np.isfinite(opt.alpha_score):
            self.apply_best_btn.setEnabled(True)
            self.restore_btn.setEnabled(True)
            label, fmt = self._score_format()
            self.status_label.setText(
                f"Stopped after {opt.evaluations_done} candidates. Starting gains "
                f"put back; best found was {label} {fmt(opt.alpha_score)} "
                f"('Apply Best')."
            )
        else:
            self.status_label.setText("Stopped. Starting gains put back.")

    # -------------------------------------------------------------- gains
    def _push_ki(self, position):
        worker = self._get_worker()
        if worker is None:
            return False
        for i, (order, kp, _) in enumerate(self._channels):
            worker.set_coefficient(order, kp, complex(position[2*i], position[2*i + 1]))
        return True

    def _apply_best(self):
        if self._opt is None or not np.isfinite(self._opt.alpha_score):
            return
        label, fmt = self._score_format()
        if self._push_ki(self._opt.alpha_position):
            self.status_label.setText(
                f"Best gains ({label} {fmt(self._opt.alpha_score)}) sent to the device."
            )

    def _restore_initial(self):
        """Put back the Ki the run started from. Called by Stop, by any abort,
        and on close -- a half-finished search must not leave an untested
        candidate running the hardware."""
        if not self._initial_ki:
            return
        worker = self._get_worker()
        if worker is None:
            return
        for (order, kp, _), ki in zip(self._channels, self._initial_ki):
            worker.set_coefficient(order, kp, ki)
        self.restore_btn.setEnabled(True)

    def closeEvent(self, event):
        if self._tick_timer.isActive():
            self._tick_timer.stop()
            self._set_running(False)
            self._restore_initial()
        super().closeEvent(event)
