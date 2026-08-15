"""Standalone FFT viewer window for a single monitored signal."""

import numpy as np
import pyqtgraph as pg
from PyQt5.QtCore import QTimer
from PyQt5.QtWidgets import (
    QAbstractItemView,
    QCheckBox,
    QDoubleSpinBox,
    QHBoxLayout,
    QHeaderView,
    QLabel,
    QMainWindow,
    QTableWidget,
    QTableWidgetItem,
    QVBoxLayout,
    QWidget,
)

from protocol import STREAM_RATE_HZ

FFT_WINDOW_SAMPLES = 2048     # ~2s of history at 1kHz
UPDATE_INTERVAL_MS = 250
DB_FLOOR = 1e-12              # clamp before log10 so a true-zero bin doesn't give -inf

HARMONIC_ORDERS = (1, 3, 5, 7, 9)
F0_SEARCH_LO_HZ = 30.0
F0_SEARCH_HI_HZ = 90.0
DEFAULT_F0_HZ = 50.0

# Phases of every signal are measured against this signal's fundamental, so a
# harmonic angle reads as "relative to the line voltage" no matter which signal
# is being viewed.
PHASE_REF_SIGNAL = "voltage"

MARKER_COLOR = "#ffd400"


class FFTWindow(QMainWindow):
    """Live amplitude spectrum of `signal_name`, recomputed from the shared
    ring buffer owned by the main window. Closing this window just stops its
    own refresh timer; it does not affect data acquisition.

    Amplitude is calibrated peak amplitude by default: the Hann window's
    coherent gain (mean 0.5) and the one-sided-spectrum factor of 2 (every
    AC bin's energy is split between the discarded negative-frequency bin
    and this one) are both corrected for, unlike a naive |FFT|/N. The RMS
    checkbox divides by sqrt(2) (skipping the DC bin, which has no such
    factor); the dB checkbox applies 20*log10 on top of whichever of those
    is selected.

    Below the plot, the odd harmonics H1/H3/H5/H7/H9 of the detected
    fundamental are tabulated with amplitude, share of the fundamental and
    phase. Phase is reported as phi_h - h*phi_1 against the fundamental of
    PHASE_REF_SIGNAL, which makes it independent of where the analysis window
    happens to start -- a raw np.angle() would jitter randomly every refresh.
    Angles follow the usual DFT cosine convention, so a signal thought of as
    a sum of sines reads with a fixed (h-1)*90 deg offset per order.
    """

    def __init__(self, signal_name: str, label: str, get_buffer_fn, parent=None):
        super().__init__(parent)
        self._signal_name = signal_name
        self._get_buffer_fn = get_buffer_fn  # callable -> numpy array of recent samples

        self.setWindowTitle(f"FFT - {label}")
        self.resize(760, 620)

        central = QWidget()
        layout = QVBoxLayout(central)
        self.setCentralWidget(central)

        controls = QHBoxLayout()
        self.db_checkbox = QCheckBox("dB")
        self.db_checkbox.toggled.connect(self._refresh)
        controls.addWidget(self.db_checkbox)

        self.rms_checkbox = QCheckBox("RMS")
        self.rms_checkbox.toggled.connect(self._refresh)
        controls.addWidget(self.rms_checkbox)

        controls.addSpacing(16)
        self.auto_f0_checkbox = QCheckBox("Auto f0")
        self.auto_f0_checkbox.setChecked(True)
        self.auto_f0_checkbox.toggled.connect(self._on_auto_f0_toggled)
        controls.addWidget(self.auto_f0_checkbox)

        controls.addWidget(QLabel("f0:"))
        self.f0_spin = QDoubleSpinBox()
        self.f0_spin.setDecimals(2)
        self.f0_spin.setSingleStep(0.5)
        self.f0_spin.setRange(10.0, 200.0)
        self.f0_spin.setValue(DEFAULT_F0_HZ)
        self.f0_spin.setSuffix(" Hz")
        self.f0_spin.setFixedWidth(100)
        self.f0_spin.setEnabled(False)  # driven by auto-detect until unchecked
        self.f0_spin.setToolTip("Fundamental frequency the harmonics are derived from")
        self.f0_spin.valueChanged.connect(self._refresh)
        controls.addWidget(self.f0_spin)

        controls.addStretch(1)
        layout.addLayout(controls)

        self.plot = pg.PlotWidget(background="#1e1e1e")
        self.plot.showGrid(x=True, y=True, alpha=0.3)
        self.plot.setLabel("bottom", "Frequency", units="Hz")
        self.plot.setTitle(f"{label} - Spectrum")
        self.curve = self.plot.plot(pen=pg.mkPen("#00d0ff", width=1.5))

        self.harmonic_markers = pg.ScatterPlotItem(
            symbol="o", size=8, pen=pg.mkPen(MARKER_COLOR), brush=pg.mkBrush(MARKER_COLOR)
        )
        self.plot.addItem(self.harmonic_markers)

        self.harmonic_texts = []
        for h in HARMONIC_ORDERS:
            text = pg.TextItem(f"H{h}", color=MARKER_COLOR, anchor=(0.5, 1.2))
            text.setVisible(False)
            self.plot.addItem(text)
            self.harmonic_texts.append(text)

        layout.addWidget(self.plot, stretch=1)

        self.table = QTableWidget(len(HARMONIC_ORDERS), 6)
        self.table.setHorizontalHeaderLabels(
            ["Harmonic", "Freq (Hz)", "Amplitude", "% of H1", "Phase (deg)", "Phasor (a+bj)"]
        )
        self.table.setEditTriggers(QAbstractItemView.NoEditTriggers)
        self.table.setSelectionMode(QAbstractItemView.NoSelection)
        self.table.verticalHeader().setVisible(False)
        self.table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        row_h = 24
        self.table.verticalHeader().setDefaultSectionSize(row_h)
        self.table.setFixedHeight(
            self.table.horizontalHeader().height() + row_h * len(HARMONIC_ORDERS) + 4
        )
        for row in range(len(HARMONIC_ORDERS)):
            for col in range(6):
                self.table.setItem(row, col, QTableWidgetItem("-"))
        layout.addWidget(self.table)

        self._timer = QTimer(self)
        self._timer.timeout.connect(self._refresh)
        self._timer.start(UPDATE_INTERVAL_MS)
        self._refresh()

    # ------------------------------------------------------------- helpers
    def _on_auto_f0_toggled(self, checked: bool):
        self.f0_spin.setEnabled(not checked)
        self._refresh()

    def _apply_scaling(self, amplitude, is_dc=False):
        """Peak amplitude -> the units currently on the y-axis. Accepts a
        scalar or an array so the curve and the table can never drift apart."""
        values = np.asarray(amplitude, dtype=np.float64)
        if self.rms_checkbox.isChecked() and not is_dc:
            values = values / np.sqrt(2.0)
        if self.db_checkbox.isChecked():
            values = 20.0 * np.log10(np.maximum(values, DB_FLOOR))
        return values

    def _y_axis_label(self):
        use_rms = self.rms_checkbox.isChecked()
        if self.db_checkbox.isChecked():
            return "Magnitude (dB" + (", RMS ref" if use_rms else ", peak ref") + ")"
        return "Amplitude (RMS)" if use_rms else "Amplitude (peak)"

    @staticmethod
    def _harmonic_component(data, window, f_hz):
        """Complex amplitude at an arbitrary frequency, using the same
        peak calibration as the plotted curve (coherent gain of the window,
        plus the one-sided factor of 2). A single-frequency DFT rather than a
        bin lookup: at ~0.49 Hz bin spacing a harmonic almost never lands on a
        bin centre, and the resulting leakage would wreck the phase."""
        n = len(data)
        t = np.arange(n) / STREAM_RATE_HZ
        acc = np.sum(data * window * np.exp(-2j * np.pi * f_hz * t))
        return acc * 2.0 / np.sum(window)

    @staticmethod
    def _detect_f0(freqs, amplitude):
        """Largest peak inside the fundamental search band, refined by
        parabolic interpolation over its two neighbours. None if the band is
        empty or the peak sits on its edge (no neighbours to interpolate)."""
        idx = np.flatnonzero((freqs >= F0_SEARCH_LO_HZ) & (freqs <= F0_SEARCH_HI_HZ))
        if idx.size == 0:
            return None
        k = int(idx[np.argmax(amplitude[idx])])
        if k <= 0 or k >= len(amplitude) - 1:
            return None
        y0, y1, y2 = amplitude[k - 1], amplitude[k], amplitude[k + 1]
        denom = y0 - 2.0 * y1 + y2
        delta = 0.0 if denom == 0 else 0.5 * (y0 - y2) / denom
        if not np.isfinite(delta) or abs(delta) > 1.0:
            delta = 0.0
        bin_width = freqs[1] - freqs[0]
        return float(freqs[k] + delta * bin_width)

    @staticmethod
    def _wrap180(degrees):
        return (degrees + 180.0) % 360.0 - 180.0

    def _set_cell(self, row, col, text):
        self.table.item(row, col).setText(text)

    # ------------------------------------------------------------- refresh
    def _refresh(self):
        data = self._get_buffer_fn(self._signal_name, FFT_WINDOW_SAMPLES)
        if data is None or len(data) < 16:
            return

        n = len(data)
        window = np.hanning(n)
        spectrum = np.fft.rfft(data * window)
        freqs = np.fft.rfftfreq(n, d=1.0 / STREAM_RATE_HZ)

        # Calibrated peak amplitude: undo the window's coherent gain (its
        # mean, not n) and restore the factor of 2 that a one-sided
        # spectrum drops for every bin except DC (and Nyquist, for even n).
        scale = np.full(spectrum.shape, 2.0)
        scale[0] = 1.0
        if n % 2 == 0:
            scale[-1] = 1.0
        amplitude = np.abs(spectrum) * scale / np.sum(window)

        values = self._apply_scaling(amplitude)
        values[0] = self._apply_scaling(amplitude[0], is_dc=True)

        self.plot.setLabel("left", self._y_axis_label())
        self.curve.setData(freqs, values)

        self._refresh_harmonics(data, window, freqs, amplitude)

    def _refresh_harmonics(self, data, window, freqs, amplitude):
        # The phase reference is another signal's fundamental (unless this
        # window already plots it). Both live buffers are appended once per
        # frame, so their trailing samples share a time origin -- trim from
        # the end to keep that true while the buffers are still filling.
        ref_data, ref_window = data, window
        ref_is_self = self._signal_name == PHASE_REF_SIGNAL
        if not ref_is_self:
            ref = self._get_buffer_fn(PHASE_REF_SIGNAL, FFT_WINDOW_SAMPLES)
            if ref is None or len(ref) < 16:
                ref_is_self = True
            else:
                m = min(len(data), len(ref))
                data = data[-m:]
                ref_data = ref[-m:]
                ref_window = window = np.hanning(m)
                ref_spectrum = np.fft.rfft(ref_data * ref_window)
                ref_freqs = np.fft.rfftfreq(m, d=1.0 / STREAM_RATE_HZ)
                ref_scale = np.full(ref_spectrum.shape, 2.0)
                ref_scale[0] = 1.0
                if m % 2 == 0:
                    ref_scale[-1] = 1.0
                ref_amplitude = np.abs(ref_spectrum) * ref_scale / np.sum(ref_window)

        if ref_is_self:
            ref_freqs, ref_amplitude = freqs, amplitude

        # f0 comes from the reference signal: the error signal may carry very
        # little energy at the line frequency, but the voltage always does.
        if self.auto_f0_checkbox.isChecked():
            detected = self._detect_f0(ref_freqs, ref_amplitude)
            if detected is not None:
                self.f0_spin.blockSignals(True)
                self.f0_spin.setValue(detected)
                self.f0_spin.blockSignals(False)
        f0 = self.f0_spin.value()

        phase_header = "Phase (deg, self)" if ref_is_self else "Phase (deg, re V H1)"
        self.table.horizontalHeaderItem(4).setText(phase_header)

        nyquist = STREAM_RATE_HZ / 2.0
        ref_phase = np.angle(self._harmonic_component(ref_data, ref_window, f0))

        marker_points = []
        fundamental_amp = None
        for row, h in enumerate(HARMONIC_ORDERS):
            f_h = h * f0
            self._set_cell(row, 0, f"H{h}")
            text = self.harmonic_texts[row]

            if f_h > nyquist:
                self._set_cell(row, 1, f"{f_h:.2f}")
                for col in (2, 3, 4, 5):
                    self._set_cell(row, col, "-")
                text.setVisible(False)
                continue

            comp = self._harmonic_component(data, window, f_h)
            amp = float(np.abs(comp))
            if h == 1:
                fundamental_amp = amp

            plotted = float(self._apply_scaling(amp))
            self._set_cell(row, 1, f"{f_h:.2f}")
            self._set_cell(row, 2, f"{plotted:.4g}")

            if fundamental_amp is None or fundamental_amp <= DB_FLOOR:
                self._set_cell(row, 3, "-")
            else:
                self._set_cell(row, 3, f"{100.0 * amp / fundamental_amp:.2f}")

            rel_phase = np.angle(comp) - h * ref_phase
            self._set_cell(row, 4, f"{self._wrap180(np.degrees(rel_phase)):+.1f}")

            # Same phasor in rectangular form. Its magnitude is the linear
            # amplitude in the current peak/RMS units -- never dB, which is
            # logarithmic and has no meaningful real/imaginary parts.
            linear = amp / np.sqrt(2.0) if self.rms_checkbox.isChecked() else amp
            phasor = linear * np.exp(1j * rel_phase)
            self._set_cell(row, 5, f"{phasor.real:.4g}{phasor.imag:+.4g}j")

            marker_points.append({"pos": (f_h, plotted)})
            text.setPos(f_h, plotted)
            text.setVisible(True)

        self.harmonic_markers.setData(marker_points)

    def closeEvent(self, event):
        self._timer.stop()
        super().closeEvent(event)
