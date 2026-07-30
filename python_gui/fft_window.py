"""Standalone FFT viewer window for a single monitored signal."""

import numpy as np
import pyqtgraph as pg
from PyQt5.QtCore import QTimer
from PyQt5.QtWidgets import QCheckBox, QHBoxLayout, QMainWindow, QVBoxLayout, QWidget

from protocol import STREAM_RATE_HZ

FFT_WINDOW_SAMPLES = 2048     # ~2s of history at 1kHz
UPDATE_INTERVAL_MS = 250
DB_FLOOR = 1e-12              # clamp before log10 so a true-zero bin doesn't give -inf


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
    """

    def __init__(self, signal_name: str, label: str, get_buffer_fn, parent=None):
        super().__init__(parent)
        self._signal_name = signal_name
        self._get_buffer_fn = get_buffer_fn  # callable -> numpy array of recent samples

        self.setWindowTitle(f"FFT - {label}")
        self.resize(700, 450)

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

        controls.addStretch(1)
        layout.addLayout(controls)

        self.plot = pg.PlotWidget(background="#1e1e1e")
        self.plot.showGrid(x=True, y=True, alpha=0.3)
        self.plot.setLabel("bottom", "Frequency", units="Hz")
        self.plot.setTitle(f"{label} - Spectrum")
        self.curve = self.plot.plot(pen=pg.mkPen("#00d0ff", width=1.5))
        layout.addWidget(self.plot)

        self._timer = QTimer(self)
        self._timer.timeout.connect(self._refresh)
        self._timer.start(UPDATE_INTERVAL_MS)
        self._refresh()

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

        use_rms = self.rms_checkbox.isChecked()
        use_db = self.db_checkbox.isChecked()

        if use_rms:
            amplitude = amplitude.copy()
            amplitude[1:] /= np.sqrt(2.0)  # DC has no sqrt(2) factor -- it doesn't oscillate

        if use_db:
            values = 20.0 * np.log10(np.maximum(amplitude, DB_FLOOR))
            label = "Magnitude (dB" + (", RMS ref" if use_rms else ", peak ref") + ")"
        else:
            values = amplitude
            label = "Amplitude (RMS)" if use_rms else "Amplitude (peak)"

        self.plot.setLabel("left", label)
        self.curve.setData(freqs, values)

    def closeEvent(self, event):
        self._timer.stop()
        super().closeEvent(event)
