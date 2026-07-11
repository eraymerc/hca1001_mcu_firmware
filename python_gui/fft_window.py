"""Standalone FFT viewer window for a single monitored signal."""

import numpy as np
import pyqtgraph as pg
from PyQt5.QtCore import QTimer
from PyQt5.QtWidgets import QMainWindow, QVBoxLayout, QWidget

from protocol import STREAM_RATE_HZ

FFT_WINDOW_SAMPLES = 2048     # ~2s of history at 1kHz
UPDATE_INTERVAL_MS = 250


class FFTWindow(QMainWindow):
    """Live magnitude spectrum of `signal_name`, recomputed from the shared
    ring buffer owned by the main window. Closing this window just stops its
    own refresh timer; it does not affect data acquisition.
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

        self.plot = pg.PlotWidget(background="#1e1e1e")
        self.plot.showGrid(x=True, y=True, alpha=0.3)
        self.plot.setLabel("bottom", "Frequency", units="Hz")
        self.plot.setLabel("left", "Magnitude")
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
        windowed = data * np.hanning(n)
        spectrum = np.fft.rfft(windowed)
        freqs = np.fft.rfftfreq(n, d=1.0 / STREAM_RATE_HZ)
        magnitude = np.abs(spectrum) / n

        self.curve.setData(freqs, magnitude)

    def closeEvent(self, event):
        self._timer.stop()
        super().closeEvent(event)
