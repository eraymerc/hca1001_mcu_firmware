package main

import (
	"time"

	"fyne.io/fyne/v2"
	"fyne.io/fyne/v2/container"
	"fyne.io/fyne/v2/widget"
)

// showFFTWindow opens a standalone live spectrum window for one signal,
// recomputed from the shared ring buffer owned by the Monitor. Closing it just
// stops its own refresh ticker; it does not affect data acquisition.
func showFFTWindow(app fyne.App, name, label string, get func(string, int) []float64, onClosed func()) {
	win := app.NewWindow("FFT - " + label)
	win.Resize(fyne.NewSize(720, 460))

	plot := NewPlotWidget("Amplitude (peak)", "Frequency (Hz)")

	dbCheck := widget.NewCheck("dB", nil)
	rmsCheck := widget.NewCheck("RMS", nil)

	refresh := func() {
		data := get(name, FFTWindowSamples)
		res, ok := computeFFT(data, rmsCheck.Checked, dbCheck.Checked)
		if !ok {
			return
		}
		plot.SetYLabel(res.YLabel)
		plot.SetData(res.Freqs, res.Values, false, 0)
	}

	dbCheck.OnChanged = func(bool) { refresh() }
	rmsCheck.OnChanged = func(bool) { refresh() }

	hint := widget.NewLabel("Hann window, " + itoa(FFTWindowSamples) + " samples, Nyquist " +
		formatTick(StreamRateHz/2, 1) + " Hz")
	hint.Importance = widget.LowImportance

	controls := container.NewHBox(dbCheck, rmsCheck, widget.NewSeparator(), hint)
	win.SetContent(container.NewBorder(controls, nil, nil, nil, plot))

	stop := make(chan struct{})
	go func() {
		ticker := time.NewTicker(FFTUpdateInterval * time.Millisecond)
		defer ticker.Stop()
		for {
			select {
			case <-stop:
				return
			case <-ticker.C:
				fyne.Do(refresh)
			}
		}
	}()

	win.SetOnClosed(func() {
		close(stop)
		if onClosed != nil {
			onClosed()
		}
	})

	refresh()
	win.Show()
}
