package main

import (
	"math"
	"math/rand"
	"os"
	"testing"

	"fyne.io/fyne/v2"
	"fyne.io/fyne/v2/test"
)

// TestWindowScreenshot renders the whole window with Fyne's software driver so
// the layout can be eyeballed without a display.
func TestWindowScreenshot(t *testing.T) {
	out := os.Getenv("PLOT_OUT")
	if out == "" {
		t.Skip("set PLOT_OUT to write a window screenshot")
	}

	a := test.NewApp()
	defer test.NewApp() // reset global app for other tests
	a.Settings().SetTheme(darkTheme{})

	win := test.NewWindow(nil)
	win.Resize(fyne.NewSize(1180, 760))

	u := NewUI(a, win)

	// Feed synthetic telemetry: 50Hz fundamental on voltage, decaying error.
	rng := rand.New(rand.NewSource(7))
	for i := 0; i < 8000; i++ {
		tt := float64(i) / StreamRateHz
		u.monitor.Add(AdcFrame{
			Seq:         uint32(i),
			TimestampMs: uint32(i),
			Voltage:     12.0*math.Sin(2*math.Pi*50*tt) + 0.15*rng.NormFloat64(),
			Error:       0.4 * math.Exp(-tt/2.0) * math.Cos(2*math.Pi*7*tt),
		})
	}

	// Drive the real state transitions rather than poking widgets directly, so
	// the screenshot reflects what onConnected/onStreamClicked actually render.
	u.onConnected("/dev/ttyACM0")
	u.setStatus("Device identified: HCA1001_ADC_STREAM_V1")
	u.rowByKey["error"].window.set(1.0, true)

	u.triggerEnable.SetChecked(true)
	u.triggerLevel.set(0, true)

	u.update()
	win.Resize(fyne.NewSize(1180, 760))

	writePNG(t, out+"/window.png", win.Canvas().Capture())
}
