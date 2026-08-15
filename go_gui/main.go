// HCA1001 ADC Monitor -- dark-mode Fyne GUI.
//
// Connects to the STM32G474RE over the ST-LINK's virtual COM port (LPUART1,
// 2,097,000 baud, see Core/Src/main.c) -- a single USB cable, no external UART
// adapter needed. Streams the single ADC1 voltage reading plus the HCA error
// signal, plots them live, and can save the captured data as CSV and the plots
// as PNG. Each signal has its own button to open a live FFT window.
//
// Run:
//
//	go run ./...
package main

import (
	"fyne.io/fyne/v2"
	"fyne.io/fyne/v2/app"
)

func main() {
	// Declare that this app follows the fyne.Do threading model. Fyne prints a
	// "not been migrated" warning at startup unless an app opts in -- it is a
	// declaration, not a detected fault. Every background goroutine here hands
	// UI work to the main loop via fyne.Do (serial callbacks, the redraw
	// ticker, the ping timeout, the CSV writer and the FFT refresh), so the
	// claim holds. Must be set before Run reads it.
	app.SetMetadata(fyne.AppMetadata{
		ID:      "com.hca1001.adcmonitor",
		Name:    "HCA1001 ADC Monitor",
		Version: "1.0.0",
		Build:   1,
		Migrations: map[string]bool{
			"fyneDo": true,
		},
	})

	a := app.NewWithID("com.hca1001.adcmonitor")
	a.Settings().SetTheme(darkTheme{})

	win := a.NewWindow("HCA1001 ADC Monitor (STM32G474RE, ST-LINK VCP)")
	win.Resize(fyne.NewSize(1180, 720))

	u := NewUI(a, win)
	a.Lifecycle().SetOnStarted(u.StartUpdateLoop)

	win.ShowAndRun()
}
