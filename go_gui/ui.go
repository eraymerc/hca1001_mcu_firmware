package main

import (
	"fmt"
	"image/color"
	"math"
	"strconv"
	"strings"
	"time"

	"fyne.io/fyne/v2"
	"fyne.io/fyne/v2/canvas"
	"fyne.io/fyne/v2/container"
	"fyne.io/fyne/v2/dialog"
	"fyne.io/fyne/v2/layout"
	"fyne.io/fyne/v2/widget"
)

// GUI redraw cadence. Deliberately decoupled from the 1kHz data rate: the
// serial goroutine buffers parsed frames into the Monitor, and this ticker
// pulls a snapshot and redraws on its own schedule. If a redraw ever takes
// longer than this interval, the next pull just picks up more samples - there
// is no per-sample backlog that can pile up.
const guiUpdateInterval = 50 * time.Millisecond

const pingTimeout = time.Second

// signalRow is one monitored signal: export checkbox, FFT button, window
// length, live stats and the plot itself.
type signalRow struct {
	name  string
	label string

	check  *widget.Check
	window *numField
	stats  *canvas.Text
	plot   *PlotWidget
	obj    fyne.CanvasObject
}

func (r *signalRow) windowSamples() int {
	n := int(math.Round(r.window.Value() * StreamRateHz))
	if n < 1 {
		return 1
	}
	return n
}

func (r *signalRow) setStats(mean, pkpk float64) {
	text := fmt.Sprintf("Mean: %.4g   Pk-Pk: %.4g", mean, pkpk)
	if r.stats.Text != text {
		r.stats.Text = text
		r.stats.Refresh()
	}
}

// UI owns every widget and all UI-goroutine state. Everything on it is touched
// only from the Fyne goroutine; background goroutines hop across with fyne.Do.
type UI struct {
	app     fyne.App
	win     fyne.Window
	monitor *Monitor

	worker *SerialWorker

	ports      []PortInfo
	portSelect *widget.Select
	connectBtn *widget.Button
	pingBtn    *widget.Button
	streamBtn  *widget.Button
	statusText *canvas.Text
	statusBar  *widget.Label

	triggerEnable *widget.Check
	triggerSource *widget.Select
	triggerEdge   *widget.Select
	triggerLevel  *numField
	triggerStatus *canvas.Text

	rows     []*signalRow
	rowByKey map[string]*signalRow

	saveCSVBtn   *widget.Button
	samplesLabel *canvas.Text

	streaming   bool
	pingPending bool
	pingTimer   *time.Timer
	csvBusy     bool
	fftOpen     map[string]bool
}

func NewUI(app fyne.App, win fyne.Window) *UI {
	u := &UI{
		app:      app,
		win:      win,
		monitor:  NewMonitor(),
		rowByKey: make(map[string]*signalRow, len(SignalNames)),
		fftOpen:  make(map[string]bool, len(SignalNames)),
	}
	u.build()
	u.refreshPorts()
	return u
}

// ------------------------------------------------------------------- layout

func (u *UI) build() {
	u.win.SetContent(container.NewBorder(
		container.NewVBox(u.buildConnectionCard(), u.buildTriggerCard()),
		container.NewVBox(u.buildExportCard(), u.buildStatusBar()),
		nil, nil,
		u.buildSignalsCard(),
	))
	u.win.SetOnClosed(func() {
		if u.worker != nil {
			u.worker.Stop()
		}
	})
}

func (u *UI) buildConnectionCard() fyne.CanvasObject {
	u.portSelect = widget.NewSelect(nil, nil)
	u.portSelect.PlaceHolder = "(no ports found)"

	refreshBtn := widget.NewButton("Refresh", u.refreshPorts)

	u.connectBtn = widget.NewButton("Connect", u.onConnectClicked)
	u.connectBtn.Importance = widget.HighImportance

	u.pingBtn = widget.NewButton("Test Connection", u.onPingClicked)
	u.pingBtn.Disable()

	u.streamBtn = widget.NewButton("Start Streaming", u.onStreamClicked)
	u.streamBtn.Disable()

	u.statusText = newColoredText("Disconnected", colHeading, true)

	// connectBtn and streamBtn toggle their labels ("Connect"/"Disconnect"),
	// and a button's MinSize change does not re-flow the surrounding HBox on
	// its own -- the longer label would render clipped. Pin both to a width
	// that fits either label.
	left := container.NewHBox(
		widget.NewLabel("COM Port:"),
		container.New(&fixedWidthLayout{width: 260}, u.portSelect),
		refreshBtn,
		container.New(&fixedWidthLayout{width: 108}, u.connectBtn),
		u.pingBtn,
		container.New(&fixedWidthLayout{width: 140}, u.streamBtn),
	)
	return widget.NewCard("Connection", "",
		container.NewBorder(nil, nil, left, rightStatus(u.statusText, 300), layout.NewSpacer()))
}

func (u *UI) buildTriggerCard() fyne.CanvasObject {
	u.triggerEnable = widget.NewCheck("Enable", nil)

	labels := make([]string, 0, len(SignalNames))
	for _, name := range SignalNames {
		labels = append(labels, SignalLabels[name])
	}
	u.triggerSource = widget.NewSelect(labels, nil)
	u.triggerSource.SetSelectedIndex(0)

	u.triggerEdge = widget.NewSelect([]string{"Rising", "Falling"}, nil)
	u.triggerEdge.SetSelectedIndex(0)

	u.triggerLevel = newNumField(0, -1_000_000, 1_000_000, 0.1, 4, 110)

	u.triggerStatus = newColoredText("", colHeading, false)

	left := container.NewHBox(
		u.triggerEnable,
		widget.NewLabel("Source:"),
		u.triggerSource,
		widget.NewLabel("Edge:"),
		u.triggerEdge,
		widget.NewLabel("Level:"),
		u.triggerLevel.Object(),
	)
	return widget.NewCard("Trigger", "",
		container.NewBorder(nil, nil, left, rightStatus(u.triggerStatus, 190), layout.NewSpacer()))
}

func (u *UI) buildSignalsCard() fyne.CanvasObject {
	objs := make([]fyne.CanvasObject, 0, len(SignalNames))

	for _, name := range SignalNames {
		label := SignalLabels[name]
		row := &signalRow{name: name, label: label}

		row.check = widget.NewCheck("Export", nil)
		row.check.SetChecked(true)

		fftBtn := widget.NewButton("FFT", func() { u.openFFT(row.name, row.label) })

		row.window = newNumField(DefaultLiveWindowSeconds,
			MinLiveWindowSeconds, MaxLiveWindowSeconds, 0.01, 3, 92)

		row.stats = newColoredText("Mean: -   Pk-Pk: -", colHeading, false)
		row.stats.TextStyle = fyne.TextStyle{Monospace: true}

		row.plot = NewPlotWidget(label, "Time (s)")

		side := container.NewVBox(
			newColoredText(label, colHeading, true),
			row.check,
			fftBtn,
			newColoredText("Window", colText, false),
			row.window.Object(),
			layout.NewSpacer(),
		)

		row.obj = container.NewBorder(
			nil, nil,
			container.New(&fixedWidthLayout{width: 150}, side), nil,
			container.NewBorder(row.stats, nil, nil, nil, row.plot),
		)

		u.rows = append(u.rows, row)
		u.rowByKey[name] = row
		objs = append(objs, row.obj)
	}

	grid := container.New(layout.NewGridLayoutWithRows(len(objs)), objs...)
	return widget.NewCard("Monitored Signals", "", grid)
}

func (u *UI) buildExportCard() fyne.CanvasObject {
	u.saveCSVBtn = widget.NewButton("Save Selected as CSV", u.saveCSV)
	savePNGBtn := widget.NewButton("Save Selected as PNG", u.savePNG)
	u.samplesLabel = newColoredText("0 samples captured", colText, false)

	left := container.NewHBox(u.saveCSVBtn, savePNGBtn)
	return widget.NewCard("Export", "",
		container.NewBorder(nil, nil, left, rightStatus(u.samplesLabel, 210), layout.NewSpacer()))
}

func (u *UI) buildStatusBar() fyne.CanvasObject {
	u.statusBar = widget.NewLabel("Select a COM port and click Connect.")
	return container.NewVBox(widget.NewSeparator(), u.statusBar)
}

func newColoredText(text string, col color.Color, bold bool) *canvas.Text {
	t := canvas.NewText(text, col)
	t.TextSize = 13
	t.TextStyle = fyne.TextStyle{Bold: bold}
	return t
}

// rightStatus pins a status label to a fixed width and right-aligns it. A bare
// canvas.Text grows its MinSize as the string gets longer, which only takes
// effect on the next parent re-layout -- so a longer message (e.g. "Connected:
// /dev/ttyACM0" replacing "Disconnected") would render clipped until the window
// was resized. Fixing the width sidesteps that entirely.
func rightStatus(t *canvas.Text, width float32) fyne.CanvasObject {
	t.Alignment = fyne.TextAlignTrailing
	return container.New(&fixedWidthLayout{width: width}, t)
}

// ---------------------------------------------------------------- connection

func (u *UI) setStatus(msg string) {
	u.statusBar.SetText(msg)
}

func (u *UI) refreshPorts() {
	u.ports = ListSerialPorts()

	opts := make([]string, 0, len(u.ports))
	for _, p := range u.ports {
		opts = append(opts, fmt.Sprintf("%s  (%s)", p.Device, p.Description))
	}
	u.portSelect.Options = opts
	if len(opts) > 0 {
		u.portSelect.SetSelectedIndex(0)
	} else {
		u.portSelect.ClearSelected()
	}
	u.portSelect.Refresh()
}

func (u *UI) onConnectClicked() {
	if u.worker != nil {
		u.connectBtn.Disable()
		u.setStatus("Disconnecting...")
		w := u.worker
		go w.Stop()
		return
	}

	idx := u.portSelect.SelectedIndex()
	if idx < 0 || idx >= len(u.ports) {
		dialog.ShowInformation("No port selected", "No COM ports found. Click Refresh.", u.win)
		return
	}
	portName := u.ports[idx].Device

	w := NewSerialWorker(portName)
	w.onConnected = func(p string) { fyne.Do(func() { u.onConnected(p) }) }
	w.onDisconnected = func() { fyne.Do(u.onDisconnected) }
	w.onError = func(msg string) { fyne.Do(func() { u.onError(msg) }) }
	w.onPingOK = func() { fyne.Do(u.onPingOK) }

	if err := w.Start(); err != nil {
		dialog.ShowError(err, u.win)
		u.setStatus(err.Error())
		return
	}

	u.worker = w
	u.connectBtn.Disable()
	go u.consume(w)
}

func (u *UI) consume(w *SerialWorker) {
	for f := range w.Frames() {
		if u.monitor.Add(f) {
			fyne.Do(func() {
				u.setStatus("Session buffer full (2,000,000 samples) - save and reconnect to continue capturing.")
			})
		}
	}
}

func (u *UI) onConnected(port string) {
	u.statusText.Text = "Connected: " + port
	u.statusText.Color = colOK
	u.statusText.Refresh()

	u.connectBtn.SetText("Disconnect")
	u.connectBtn.Enable()
	u.pingBtn.Enable()
	u.streamBtn.Enable()
	u.setStatus("Connected to " + port + ".")
}

func (u *UI) onDisconnected() {
	u.worker = nil
	u.streaming = false

	u.statusText.Text = "Disconnected"
	u.statusText.Color = colHeading
	u.statusText.Refresh()

	u.connectBtn.SetText("Connect")
	u.connectBtn.Enable()
	u.pingBtn.Disable()
	u.streamBtn.SetText("Start Streaming")
	u.streamBtn.Disable()
}

func (u *UI) onError(msg string) {
	u.statusText.Text = "Error"
	u.statusText.Color = colError
	u.statusText.Refresh()

	u.setStatus(msg)
	dialog.ShowError(fmt.Errorf("%s", msg), u.win)

	if u.worker != nil {
		w := u.worker
		go w.Stop()
	}
}

func (u *UI) onPingClicked() {
	if u.worker == nil {
		return
	}
	u.pingPending = true
	u.worker.Ping()
	u.setStatus("Pinging device...")

	if u.pingTimer != nil {
		u.pingTimer.Stop()
	}
	u.pingTimer = time.AfterFunc(pingTimeout, func() { fyne.Do(u.onPingTimeout) })
}

func (u *UI) onPingOK() {
	if !u.pingPending {
		return
	}
	u.pingPending = false
	if u.pingTimer != nil {
		u.pingTimer.Stop()
	}
	u.setStatus("Device identified: HCA1001_ADC_STREAM_V1")
}

func (u *UI) onPingTimeout() {
	if !u.pingPending {
		return
	}
	u.pingPending = false
	u.setStatus("No reply from device. Wrong COM port, or firmware not flashed.")
}

func (u *UI) onStreamClicked() {
	if u.worker == nil {
		return
	}
	if u.streaming {
		u.worker.StopStreaming()
		u.streaming = false
		u.streamBtn.SetText("Start Streaming")
		return
	}
	u.worker.StartStreaming()
	u.streaming = true
	u.streamBtn.SetText("Stop Streaming")
}

// --------------------------------------------------------------- update loop

// StartUpdateLoop begins the redraw ticker. Call it from the app's OnStarted
// lifecycle hook: fyne.Do hands work to the main event loop, so the loop has
// to be running before the first tick fires.
func (u *UI) StartUpdateLoop() {
	go func() {
		ticker := time.NewTicker(guiUpdateInterval)
		defer ticker.Stop()
		for range ticker.C {
			fyne.Do(u.update)
		}
	}()
}

func (u *UI) update() {
	snap := u.monitor.Snapshot()
	t := snap.T

	// Trigger: find the most recent edge crossing that still leaves a full
	// window's worth of samples after it, so every row -- each of which may
	// have a different window length -- is sliced from the exact same point
	// in time and stays aligned with the others.
	trigIdx := -1
	if u.triggerEnable.Checked && len(t) > 1 {
		sourceName := SignalNames[maxInt(u.triggerSource.SelectedIndex(), 0)]
		maxWindow := 0
		for _, row := range u.rows {
			if n := row.windowSamples(); n > maxWindow {
				maxWindow = n
			}
		}
		trigIdx = findTriggerIndex(snap.Signals[sourceName],
			u.triggerEdge.Selected, u.triggerLevel.Value(), maxWindow)

		if trigIdx >= 0 {
			u.setTriggerStatus("Triggered", colOK)
		} else {
			u.setTriggerStatus("Waiting for trigger...", colWarning)
		}
	} else {
		u.setTriggerStatus("", colHeading)
	}

	for _, row := range u.rows {
		y := snap.Signals[row.name]
		n := minInt(len(t), row.windowSamples())

		var tw, yw []float64
		if trigIdx >= 0 {
			end := minInt(trigIdx+n, len(y))
			yw = y[trigIdx:end]
			tw = make([]float64, end-trigIdx)
			for i := range tw {
				tw[i] = t[trigIdx+i] - t[trigIdx]
			}
		} else {
			yw = y[len(y)-minInt(n, len(y)):]
			tw = t[len(t)-minInt(n, len(t)):]
		}

		row.plot.SetData(tw, yw, trigIdx >= 0, 0)
		mean, pkpk := meanPkPk(yw)
		row.setStats(mean, pkpk)
	}

	text := commafy(snap.SessionLen) + " samples captured"
	if u.samplesLabel.Text != text {
		u.samplesLabel.Text = text
		u.samplesLabel.Refresh()
	}
}

func (u *UI) setTriggerStatus(text string, col color.Color) {
	if u.triggerStatus.Text == text {
		return
	}
	u.triggerStatus.Text = text
	u.triggerStatus.Color = col
	u.triggerStatus.Refresh()
}

// ----------------------------------------------------------------------- FFT

func (u *UI) openFFT(name, label string) {
	if u.fftOpen[name] {
		return
	}
	u.fftOpen[name] = true
	showFFTWindow(u.app, name, label, u.monitor.LiveBuffer, func() {
		fyne.Do(func() { u.fftOpen[name] = false })
	})
}

// -------------------------------------------------------------------- export

func (u *UI) selectedRows() []*signalRow {
	out := make([]*signalRow, 0, len(u.rows))
	for _, row := range u.rows {
		if row.check.Checked {
			out = append(out, row)
		}
	}
	return out
}

func (u *UI) saveCSV() {
	if u.csvBusy {
		dialog.ShowInformation("Export in progress", "A CSV export is already running.", u.win)
		return
	}
	rows := u.selectedRows()
	if len(rows) == 0 {
		dialog.ShowInformation("Nothing selected", "Check at least one signal to export.", u.win)
		return
	}
	records := u.monitor.SessionSnapshot()
	if len(records) == 0 {
		dialog.ShowInformation("No data", "No samples captured yet.", u.win)
		return
	}

	names := make([]string, 0, len(rows))
	for _, row := range rows {
		names = append(names, row.name)
	}

	save := dialog.NewFileSave(func(wc fyne.URIWriteCloser, err error) {
		if err != nil {
			dialog.ShowError(err, u.win)
			return
		}
		if wc == nil {
			return
		}

		u.csvBusy = true
		u.saveCSVBtn.Disable()
		path := wc.URI().Path()
		u.setStatus(fmt.Sprintf("Saving %s rows to %s ...", commafy(len(records)), path))

		go func() {
			count, werr := writeCSV(wc, names, records)
			cerr := wc.Close()
			if werr == nil {
				werr = cerr
			}
			fyne.Do(func() {
				u.csvBusy = false
				u.saveCSVBtn.Enable()
				if werr != nil {
					dialog.ShowError(werr, u.win)
					u.setStatus("Save failed: " + werr.Error())
					return
				}
				u.setStatus(fmt.Sprintf("Saved %s rows to %s", commafy(count), path))
			})
		}()
	}, u.win)
	save.SetFileName("hca1001_adc_log.csv")
	save.Show()
}

func (u *UI) savePNG() {
	rows := u.selectedRows()
	if len(rows) == 0 {
		dialog.ShowInformation("Nothing selected", "Check at least one signal to export.", u.win)
		return
	}

	dialog.ShowFolderOpen(func(dir fyne.ListableURI, err error) {
		if err != nil {
			dialog.ShowError(err, u.win)
			return
		}
		if dir == nil {
			return
		}
		saved, werr := writePNGs(dir, rows)
		if werr != nil {
			dialog.ShowError(werr, u.win)
			u.setStatus("PNG export failed: " + werr.Error())
			return
		}
		u.setStatus(fmt.Sprintf("Saved %d PNG file(s) to %s", len(saved), dir.Path()))
	}, u.win)
}

// -------------------------------------------------------------------- helpers

func minInt(a, b int) int {
	if a < b {
		return a
	}
	return b
}

func maxInt(a, b int) int {
	if a > b {
		return a
	}
	return b
}

func itoa(v int) string { return strconv.Itoa(v) }

// commafy renders 1234567 as "1,234,567", matching the Python GUI's "{:,}".
func commafy(v int) string {
	s := strconv.Itoa(v)
	neg := strings.HasPrefix(s, "-")
	if neg {
		s = s[1:]
	}

	var b strings.Builder
	for i, c := range s {
		if i > 0 && (len(s)-i)%3 == 0 {
			b.WriteByte(',')
		}
		b.WriteRune(c)
	}
	if neg {
		return "-" + b.String()
	}
	return b.String()
}
