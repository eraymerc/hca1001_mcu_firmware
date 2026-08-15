package main

import (
	"encoding/csv"
	"image/png"
	"io"
	"strconv"
	"time"

	"fyne.io/fyne/v2"
	"fyne.io/fyne/v2/storage"
)

// writeCSV streams the session record to w. It runs on its own goroutine:
// writing up to MaxSessionSamples (2,000,000) rows synchronously would freeze
// the UI for a noticeable time. It only touches the caller's snapshot, never
// live widgets, so it is safe off the UI goroutine.
func writeCSV(w io.Writer, signals []string, records []Record) (int, error) {
	cw := csv.NewWriter(w)

	header := []string{"seq", "timestamp_ms"}
	for _, name := range signals {
		header = append(header, SignalLabels[name])
	}
	if err := cw.Write(header); err != nil {
		return 0, err
	}

	row := make([]string, len(header))
	for _, rec := range records {
		row[0] = strconv.FormatUint(uint64(rec.Seq), 10)
		row[1] = strconv.FormatUint(uint64(rec.TimestampMs), 10)
		for i, name := range signals {
			v := rec.Voltage
			if name == "error" {
				v = rec.Error
			}
			row[2+i] = strconv.FormatFloat(v, 'g', -1, 64)
		}
		if err := cw.Write(row); err != nil {
			return 0, err
		}
	}

	cw.Flush()
	return len(records), cw.Error()
}

const (
	pngExportWidth  = 1400
	pngExportHeight = 560
	pngExportScale  = 2.0
)

// writePNGs renders each selected plot at export resolution into dir.
func writePNGs(dir fyne.ListableURI, rows []*signalRow) ([]string, error) {
	stamp := time.Now().Format("20060102_150405")
	saved := make([]string, 0, len(rows))

	for _, row := range rows {
		img := row.plot.Snapshot(pngExportWidth, pngExportHeight, pngExportScale)

		child, err := storage.Child(dir, row.name+"_"+stamp+".png")
		if err != nil {
			return saved, err
		}
		wc, err := storage.Writer(child)
		if err != nil {
			return saved, err
		}
		if err := png.Encode(wc, img); err != nil {
			wc.Close()
			return saved, err
		}
		if err := wc.Close(); err != nil {
			return saved, err
		}
		saved = append(saved, child.Name())
	}

	return saved, nil
}
