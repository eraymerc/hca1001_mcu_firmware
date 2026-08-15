package main

import (
	"math"
	"strconv"

	"fyne.io/fyne/v2"
	"fyne.io/fyne/v2/container"
	"fyne.io/fyne/v2/theme"
	"fyne.io/fyne/v2/widget"
)

// numField is a compact numeric entry with step buttons -- Fyne has no
// spinbox, and typing into a bare Entry is awkward for values you want to
// nudge (plot window length, trigger level).
type numField struct {
	entry *widget.Entry
	obj   fyne.CanvasObject

	value    float64
	min, max float64
	step     float64
	decimals int

	OnChanged func(float64)
}

func newNumField(value, min, max, step float64, decimals int, width float32) *numField {
	n := &numField{
		value:    value,
		min:      min,
		max:      max,
		step:     step,
		decimals: decimals,
	}

	n.entry = widget.NewEntry()
	n.entry.SetText(n.format(value))
	n.entry.OnChanged = func(s string) {
		v, err := strconv.ParseFloat(s, 64)
		if err != nil {
			return // mid-typing ("", "-", "1."): keep the last good value
		}
		n.set(v, false)
	}
	n.entry.OnSubmitted = func(string) { n.entry.SetText(n.format(n.value)) }

	dec := widget.NewButtonWithIcon("", theme.ContentRemoveIcon(), func() { n.nudge(-1) })
	inc := widget.NewButtonWithIcon("", theme.ContentAddIcon(), func() { n.nudge(+1) })
	dec.Importance = widget.LowImportance
	inc.Importance = widget.LowImportance

	sized := container.New(&fixedWidthLayout{width: width}, n.entry)
	n.obj = container.NewHBox(dec, sized, inc)
	return n
}

func (n *numField) format(v float64) string {
	return strconv.FormatFloat(v, 'f', n.decimals, 64)
}

func (n *numField) clamp(v float64) float64 {
	return math.Min(math.Max(v, n.min), n.max)
}

// set updates the value, optionally rewriting the entry text. Text is left
// alone when the change came from typing, so the caret does not jump.
func (n *numField) set(v float64, syncText bool) {
	v = n.clamp(v)
	if v == n.value && !syncText {
		return
	}
	n.value = v
	if syncText {
		n.entry.SetText(n.format(v))
	}
	if n.OnChanged != nil {
		n.OnChanged(v)
	}
}

func (n *numField) nudge(dir float64) {
	n.set(n.value+dir*n.step, true)
}

func (n *numField) Value() float64 { return n.value }

func (n *numField) Object() fyne.CanvasObject { return n.obj }

// fixedWidthLayout pins its content to a set width while letting Fyne pick
// the height, so entries in the side panels line up.
type fixedWidthLayout struct{ width float32 }

func (l *fixedWidthLayout) MinSize(objects []fyne.CanvasObject) fyne.Size {
	h := float32(0)
	for _, o := range objects {
		if mh := o.MinSize().Height; mh > h {
			h = mh
		}
	}
	return fyne.NewSize(l.width, h)
}

func (l *fixedWidthLayout) Layout(objects []fyne.CanvasObject, size fyne.Size) {
	for _, o := range objects {
		o.Resize(fyne.NewSize(l.width, size.Height))
		o.Move(fyne.NewPos(0, 0))
	}
}
