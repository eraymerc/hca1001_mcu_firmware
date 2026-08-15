// A scope-style line plot rendered straight to a raster image.
//
// Fyne's canvas is a retained scene graph, so representing 30,000 samples as
// canvas objects would be hopeless. Instead the whole plot -- grid, axes, tick
// labels and curve -- is drawn into an image.RGBA that Fyne blits as a single
// object, and the curve is min/max downsampled to one vertical segment per
// pixel column. That is the same trick pyqtgraph uses, and it makes redraw
// cost depend on plot width rather than sample count.
//
// Antialiasing is deliberately off for the curve (as it was in the Python GUI,
// see dark_theme.py) -- it is the per-frame budget the update timer depends on
// staying cheap.
package main

import (
	"image"
	"image/color"
	"image/draw"
	"math"
	"strconv"
	"sync"

	"fyne.io/fyne/v2"
	"fyne.io/fyne/v2/canvas"
	"fyne.io/fyne/v2/widget"
	"golang.org/x/image/font"
	"golang.org/x/image/font/gofont/goregular"
	"golang.org/x/image/font/opentype"
	"golang.org/x/image/math/fixed"
)

var (
	plotGrid  = color.NRGBA{R: 0x2f, G: 0x2f, B: 0x2f, A: 0xff}
	plotAxis  = color.NRGBA{R: 0x4a, G: 0x4a, B: 0x4a, A: 0xff}
	plotLabel = color.NRGBA{R: 0x9a, G: 0x9a, B: 0x9a, A: 0xff}
	plotTitle = colHeading
	plotTrig  = colWarning
)

// ---------------------------------------------------------------- font cache

var (
	fontOnce   sync.Once
	fontParsed *opentype.Font
	faceMu     sync.Mutex
	faces      = map[int]font.Face{}
)

func faceAt(px float64) font.Face {
	fontOnce.Do(func() {
		fontParsed, _ = opentype.Parse(goregular.TTF)
	})
	if fontParsed == nil {
		return nil
	}

	key := int(math.Round(px * 2))
	if key < 12 {
		key = 12
	}

	faceMu.Lock()
	defer faceMu.Unlock()

	if f, ok := faces[key]; ok {
		return f
	}
	f, err := opentype.NewFace(fontParsed, &opentype.FaceOptions{
		Size:    float64(key) / 2,
		DPI:     72,
		Hinting: font.HintingFull,
	})
	if err != nil {
		return nil
	}
	faces[key] = f
	return f
}

func drawText(dst *image.RGBA, face font.Face, col color.Color, x, y int, s string) {
	if face == nil {
		return
	}
	d := &font.Drawer{
		Dst:  dst,
		Src:  image.NewUniform(col),
		Face: face,
		Dot:  fixed.P(x, y),
	}
	d.DrawString(s)
}

func textWidth(face font.Face, s string) int {
	if face == nil {
		return 0
	}
	return font.MeasureString(face, s).Round()
}

// ------------------------------------------------------------- tick helpers

// niceTicks picks round tick values covering [lo, hi], Heckbert's algorithm.
func niceTicks(lo, hi float64, target int) (start, step float64, count int) {
	if target < 2 {
		target = 2
	}
	span := hi - lo
	if span <= 0 || math.IsNaN(span) || math.IsInf(span, 0) {
		return lo, 0, 0
	}

	rough := span / float64(target-1)
	mag := math.Pow(10, math.Floor(math.Log10(rough)))
	norm := rough / mag
	switch {
	case norm < 1.5:
		step = 1 * mag
	case norm < 3:
		step = 2 * mag
	case norm < 7:
		step = 5 * mag
	default:
		step = 10 * mag
	}

	start = math.Ceil(lo/step) * step
	for v := start; v <= hi+step*1e-9; v += step {
		count++
	}
	return start, step, count
}

func formatTick(v, step float64) string {
	if step <= 0 {
		return strconv.FormatFloat(v, 'g', 4, 64)
	}
	if v == 0 {
		return "0"
	}
	av := math.Abs(v)
	if av >= 1e5 || av < 1e-4 {
		return strconv.FormatFloat(v, 'e', 1, 64)
	}
	d := int(math.Ceil(-math.Log10(step)))
	if d < 0 {
		d = 0
	}
	if d > 6 {
		d = 6
	}
	return strconv.FormatFloat(v, 'f', d, 64)
}

// ---------------------------------------------------------- raster primitives

func fillRect(dst *image.RGBA, r image.Rectangle, c color.Color) {
	draw.Draw(dst, r.Intersect(dst.Bounds()), image.NewUniform(c), image.Point{}, draw.Src)
}

// blendRect composites a (usually translucent) colour over what is already there.
func blendRect(dst *image.RGBA, r image.Rectangle, c color.Color) {
	draw.Draw(dst, r.Intersect(dst.Bounds()), image.NewUniform(c), image.Point{}, draw.Over)
}

func hLine(dst *image.RGBA, x0, x1, y int, c color.Color) {
	if x0 > x1 {
		x0, x1 = x1, x0
	}
	fillRect(dst, image.Rect(x0, y, x1+1, y+1), c)
}

func vLine(dst *image.RGBA, x, y0, y1 int, c color.Color) {
	if y0 > y1 {
		y0, y1 = y1, y0
	}
	fillRect(dst, image.Rect(x, y0, x+1, y1+1), c)
}

// line draws a 1px Bresenham segment, clipped to clip.
func line(dst *image.RGBA, x0, y0, x1, y1 int, clip image.Rectangle, c color.Color) {
	dx := abs(x1 - x0)
	dy := -abs(y1 - y0)
	sx, sy := -1, -1
	if x0 < x1 {
		sx = 1
	}
	if y0 < y1 {
		sy = 1
	}
	err := dx + dy

	for {
		if x0 >= clip.Min.X && x0 < clip.Max.X && y0 >= clip.Min.Y && y0 < clip.Max.Y {
			dst.Set(x0, y0, c)
		}
		if x0 == x1 && y0 == y1 {
			return
		}
		e2 := 2 * err
		if e2 >= dy {
			err += dy
			x0 += sx
		}
		if e2 <= dx {
			err += dx
			y0 += sy
		}
	}
}

func abs(v int) int {
	if v < 0 {
		return -v
	}
	return v
}

func clampInt(v, lo, hi int) int {
	if v < lo {
		return lo
	}
	if v > hi {
		return hi
	}
	return v
}

// ------------------------------------------------------------------ rendering

type plotData struct {
	xs, ys     []float64
	xLabel     string
	yLabel     string
	curve      color.NRGBA
	hasTrigger bool
	triggerX   float64
}

func renderPlot(w, h int, scale float64, d plotData) *image.RGBA {
	if w < 1 {
		w = 1
	}
	if h < 1 {
		h = 1
	}
	img := image.NewRGBA(image.Rect(0, 0, w, h))
	fillRect(img, img.Bounds(), colBackground)

	if scale < 1 {
		scale = 1
	}
	if scale > 4 {
		scale = 4
	}
	px := func(v float64) int { return int(math.Round(v * scale)) }

	face := faceAt(11 * scale)
	small := faceAt(10 * scale)

	// Reserve room for the y tick labels on the left and x ticks below.
	left := px(52)
	right := w - px(10)
	top := px(8)
	bottom := h - px(22)

	if right-left < 8 || bottom-top < 8 {
		return img
	}
	plotRect := image.Rect(left, top, right, bottom)

	// ---- data bounds ----
	n := len(d.xs)
	if n > len(d.ys) {
		n = len(d.ys)
	}

	xMin, xMax := 0.0, 1.0
	yMin, yMax := -1.0, 1.0
	if n > 0 {
		xMin, xMax = d.xs[0], d.xs[n-1]
		yMin, yMax = d.ys[0], d.ys[0]
		for i := 0; i < n; i++ {
			if d.ys[i] < yMin {
				yMin = d.ys[i]
			}
			if d.ys[i] > yMax {
				yMax = d.ys[i]
			}
		}
	}
	if !(xMax > xMin) {
		xMax = xMin + 1e-6
	}
	if yMax-yMin < 1e-12 {
		pad := math.Max(math.Abs(yMax)*0.05, 1e-6)
		yMin, yMax = yMin-pad, yMax+pad
	} else {
		pad := (yMax - yMin) * 0.06
		yMin, yMax = yMin-pad, yMax+pad
	}

	sx := func(v float64) int {
		return left + int(math.Round((v-xMin)/(xMax-xMin)*float64(right-left-1)))
	}
	sy := func(v float64) int {
		return bottom - 1 - int(math.Round((v-yMin)/(yMax-yMin)*float64(bottom-top-1)))
	}

	// ---- grid + tick labels ----
	yStart, yStep, yCount := niceTicks(yMin, yMax, 6)
	for i := 0; i < yCount; i++ {
		v := yStart + float64(i)*yStep
		y := sy(v)
		if y < top || y >= bottom {
			continue
		}
		hLine(img, left, right-1, y, plotGrid)

		label := formatTick(v, yStep)
		tw := textWidth(face, label)
		drawText(img, face, plotLabel, left-px(6)-tw, y+px(4), label)
	}

	xStart, xStep, xCount := niceTicks(xMin, xMax, 7)
	for i := 0; i < xCount; i++ {
		v := xStart + float64(i)*xStep
		x := sx(v)
		if x < left || x >= right {
			continue
		}
		vLine(img, x, top, bottom-1, plotGrid)

		label := formatTick(v, xStep)
		tw := textWidth(face, label)
		lx := clampInt(x-tw/2, 0, w-tw)
		drawText(img, face, plotLabel, lx, bottom+px(14), label)
	}

	// ---- axes ----
	vLine(img, left, top, bottom-1, plotAxis)
	hLine(img, left, right-1, bottom-1, plotAxis)

	// Captions go inside the plot area, top-left and top-right, so they can
	// never collide with the tick labels along the axes. They are drawn after
	// the curve (below) so they stay readable over dense traces.
	// A dense trace runs straight through the caption row, so each caption gets
	// a translucent plate behind it to stay readable.
	plate := color.NRGBA{R: 0x1e, G: 0x1e, B: 0x1e, A: 0xcc}
	caption := func(x, baseline int, col color.Color, s string) {
		tw := textWidth(small, s)
		blendRect(img, image.Rect(x-px(3), baseline-px(11), x+tw+px(3), baseline+px(4)), plate)
		drawText(img, small, col, x, baseline, s)
	}

	drawCaptions := func() {
		baseline := top + px(12)
		if d.yLabel != "" {
			caption(left+px(6), baseline, plotTitle, d.yLabel)
		}
		if d.xLabel != "" {
			caption(right-px(6)-textWidth(small, d.xLabel), baseline, plotLabel, d.xLabel)
		}
	}

	if n == 0 {
		drawCaptions()
		msg := "waiting for data"
		tw := textWidth(face, msg)
		drawText(img, face, plotLabel, left+(right-left-tw)/2, (top+bottom)/2, msg)
		return img
	}

	// ---- curve ----
	curveCol := d.curve
	if curveCol.A == 0 {
		curveCol = colAccent
	}
	plotW := right - left

	if n > plotW*2 && plotW > 0 {
		// Dense: one vertical min/max segment per pixel column, joined to the
		// previous column so the trace stays continuous.
		colMin := make([]int, plotW)
		colMax := make([]int, plotW)
		colSet := make([]bool, plotW)
		colLast := make([]int, plotW)

		for i := 0; i < n; i++ {
			c := int((d.xs[i] - xMin) / (xMax - xMin) * float64(plotW-1))
			c = clampInt(c, 0, plotW-1)
			y := sy(d.ys[i])
			if !colSet[c] {
				colSet[c], colMin[c], colMax[c] = true, y, y
			} else {
				if y < colMin[c] {
					colMin[c] = y
				}
				if y > colMax[c] {
					colMax[c] = y
				}
			}
			colLast[c] = y
		}

		prevX, prevY, havePrev := 0, 0, false
		for c := 0; c < plotW; c++ {
			if !colSet[c] {
				continue
			}
			x := left + c
			lo := clampInt(colMin[c], top, bottom-1)
			hi := clampInt(colMax[c], top, bottom-1)
			vLine(img, x, lo, hi, curveCol)

			if havePrev {
				line(img, prevX, prevY, x, clampInt(colMin[c], top, bottom-1), plotRect, curveCol)
			}
			prevX, prevY, havePrev = x, clampInt(colLast[c], top, bottom-1), true
		}
	} else {
		for i := 1; i < n; i++ {
			line(img, sx(d.xs[i-1]), sy(d.ys[i-1]), sx(d.xs[i]), sy(d.ys[i]), plotRect, curveCol)
		}
		if n == 1 {
			x, y := sx(d.xs[0]), sy(d.ys[0])
			if plotRect.Min.X <= x && x < plotRect.Max.X {
				vLine(img, x, clampInt(y, top, bottom-1), clampInt(y, top, bottom-1), curveCol)
			}
		}
	}

	// ---- trigger marker ----
	if d.hasTrigger {
		x := sx(d.triggerX)
		if x >= left && x < right {
			for y := top; y < bottom; y++ {
				if (y/px(4))%2 == 0 { // dashed, so it stays readable over the trace
					img.Set(x, y, plotTrig)
				}
			}
		}
	}

	drawCaptions()
	return img
}

// ------------------------------------------------------------------- widget

// PlotWidget is a Fyne widget wrapping renderPlot. SetData touches the UI
// (raster Refresh), so like any other UI call it belongs on the main loop --
// callers on a background goroutine must wrap it in fyne.Do.
type PlotWidget struct {
	widget.BaseWidget

	mu     sync.Mutex
	data   plotData
	raster *canvas.Raster
}

func NewPlotWidget(yLabel, xLabel string) *PlotWidget {
	p := &PlotWidget{}
	p.data.yLabel = yLabel
	p.data.xLabel = xLabel
	p.data.curve = colAccent
	p.ExtendBaseWidget(p)
	return p
}

func (p *PlotWidget) CreateRenderer() fyne.WidgetRenderer {
	p.mu.Lock()
	if p.raster == nil {
		p.raster = canvas.NewRaster(p.generate)
	}
	r := p.raster
	p.mu.Unlock()
	return widget.NewSimpleRenderer(r)
}

func (p *PlotWidget) MinSize() fyne.Size {
	return fyne.NewSize(260, 130)
}

func (p *PlotWidget) generate(w, h int) image.Image {
	p.mu.Lock()
	d := p.data
	p.mu.Unlock()

	scale := 1.0
	if lw := p.Size().Width; lw > 0 {
		scale = float64(w) / float64(lw)
	}
	return renderPlot(w, h, scale, d)
}

// SetData replaces the plotted series. The caller must not mutate xs/ys after
// handing them over (every caller passes freshly allocated snapshot slices).
func (p *PlotWidget) SetData(xs, ys []float64, hasTrigger bool, triggerX float64) {
	p.mu.Lock()
	p.data.xs, p.data.ys = xs, ys
	p.data.hasTrigger, p.data.triggerX = hasTrigger, triggerX
	r := p.raster
	p.mu.Unlock()

	if r != nil {
		r.Refresh()
	}
}

func (p *PlotWidget) SetYLabel(label string) {
	p.mu.Lock()
	p.data.yLabel = label
	r := p.raster
	p.mu.Unlock()

	if r != nil {
		r.Refresh()
	}
}

// Snapshot renders the current series at an explicit size, for PNG export.
func (p *PlotWidget) Snapshot(w, h int, scale float64) image.Image {
	p.mu.Lock()
	d := p.data
	p.mu.Unlock()

	return renderPlot(w, h, scale, d)
}
