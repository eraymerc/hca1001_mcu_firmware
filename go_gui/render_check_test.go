package main

import (
	"encoding/binary"
	"image"
	"image/png"
	"math"
	"math/rand"
	"os"
	"testing"
)

func TestProtocolRoundTrip(t *testing.T) {
	// Build a frame exactly the way the firmware does, then parse it.
	buf := make([]byte, FrameSize)
	buf[0], buf[1] = Sync0, Sync1
	binary.LittleEndian.PutUint32(buf[2:6], 12345)
	binary.LittleEndian.PutUint32(buf[6:10], 987654)
	binary.LittleEndian.PutUint32(buf[10:14], math.Float32bits(3.25))
	binary.LittleEndian.PutUint32(buf[14:18], math.Float32bits(-0.125))
	buf[18] = checksum8(buf[2:18])

	f, ok := ParseFrame(buf)
	if !ok {
		t.Fatal("valid frame rejected")
	}
	if f.Seq != 12345 || f.TimestampMs != 987654 || f.Voltage != 3.25 || f.Error != -0.125 {
		t.Fatalf("round trip mismatch: %+v", f)
	}

	bad := make([]byte, FrameSize)
	copy(bad, buf)
	bad[18] ^= 0xFF
	if _, ok := ParseFrame(bad); ok {
		t.Fatal("bad checksum accepted")
	}

	bad2 := make([]byte, FrameSize)
	copy(bad2, buf)
	bad2[0] = 0x00
	if _, ok := ParseFrame(bad2); ok {
		t.Fatal("bad sync accepted")
	}
}

func TestFindTriggerIndex(t *testing.T) {
	y := []float64{-1, -1, 1, 1, -1, -1, 1, 1, 1, 1}

	// Rising crossings of 0 are at index 2 and 6. With minPost=3 the search
	// window ends at 7, so the most recent qualifying crossing is 6.
	if got := findTriggerIndex(y, "Rising", 0, 3); got != 6 {
		t.Fatalf("rising: got %d, want 6", got)
	}
	if got := findTriggerIndex(y, "Falling", 0, 3); got != 4 {
		t.Fatalf("falling: got %d, want 4", got)
	}
	// Not enough post-trigger samples for any crossing.
	if got := findTriggerIndex(y, "Rising", 0, 10); got != -1 {
		t.Fatalf("insufficient post-samples: got %d, want -1", got)
	}
}

func TestFFTFindsTone(t *testing.T) {
	// n = 1000 at 1kHz puts bin spacing at exactly 1Hz, so the 50Hz tone lands
	// dead on bin 50. Off-bin tones lose a few percent to scalloping, which is
	// inherent to the DFT (numpy behaves identically) -- not worth asserting on.
	const n = 1000
	data := make([]float64, n)
	for i := range data {
		data[i] = 2.0 * math.Sin(2*math.Pi*50.0*float64(i)/StreamRateHz)
	}

	res, ok := computeFFT(data, false, false)
	if !ok {
		t.Fatal("computeFFT rejected valid data")
	}

	peak := 0
	for i, v := range res.Values {
		if v > res.Values[peak] {
			peak = i
		}
	}
	if math.Abs(res.Freqs[peak]-50.0) > 2.0 {
		t.Fatalf("peak at %.2f Hz, want ~50 Hz", res.Freqs[peak])
	}
	// Calibrated peak amplitude should recover the 2.0 amplitude.
	if math.Abs(res.Values[peak]-2.0) > 0.05 {
		t.Fatalf("peak amplitude %.4f, want ~2.0", res.Values[peak])
	}
}

func TestRingWrap(t *testing.T) {
	r := newRing(4)
	for i := 1; i <= 6; i++ {
		r.push(float64(i))
	}
	got := r.slice()
	want := []float64{3, 4, 5, 6}
	if len(got) != len(want) {
		t.Fatalf("len %d, want %d", len(got), len(want))
	}
	for i := range want {
		if got[i] != want[i] {
			t.Fatalf("slice %v, want %v", got, want)
		}
	}
	last := r.lastN(2)
	if len(last) != 2 || last[0] != 5 || last[1] != 6 {
		t.Fatalf("lastN %v, want [5 6]", last)
	}
}

// TestRenderPlotImage writes sample renders so the plot output can be eyeballed.
func TestRenderPlotImage(t *testing.T) {
	out := os.Getenv("PLOT_OUT")
	if out == "" {
		t.Skip("set PLOT_OUT to write sample renders")
	}

	// Dense case: 30,000 samples over 30s, exercising min/max downsampling.
	const n = 30000
	xs := make([]float64, n)
	ys := make([]float64, n)
	rng := rand.New(rand.NewSource(1))
	for i := range xs {
		tt := float64(i) / StreamRateHz
		xs[i] = tt
		ys[i] = 12.0*math.Sin(2*math.Pi*50*tt) +
			1.5*math.Sin(2*math.Pi*150*tt) +
			0.4*rng.NormFloat64()
	}

	img := renderPlot(1400, 420, 2.0, plotData{
		xs: xs, ys: ys,
		yLabel: "Voltage (V)", xLabel: "Time (s)",
		curve:      colAccent,
		hasTrigger: true, triggerX: 15.0,
	})
	writePNG(t, out+"/plot_dense.png", img)

	// Sparse case: 40 samples, exercising the polyline path.
	sxs := make([]float64, 40)
	sys := make([]float64, 40)
	for i := range sxs {
		sxs[i] = float64(i) / StreamRateHz
		sys[i] = math.Exp(-float64(i)/12.0) * math.Cos(float64(i)/2.0)
	}
	img2 := renderPlot(900, 300, 2.0, plotData{
		xs: sxs, ys: sys,
		yLabel: "HCA Error (p.u.)", xLabel: "Time (s)",
		curve: colAccent,
	})
	writePNG(t, out+"/plot_sparse.png", img2)

	// Empty case.
	img3 := renderPlot(900, 300, 2.0, plotData{yLabel: "Voltage (V)", xLabel: "Time (s)"})
	writePNG(t, out+"/plot_empty.png", img3)

	// Spectrum case.
	data := make([]float64, FFTWindowSamples)
	for i := range data {
		tt := float64(i) / StreamRateHz
		data[i] = 12*math.Sin(2*math.Pi*50*tt) + 3*math.Sin(2*math.Pi*150*tt)
	}
	res, _ := computeFFT(data, false, true)
	img4 := renderPlot(1000, 340, 2.0, plotData{
		xs: res.Freqs, ys: res.Values,
		yLabel: res.YLabel, xLabel: "Frequency (Hz)",
		curve: colAccent,
	})
	writePNG(t, out+"/plot_fft.png", img4)
}

func writePNG(t *testing.T, path string, img image.Image) {
	t.Helper()

	f, err := os.Create(path)
	if err != nil {
		t.Fatalf("create %s: %v", path, err)
	}
	defer f.Close()

	if err := png.Encode(f, img); err != nil {
		t.Fatalf("encode %s: %v", path, err)
	}
}
