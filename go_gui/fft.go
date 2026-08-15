package main

import (
	"math"
	"math/cmplx"
	"sync"

	"gonum.org/v1/gonum/dsp/fourier"
)

const (
	FFTWindowSamples  = 2048 // ~2s of history at 1kHz
	FFTUpdateInterval = 250  // ms
	dbFloor           = 1e-12
	minFFTSamples     = 16
)

// FFT plans are relatively expensive to build and the sample count is stable
// once the buffer has filled, so keep one per length around.
var (
	planMu sync.Mutex
	plans  = map[int]*fourier.FFT{}
)

func fftPlan(n int) *fourier.FFT {
	planMu.Lock()
	defer planMu.Unlock()

	if p, ok := plans[n]; ok {
		return p
	}
	p := fourier.NewFFT(n)
	plans[n] = p
	return p
}

// FFTResult is one computed spectrum, ready to plot.
type FFTResult struct {
	Freqs  []float64
	Values []float64
	YLabel string
}

// computeFFT returns the live amplitude spectrum of data.
//
// Amplitude is calibrated peak amplitude by default: the Hann window's
// coherent gain (its mean, not n) and the one-sided-spectrum factor of 2
// (every AC bin's energy is split between the discarded negative-frequency
// bin and this one) are both corrected for, unlike a naive |FFT|/N. useRMS
// divides by sqrt(2) (skipping the DC bin, which has no such factor); useDB
// applies 20*log10 on top of whichever of those is selected.
func computeFFT(data []float64, useRMS, useDB bool) (FFTResult, bool) {
	n := len(data)
	if n < minFFTSamples {
		return FFTResult{}, false
	}

	// Symmetric Hann window, matching numpy's np.hanning(n).
	windowed := make([]float64, n)
	windowSum := 0.0
	for i := 0; i < n; i++ {
		w := 0.5 - 0.5*math.Cos(2*math.Pi*float64(i)/float64(n-1))
		windowed[i] = data[i] * w
		windowSum += w
	}
	if windowSum == 0 {
		return FFTResult{}, false
	}

	coeff := fftPlan(n).Coefficients(nil, windowed)

	freqs := make([]float64, len(coeff))
	values := make([]float64, len(coeff))
	for i, c := range coeff {
		freqs[i] = float64(i) * StreamRateHz / float64(n)

		// Restore the factor of 2 that a one-sided spectrum drops for every
		// bin except DC (and Nyquist, for even n).
		scale := 2.0
		if i == 0 || (n%2 == 0 && i == len(coeff)-1) {
			scale = 1.0
		}
		amp := cmplx.Abs(c) * scale / windowSum

		if useRMS && i > 0 { // DC has no sqrt(2) factor -- it doesn't oscillate
			amp /= math.Sqrt2
		}
		values[i] = amp
	}

	label := "Amplitude (peak)"
	if useRMS {
		label = "Amplitude (RMS)"
	}
	if useDB {
		for i, v := range values {
			values[i] = 20.0 * math.Log10(math.Max(v, dbFloor))
		}
		if useRMS {
			label = "Magnitude (dB, RMS ref)"
		} else {
			label = "Magnitude (dB, peak ref)"
		}
	}

	return FFTResult{Freqs: freqs, Values: values, YLabel: label}, true
}
