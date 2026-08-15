package main

// ring is a fixed-capacity circular buffer of float64 samples. Pushing past
// capacity overwrites the oldest sample, which is what a live scope window
// wants: the buffer is sized for the largest window any row could ask for,
// and each row just slices the trailing part of it that it displays.
type ring struct {
	buf  []float64
	head int // index of the next write
	size int // number of valid samples, <= len(buf)
}

func newRing(capacity int) *ring {
	if capacity < 1 {
		capacity = 1
	}
	return &ring{buf: make([]float64, capacity)}
}

func (r *ring) push(v float64) {
	r.buf[r.head] = v
	r.head = (r.head + 1) % len(r.buf)
	if r.size < len(r.buf) {
		r.size++
	}
}

func (r *ring) len() int { return r.size }

// slice returns every buffered sample in chronological order.
func (r *ring) slice() []float64 {
	out := make([]float64, r.size)
	start := (r.head - r.size + len(r.buf)) % len(r.buf)
	for i := 0; i < r.size; i++ {
		out[i] = r.buf[(start+i)%len(r.buf)]
	}
	return out
}

// lastN returns the most recent n samples in chronological order, or every
// buffered sample if fewer than n are available.
func (r *ring) lastN(n int) []float64 {
	if n > r.size {
		n = r.size
	}
	if n < 0 {
		n = 0
	}
	out := make([]float64, n)
	start := (r.head - n + len(r.buf)) % len(r.buf)
	for i := 0; i < n; i++ {
		out[i] = r.buf[(start+i)%len(r.buf)]
	}
	return out
}
