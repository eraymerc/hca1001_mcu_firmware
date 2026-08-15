package main

import "sync"

const (
	DefaultLiveWindowSeconds = 5.0
	MinLiveWindowSeconds     = 0.005
	MaxLiveWindowSeconds     = 30.0

	// Capture buffer is sized for the largest window any row could ask for;
	// each row's window field just controls how much of that trailing history
	// it displays, so changing a field never needs to resize a ring.
	MaxLiveSamples = int(MaxLiveWindowSeconds * StreamRateHz)

	// ~33 min at 1kHz, bounds RAM use for CSV export.
	MaxSessionSamples = 2_000_000
)

// Record is one full-session row kept for CSV export.
type Record struct {
	Seq         uint32
	TimestampMs uint32
	Voltage     float64
	Error       float64
}

// Snapshot is a consistent view of the live buffers, taken under the lock so
// the UI never reads a half-updated set of signals.
type Snapshot struct {
	T          []float64
	Signals    map[string][]float64
	SessionLen int
	Capped     bool
}

// Monitor owns every captured sample. The serial consumer goroutine writes
// through Add; the UI goroutine reads through Snapshot / LiveBuffer.
type Monitor struct {
	mu       sync.Mutex
	t        *ring
	signals  map[string]*ring
	session  []Record
	capped   bool
	t0Ms     uint32
	haveT0   bool
	justCapd bool
}

func NewMonitor() *Monitor {
	m := &Monitor{
		t:       newRing(MaxLiveSamples),
		signals: make(map[string]*ring, len(SignalNames)),
	}
	for _, name := range SignalNames {
		m.signals[name] = newRing(MaxLiveSamples)
	}
	return m
}

// Add records one frame. It reports true exactly once, on the frame that
// fills the session buffer, so the caller can surface that to the user.
func (m *Monitor) Add(f AdcFrame) (justCapped bool) {
	m.mu.Lock()
	defer m.mu.Unlock()

	if !m.haveT0 {
		m.t0Ms = f.TimestampMs
		m.haveT0 = true
	}

	// Wraps correctly across the uint32 HAL_GetTick() rollover (~49 days).
	m.t.push(float64(f.TimestampMs-m.t0Ms) / 1000.0)
	m.signals["voltage"].push(f.Voltage)
	m.signals["error"].push(f.Error)

	if !m.capped {
		m.session = append(m.session, Record{f.Seq, f.TimestampMs, f.Voltage, f.Error})
		if len(m.session) >= MaxSessionSamples {
			m.capped = true
			return true
		}
	}
	return false
}

func (m *Monitor) Snapshot() Snapshot {
	m.mu.Lock()
	defer m.mu.Unlock()

	sigs := make(map[string][]float64, len(m.signals))
	for name, r := range m.signals {
		sigs[name] = r.slice()
	}
	return Snapshot{
		T:          m.t.slice(),
		Signals:    sigs,
		SessionLen: len(m.session),
		Capped:     m.capped,
	}
}

// LiveBuffer returns the most recent n samples of one signal, for the FFT view.
func (m *Monitor) LiveBuffer(name string, n int) []float64 {
	m.mu.Lock()
	defer m.mu.Unlock()

	r, ok := m.signals[name]
	if !ok {
		return nil
	}
	return r.lastN(n)
}

// SessionSnapshot copies the session record so the CSV writer never iterates
// a slice that the serial consumer is concurrently appending to.
func (m *Monitor) SessionSnapshot() []Record {
	m.mu.Lock()
	defer m.mu.Unlock()

	out := make([]Record, len(m.session))
	copy(out, m.session)
	return out
}

// findTriggerIndex searches backward for the most recent edge crossing that
// still leaves at least minPost samples after it, so every row -- each of
// which may have a different window length -- can be sliced from the exact
// same point in time and stay aligned with one another. Returns -1 if no
// qualifying crossing exists yet (just connected, or the signal never
// crosses level).
func findTriggerIndex(y []float64, edge string, level float64, minPost int) int {
	searchEnd := len(y) - minPost
	if searchEnd < 1 {
		return -1
	}
	if edge == "Rising" {
		for i := searchEnd - 1; i > 0; i-- {
			if y[i-1] < level && level <= y[i] {
				return i
			}
		}
		return -1
	}
	for i := searchEnd - 1; i > 0; i-- {
		if y[i-1] > level && level >= y[i] {
			return i
		}
	}
	return -1
}

func meanPkPk(y []float64) (mean, pkpk float64) {
	if len(y) == 0 {
		return 0, 0
	}
	lo, hi, sum := y[0], y[0], 0.0
	for _, v := range y {
		sum += v
		if v < lo {
			lo = v
		}
		if v > hi {
			hi = v
		}
	}
	return sum / float64(len(y)), hi - lo
}
