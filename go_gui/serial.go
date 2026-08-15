// Background goroutine that owns the serial port exclusively: it is the only
// goroutine that ever calls Read()/Write() on it. Connects, resyncs on the
// 0xA5 0x5A frame marker, validates checksums, and hands parsed frames to the
// app through a buffered channel.
//
// Commands from the UI (start/stop/ping) are handed over through a channel
// instead of writing to the port directly, so a Write() that ever blocks
// (USB hiccup, full OS buffer) can never stall a UI-originated call. The read
// loop drains that channel between reads, so all serial I/O stays on this one
// goroutine -- the same design as the Python serial_worker.SerialWorker.
//
// Frames go into a large buffered channel rather than being pushed at the UI.
// The frontend pulls on its own timer (see GetPlot), so redraw cost is
// decoupled from arrival rate: a slow render just means fewer, larger pulls,
// never a growing backlog.
package main

import (
	"bytes"
	"fmt"
	"sync/atomic"
	"time"

	"go.bug.st/serial"
	"go.bug.st/serial/enumerator"
)

const (
	Baudrate  = 2097000 // must match hlpuart1.Init.BaudRate in Core/Src/main.c
	readChunk = 4096
	readWait  = 50 * time.Millisecond

	// ~65s of 1kHz frames of slack before the reader starts dropping.
	frameChanDepth = 65536
)

// PortInfo is one entry of the COM port dropdown.
type PortInfo struct {
	Device      string `json:"device"`
	Description string `json:"description"`
}

// ListSerialPorts mirrors pyserial's serial.tools.list_ports.comports().
func ListSerialPorts() []PortInfo {
	out := []PortInfo{}

	detailed, err := enumerator.GetDetailedPortsList()
	if err == nil && len(detailed) > 0 {
		for _, p := range detailed {
			desc := p.Product
			if desc == "" {
				desc = "n/a"
			}
			out = append(out, PortInfo{Device: p.Name, Description: desc})
		}
		return out
	}

	names, err := serial.GetPortsList()
	if err != nil {
		return out
	}
	for _, n := range names {
		out = append(out, PortInfo{Device: n, Description: "n/a"})
	}
	return out
}

// SerialWorker owns one open port for its lifetime.
type SerialWorker struct {
	portName string
	frames   chan AdcFrame

	cmds      chan []byte
	stop      chan struct{}
	stopped   chan struct{}
	streaming atomic.Bool

	onConnected    func(port string)
	onDisconnected func()
	onError        func(msg string)
	onPingOK       func()
}

func NewSerialWorker(portName string) *SerialWorker {
	return &SerialWorker{
		portName: portName,
		frames:   make(chan AdcFrame, frameChanDepth),
		cmds:     make(chan []byte, 16),
		stop:     make(chan struct{}),
		stopped:  make(chan struct{}),
	}
}

func (w *SerialWorker) PortName() string { return w.portName }

func (w *SerialWorker) Frames() <-chan AdcFrame { return w.frames }

// --- control API, safe to call from any goroutine ---
// These only enqueue; the actual Write() happens inside run().

func (w *SerialWorker) StartStreaming() {
	w.streaming.Store(true)
	w.enqueue(CmdStart)
}

func (w *SerialWorker) StopStreaming() {
	w.streaming.Store(false)
	w.enqueue(CmdStop)
}

func (w *SerialWorker) Ping() { w.enqueue(CmdPing) }

func (w *SerialWorker) enqueue(cmd []byte) {
	select {
	case w.cmds <- cmd:
	case <-w.stop:
	default: // command backlog: the port is wedged, dropping is better than blocking
	}
}

// Stop asks the read loop to exit and waits up to 2s for it to finish.
func (w *SerialWorker) Stop() {
	select {
	case <-w.stop:
	default:
		close(w.stop)
	}
	select {
	case <-w.stopped:
	case <-time.After(2 * time.Second):
	}
}

// Start opens the port and launches the read loop. A non-nil error means the
// port never opened and no callbacks will fire.
func (w *SerialWorker) Start() error {
	mode := &serial.Mode{
		BaudRate: Baudrate,
		DataBits: 8,
		Parity:   serial.NoParity,
		StopBits: serial.OneStopBit,
	}
	port, err := serial.Open(w.portName, mode)
	if err != nil {
		return fmt.Errorf("could not open %s: %w", w.portName, err)
	}
	if err := port.SetReadTimeout(readWait); err != nil {
		port.Close()
		return fmt.Errorf("could not configure %s: %w", w.portName, err)
	}

	go w.run(port)
	return nil
}

func (w *SerialWorker) drainCommands(port serial.Port) {
	for {
		select {
		case cmd := <-w.cmds:
			if _, err := port.Write(cmd); err != nil && w.onError != nil {
				w.onError(err.Error())
			}
		default:
			return
		}
	}
}

func (w *SerialWorker) run(port serial.Port) {
	// Closing frames lets the consumer's range loop finish on disconnect.
	// Every send happens on this goroutine, before these deferred closes.
	defer close(w.stopped)
	defer close(w.frames)

	if w.onConnected != nil {
		w.onConnected(w.portName)
	}

	buf := make([]byte, 0, readChunk*2)
	chunk := make([]byte, readChunk)
	syncPair := []byte{Sync0, Sync1}

loop:
	for {
		select {
		case <-w.stop:
			break loop
		default:
		}

		w.drainCommands(port)

		n, err := port.Read(chunk)
		if err != nil {
			if w.onError != nil {
				w.onError(fmt.Sprintf("Serial read error: %v", err))
			}
			break
		}
		if n == 0 {
			continue
		}
		buf = append(buf, chunk[:n]...)

		// Handle the ASCII ping reply separately (not a fixed-size binary frame)
		if bytes.HasPrefix(buf, PingReplyPrefix) {
			if nl := bytes.IndexByte(buf, '\n'); nl != -1 {
				if w.onPingOK != nil {
					w.onPingOK()
				}
				buf = append(buf[:0], buf[nl+1:]...)
			}
		}

		// Resync on the two sync bytes, then parse fixed-size frames
		for {
			idx := bytes.Index(buf, syncPair)
			if idx == -1 {
				if len(buf) > FrameSize*4 {
					// keep tail, avoid unbounded growth
					buf = append(buf[:0], buf[len(buf)-FrameSize:]...)
				}
				break
			}
			if idx > 0 {
				buf = append(buf[:0], buf[idx:]...)
			}
			if len(buf) < FrameSize {
				break
			}

			frame, ok := ParseFrame(buf[:FrameSize])
			if !ok {
				buf = append(buf[:0], buf[1:]...) // bad checksum/sync collision, slide forward one byte
				continue
			}
			buf = append(buf[:0], buf[FrameSize:]...)

			select {
			case w.frames <- frame:
			default: // consumer far behind; drop oldest-arriving sample rather than stall the reader
			}
		}
	}

	if w.streaming.Load() {
		port.Write(CmdStop)
	}
	port.Close()

	if w.onDisconnected != nil {
		w.onDisconnected()
	}
}
