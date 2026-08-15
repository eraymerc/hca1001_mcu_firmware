package main

import (
	"encoding/binary"
	"math"
	"os"
	"os/exec"
	"path/filepath"
	"testing"
	"time"
)

// buildFrame encodes one AdcStreamFrame_t exactly as the firmware does.
func buildFrame(seq, ts uint32, voltage, errSig float32) []byte {
	buf := make([]byte, FrameSize)
	buf[0], buf[1] = Sync0, Sync1
	binary.LittleEndian.PutUint32(buf[2:6], seq)
	binary.LittleEndian.PutUint32(buf[6:10], ts)
	binary.LittleEndian.PutUint32(buf[10:14], math.Float32bits(voltage))
	binary.LittleEndian.PutUint32(buf[14:18], math.Float32bits(errSig))
	buf[18] = checksum8(buf[2:18])
	return buf
}

// TestSerialWorkerAgainstPTY runs the real SerialWorker against a socat pty
// pair standing in for the Nucleo: it checks the ping reply, frame parsing,
// resync after corruption, and that UI commands reach the device.
func TestSerialWorkerAgainstPTY(t *testing.T) {
	if _, err := exec.LookPath("socat"); err != nil {
		t.Skip("socat not available")
	}

	dir := t.TempDir()
	hostLink := filepath.Join(dir, "host")
	devLink := filepath.Join(dir, "dev")

	cmd := exec.Command("socat",
		"PTY,raw,echo=0,link="+hostLink,
		"PTY,raw,echo=0,link="+devLink)
	if err := cmd.Start(); err != nil {
		t.Fatalf("start socat: %v", err)
	}
	defer func() {
		cmd.Process.Kill()
		cmd.Wait()
	}()

	// Wait for socat to create both symlinks.
	deadline := time.Now().Add(5 * time.Second)
	for {
		_, e1 := os.Stat(hostLink)
		_, e2 := os.Stat(devLink)
		if e1 == nil && e2 == nil {
			break
		}
		if time.Now().After(deadline) {
			t.Skip("socat did not create pty links")
		}
		time.Sleep(20 * time.Millisecond)
	}

	// The "firmware" end.
	dev, err := os.OpenFile(devLink, os.O_RDWR, 0)
	if err != nil {
		t.Fatalf("open device end: %v", err)
	}
	defer dev.Close()

	w := NewSerialWorker(hostLink)

	pinged := make(chan struct{}, 1)
	connected := make(chan struct{}, 1)
	w.onPingOK = func() {
		select {
		case pinged <- struct{}{}:
		default:
		}
	}
	w.onConnected = func(string) {
		select {
		case connected <- struct{}{}:
		default:
		}
	}
	w.onError = func(msg string) { t.Logf("worker error: %s", msg) }

	if err := w.Start(); err != nil {
		t.Skipf("cannot open pty as serial port: %v", err)
	}
	defer w.Stop()

	select {
	case <-connected:
	case <-time.After(2 * time.Second):
		t.Fatal("onConnected never fired")
	}

	// 1. Ping reply, exactly as HandleStreamCommand sends it.
	if _, err := dev.Write([]byte(string(PingReplyPrefix) + "\n")); err != nil {
		t.Fatalf("write ping reply: %v", err)
	}
	select {
	case <-pinged:
	case <-time.After(2 * time.Second):
		t.Fatal("ping reply not recognised")
	}

	// 2. Garbage, then valid frames: the worker must resync on 0xA5 0x5A and
	//    lose only the corrupted bytes.
	var payload []byte
	payload = append(payload, 0x11, 0x22, 0x33, 0xA5, 0x00, 0x7F)
	const nFrames = 200
	for i := 0; i < nFrames; i++ {
		payload = append(payload, buildFrame(uint32(i), uint32(i), float32(i)*0.25, float32(i)*-0.5)...)
	}
	if _, err := dev.Write(payload); err != nil {
		t.Fatalf("write frames: %v", err)
	}

	got := make([]AdcFrame, 0, nFrames)
	timeout := time.After(5 * time.Second)
	for len(got) < nFrames {
		select {
		case f := <-w.Frames():
			got = append(got, f)
		case <-timeout:
			t.Fatalf("only received %d of %d frames", len(got), nFrames)
		}
	}

	for i, f := range got {
		if f.Seq != uint32(i) {
			t.Fatalf("frame %d: seq %d", i, f.Seq)
		}
		if f.Voltage != float64(float32(i)*0.25) || f.Error != float64(float32(i)*-0.5) {
			t.Fatalf("frame %d: payload mismatch %+v", i, f)
		}
	}

	// 3. Commands from the UI must reach the device end.
	w.StartStreaming()
	dev.SetReadDeadline(time.Now().Add(2 * time.Second))
	cmdBuf := make([]byte, 1)
	if _, err := dev.Read(cmdBuf); err != nil {
		t.Fatalf("read command: %v", err)
	}
	if cmdBuf[0] != CmdStart[0] {
		t.Fatalf("got command %q, want %q", cmdBuf[0], CmdStart[0])
	}
}
