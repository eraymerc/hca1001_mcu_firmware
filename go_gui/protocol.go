// Wire protocol shared with the firmware (Core/Inc/main.h, AdcStreamFrame_t).
//
// Keep this file in sync with the C struct by hand -- there is no code
// generation step. If you change the frame layout in firmware, update
// FrameSize and ParseFrame here to match.
package main

import (
	"encoding/binary"
	"math"
)

const (
	Sync0 = 0xA5
	Sync1 = 0x5A

	// Little-endian (Cortex-M4 and x86/x64 both are), matches:
	//   uint8_t sync0, sync1; uint32_t seq, timestamp_ms; float voltage, error; uint8_t checksum;
	FrameSize = 1 + 1 + 4 + 4 + 4 + 4 + 1 // 19

	// Must match ADC_STREAM_DECIMATION in Core/Src/main.c (40kHz / 40 = 1kHz)
	StreamRateHz = 1000.0
)

// Single-byte host -> device commands (see Core/Src/main.c, HandleStreamCommand)
var (
	CmdStart        = []byte("S")
	CmdStop         = []byte("X")
	CmdPing         = []byte("P")
	PingReplyPrefix = []byte("HCA1001_ADC_STREAM_V1")
)

var SignalNames = []string{"voltage", "error"}

var SignalLabels = map[string]string{
	"voltage": "Voltage (V)",
	"error":   "HCA Error (p.u.)",
}

// AdcFrame is one decoded telemetry sample.
type AdcFrame struct {
	Seq         uint32  `json:"seq"`
	TimestampMs uint32  `json:"timestampMs"`
	Voltage     float64 `json:"voltage"`
	Error       float64 `json:"error"`
}

func checksum8(payload []byte) byte {
	var sum byte
	for _, b := range payload {
		sum += b
	}
	return sum
}

// ParseFrame parses and validates one FrameSize-byte buffer.
// The second return value is false if the buffer is not a valid frame.
func ParseFrame(buf []byte) (AdcFrame, bool) {
	if len(buf) != FrameSize {
		return AdcFrame{}, false
	}
	if buf[0] != Sync0 || buf[1] != Sync1 {
		return AdcFrame{}, false
	}
	// payload = seq .. error, matches StreamChecksum() in firmware
	if checksum8(buf[2:FrameSize-1]) != buf[FrameSize-1] {
		return AdcFrame{}, false
	}
	return AdcFrame{
		Seq:         binary.LittleEndian.Uint32(buf[2:6]),
		TimestampMs: binary.LittleEndian.Uint32(buf[6:10]),
		Voltage:     float64(math.Float32frombits(binary.LittleEndian.Uint32(buf[10:14]))),
		Error:       float64(math.Float32frombits(binary.LittleEndian.Uint32(buf[14:18]))),
	}, true
}
