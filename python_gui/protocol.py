"""Wire protocol shared with the firmware (Core/Inc/main.h, AdcStreamFrame_t).

Keep this file in sync with the C struct by hand -- there is no code
generation step. If you change the frame layout in firmware, update
FRAME_FORMAT here to match.
"""

import struct
from typing import NamedTuple, Optional

SYNC0 = 0xA5
SYNC1 = 0x5A

# Samples carried by one wire frame. Must match STREAM_BATCH_SAMPLES in main.h.
# Batching amortises the 11 bytes of framing over 8 samples (9.4 B/sample rather
# than 19), which is what keeps the 5kHz stream inside what the ST-LINK VCP will
# actually carry -- see the AdcStreamFrame_t comment in Core/Inc/main.h.
STREAM_BATCH_SAMPLES = 8

# '<' little-endian (Cortex-M4 and x86/x64 both are), matches:
#   uint8_t sync0, sync1; uint32_t seq, timestamp_ms;
#   struct { float voltage, error; } samples[STREAM_BATCH_SAMPLES]; uint8_t checksum;
FRAME_FORMAT = "<BBII" + "ff" * STREAM_BATCH_SAMPLES + "B"
FRAME_SIZE = struct.calcsize(FRAME_FORMAT)  # 75

# Single-byte host -> device commands (see Core/Src/main.c, HandleStreamCommand)
CMD_START = b"S"
CMD_STOP = b"X"
CMD_PING = b"P"
CMD_GET_COEFFS = b"G"   # ask the device to report every channel's Kp/Ki
CMD_RESET_INTEGRATORS = b"R"  # zero every channel's integrator + the disperser window
CMD_SET_COEFF = b"C"    # followed by COEFF_CMD_PAYLOAD, see build_set_coeff_command
PING_REPLY_PREFIX = b"HCA1001_ADC_STREAM_V1"

# --- HCA coefficient frames (device -> host), see HcaCoeffFrame_t in main.h ---
# Same first sync byte as a stream frame but a different second one, so the
# reader resyncs on SYNC0 and picks the frame length from the byte after it.
COEFF_SYNC1 = 0x5B
COEFF_FRAME_FORMAT = "<BBBBBffffB"  # sync0 sync1 index count order kp_re kp_im ki_re ki_im chk
COEFF_FRAME_SIZE = struct.calcsize(COEFF_FRAME_FORMAT)  # 22

# Payload following the CMD_SET_COEFF byte (host -> device): order, the four
# gain components, then an 8-bit additive checksum over those 17 bytes.
COEFF_CMD_FORMAT = "<BffffB"
COEFF_CMD_SIZE = struct.calcsize(COEFF_CMD_FORMAT)  # 18

# Highest harmonic order the GUI offers. MAX_HARMONICS in Core/Inc/hca_lib.h
# must be at least this large or the device runs out of channel slots.
MAX_HARMONIC_ORDER = 50

# Must match ADC_STREAM_DECIMATION in Core/Src/main.c (40kHz / 8 = 5kHz).
# Nyquist is therefore 2.5kHz, which is what bounds the FFT view's frequency axis.
STREAM_RATE_HZ = 5000.0

SIGNAL_NAMES = ("voltage", "error")
SIGNAL_LABELS = {
    "voltage": "Voltage (V)",
    "error": "HCA Error (p.u.)",
}


class CoeffFrame(NamedTuple):
    """One channel's gains as reported by the device."""
    index: int    # channel slot on the device
    count: int    # how many channels are active, so the host knows when it has them all
    order: int
    kp_real: float
    kp_imag: float
    ki_real: float
    ki_imag: float


class AdcFrame(NamedTuple):
    """One sample. Wire frames carry STREAM_BATCH_SAMPLES of these; parse_frame
    expands a frame back into per-sample records so everything downstream stays
    sample-oriented."""
    seq: int            # per-sample index: batch seq * STREAM_BATCH_SAMPLES + position
    timestamp_ms: float # device clock, interpolated within the batch
    voltage: float
    error: float


def checksum8(payload: bytes) -> int:
    return sum(payload) & 0xFF


def parse_frame(buf: bytes):
    """Parse and validate one FRAME_SIZE-byte buffer, returning its
    STREAM_BATCH_SAMPLES samples as a list of AdcFrame. None if invalid.

    The device stamps only the first sample of a batch, so the rest are spaced
    at the nominal sample interval. That keeps per-sample time resolution finer
    than HAL_GetTick's 1ms, and the batch's own timestamp still anchors the
    stream-rate measurement in the GUI.
    """
    if len(buf) != FRAME_SIZE:
        return None
    fields = struct.unpack(FRAME_FORMAT, buf)
    sync0, sync1, seq, ts = fields[:4]
    chk = fields[-1]
    if sync0 != SYNC0 or sync1 != SYNC1:
        return None
    payload = buf[2:-1]  # seq .. samples, matches StreamChecksum() in firmware
    if checksum8(payload) != chk:
        return None

    sample_interval_ms = 1000.0 / STREAM_RATE_HZ
    base = seq * STREAM_BATCH_SAMPLES
    pairs = fields[4:-1]
    return [
        AdcFrame(base + i, ts + i * sample_interval_ms, pairs[2 * i], pairs[2 * i + 1])
        for i in range(STREAM_BATCH_SAMPLES)
    ]


def parse_coeff_frame(buf: bytes) -> Optional[CoeffFrame]:
    """Parse and validate one COEFF_FRAME_SIZE-byte buffer. Returns None if invalid."""
    if len(buf) != COEFF_FRAME_SIZE:
        return None
    (sync0, sync1, index, count, order,
     kp_re, kp_im, ki_re, ki_im, chk) = struct.unpack(COEFF_FRAME_FORMAT, buf)
    if sync0 != SYNC0 or sync1 != COEFF_SYNC1:
        return None
    payload = buf[2:-1]  # index .. ki_imag, matches CoeffChecksum() in firmware
    if checksum8(payload) != chk:
        return None
    return CoeffFrame(index, count, order, kp_re, kp_im, ki_re, ki_im)


def build_set_coeff_command(order: int, kp: complex, ki: complex) -> bytes:
    """CMD_SET_COEFF plus its payload, ready to write to the port."""
    body = struct.pack(
        "<Bffff", order, kp.real, kp.imag, ki.real, ki.imag
    )
    return CMD_SET_COEFF + body + bytes([checksum8(body)])
