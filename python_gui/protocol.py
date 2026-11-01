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
# Batching amortises the 11 bytes of framing over 8 samples (7.4 B/sample), see
# the AdcStreamFrame_t comment in Core/Inc/main.h.
STREAM_BATCH_SAMPLES = 8

# '<' little-endian (Cortex-M4 and x86/x64 both are), matches:
#   uint8_t sync0, sync1; uint32_t seq, timestamp_ms;
#   struct { int16_t voltage, reference, error; } samples[STREAM_BATCH_SAMPLES];
#   uint8_t checksum;
FRAME_FORMAT = "<BBII" + "hhh" * STREAM_BATCH_SAMPLES + "B"
FRAME_SIZE = struct.calcsize(FRAME_FORMAT)  # 59

# Fixed-point scales of the int16 sample fields. Must match STREAM_VOLT_SCALE and
# STREAM_PU_SCALE in Core/Src/main.c.
STREAM_VOLT_SCALE = 16.0    # voltage counts per volt
STREAM_PU_SCALE = 2048.0    # reference/error counts per p.u.

# Volts per p.u. of the reference: the firmware forms error = r - v/V_PEAK_NOM.
# Must match V_PEAK_NOM in Core/Src/main.c.
V_PEAK_NOM = 200.0

# Single-byte host -> device commands (see Core/Src/main.c, HandleStreamCommand)
CMD_START = b"S"
CMD_STOP = b"X"
CMD_PING = b"P"
CMD_GET_COEFFS = b"G"   # ask the device to report every channel's Kp/Ki
CMD_RESET_INTEGRATORS = b"R"  # zero every channel's integrator + the disperser window
CMD_SET_COEFF = b"C"    # followed by COEFF_CMD_PAYLOAD, see build_set_coeff_command
CMD_SET_REF = b"M"      # followed by REF_CMD_PAYLOAD, see build_set_ref_command
CMD_GET_REF = b"N"      # ask the device to report the reference multiplier in force
CMD_CALIBRATE = b"K"    # followed by CAL_CMD_PAYLOAD, see build_calibrate_command
CMD_GET_CAL = b"Q"      # ask the device to report the sensor calibration in force
CMD_CAL_DEFAULT = b"D"  # restore the build-time sensor calibration
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

# --- reference multiplier frames (device -> host), see HcaRefFrame_t in main.h ---
# A third second-sync-byte variant on the same link.
REF_SYNC1 = 0x5C
REF_FRAME_FORMAT = "<BBfBfB"  # sync0 sync1 value open_loop limit chk
REF_FRAME_SIZE = struct.calcsize(REF_FRAME_FORMAT)  # 12

# Payload following the CMD_SET_REF byte: the multiplier, then an 8-bit
# additive checksum over those 4 bytes.
REF_CMD_FORMAT = "<fB"
REF_CMD_SIZE = struct.calcsize(REF_CMD_FORMAT)  # 5

# --- sensor calibration frames (device -> host), see HcaCalFrame_t in main.h ---
CAL_SYNC1 = 0x5D
CAL_FRAME_FORMAT = "<BBBffffffB"  # sync0 sync1 status vdc gain offset raw_peak raw_dc exp_peak chk
CAL_FRAME_SIZE = struct.calcsize(CAL_FRAME_FORMAT)  # 28

# Payload following the CMD_CALIBRATE byte: the DC bus voltage, then an 8-bit
# additive checksum over those 4 bytes.
CAL_CMD_FORMAT = "<fB"
CAL_CMD_SIZE = struct.calcsize(CAL_CMD_FORMAT)  # 5

# HcaCalFrame_t.status -- must match the CAL_STATUS_* defines in main.h.
CAL_STATUS_OK = 0
CAL_STATUS_REPORT = 1
CAL_STATUS_RESTORED = 2
CAL_STATUS_BUSY = 3
CAL_STATUS_BAD_VDC = 4
CAL_STATUS_NO_SIGNAL = 5
CAL_STATUS_OUT_OF_RANGE = 6

CAL_STATUS_TEXT = {
    CAL_STATUS_OK: "Calibration applied.",
    CAL_STATUS_REPORT: "Calibration currently in force.",
    CAL_STATUS_RESTORED: "Build-time calibration restored.",
    CAL_STATUS_BUSY: "A calibration is already running -- wait for it to finish.",
    CAL_STATUS_BAD_VDC: "Vdc outside the accepted 10-1000 V range; nothing changed.",
    CAL_STATUS_NO_SIGNAL: (
        "No usable signal: the output was not switching, the reference is at "
        "zero, or the sensor is disconnected. Nothing changed."
    ),
    CAL_STATUS_OUT_OF_RANGE: (
        "Result implausible (more than 4x off nominal) -- check Vdc and the "
        "sensor wiring. Nothing changed."
    ),
}

# Highest harmonic order the GUI offers. MAX_HARMONICS in Core/Inc/hca_lib.h
# must be at least this large or the device runs out of channel slots.
MAX_HARMONIC_ORDER = 50

# The device streams every sample of its 40kHz control loop. Nyquist is
# therefore 20kHz, which is what bounds the FFT view's frequency axis.
STREAM_RATE_HZ = 40000.0

SIGNAL_NAMES = ("voltage", "reference", "error")
SIGNAL_LABELS = {
    "voltage": "Voltage (V)",
    "reference": "HCA Reference (p.u.)",
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


class RefFrame(NamedTuple):
    """The reference multiplier the device is actually running.

    ``limit`` is the largest value this firmware build accepts: open-loop
    builds allow overmodulation (above the modulation index), closed-loop ones
    cap at the modulation index so the controller keeps headroom to correct
    with. The device clamps, so ``value`` is what is really in force -- it may
    be below what was sent.
    """
    value: float
    open_loop: bool
    limit: float


class CalFrame(NamedTuple):
    """Sensor calibration state as reported by the device.

    ``gain``/``offset`` are the coefficients now in force (volts = raw*gain +
    offset). ``raw_peak``/``raw_dc`` are what the sensor measured uncalibrated
    and ``expected_peak`` what the modulator must have produced given the Vdc
    supplied -- their ratio is the gain, and showing both is what makes a bad
    run (dead output, wrong Vdc) obvious rather than silently accepted.
    Only CAL_STATUS_OK and CAL_STATUS_RESTORED changed anything.
    """
    status: int
    vdc: float
    gain: float
    offset: float
    raw_peak: float
    raw_dc: float
    expected_peak: float


class AdcFrame(NamedTuple):
    """One sample. Wire frames carry STREAM_BATCH_SAMPLES of these; parse_frame
    expands a frame back into per-sample records so everything downstream stays
    sample-oriented."""
    seq: int            # per-sample index: batch seq * STREAM_BATCH_SAMPLES + position
    timestamp_ms: float # device clock, interpolated within the batch
    voltage: float      # V
    reference: float    # p.u.
    error: float        # p.u.


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
    triples = fields[4:-1]
    return [
        AdcFrame(
            base + i,
            ts + i * sample_interval_ms,
            triples[3 * i] / STREAM_VOLT_SCALE,
            triples[3 * i + 1] / STREAM_PU_SCALE,
            triples[3 * i + 2] / STREAM_PU_SCALE,
        )
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


def parse_ref_frame(buf: bytes) -> Optional[RefFrame]:
    """Parse and validate one REF_FRAME_SIZE-byte buffer. Returns None if invalid."""
    if len(buf) != REF_FRAME_SIZE:
        return None
    sync0, sync1, value, open_loop, limit, chk = struct.unpack(REF_FRAME_FORMAT, buf)
    if sync0 != SYNC0 or sync1 != REF_SYNC1:
        return None
    payload = buf[2:-1]  # value .. limit, matches SendRefFrame() in firmware
    if checksum8(payload) != chk:
        return None
    return RefFrame(value, bool(open_loop), limit)


def build_set_ref_command(value: float) -> bytes:
    """CMD_SET_REF plus its payload, ready to write to the port."""
    body = struct.pack("<f", value)
    return CMD_SET_REF + body + bytes([checksum8(body)])


def parse_cal_frame(buf: bytes) -> Optional[CalFrame]:
    """Parse and validate one CAL_FRAME_SIZE-byte buffer. Returns None if invalid."""
    if len(buf) != CAL_FRAME_SIZE:
        return None
    (sync0, sync1, status, vdc, gain, offset,
     raw_peak, raw_dc, expected_peak, chk) = struct.unpack(CAL_FRAME_FORMAT, buf)
    if sync0 != SYNC0 or sync1 != CAL_SYNC1:
        return None
    payload = buf[2:-1]  # status .. expected_peak, matches SendCalFrame() in firmware
    if checksum8(payload) != chk:
        return None
    return CalFrame(status, vdc, gain, offset, raw_peak, raw_dc, expected_peak)


def build_calibrate_command(vdc: float) -> bytes:
    """CMD_CALIBRATE plus its payload, ready to write to the port."""
    body = struct.pack("<f", vdc)
    return CMD_CALIBRATE + body + bytes([checksum8(body)])
