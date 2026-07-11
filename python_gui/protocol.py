"""Wire protocol shared with the firmware (Core/Inc/main.h, AdcStreamFrame_t).

Keep this file in sync with the C struct by hand -- there is no code
generation step. If you change the frame layout in firmware, update
FRAME_FORMAT here to match.
"""

import struct
from typing import NamedTuple, Optional

SYNC0 = 0xA5
SYNC1 = 0x5A

# '<' little-endian (Cortex-M4 and x86/x64 both are), matches:
#   uint8_t sync0, sync1; uint32_t seq, timestamp_ms;
#   float voltage, current, encoder, error; uint8_t checksum;
FRAME_FORMAT = "<BBIIffffB"
FRAME_SIZE = struct.calcsize(FRAME_FORMAT)

# Single-byte host -> device commands (see USB_DEVICE/App/usbd_cdc_if.c)
CMD_START = b"S"
CMD_STOP = b"X"
CMD_PING = b"P"
PING_REPLY_PREFIX = b"HCA1001_ADC_STREAM_V1"

# Must match ADC_STREAM_DECIMATION in Core/Src/main.c (40kHz / 40 = 1kHz)
STREAM_RATE_HZ = 1000.0

SIGNAL_NAMES = ("voltage", "current", "encoder", "error")
SIGNAL_LABELS = {
    "voltage": "Voltage (V)",
    "current": "Current (raw ADC V)",
    "encoder": "Encoder (raw ADC V)",
    "error": "HCA Error (p.u.)",
}


class AdcFrame(NamedTuple):
    seq: int
    timestamp_ms: int
    voltage: float
    current: float
    encoder: float
    error: float


def checksum8(payload: bytes) -> int:
    return sum(payload) & 0xFF


def parse_frame(buf: bytes) -> Optional[AdcFrame]:
    """Parse and validate one FRAME_SIZE-byte buffer. Returns None if invalid."""
    if len(buf) != FRAME_SIZE:
        return None
    sync0, sync1, seq, ts, voltage, current, encoder, error, chk = struct.unpack(FRAME_FORMAT, buf)
    if sync0 != SYNC0 or sync1 != SYNC1:
        return None
    payload = buf[2:-1]  # seq .. error, matches StreamChecksum() in firmware
    if checksum8(payload) != chk:
        return None
    return AdcFrame(seq, ts, voltage, current, encoder, error)
