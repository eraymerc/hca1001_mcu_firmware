"""Background thread that owns the serial port: connects, sends start/stop
commands, resyncs on the 0xA5 0x5A frame marker, validates checksums, and
emits parsed frames back to the GUI thread in small batches.
"""

from PyQt5.QtCore import QThread, pyqtSignal
import serial

from protocol import (
    CMD_PING,
    CMD_START,
    CMD_STOP,
    FRAME_SIZE,
    PING_REPLY_PREFIX,
    SYNC0,
    SYNC1,
    parse_frame,
)

BAUDRATE = 115200  # ignored by the USB CDC-ACM link itself, pyserial still wants a value
READ_CHUNK = 4096
EMIT_BATCH_INTERVAL_MS = 20  # ~50 GUI updates/sec


class SerialWorker(QThread):
    frames_received = pyqtSignal(list)   # list[AdcFrame]
    connected = pyqtSignal(str)          # port name
    disconnected = pyqtSignal()
    error = pyqtSignal(str)
    ping_ok = pyqtSignal(bool)

    def __init__(self, port_name: str, parent=None):
        super().__init__(parent)
        self._port_name = port_name
        self._ser: serial.Serial | None = None
        self._running = False
        self._streaming = False

    # --- public control API, safe to call from the GUI thread ---
    def start_streaming(self):
        self._streaming = True
        self._send(CMD_START)

    def stop_streaming(self):
        self._streaming = False
        self._send(CMD_STOP)

    def ping(self):
        self._send(CMD_PING)

    def stop(self):
        self._running = False

    # --- internal ---
    def _send(self, data: bytes):
        if self._ser is not None and self._ser.is_open:
            try:
                self._ser.write(data)
            except serial.SerialException as exc:
                self.error.emit(str(exc))

    def run(self):
        try:
            self._ser = serial.Serial(self._port_name, BAUDRATE, timeout=0.05)
        except serial.SerialException as exc:
            self.error.emit(f"Could not open {self._port_name}: {exc}")
            return

        self.connected.emit(self._port_name)
        self._running = True
        buf = bytearray()
        pending_frames = []
        import time
        last_emit = time.monotonic()

        while self._running:
            try:
                chunk = self._ser.read(READ_CHUNK)
            except serial.SerialException as exc:
                self.error.emit(f"Serial read error: {exc}")
                break

            if chunk:
                buf.extend(chunk)

                # Handle the ASCII ping reply separately (not a fixed-size binary frame)
                if buf.startswith(PING_REPLY_PREFIX):
                    nl = buf.find(b"\n")
                    if nl != -1:
                        self.ping_ok.emit(True)
                        del buf[: nl + 1]

                # Resync on the two sync bytes, then parse fixed-size frames
                while True:
                    idx = buf.find(bytes([SYNC0, SYNC1]))
                    if idx == -1:
                        if len(buf) > FRAME_SIZE * 4:
                            del buf[: len(buf) - FRAME_SIZE]  # keep tail, avoid unbounded growth
                        break
                    if idx > 0:
                        del buf[:idx]
                    if len(buf) < FRAME_SIZE:
                        break

                    frame = parse_frame(bytes(buf[:FRAME_SIZE]))
                    if frame is None:
                        del buf[:1]  # bad checksum/sync collision, slide forward one byte
                        continue

                    del buf[:FRAME_SIZE]
                    pending_frames.append(frame)

            now = time.monotonic()
            if pending_frames and (now - last_emit) * 1000 >= EMIT_BATCH_INTERVAL_MS:
                self.frames_received.emit(pending_frames)
                pending_frames = []
                last_emit = now

        if pending_frames:
            self.frames_received.emit(pending_frames)

        if self._ser is not None:
            try:
                if self._streaming:
                    self._ser.write(CMD_STOP)
                self._ser.close()
            except serial.SerialException:
                pass
        self.disconnected.emit()
