"""Background thread that owns the serial port exclusively: it is the only
thread that ever calls read()/write() on it. Connects, resyncs on the
0xA5 0x5A frame marker, validates checksums, and buffers parsed frames for
the GUI thread to pull.

Commands from the GUI thread (start/stop/ping) are handed over through a
thread-safe queue.Queue instead of calling _ser.write() directly - a plain
method call from the GUI thread would run write() on the GUI thread's own
stack (Python threading doesn't move it onto this QThread), and a write()
that ever blocks (e.g. USB hiccup, full OS buffer) would freeze the whole
UI. Draining the queue here keeps all serial I/O off the GUI thread.

Parsed frames go into a plain thread-safe queue.Queue too, *not* a Qt
signal emitted on a fixed timer. Emitting a signal every N ms regardless
of whether the GUI has finished handling the previous one lets Qt's
cross-thread event queue grow without bound the moment rendering falls
behind arrival rate - exactly the "freezes more and more over time"
symptom. Instead, MainWindow pulls with drain_frames() on its own GUI
timer, so redraw cost is decoupled from how fast data arrives: a slow
render just means fewer, larger pulls, never a growing backlog.
"""

import queue

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
WRITE_TIMEOUT_S = 1.0  # bound worst-case write() blocking time, belt-and-braces


class SerialWorker(QThread):
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
        self._cmd_queue: "queue.Queue[bytes]" = queue.Queue()
        self._frame_queue: "queue.Queue" = queue.Queue()  # AdcFrame items, GUI-thread pulls

    # --- public control API, safe to call from the GUI thread ---
    # These only enqueue; the actual serial.write() happens inside run(),
    # on this worker thread.
    def start_streaming(self):
        self._streaming = True
        self._cmd_queue.put(CMD_START)

    def stop_streaming(self):
        self._streaming = False
        self._cmd_queue.put(CMD_STOP)

    def ping(self):
        self._cmd_queue.put(CMD_PING)

    def stop(self):
        self._running = False

    def drain_frames(self):
        """Non-blocking pull of every AdcFrame parsed since the last call.
        Call this from a GUI-thread timer, not from run()."""
        frames = []
        while True:
            try:
                frames.append(self._frame_queue.get_nowait())
            except queue.Empty:
                break
        return frames

    # --- internal, only ever runs on this thread ---
    def _drain_command_queue(self):
        while True:
            try:
                cmd = self._cmd_queue.get_nowait()
            except queue.Empty:
                return
            try:
                self._ser.write(cmd)
            except serial.SerialException as exc:
                self.error.emit(str(exc))

    def run(self):
        try:
            self._ser = serial.Serial(
                self._port_name, BAUDRATE, timeout=0.05, write_timeout=WRITE_TIMEOUT_S
            )
        except serial.SerialException as exc:
            self.error.emit(f"Could not open {self._port_name}: {exc}")
            return

        self.connected.emit(self._port_name)
        self._running = True
        buf = bytearray()

        while self._running:
            self._drain_command_queue()

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
                    self._frame_queue.put(frame)

        if self._ser is not None:
            try:
                if self._streaming:
                    self._ser.write(CMD_STOP)
                self._ser.close()
            except serial.SerialException:
                pass
        self.disconnected.emit()
