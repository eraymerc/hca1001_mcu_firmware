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
    CAL_FRAME_SIZE,
    CAL_SYNC1,
    CMD_CAL_DEFAULT,
    CMD_GET_CAL,
    CMD_GET_COEFFS,
    CMD_GET_REF,
    CMD_PING,
    CMD_RESET_INTEGRATORS,
    CMD_START,
    CMD_STOP,
    COEFF_FRAME_SIZE,
    COEFF_SYNC1,
    FRAME_SIZE,
    PING_REPLY_PREFIX,
    REF_FRAME_SIZE,
    REF_SYNC1,
    SYNC0,
    SYNC1,
    build_calibrate_command,
    build_set_coeff_command,
    build_set_ref_command,
    parse_cal_frame,
    parse_coeff_frame,
    parse_frame,
    parse_ref_frame,
)

BAUDRATE = 2097000  # must match hlpuart1.Init.BaudRate in Core/Src/main.c
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
        self._coeff_queue: "queue.Queue" = queue.Queue()  # CoeffFrame items, likewise
        self._ref_queue: "queue.Queue" = queue.Queue()    # RefFrame items, likewise
        self._cal_queue: "queue.Queue" = queue.Queue()    # CalFrame items, likewise
        # Plain ints, written here and read from the GUI thread. Only ever
        # incremented, so a torn read just shows a slightly stale count.
        self._bytes_read = 0
        self._frames_ok = 0
        self._frames_rejected = 0   # sync matched but the frame failed to parse

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

    def request_coefficients(self):
        """Ask the device to report the Kp/Ki of every active HCA channel."""
        self._cmd_queue.put(CMD_GET_COEFFS)

    def reset_integrators(self):
        """Clear every channel's integrator state on the device. The device
        replies with its full coefficient report, which serves as the ack."""
        self._cmd_queue.put(CMD_RESET_INTEGRATORS)

    def set_coefficient(self, order: int, kp: complex, ki: complex):
        """Push new gains for one harmonic order. The device echoes the channel
        back once applied, so the reply -- not this call -- is the confirmation."""
        self._cmd_queue.put(build_set_coeff_command(order, kp, ki))

    def request_reference_multiplier(self):
        """Ask the device for the reference multiplier it is running."""
        self._cmd_queue.put(CMD_GET_REF)

    def set_reference_multiplier(self, value: float):
        """Push a new reference multiplier. The device clamps it to what this
        build allows and echoes the applied value back, so the reply -- not this
        call -- says what is actually in force."""
        self._cmd_queue.put(build_set_ref_command(value))

    def calibrate(self, vdc: float):
        """Run the sensor self-calibration against a known DC bus voltage. Takes
        the device ~0.6s; the CalFrame it replies with carries the outcome."""
        self._cmd_queue.put(build_calibrate_command(vdc))

    def request_calibration(self):
        """Ask the device for the sensor calibration it is running."""
        self._cmd_queue.put(CMD_GET_CAL)

    def restore_default_calibration(self):
        """Put the build-time sensor calibration back."""
        self._cmd_queue.put(CMD_CAL_DEFAULT)

    def stop(self):
        self._running = False

    def drain_frames(self):
        """Non-blocking pull of every AdcFrame parsed since the last call.
        Call this from a GUI-thread timer, not from run()."""
        return self._drain(self._frame_queue)

    def stats(self):
        """(bytes read, frames accepted, frames rejected) since connect. Lets the
        GUI tell 'no bytes arriving' from 'bytes arriving but not framing'."""
        return self._bytes_read, self._frames_ok, self._frames_rejected

    def drain_coeff_frames(self):
        """Non-blocking pull of every CoeffFrame parsed since the last call."""
        return self._drain(self._coeff_queue)

    def drain_ref_frames(self):
        """Non-blocking pull of every RefFrame parsed since the last call."""
        return self._drain(self._ref_queue)

    def drain_cal_frames(self):
        """Non-blocking pull of every CalFrame parsed since the last call."""
        return self._drain(self._cal_queue)

    @staticmethod
    def _drain(q):
        items = []
        while True:
            try:
                items.append(q.get_nowait())
            except queue.Empty:
                break
        return items

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

        try:
            self._read_loop(buf)
        except Exception as exc:  # noqa: BLE001 - a dead thread must not be silent
            self.error.emit(f"Serial reader stopped: {type(exc).__name__}: {exc}")

        if self._ser is not None:
            try:
                if self._streaming:
                    self._ser.write(CMD_STOP)
                self._ser.close()
            except serial.SerialException:
                pass
        self.disconnected.emit()

    def _read_loop(self, buf):
        while self._running:
            self._drain_command_queue()

            try:
                chunk = self._ser.read(READ_CHUNK)
            except serial.SerialException as exc:
                self.error.emit(f"Serial read error: {exc}")
                break

            if chunk:
                self._bytes_read += len(chunk)
                buf.extend(chunk)

                # Handle the ASCII ping reply separately (not a fixed-size binary frame)
                if buf.startswith(PING_REPLY_PREFIX):
                    nl = buf.find(b"\n")
                    if nl != -1:
                        self.ping_ok.emit(True)
                        del buf[: nl + 1]

                # Resync on SYNC0, then let the following byte pick the frame
                # type: SYNC1 is an ADC sample, COEFF_SYNC1 a coefficient
                # report, REF_SYNC1 the reference multiplier, CAL_SYNC1 a
                # calibration report. All are
                # fixed-size and checksummed, so a sync-byte
                # collision inside float payload data just fails to parse and
                # the loop slides forward a byte.
                while True:
                    idx = buf.find(bytes([SYNC0]))
                    if idx == -1:
                        # No frame start in sight. Keep a short tail (a split
                        # ASCII ping reply lives here too) but never let the
                        # buffer grow without bound.
                        if len(buf) > COEFF_FRAME_SIZE * 4:
                            del buf[: len(buf) - COEFF_FRAME_SIZE]
                        break
                    if idx > 0:
                        del buf[:idx]
                    if len(buf) < 2:
                        break

                    if buf[1] == SYNC1:
                        # A sample frame parses to a list (it carries a batch);
                        # a coefficient frame to a single record.
                        size, parse, sink, batched = (
                            FRAME_SIZE, parse_frame, self._frame_queue, True
                        )
                    elif buf[1] == COEFF_SYNC1:
                        size, parse, sink, batched = (
                            COEFF_FRAME_SIZE, parse_coeff_frame, self._coeff_queue, False
                        )
                    elif buf[1] == REF_SYNC1:
                        size, parse, sink, batched = (
                            REF_FRAME_SIZE, parse_ref_frame, self._ref_queue, False
                        )
                    elif buf[1] == CAL_SYNC1:
                        size, parse, sink, batched = (
                            CAL_FRAME_SIZE, parse_cal_frame, self._cal_queue, False
                        )
                    else:
                        del buf[:1]  # 0xA5 that starts no frame
                        continue

                    if len(buf) < size:
                        break

                    frame = parse(bytes(buf[:size]))
                    if frame is None:
                        self._frames_rejected += 1
                        del buf[:1]  # bad checksum/sync collision, slide forward one byte
                        continue

                    self._frames_ok += 1
                    del buf[:size]
                    if batched:
                        for sample in frame:
                            sink.put(sample)
                    else:
                        sink.put(frame)
