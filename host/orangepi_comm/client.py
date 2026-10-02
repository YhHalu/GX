"""One serial owner; application requests expire independently of UART refresh."""
from dataclasses import dataclass, field
from datetime import datetime, timezone
import json
import logging
from logging.handlers import RotatingFileHandler
import math
from pathlib import Path
import secrets
import threading
import time

from .protocol import Frame, Framer, MAX_SEQ, ProtocolError, ZERO_SID, encode, uint

class CommunicationError(RuntimeError):
    pass

@dataclass
class _Request:
    frame: Frame
    safety: bool = False
    sent: float | None = None
    event: threading.Event = field(default_factory=threading.Event)
    result: Frame | None = None
    error: Exception | None = None

class Client:
    PERIOD = 0.100
    ACK_TIMEOUT = 0.300
    APP_TIMEOUT = 0.300

    def __init__(self, port: str, *, zero_only=False, log_path=None, serial_factory=None):
        self.port_name = port
        self.zero_only = zero_only
        self._factory = serial_factory
        self._lock = threading.RLock()
        self._end = threading.Event()
        self._thread = None
        self._serial = None
        self.state = "DISCONNECTED"
        self.fault = None
        self.info = None
        self.sid = ZERO_SID
        self._seq = self._confirmed = 0
        self._status_floor = 0
        self._pending = None
        self._target = None
        self._app_deadline = self._next_set = 0.0
        self._status = None
        self._status_time = 0.0
        self._safety_remaining = 0
        self._safety_next = 0.0
        self._generation = 0
        self._log = logging.Logger(f"gx-uart-{id(self)}")
        self._log_path = Path(log_path) if log_path else None
        self._setup_log()

    def _setup_log(self):
        if self._log.handlers:
            return
        if self._log_path:
            path = self._log_path
            path.parent.mkdir(parents=True, exist_ok=True)
            handler = RotatingFileHandler(path, maxBytes=5*1024*1024, backupCount=3, encoding="utf-8")
            handler.setFormatter(logging.Formatter("%(message)s"))
            self._log.addHandler(handler)
        else:
            self._log.addHandler(logging.NullHandler())

    def _record(self, event, **fields):
        self._log.info(json.dumps({"utc": datetime.now(timezone.utc).isoformat(),
                                  "monotonic": time.monotonic(), "event": event,
                                  "host_state": self.state, "sid": self.sid, **fields}, ensure_ascii=False))

    def _serial_open(self):
        factory = self._factory
        if factory is None:
            import serial
            factory = serial.Serial
        return factory(self.port_name, baudrate=115200, bytesize=8, parity="N", stopbits=1,
                       timeout=0.01, write_timeout=0.05, xonxoff=False, rtscts=False,
                       dsrdtr=False, exclusive=True)

    def _finish(self, result=None, error=None):
        request = self._pending
        self._pending = None
        if request:
            request.result, request.error = result, error
            request.event.set()

    def _fault(self, reason, *, transport=False):
        self._generation += 1
        first = self.state != "FAULT"
        self._target = None
        self._app_deadline = 0.0
        self.state = "FAULT"
        self.fault = reason
        self._finish(error=CommunicationError(reason))
        if first and not transport:
            self._safety_remaining = 3
            self._safety_next = time.monotonic()
        if transport:
            self._safety_remaining = 0
        self._record("fault", reason=reason)

    def _next_sequence(self, *, stopping=False):
        if self._seq >= (MAX_SEQ if stopping else MAX_SEQ - 4):
            raise CommunicationError("sequence exhausted; recover with a new session")
        self._seq += 1
        return self._seq

    def _write(self, request):
        raw = encode(request.frame)
        started = time.monotonic()
        offset = 0
        while offset < len(raw):
            count = self._serial.write(raw[offset:])
            if not isinstance(count, int) or count <= 0 or count > len(raw)-offset:
                raise CommunicationError("partial write made no progress")
            offset += count
            if time.monotonic() - started >= 0.05 and offset != len(raw):
                raise CommunicationError("partial write deadline exceeded")
        request.sent = started
        self._record("tx", seq=request.frame.seq, raw=raw.decode("ascii"), hex=raw.hex())

    def _tick(self, now):
        if self._target is not None and now >= self._app_deadline:
            self._fault("APPLICATION_TIMEOUT")
        request = self._pending
        if request and request.sent is not None and now-request.sent >= self.ACK_TIMEOUT:
            safety = request.safety
            op = request.frame.kind
            self._finish(error=CommunicationError("ACK_TIMEOUT"))
            if safety:
                self._safety_next = now + self.PERIOD
            elif self.state not in {"HANDSHAKING", "STOPPING"}:
                self._fault("ACK_TIMEOUT")
            self._record("request_timeout", op=op)
        if self._pending is None and self._safety_remaining and now >= self._safety_next:
            self._safety_remaining -= 1
            self._pending = _Request(Frame("STOP", self.sid, self._next_sequence(stopping=True)), safety=True)
        if self._pending is None and self._target is not None and self.state in {"IDLE", "MOVING"} and now >= self._next_set:
            if self._seq >= MAX_SEQ - 4:
                self._fault("SEQ_EXHAUSTED")
                return
            seq = self._next_sequence()
            self._pending = _Request(Frame("SET", self.sid, seq, tuple(str(x) for x in self._target)))
            self._next_set = now + self.PERIOD
        if self._pending and self._pending.sent is None:
            self._write(self._pending)

    def _handle(self, frame):
        self._record("rx", raw=encode(frame).decode("ascii"), seq=frame.seq)
        if frame.kind == "READY":
            if self.state == "HANDSHAKING" and self.info is not None:
                self._generation += 1
                self.info = None
                self._finish(error=CommunicationError("LOWER_RESTART"))
            elif self.state != "HANDSHAKING":
                self._fault("LOWER_RESTART")
            return
        if frame.sid != self.sid:
            if frame.kind == "STATUS" and self.state in {"WAIT_START", "IDLE", "MOVING"}:
                self._fault("SESSION_CHANGED")
            return
        request = self._pending
        if frame.kind == "ERR":
            if self.state == "HANDSHAKING":
                if request and frame.seq == request.frame.seq:
                    self._finish(error=CommunicationError(frame.args[0]))
            elif self.state == "STOPPING":
                if request and frame.seq == request.frame.seq:
                    self._finish(error=CommunicationError(frame.args[0]))
            elif self.state != "FAULT":
                self._fault(frame.args[0])
            return
        if frame.kind == "STATUS":
            if frame.seq < self._status_floor or (self._status and frame.seq < self._status["sequence"]):
                return
            if frame.seq > self._seq:
                raise ProtocolError("unissued sequence in status")
            state, p1, p2, c1, c2, bits, age, fault, tick = frame.args
            previous = self._status
            if previous:
                old = previous["uptime_ms"]
                new = int(tick)
                if new < old and old-new < 0x80000000:
                    self._fault("LOWER_RESTART")
                    return
            self._status = {"sid": frame.sid, "sequence": frame.seq, "state": state,
                            "target_permille": [int(p1), int(p2)],
                            "measured_cps": [int(c1)/10, int(c2)/10],
                            "valid_bits": int(bits), "sample_age_ms": int(age),
                            "fault": fault, "uptime_ms": int(tick)}
            self._status_time = time.monotonic()
            if self.state in {"WAIT_START", "IDLE", "MOVING"}:
                if state == "FAULT":
                    self._fault(fault)
                elif state in {"UNBOUND", "BOUND"}:
                    self._fault("LOWER_NOT_ARMED")
                elif self.state == "WAIT_START":
                    if state == "IDLE":
                        self.state = "IDLE"
                        self._record("button_started")
                    elif state == "MOVING":
                        self._fault("UNEXPECTED_NONZERO_TARGET")
                elif state == "WAIT_START":
                    self._fault("START_PERMISSION_LOST")
                elif self.zero_only and (int(p1) or int(p2)):
                    self._fault("UNEXPECTED_NONZERO_TARGET")
            return
        if not request or frame.seq != request.frame.seq:
            self._record("unmatched_reply", seq=frame.seq)
            return
        if frame.kind == "INFO" and request.frame.kind == "HELLO":
            if frame.args[0] not in {"GXUART2", "GXUART2_BTN"} or tuple(map(uint, frame.args[1:])) != (6000,6000,300,500,100):
                raise ProtocolError("unsupported firmware capabilities")
            self.info = {"firmware": frame.args[0], "scale_cps": [6000,6000], "pwm_max_permille": 300,
                         "lease_ms": 500, "partial_ms": 100,
                         "start_button": frame.args[0] == "GXUART2_BTN"}
            self._finish(result=frame)
        elif frame.kind == "ACK" and request.frame.kind in {"SET", "STOP"}:
            expected = request.frame.args if request.frame.kind == "SET" else ("0", "0")
            if frame.args[0] != request.frame.kind or tuple(map(int, frame.args[2:])) != tuple(map(int, expected)):
                raise ProtocolError("ACK does not match request")
            self._confirmed = frame.seq
            if request.frame.kind == "STOP":
                # Exclude queued pre-STOP measurements (including BOUND).
                # SET telemetry may legitimately precede the latest SET ACK.
                self._status_floor = frame.seq
                if self._status is not None and self._status["sequence"] < self._status_floor:
                    self._status = None
                    self._status_time = 0.0
            if request.safety:
                self._safety_remaining = 0
            if request.frame.kind == "SET" and self.state in {"IDLE", "MOVING"}:
                self.state = frame.args[1]
            self._finish(result=frame)

    def _run(self):
        decoder = Framer()
        try:
            while not self._end.is_set():
                with self._lock:
                    self._tick(time.monotonic())
                data = self._serial.read(256)
                with self._lock:
                    if data:
                        self._record("rx_bytes", hex=data.hex())
                    for item in decoder.feed(data):
                        if isinstance(item, ProtocolError):
                            if self.state == "HANDSHAKING":
                                self._record("bad_handshake_input", reason=str(item))
                            else:
                                self._fault("PROTOCOL: " + str(item))
                        else:
                            try:
                                self._handle(item)
                            except ProtocolError as error:
                                if self.state == "HANDSHAKING":
                                    self._finish(error=CommunicationError(str(error)))
                                else:
                                    self._fault("PROTOCOL: " + str(error))
        except Exception as error:
            with self._lock:
                self._fault("TRANSPORT: " + str(error), transport=True)
        finally:
            self._serial.close()
            with self._lock:
                self._finish(error=CommunicationError("serial owner exited"))

    def _exchange(self, frame, *, preempt=False):
        with self._lock:
            if not self._thread or not self._thread.is_alive():
                raise CommunicationError("serial owner is not running")
            if self._pending:
                if not preempt:
                    raise CommunicationError("request already pending")
                self._finish(error=CommunicationError("preempted by STOP"))
            request = _Request(frame)
            self._pending = request
        if not request.event.wait(self.ACK_TIMEOUT + 0.15):
            with self._lock:
                if self._pending is request:
                    self._finish(error=CommunicationError("serial owner response deadline"))
            raise CommunicationError("serial owner response deadline")
        if request.error:
            raise CommunicationError(str(request.error))
        return request.result

    def _terminate(self):
        with self._lock:
            self._target = None
            self._end.set()
        if self._thread:
            self._thread.join(1.5)
            if self._thread.is_alive():
                raise CommunicationError("serial owner did not exit; cannot reopen port")
        self._thread = None

    def connect(self):
        self._setup_log()
        self._terminate()
        with self._lock:
            self.state = "HANDSHAKING"
            self.fault = None
            self.info = self._status = None
            self._status_time = 0.0
            self._safety_remaining = 0
            self._pending = None
            self._end.clear()
        try:
            self._serial = self._serial_open()
            self._serial.reset_input_buffer()
            self._serial.reset_output_buffer()
            # Delimit any partial frame before the first HELLO.
            if self._serial.write(b"\n") != 1:
                raise CommunicationError("cannot delimit UART")
            self._thread = threading.Thread(target=self._run, name="gx-uart", daemon=True)
            self._thread.start()
            last_error = None
            for attempt in range(3):
                with self._lock:
                    self.sid = f"{secrets.randbelow((1 << 128) - 1) + 1:032X}"
                    self._seq = self._confirmed = 0
                    self._status_floor = 0
                    self._status = None
                    self.info = None
                    generation = self._generation
                try:
                    self._exchange(Frame("HELLO", self.sid, 0))
                    with self._lock:
                        seq = self._next_sequence(stopping=True)
                    stopped = self._exchange(Frame("STOP", self.sid, seq))
                    with self._lock:
                        if self._generation != generation or self.state != "HANDSHAKING":
                            raise CommunicationError("handshake invalidated by a fault")
                        self.state = stopped.args[1]
                        self._record("connected")
                    return self.info.copy()
                except CommunicationError as error:
                    last_error = error
                    time.sleep(self.PERIOD)
            raise CommunicationError(f"zero handshake failed: {last_error}")
        except Exception as error:
            self._terminate()
            if self._serial:
                self._serial.close()
            with self._lock:
                self.state, self.fault = "FAULT", str(error)
            raise CommunicationError(str(error)) from error

    @staticmethod
    def _percent(value):
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or not 0 <= value <= 100:
            raise ValueError("reference speed must be finite and within 0..100 percent")
        return math.floor(value * 10 + 0.5)

    def set_speed(self, cn1_percent, cn2_percent):
        target = (self._percent(cn1_percent), self._percent(cn2_percent))
        if self.zero_only and any(target):
            raise ValueError("zero-only mode forbids nonzero targets")
        if not any(target):
            return self.stop()
        with self._lock:
            if self.state == "WAIT_START":
                raise CommunicationError("请先按下下位机 SW2 启动键；运动请求未排队")
            if self.state not in {"IDLE", "MOVING"}:
                raise CommunicationError("new motion requires a healthy connected session")
            if self._target != target:
                self._next_set = 0.0
            self._target = target
            self._app_deadline = time.monotonic() + self.APP_TIMEOUT
        return target

    def stop(self):
        with self._lock:
            if self.state == "DISCONNECTED":
                raise CommunicationError("not connected; stop unconfirmed")
            if self.state == "HANDSHAKING":
                raise CommunicationError("handshake is in progress")
            preserve_fault = self.state == "FAULT"
            reason = self.fault
            self._target = None
            self._safety_remaining = 0
            self.state = "STOPPING"
            generation = self._generation
        last_error = None
        for attempt in range(3):
            try:
                with self._lock:
                    seq = self._next_sequence(stopping=True)
                result = self._exchange(Frame("STOP", self.sid, seq), preempt=True)
                with self._lock:
                    if self._generation != generation or self.state != "STOPPING":
                        raise CommunicationError("stop invalidated by a fault")
                    self.state = "FAULT" if preserve_fault else result.args[1]
                    self.fault = reason if preserve_fault else None
                return result
            except CommunicationError as error:
                last_error = error
                time.sleep(self.PERIOD)
        with self._lock:
            self._fault("STOP_UNCONFIRMED: " + str(last_error), transport=True)
        raise CommunicationError(str(self.fault))

    def recover(self):
        return self.connect()

    def wait_for_start(self, timeout=30.0):
        """Wait for SW2 authorization without submitting or caching motion."""
        if isinstance(timeout, bool) or not isinstance(timeout, (int, float)) or not math.isfinite(timeout) or timeout <= 0:
            raise ValueError("start timeout must be finite and positive")
        deadline = time.monotonic() + timeout
        while True:
            with self._lock:
                if self.state == "IDLE":
                    return
                if self.state != "WAIT_START":
                    raise CommunicationError("cannot wait for start: " + str(self.fault or self.state))
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise CommunicationError("等待 SW2 启动键超时；未发送运动目标")
            time.sleep(min(0.02, remaining))

    def get_status(self, *, max_age=0.3):
        with self._lock:
            if self._status is None or time.monotonic()-self._status_time > max_age:
                raise CommunicationError("fresh status is unavailable")
            result = self._status.copy()
            result["target_permille"] = result["target_permille"].copy()
            result["measured_cps"] = result["measured_cps"].copy()
            result.update(host_state=self.state, host_fault=self.fault)
            return result

    def close(self):
        error = None
        if self._thread and self._thread.is_alive() and self.state != "HANDSHAKING":
            try:
                self.stop()
            except CommunicationError as exception:
                error = exception
        self._terminate()
        with self._lock:
            if error is None:
                self.state = "DISCONNECTED"
        for handler in self._log.handlers[:]:
            handler.close()
            self._log.removeHandler(handler)
        if error:
            raise error

    def __enter__(self):
        self.connect()
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        try:
            self.close()
        except CommunicationError:
            if exc_type is None:
                raise
