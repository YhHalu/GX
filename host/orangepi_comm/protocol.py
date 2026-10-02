"""GX UART v2. All units and grammar are shared with motor_command.c."""
from dataclasses import dataclass
import binascii
import re

MAX_FRAME = 192
MAX_SEQ = 0xFFFFFFFF
ZERO_SID = "0" * 32
STATES = {"UNBOUND", "BOUND", "WAIT_START", "IDLE", "MOVING", "FAULT"}

class ProtocolError(ValueError):
    pass

@dataclass(frozen=True)
class Frame:
    kind: str
    sid: str
    seq: int
    args: tuple[str, ...] = ()

def crc(data: bytes) -> int:
    return binascii.crc_hqx(data, 0xFFFF)

def uint(value: str, limit: int = MAX_SEQ) -> int:
    if not value or not value.isascii() or not value.isdecimal() or len(value) > 10:
        raise ProtocolError("invalid unsigned integer")
    number = int(value)
    if number > limit:
        raise ProtocolError("integer out of range")
    return number

def signed(value: str) -> int:
    magnitude = value[1:] if value.startswith("-") else value
    number = uint(magnitude, 0x80000000 if value.startswith("-") else 0x7FFFFFFF)
    return -number if value.startswith("-") else number

def validate(frame: Frame) -> None:
    kind, sid, seq, args = frame.kind, frame.sid, frame.seq, frame.args
    if not isinstance(sid, str) or not re.fullmatch(r"[0-9A-F]{32}", sid) or type(seq) is not int or not 0 <= seq <= MAX_SEQ:
        raise ProtocolError("invalid session or sequence")
    if not isinstance(kind, str) or not isinstance(args, tuple) or not all(isinstance(arg, str) for arg in args):
        raise ProtocolError("invalid message fields")
    counts = {"HELLO": 0, "STOP": 0, "SET": 2, "READY": 6, "INFO": 6,
              "ACK": 4, "ERR": 2, "STATUS": 9}
    if kind not in counts or len(args) != counts[kind]:
        raise ProtocolError("unknown message or wrong field count")
    if kind in {"HELLO", "STOP", "SET", "INFO", "ACK"} and sid == ZERO_SID:
        raise ProtocolError("zero session")
    if kind in {"HELLO", "READY", "INFO"} and seq != 0:
        raise ProtocolError("handshake sequence must be zero")
    if kind == "READY" and sid != ZERO_SID:
        raise ProtocolError("READY must be unbound")
    if kind in {"STOP", "SET", "ACK"} and seq == 0:
        raise ProtocolError("request sequence must be positive")
    if kind == "SET":
        for arg in args:
            uint(arg, 1000)
    elif kind in {"INFO", "READY"}:
        if not re.fullmatch(r"[A-Z0-9_]{1,24}", args[0]):
            raise ProtocolError("invalid firmware identifier")
        for arg in args[1:]:
            uint(arg)
        if not all(uint(arg) > 0 for arg in args[1:]):
            raise ProtocolError("invalid capabilities")
        uint(args[3], 1000)
    elif kind == "ACK":
        op, state, p1, p2 = args
        targets = (uint(p1, 1000), uint(p2, 1000))
        expected_states = {"MOVING"} if any(targets) else {"IDLE", "WAIT_START"}
        if op not in {"STOP", "SET"} or state not in expected_states or (op == "STOP" and any(targets)):
            raise ProtocolError("inconsistent acknowledgement")
    elif kind == "ERR":
        if not re.fullmatch(r"[A-Z0-9_]{1,31}", args[0]) or args[1] != "FAULT":
            raise ProtocolError("invalid fault")
    elif kind == "STATUS":
        state, p1, p2, c1, c2, valid, age, fault, tick = args
        targets = (uint(p1, 1000), uint(p2, 1000))
        feedback = (signed(c1), signed(c2))
        bits = uint(valid, 3)
        age_ms = uint(age)
        uint(tick)
        if state not in STATES or not re.fullmatch(r"[A-Z0-9_]{1,31}", fault):
            raise ProtocolError("invalid state")
        if (state == "MOVING") != any(targets) or ((state == "FAULT") != (fault != "NONE")):
            raise ProtocolError("inconsistent state")
        if (bits and age_ms > 100) or any(feedback[i] and not bits & (1 << i) for i in range(2)):
            raise ProtocolError("invalid feedback validity")
        if state == "UNBOUND" and sid != ZERO_SID:
            raise ProtocolError("invalid unbound session")

def encode(frame: Frame) -> bytes:
    validate(frame)
    body = ",".join(("2", frame.kind, frame.sid, str(frame.seq), *frame.args)).encode("ascii")
    result = b"@" + body + f"*{crc(body):04X}\n".encode("ascii")
    if len(result) > MAX_FRAME:
        raise ProtocolError("frame too long")
    return result

def decode(raw: bytes) -> Frame:
    if not raw.endswith(b"\n") or len(raw) > MAX_FRAME:
        raise ProtocolError("incomplete or oversized frame")
    line = raw[:-1]
    if line.endswith(b"\r"):
        line = line[:-1]
    if len(line) < 8 or line[:1] != b"@" or line[-5:-4] != b"*":
        raise ProtocolError("bad frame boundary")
    body, checksum = line[1:-5], line[-4:]
    if not re.fullmatch(rb"[0-9A-F]{4}", checksum) or crc(body) != int(checksum, 16):
        raise ProtocolError("CRC mismatch")
    if any(byte < 33 or byte > 126 for byte in body):
        raise ProtocolError("invalid ASCII")
    fields = body.decode("ascii").split(",")
    if len(fields) < 4 or fields[0] != "2":
        raise ProtocolError("bad version or fields")
    result = Frame(fields[1], fields[2], uint(fields[3]), tuple(fields[4:]))
    validate(result)
    return result

class Framer:
    """Bounded stream decoder; discard an oversized line through LF."""
    def __init__(self):
        self.buffer = bytearray()
        self.discard = False

    def feed(self, data: bytes):
        for byte in data:
            if self.discard:
                if byte == 10:
                    self.discard = False
                continue
            self.buffer.append(byte)
            if len(self.buffer) > MAX_FRAME:
                self.buffer.clear()
                self.discard = byte != 10
                yield ProtocolError("oversized stream line")
            elif byte == 10:
                raw = bytes(self.buffer)
                self.buffer.clear()
                if raw in {b"\n", b"\r\n"}:
                    continue
                try:
                    yield decode(raw)
                except ProtocolError as error:
                    yield error
