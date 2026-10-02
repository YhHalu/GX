"""Explicit hardware test: only HELLO/STOP, never a nonzero request.

Run after the continuous zero test. An external SWD reset is coordinated
through --reset-ready; this script itself does not reset or flash hardware.
"""
import argparse
from datetime import datetime, timezone
import json
from pathlib import Path
import time
import uuid

import serial
from orangepi_comm import Client
from orangepi_comm.protocol import Frame, Framer, ProtocolError, encode


def report(event, **fields):
    print(json.dumps({"event": event, **fields}, ensure_ascii=False), flush=True)


def verify_idle(client):
    deadline = time.monotonic() + 1.0
    while time.monotonic() < deadline:
        try:
            status = client.get_status()
        except RuntimeError:
            time.sleep(0.02)
            continue
        if (status["host_state"] not in {"IDLE", "WAIT_START"} or status["state"] not in {"IDLE", "WAIT_START"} or
                status["target_permille"] != [0, 0] or status["measured_cps"] != [0, 0] or
                status["valid_bits"] != 3 or status["fault"] != "NONE"):
            raise RuntimeError(f"zero verification failed: {status}")
        report("idle_verified", status=status)
        return status
    raise RuntimeError("no fresh status after recovery")


class RawZeroLink:
    def __init__(self, port, log_path):
        self.sid = uuid.uuid4().hex.upper()
        self.decoder = Framer()
        self.log = log_path.open("a", encoding="utf-8")
        try:
            self.serial = serial.Serial(port, baudrate=115200, timeout=0.02,
                                        write_timeout=0.1, exclusive=True)
        except Exception:
            self.log.close()
            raise

    def record(self, direction, data):
        self.log.write(json.dumps({"utc": datetime.now(timezone.utc).isoformat(),
                                   "direction": direction, "hex": data.hex()}) + "\n")
        self.log.flush()

    def wire(self, data):
        if self.serial.write(data) != len(data):
            raise RuntimeError("incomplete test frame write")
        self.record("tx", data)

    def send(self, kind, seq):
        if kind not in {"HELLO", "STOP"}:
            raise ValueError("hardware tool permits only HELLO and STOP")
        self.wire(encode(Frame(kind, self.sid, seq)))

    def wait(self, predicate, *, initial=False):
        deadline = time.monotonic() + 1.5
        while time.monotonic() < deadline:
            data = self.serial.read(512)
            if data:
                self.record("rx", data)
            for frame in self.decoder.feed(data):
                if isinstance(frame, ProtocolError):
                    if initial:
                        continue  # Opening mid-STATUS can leave a truncated old line.
                    raise RuntimeError(str(frame))
                if frame.sid != self.sid:
                    continue
                if frame.kind in {"ACK", "STATUS"}:
                    targets = frame.args[2:] if frame.kind == "ACK" else frame.args[1:3]
                    if any(map(int, targets)):
                        raise RuntimeError("unexpected nonzero target")
                if predicate(frame):
                    return frame
        raise RuntimeError("hardware reply timed out")

    def establish(self):
        self.serial.reset_input_buffer()
        self.wire(b"\n")
        self.send("HELLO", 0)
        self.wait(lambda frame: frame.kind == "INFO" and frame.seq == 0, initial=True)
        self.send("STOP", 1)
        self.wait(lambda frame: frame.kind == "ACK" and frame.seq == 1)

    def close(self):
        try:
            self.serial.close()
        finally:
            self.log.close()


def main():
    parser = argparse.ArgumentParser(description="实机零目标异常及恢复检查")
    parser.add_argument("--port", required=True)
    parser.add_argument("--log", default="logs/zero-recovery.jsonl")
    parser.add_argument("--reset-ready", type=Path,
                        help="写入准备标记后，等待外部执行 STM32 复位（40秒上限）")
    args = parser.parse_args()
    log = Path(args.log)
    log.parent.mkdir(parents=True, exist_ok=True)

    for fault in ("CRC", "PARTIAL_TIMEOUT"):
        raw = RawZeroLink(args.port, log.with_suffix(".raw.jsonl"))
        try:
            raw.establish()
            if fault == "CRC":
                frame = encode(Frame("STOP", raw.sid, 2))
                checksum = b"0000" if frame[-5:-1] != b"0000" else b"FFFF"
                raw.wire(frame[:-5] + checksum + b"\n")
            else:
                raw.wire(b"@2,STOP,")
            reply = raw.wait(lambda frame: frame.kind == "ERR" and frame.args[0] == fault)
            status = raw.wait(lambda frame: frame.kind == "STATUS" and frame.args[0] == "FAULT")
            report("fault_latched", fault=fault, reply=reply.args, status=status.args)
            raw.wire(b"\n")
        finally:
            raw.close()
        with Client(args.port, zero_only=True, log_path=log) as client:
            if client.sid == raw.sid:
                raise RuntimeError("recovery reused old session")
            verify_idle(client)

    with Client(args.port, zero_only=True, log_path=log) as client:
        previous_sid = client.sid
        verify_idle(client)
        client.close()
        client.recover()
        if client.sid == previous_sid:
            raise RuntimeError("reconnect reused old session")
        verify_idle(client)
        report("port_reopen_passed")
        if args.reset_ready:
            previous_sid = client.sid
            uptime_before = client.get_status()["uptime_ms"]
            args.reset_ready.write_text(client.sid + "\n")
            report("ready_for_external_reset", sid=client.sid)
            try:
                deadline = time.monotonic() + 40
                while client.state != "FAULT" and time.monotonic() < deadline:
                    time.sleep(0.02)
                # Reset can truncate an active STATUS and corrupt the READY line.
                # A protocol fault must revoke motion just like a complete READY.
                reason = client.fault or ""
                if client.state != "FAULT" or not (reason in {"LOWER_RESTART", "SESSION_CHANGED"} or
                                                    reason.startswith("PROTOCOL: ")):
                    raise RuntimeError(f"reset not detected: {client.state}, {client.fault}")
                report("reset_detected", fault=client.fault)
                client.recover()
                if client.sid == previous_sid:
                    raise RuntimeError("reset recovery reused old session")
                status = verify_idle(client)
                if status["uptime_ms"] >= uptime_before:
                    raise RuntimeError("STM32 uptime did not restart")
            finally:
                args.reset_ready.unlink(missing_ok=True)
    report("zero_recovery_passed", external_reset=bool(args.reset_ready))


if __name__ == "__main__":
    main()
