"""Terminal tools; zero-test cannot encode a nonzero SET."""
import argparse
import json
import signal
import sys
import time

from .client import Client, CommunicationError

def _seconds(value):
    number = float(value)
    if not 0.1 <= number <= 86400:
        raise argparse.ArgumentTypeError("seconds must be within 0.1..86400")
    return number

def main(argv=None):
    parser = argparse.ArgumentParser(description="香橙派与 GX STM32 UART v2 通信")
    parser.add_argument("--port", required=True, help="实际 UART 设备路径")
    parser.add_argument("--zero-only", action="store_true", help="禁止所有非零目标")
    parser.add_argument("--log", default="logs/uart.jsonl", help="JSONL 日志（5MB×4）")
    sub = parser.add_subparsers(dest="command", required=True)
    sub.add_parser("handshake")
    sub.add_parser("stop")
    status = sub.add_parser("status")
    status.add_argument("--seconds", type=_seconds, default=5.0)
    test = sub.add_parser("zero-test")
    test.add_argument("--seconds", type=_seconds, default=60.0, help="验收时长，默认60秒")
    drive = sub.add_parser("drive")
    drive.add_argument("--cn1", type=float, required=True)
    drive.add_argument("--cn2", type=float, required=True)
    drive.add_argument("--seconds", type=_seconds, required=True)
    drive.add_argument("--start-timeout", type=_seconds, default=30.0, help="等待按下SW2的秒数，默认30秒；等待期间不发送运动目标")
    args = parser.parse_args(argv)
    if args.command == "drive":
        if args.seconds > 2:
            parser.error("drive is a short test limited to two seconds")
        # Validate before opening the port.
        p1, p2 = Client._percent(args.cn1), Client._percent(args.cn2)
        if args.zero_only and (p1 or p2):
            parser.error("zero-only mode forbids nonzero targets")
    def terminate(signum, frame):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, terminate)
    try:
        with Client(args.port, zero_only=args.zero_only or args.command == "zero-test", log_path=args.log) as client:
            print(json.dumps({"connected": client.info, "sid": client.sid}, ensure_ascii=False), flush=True)
            if args.command == "stop":
                client.stop()
                print('{"stop_confirmed": true}', flush=True)
            elif args.command in {"status", "zero-test"}:
                deadline = time.monotonic() + args.seconds
                next_print = 0.0
                valid_samples = 0
                # The first scheduled STATUS may follow the handshake by 100ms.
                start = time.monotonic()
                while time.monotonic() < deadline:
                    now = time.monotonic()
                    try:
                        snapshot = client.get_status()
                    except CommunicationError:
                        if now-start > 0.3:
                            raise
                        time.sleep(0.01)
                        continue
                    if snapshot["host_state"] not in {"IDLE", "WAIT_START"} or snapshot["state"] not in {"IDLE", "WAIT_START"}:
                        raise CommunicationError("zero/idle state lost")
                    if args.command == "zero-test":
                        if snapshot["target_permille"] != [0,0] or snapshot["valid_bits"] != 3 or snapshot["measured_cps"] != [0,0]:
                            raise CommunicationError("zero-test feedback or target check failed")
                        valid_samples += 1
                    if now >= next_print:
                        print(json.dumps(snapshot, ensure_ascii=False), flush=True)
                        next_print = now + 1
                    time.sleep(0.05)
                if args.command == "zero-test":
                    if not valid_samples:
                        raise CommunicationError("zero-test received no valid status")
                    print(json.dumps({"zero_test_passed": True, "seconds": args.seconds, "observations": valid_samples}), flush=True)
            elif args.command == "drive":
                if client.state == "WAIT_START" and (args.cn1 or args.cn2):
                    print("请按下下位机 SW2 启动键；当前电机保持停止。", flush=True)
                    client.wait_for_start(args.start_timeout)
                deadline = time.monotonic() + args.seconds
                while time.monotonic() < deadline:
                    client.set_speed(args.cn1, args.cn2)
                    time.sleep(0.05)
                client.stop()
                print('{"stop_confirmed": true}', flush=True)
    except KeyboardInterrupt:
        raise SystemExit(130)
    except (CommunicationError, ValueError, OSError) as error:
        print(str(error), file=sys.stderr)
        raise SystemExit(1)
