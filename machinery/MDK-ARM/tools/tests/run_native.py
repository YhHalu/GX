"""GCC C regression tests plus Python/C interoperability; never access hardware."""
from pathlib import Path
import argparse
import os
import subprocess
import sys

root = Path(__file__).resolve().parents[3]
tests = Path(__file__).resolve().parent
src = root / "Core/Src"
repo = root.parent
out = root / "MDK-ARM/build/motor-tests"

def run(args, **kwargs):
    subprocess.run([str(x) for x in args], check=True, **kwargs)

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--sanitize", action="store_true")
    args = parser.parse_args()
    out.mkdir(parents=True, exist_ok=True)
    common = ["gcc", "-std=c99", "-Wall", "-Wextra", "-Werror"]
    if args.sanitize:
        common += ["-fsanitize=address,undefined", "-fno-omit-frame-pointer"]
    includes = ["-I"+str(tests), "-I"+str(root/"Core/Inc")]
    cases = [("pid", ["test_pid.c"], ["pid.c"], []),
             ("motor", ["test_motor.c"], ["motor.c","pid.c"], []),
             ("encoder", ["test_encoder.c","test_motor.c"], ["motor.c","pid.c"], ["-DMOTOR_TEST_HAL_ONLY"]),
             ("command", ["test_command.c"], ["motor_command.c","pid.c"], []),
             ("uart", ["test_uart.c"], ["motor_uart.c","motor_command.c","pid.c"], ["-I"+str(tests/"uart")])]
    for name, files, sources, extra in cases:
        executable=out/("test_"+name)
        run(common + extra + includes + [tests/f for f in files] + [src/s for s in sources] + ["-lm","-o",executable])
        run([executable])
    if not args.sanitize:
        bridge=out/"protocol_bridge.so"
        run(common+includes+["-shared","-fPIC",tests/"protocol_bridge.c",src/"motor_command.c",src/"pid.c","-lm","-o",bridge])
        env=os.environ.copy()
        env.update(PYTHONPATH=str(repo/"host"), GX_PROTOCOL_BRIDGE=str(bridge))
        run([sys.executable,"-B","-m","unittest","discover","-s",repo/"host/tests","-v"],env=env)
if __name__ == "__main__":
    main()
