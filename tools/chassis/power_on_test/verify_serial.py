"""Reset and check A15 command sequencing ONLY with actuator power disconnected.

This intentionally restarts the automatic firmware. It cannot verify wiring,
physical wheel direction, servo angles, or whether actuator power is really off.
Requires pyserial. Never use this as a powered movement test.
"""
import argparse
from datetime import datetime
from pathlib import Path
import re
import time

import serial


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", required=True)
    parser.add_argument("--power-off-confirmed", action="store_true", required=True)
    parser.add_argument("--mode", choices=("full", "abort-run", "abort-countdown"), default="full")
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    lines = []
    abort_sent = False
    try:
        with serial.Serial(port=None, baudrate=115200, timeout=0.1) as connection:
            connection.dtr = False
            connection.rts = False
            connection.port = args.port
            connection.open()
            # EN low while GPIO0 is high, then release into the application.
            connection.rts = True
            time.sleep(0.15)
            connection.reset_input_buffer()
            connection.rts = False
            deadline = time.monotonic() + (14 if args.mode == "full" else 9)
            while time.monotonic() < deadline:
                line = connection.readline().decode("utf-8", errors="replace").strip()
                if line:
                    lines.append(line)
                    print(line, flush=True)
                trigger = "REAR MOTORS RUN:" if args.mode == "abort-run" else "COUNTDOWN:"
                if args.mode != "full" and trigger in line and not abort_sent:
                    connection.write(b"0")  # No newline: verify immediate cancellation.
                    connection.flush()
                    abort_sent = True
                    lines.append("HOST: sent single byte 0 without newline")
    except Exception as exc:
        lines.append(f"HOST ERROR: {type(exc).__name__}: {exc}")
        passed = False
    else:
        joined = "\n".join(lines)
        passed = joined.count("A15 v1.2 ONE SHOT") == 1 and "FAULT:" not in joined
        if args.mode == "full":
            markers = ("COUNTDOWN:", "REAR MOTORS RUN:", "MOTOR STOP command;",
                       "MOTOR INPUTS LOW.", "INITIAL CENTER command", "LEFT command",
                       "CENTER command.", "RIGHT command", "DONE: one shot finished")
            positions = [joined.find(marker) for marker in markers]
            passed = passed and all(position >= 0 for position in positions)
            passed = passed and positions == sorted(positions)
            passed = passed and all(joined.count(marker) == 1 for marker in markers)
            interval = re.search(r"drive interval=(\d+) ms", joined)
            passed = passed and interval is not None and 2000 <= int(interval[1]) <= 2010
            commands = re.findall(r"OUTPUT command=(\d+) us", joined)
            passed = passed and commands == ["1500", "1200", "1500", "1800"]
        else:
            passed = passed and abort_sent and joined.count("ABORTED: all outputs LOW") == 1
            passed = passed and "INITIAL CENTER command" not in joined and "DONE:" not in joined
            passed = passed and joined.count("REAR MOTORS RUN:") == (args.mode == "abort-run")
    result = f"{'PASS' if passed else 'FAIL'}: {args.mode}; firmware commands only; actuator power OFF"
    print(result, flush=True)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(
        f"Checked {datetime.now().astimezone().isoformat()} port={args.port}\n"
        "User confirmed USB connected and actuator power OFF. EN reset sent by host.\n"
        "No physical motion or electrical waveform measurement is claimed.\n"
        + "\n".join(lines) + "\n" + result + "\n", encoding="utf-8")
    return 0 if passed else 1


if __name__ == "__main__":
    raise SystemExit(main())
