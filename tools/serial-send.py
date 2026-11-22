#!/usr/bin/env python3
"""Send CLI lines to a USB CDC serial console and print what comes back.

Usage: serial-send.py PORT LINE [LINE ...]

Each LINE is sent with a trailing LF. After each one the reply is read until
the port has been quiet for --quiet seconds, or --max seconds have passed, so
a board streaming telemetry still returns. Needs pyserial (run through
`uvx --from pyserial`).
"""

import argparse
import sys
import time

import serial


def read_reply(port: serial.Serial, quiet_s: float, max_s: float) -> bytes:
    """Read until the port is quiet for quiet_s, or max_s has passed."""
    out = bytearray()
    start = last = time.monotonic()
    while True:
        now = time.monotonic()
        if now - last >= quiet_s or now - start >= max_s:
            return bytes(out)
        chunk = port.read(port.in_waiting or 1)
        if chunk:
            out += chunk
            last = time.monotonic()


def main() -> int:
    ap = argparse.ArgumentParser(
        description="Send CLI lines to a USB CDC serial console."
    )
    ap.add_argument("port")
    ap.add_argument("lines", nargs="+")
    ap.add_argument("--baud", type=int, default=115200, help="ignored by USB CDC")
    ap.add_argument(
        "--quiet", type=float, default=0.4, help="seconds of silence ending a reply"
    )
    ap.add_argument("--max", type=float, default=3.0, help="longest wait for one reply")
    args = ap.parse_args()

    try:
        port = serial.Serial(args.port, args.baud, timeout=0.05)
    except serial.SerialException as e:
        print(f"ERROR: {e}", file=sys.stderr)
        return 1
    with port:
        # Drop whatever the board printed before we connected.
        time.sleep(0.05)
        port.reset_input_buffer()
        for line in args.lines:
            print(f"> {line}")
            port.write(line.encode() + b"\n")
            port.flush()
            reply = read_reply(port, args.quiet, args.max)
            sys.stdout.write(reply.decode(errors="replace"))
            if reply and not reply.endswith(b"\n"):
                sys.stdout.write("\n")
            sys.stdout.flush()
    return 0


if __name__ == "__main__":
    sys.exit(main())
