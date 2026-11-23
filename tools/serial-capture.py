#!/usr/bin/env python3
"""Record a board's CSV telemetry stream to a file.

Usage: serial-capture.py PORT SECONDS OUT [--hz HZ]

Sends `stream on HZ`, keeps the header line (starting with --header-prefix)
and every row with as many numeric fields as the header for SECONDS, then
sends `stream off`. Other lines (CLI replies, boot notes) go to stderr. Needs
pyserial (run through `uvx --from pyserial`).
"""

import argparse
import sys
import time
from pathlib import Path

import serial


def is_row(line: str, ncols: int) -> bool:
    fields = line.split(",")
    if len(fields) != ncols:
        return False
    try:
        for f in fields:
            float(f)
    except ValueError:
        return False
    return True


def main() -> int:
    ap = argparse.ArgumentParser(description="Record a CSV telemetry stream.")
    ap.add_argument("port")
    ap.add_argument("seconds", type=float)
    ap.add_argument("out", type=Path)
    ap.add_argument("--hz", type=int, default=100, help="stream rate to request")
    ap.add_argument(
        "--header-prefix", default="t_ms,", help="how the header line starts"
    )
    args = ap.parse_args()

    try:
        port = serial.Serial(args.port, 115200, timeout=0.05)
    except serial.SerialException as e:
        print(f"ERROR: {e}", file=sys.stderr)
        return 1

    header = None
    rows: list[str] = []
    with port:
        time.sleep(0.05)
        port.reset_input_buffer()
        port.write(f"stream on {args.hz}\n".encode())
        buf = b""
        end = time.monotonic() + args.seconds
        while time.monotonic() < end:
            buf += port.read(port.in_waiting or 1)
            *lines, buf = buf.split(b"\n")
            for raw in lines:
                line = raw.decode(errors="replace").strip()
                if not line:
                    continue
                if line.startswith(args.header_prefix):
                    header = line
                elif header is not None and is_row(line, header.count(",") + 1):
                    rows.append(line)
                else:
                    print(line, file=sys.stderr)
        port.write(b"stream off\n")
        port.flush()

    if header is None:
        print(
            "ERROR: no telemetry header seen; is the firmware running?", file=sys.stderr
        )
        return 1
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(header + "\n" + "".join(r + "\n" for r in rows))
    print(f"{len(rows)} rows in {args.seconds:g} s -> {args.out}")
    return 0 if rows else 1


if __name__ == "__main__":
    sys.exit(main())
