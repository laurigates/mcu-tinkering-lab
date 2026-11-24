#!/usr/bin/env python3
"""Record a board's CSV telemetry stream to a file.

Usage: serial-capture.py PORT SECONDS OUT [--hz HZ]
       serial-capture.py PORT SECONDS OUT --send "log dump" --until "log end"

Sends `stream on HZ`, keeps the header line (starting with --header-prefix)
and every row with as many numeric fields as the header for SECONDS, then
sends `stream off`. With --send it sends that command instead and reads until
a line starting with --until, failing if none arrives within SECONDS (for a
dump the board prints once, such as balancebot's `log dump`). Other lines (CLI
replies, boot notes) go to stderr. Needs pyserial (run through
`uvx --from pyserial`).
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
    ap.add_argument(
        "--send", help="command to send instead of `stream on`; no `stream off` after"
    )
    ap.add_argument("--until", help="stop at a line starting with this (needs --send)")
    args = ap.parse_args()
    if args.until and not args.send:
        ap.error("--until needs --send")

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
        command = args.send if args.send else f"stream on {args.hz}"
        port.write(command.encode() + b"\n")
        buf = b""
        end = time.monotonic() + args.seconds
        done = False
        while not done and time.monotonic() < end:
            buf += port.read(port.in_waiting or 1)
            *lines, buf = buf.split(b"\n")
            for raw in lines:
                line = raw.decode(errors="replace").strip()
                if not line:
                    continue
                if args.until and line.startswith(args.until):
                    done = True
                    break
                if line.startswith(args.header_prefix):
                    header = line
                elif header is not None and is_row(line, header.count(",") + 1):
                    rows.append(line)
                else:
                    print(line, file=sys.stderr)
        if not args.send:
            port.write(b"stream off\n")
        port.flush()

    if header is None:
        print(
            "ERROR: no telemetry header seen; is the firmware running?", file=sys.stderr
        )
        return 1
    if args.until and not done:
        print(
            f"ERROR: no '{args.until}' line within {args.seconds:g} s; "
            f"{len(rows)} rows read, not saved",
            file=sys.stderr,
        )
        return 1
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(header + "\n" + "".join(r + "\n" for r in rows))
    print(f"{len(rows)} rows in {args.seconds:g} s -> {args.out}")
    return 0 if rows else 1


if __name__ == "__main__":
    sys.exit(main())
