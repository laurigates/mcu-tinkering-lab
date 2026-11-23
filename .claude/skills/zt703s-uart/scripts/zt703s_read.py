#!/usr/bin/env -S uv run --script
# /// script
# requires-python = ">=3.12"
# dependencies = ["pyserial>=3.5"]
# ///
"""Read multimeter records from a ZOYI ZT-703S's UART output.

The meter sends one ASCII record about three times a second, at 115200 8N1,
with no CR or LF between records (EEVblog ZT-70xS thread, reply #139). This
reader treats a quiet gap on the line as the end of a record, and also splits
a chunk that holds several records at each ``Label:``.

Usage:
    zt703s_read.py [--port /dev/cu.usbserial-XXXX] [--csv out.csv] [--raw]
"""

from __future__ import annotations

import argparse
import csv
import re
import sys
import time
from dataclasses import dataclass

import serial
from serial.tools import list_ports

BAUD = 115200
# A record is ~30 bytes, about 3 ms on the wire; records are ~333 ms apart.
GAP_S = 0.05
RECORD_START = re.compile(r"(?<![A-Za-z])(?=[A-Za-z]+:)")
RECORD = re.compile(r"^(?P<label>[A-Za-z]+):\s*(?P<value>\S*)\s*(?P<unit>\S*)\s*$")


@dataclass(frozen=True)
class Reading:
    label: str
    value: float | None  # None for overload ("0.L") or a blank display
    unit: str
    raw: str


def split_records(text: str) -> list[str]:
    """Split text that may hold several concatenated records."""
    return [part.strip() for part in RECORD_START.split(text) if part.strip()]


def parse_record(record: str) -> Reading | None:
    """Parse one record such as ``Voltage:-23.211 mV``; None if unrecognised."""
    match = RECORD.match(record)
    if not match:
        return None
    try:
        value: float | None = float(match["value"])
    except ValueError:
        value = None
    return Reading(match["label"], value, match["unit"], record)


def find_port() -> str:
    """Return the only USB serial adapter attached, or exit with the candidates."""
    ports = [p.device for p in list_ports.comports() if p.vid is not None]
    ports = [p for p in ports if not p.startswith("/dev/tty.")]  # macOS: cu. and tty.
    if len(ports) == 1:
        return ports[0]
    listing = "\n  ".join(ports) or "(none)"
    sys.exit(f"Pass --port; USB serial ports found:\n  {listing}")


def read_chunks(port: serial.Serial):
    """Yield the bytes received between quiet gaps on the line."""
    buf = bytearray()
    while True:
        data = port.read(256)
        if data:
            buf += data
        elif buf:
            yield bytes(buf)
            buf.clear()


def main() -> None:
    parser = argparse.ArgumentParser(description="Read a ZOYI ZT-703S UART output.")
    parser.add_argument("--port", help="serial device; default: the only USB adapter")
    parser.add_argument("--baud", type=int, default=BAUD)
    parser.add_argument("--csv", help="also append readings to this CSV file")
    parser.add_argument(
        "--raw", action="store_true", help="print each chunk as hex and text"
    )
    args = parser.parse_args()

    device = args.port or find_port()
    writer = None
    csv_file = None
    if args.csv:
        csv_file = open(args.csv, "a", newline="")  # noqa: SIM115 - closed in finally
        writer = csv.writer(csv_file)
        if csv_file.tell() == 0:
            writer.writerow(["time", "label", "value", "unit", "raw"])

    print(f"Reading {device} at {args.baud} 8N1; Ctrl-C to stop", file=sys.stderr)
    try:
        with serial.Serial(device, args.baud, timeout=GAP_S) as port:
            for chunk in read_chunks(port):
                stamp = time.strftime("%Y-%m-%dT%H:%M:%S")
                text = chunk.decode("ascii", errors="replace")
                if args.raw:
                    print(f"{stamp} {chunk.hex(' ')}  {text!r}")
                    continue
                for record in split_records(text):
                    reading = parse_record(record)
                    if reading is None:
                        print(f"{stamp} ? {record!r}", file=sys.stderr)
                        continue
                    shown = "OL" if reading.value is None else reading.value
                    print(
                        f"{stamp} {reading.label:<12} {shown} {reading.unit}",
                        flush=True,
                    )
                    if writer and csv_file:
                        writer.writerow(
                            [
                                stamp,
                                reading.label,
                                reading.value,
                                reading.unit,
                                reading.raw,
                            ]
                        )
                        csv_file.flush()
    except KeyboardInterrupt:
        pass
    finally:
        if csv_file:
            csv_file.close()


if __name__ == "__main__":
    main()
