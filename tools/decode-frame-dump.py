#!/usr/bin/env -S uv run --quiet --script
# /// script
# requires-python = ">=3.11"
# dependencies = ["pillow"]
# ///
"""Recover camera frames from a robocar-unified serial log.

The firmware (packages/robocar/unified/main/frame_dump.c) emits the exact JPEG
it hands to Gemini, base64-framed, interleaved with ordinary ESP_LOG output:

    SNAPBEGIN seq=1 len=11423 crc=5276e4c6 w=320 h=240 gain=62 exp=1248 ceil=0
    SNAP /9j/4AAQSkZJRgABAQAAAQABAAD/2wBDAAgGBgcGBQgHBwcJCQgKDBQNDAsL...
    ...
    SNAPEND seq=1 lines=238

Usage:
    ./decode-frame-dump.py <logfile> <outdir>

Writes <outdir>/frame-<seq>.jpg per recovered frame and prints luma statistics
so the "is it actually black?" question is answered numerically, before anyone
opens an image.

The CRC check is load-bearing, not decoration: the ESP32's USB-Serial-JTAG
console silently DROPS transmit bytes once the host stops reading for 50 ms
(TX_FLUSH_TIMEOUT_US in esp_driver_usb_serial_jtag) and reports success anyway.
A truncated frame decodes into a plausible-looking corrupt image, which is
exactly the false "the camera is broken" conclusion this tool exists to avoid.
"""

from __future__ import annotations

import base64
import binascii
import re
import sys
from dataclasses import dataclass, field
from pathlib import Path

BEGIN_RE = re.compile(
    r"SNAPBEGIN\s+seq=(\d+)\s+len=(\d+)\s+crc=([0-9a-fA-F]{8})"
    r"(?:\s+w=(\d+))?(?:\s+h=(\d+))?"
    r"(?:\s+gain=(\d+))?(?:\s+exp=(\d+))?(?:\s+ceil=(\d+))?"
)
LINE_RE = re.compile(r"SNAP ([A-Za-z0-9+/=]+)\s*$")
END_RE = re.compile(r"SNAPEND\s+seq=(\d+)\s+lines=(\d+)")


@dataclass
class Frame:
    seq: int
    declared_len: int
    declared_crc: int
    width: int | None = None
    height: int | None = None
    gain: int | None = None
    exposure: int | None = None
    gainceiling: int | None = None
    chunks: list[str] = field(default_factory=list)


def parse(log: str) -> list[Frame]:
    """Extract frames, tolerating ESP_LOG lines interleaved between SNAP lines."""
    frames: list[Frame] = []
    current: Frame | None = None

    for line in log.splitlines():
        if (m := BEGIN_RE.search(line)) is not None:
            if current is not None:
                # A new frame started before the previous one closed: its
                # SNAPEND was dropped on the wire. Say so — silently discarding
                # it would under-report exactly the corruption we are watching
                # for.
                print(
                    f"warning: frame seq={current.seq} was truncated (no SNAPEND "
                    f"before seq={m.group(1)} started) — discarded",
                    file=sys.stderr,
                )
            current = Frame(
                seq=int(m.group(1)),
                declared_len=int(m.group(2)),
                declared_crc=int(m.group(3), 16),
                width=int(m.group(4)) if m.group(4) else None,
                height=int(m.group(5)) if m.group(5) else None,
                gain=int(m.group(6)) if m.group(6) else None,
                exposure=int(m.group(7)) if m.group(7) else None,
                gainceiling=int(m.group(8)) if m.group(8) else None,
            )
            continue

        if current is None:
            continue

        if (m := LINE_RE.search(line)) is not None:
            current.chunks.append(m.group(1))
        elif END_RE.search(line) is not None:
            frames.append(current)
            current = None

    if current is not None:
        print(
            f"warning: frame seq={current.seq} has no SNAPEND — capture was cut short",
            file=sys.stderr,
        )
    return frames


def luma_stats(jpeg: bytes) -> str:
    """Percentile luma. A low mean with a high p95 is backlit, not unlit."""
    try:
        import io

        from PIL import Image, UnidentifiedImageError
    except ImportError:
        return "install pillow for luma stats"

    try:
        with Image.open(io.BytesIO(jpeg)) as im:
            values = sorted(im.convert("L").tobytes())
    except (UnidentifiedImageError, OSError) as exc:
        # The CRC passed but the bytes are not a decodable JPEG. That is itself
        # a finding — the camera produced a malformed frame — so report it
        # rather than dying, and keep processing the remaining frames.
        return f"UNDECODABLE ({type(exc).__name__}: {exc})"

    if not values:
        return "empty image"

    def pct(p: float) -> int:
        return values[min(len(values) - 1, int(len(values) * p))]

    mean = sum(values) / len(values)
    return (
        f"mean={mean:6.1f} p5={pct(0.05):3d} p50={pct(0.50):3d} "
        f"p95={pct(0.95):3d} max={values[-1]:3d}"
    )


def main() -> int:
    if len(sys.argv) != 3:
        print(__doc__)
        return 2

    log_path, out_dir = Path(sys.argv[1]), Path(sys.argv[2])
    if not log_path.is_file():
        print(f"error: no such log file: {log_path}", file=sys.stderr)
        return 1
    out_dir.mkdir(parents=True, exist_ok=True)

    frames = parse(log_path.read_text(errors="replace"))
    if not frames:
        print(
            "No frames found. The firmware dumps the first "
            "FRAME_DUMP_BOOT_FRAMES frames after boot — attach the monitor "
            "before resetting, or type 'snap 3' if your terminal can write to "
            "the port.",
            file=sys.stderr,
        )
        return 1

    ok = 0
    for f in frames:
        try:
            payload = base64.b64decode("".join(f.chunks))
        except (binascii.Error, ValueError) as exc:
            # Dropped console bytes break base64 quartet alignment. Report it
            # like any other corruption and keep going — letting this propagate
            # would abandon every healthy frame later in the same capture.
            print(
                f"frame {f.seq}: CORRUPT — undecodable base64 ({exc}). Re-run the capture."
            )
            continue

        crc = binascii.crc32(payload) & 0xFFFFFFFF

        if len(payload) != f.declared_len or crc != f.declared_crc:
            print(
                f"frame {f.seq}: CORRUPT — got {len(payload)} B / crc {crc:08x}, "
                f"expected {f.declared_len} B / crc {f.declared_crc:08x}. "
                "The console dropped bytes; re-run the capture."
            )
            continue

        path = out_dir / f"frame-{f.seq:03d}.jpg"
        path.write_bytes(payload)
        ok += 1

        exposure = ""
        if f.gain is not None:
            ceiling = (
                f"{1 << (f.gainceiling + 1)}x" if f.gainceiling is not None else "?"
            )
            exposure = f" | gain={f.gain} exp={f.exposure} ceiling={ceiling}"
        print(
            f"frame {f.seq}: {len(payload):6d} B  {luma_stats(payload)}{exposure}  -> {path}"
        )

    print(f"\n{ok}/{len(frames)} frame(s) recovered into {out_dir}")
    if ok:
        print(
            "\nReading the numbers:\n"
            "  low mean + gain pegged at the ceiling + high exposure -> sensor is\n"
            "      starved; raise the ceiling ('cam gainceiling 3' and up)\n"
            "  low mean + low gain + low exposure                    -> the sensor is not\n"
            "      trying; auto-exposure is not converging\n"
            "  low mean + high p95                                   -> backlit, not unlit\n"
            "  mean above ~60 with structure                         -> the frames are fine;\n"
            "      the darkness remark is coming from somewhere else"
        )
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
