#!/usr/bin/env -S uv run --quiet --script
# /// script
# requires-python = ">=3.11"
# dependencies = []
# ///
"""Recover microphone frames from a robocar-unified serial log.

The firmware (packages/robocar/unified/main/mic_dump.c) emits the exact PCM the
ambient gate measured, base64-framed, interleaved with ordinary ESP_LOG output:

    MICBEGIN seq=1 len=2048 crc=5276e4c6 rate=16000 bits=16 ch=1
    MIC AAAAAP//AQAAAP7/AwAAAP//AgD+/wEAAAD//wIA/v8BAAAA//8CAP7/AQAA...
    ...
    MICEND seq=1 lines=43

Usage:
    ./decode-mic-dump.py <logfile> <outdir>

Writes <outdir>/mic-<seq>.wav per recovered frame and prints level statistics,
so "is the microphone actually working?" is answered numerically before anyone
plays a file. That ordering is the entire point: a loudness threshold set over a
sensor nobody has listened to is guesswork, and it presents itself as a tuning
problem rather than as a dead microphone.

The CRC check is load-bearing, not decoration: the ESP32's USB-Serial-JTAG
console silently DROPS transmit bytes once the host stops reading for 50 ms
(TX_FLUSH_TIMEOUT_US in esp_driver_usb_serial_jtag) and reports success anyway.
A truncated frame decodes into audio that is short and torn — which sounds
exactly like a broken microphone, the one conclusion this tool exists to avoid
fabricating.
"""

from __future__ import annotations

import base64
import binascii
import re
import struct
import sys
from pathlib import Path

BEGIN_RE = re.compile(
    r"MICBEGIN\s+seq=(\d+)\s+len=(\d+)\s+crc=([0-9a-fA-F]{8})"
    r"\s+rate=(\d+)\s+bits=(\d+)\s+ch=(\d+)"
)
END_RE = re.compile(r"MICEND\s+seq=(\d+)\s+lines=(\d+)")
DATA_RE = re.compile(r"^\s*MIC ([A-Za-z0-9+/=]+)\s*$")


def wav_bytes(pcm: bytes, rate: int, bits: int, channels: int) -> bytes:
    """Wrap raw little-endian PCM in a canonical 44-byte WAV header."""
    block_align = channels * bits // 8
    return (
        b"RIFF"
        + struct.pack("<I", 36 + len(pcm))
        + b"WAVEfmt "
        + struct.pack(
            "<IHHIIHH", 16, 1, channels, rate, rate * block_align, block_align, bits
        )
        + b"data"
        + struct.pack("<I", len(pcm))
        + pcm
    )


def stats(pcm: bytes) -> str:
    """Level statistics. These, not the audio, are what the tool is for."""
    n = len(pcm) // 2
    if n == 0:
        return "empty"
    samples = struct.unpack(f"<{n}h", pcm[: n * 2])
    peak = max(abs(s) for s in samples)
    mean = sum(samples) / n
    # Sum of squares in ints: float accumulation over 16-bit samples is fine
    # either way at this length, but ints make the result exactly reproducible.
    rms = (sum(s * s for s in samples) / n) ** 0.5
    clipped = sum(1 for s in samples if s >= 32767 or s <= -32768)

    def dbfs(v: float) -> str:
        if v <= 0:
            return "-inf"
        import math

        return f"{20 * math.log10(v / 32768.0):6.1f}"

    return (
        f"{n:5d} samples  peak {peak:6d} ({dbfs(peak)} dBFS)  "
        f"rms {rms:8.1f} ({dbfs(rms)} dBFS)  dc {mean:+8.1f}  clipped {clipped}"
    )


def main() -> int:
    if len(sys.argv) != 3:
        print(__doc__)
        return 2
    logfile, outdir = Path(sys.argv[1]), Path(sys.argv[2])
    outdir.mkdir(parents=True, exist_ok=True)

    recovered = failed = 0
    pending: dict | None = None

    for line in logfile.read_text(errors="replace").splitlines():
        if (m := BEGIN_RE.search(line)) is not None:
            pending = {
                "seq": int(m.group(1)),
                "len": int(m.group(2)),
                "crc": int(m.group(3), 16),
                "rate": int(m.group(4)),
                "bits": int(m.group(5)),
                "ch": int(m.group(6)),
                "chunks": [],
            }
            continue
        if pending is None:
            continue
        if (m := DATA_RE.match(line)) is not None:
            pending["chunks"].append(m.group(1))
            continue
        if (m := END_RE.search(line)) is not None:
            seq = pending["seq"]
            try:
                pcm = base64.b64decode("".join(pending["chunks"]), validate=True)
            except binascii.Error as exc:
                print(f"frame {seq}: base64 decode failed ({exc}) — DISCARDED")
                failed += 1
                pending = None
                continue

            # Both checks, and neither is redundant: a dropped whole line changes
            # the length, while a corrupted line inside a chunk does not.
            if len(pcm) != pending["len"]:
                print(
                    f"frame {seq}: length {len(pcm)} != declared {pending['len']} "
                    "— console dropped bytes, DISCARDED"
                )
                failed += 1
                pending = None
                continue
            crc = binascii.crc32(pcm) & 0xFFFFFFFF
            if crc != pending["crc"]:
                print(
                    f"frame {seq}: crc {crc:08x} != declared {pending['crc']:08x} "
                    "— corrupt, DISCARDED"
                )
                failed += 1
                pending = None
                continue

            path = outdir / f"mic-{seq}.wav"
            path.write_bytes(
                wav_bytes(pcm, pending["rate"], pending["bits"], pending["ch"])
            )
            print(f"frame {seq}: {stats(pcm)}  -> {path}")
            recovered += 1
            pending = None

    print(f"\n{recovered} frame(s) recovered, {failed} discarded")
    if recovered == 0:
        print(
            "No frames recovered. The read-only monitor cannot send `mic dump`, so "
            "check that MIC_DUMP_BOOT_FRAMES is non-zero and that the log covers "
            "boot — or use screen/picocom to type the command."
        )
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
