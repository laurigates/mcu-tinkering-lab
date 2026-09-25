# /// script
# requires-python = ">=3.11"
# dependencies = [
#     "numpy",
#     "scipy",
#     "sounddevice",
# ]
# ///
"""
Bench test for the ESP32 Kids Audio Toy — hardware-in-the-loop via the mic.

Records the piezo output from the default input device (e.g. the MacBook's
built-in microphone), detects each beep, and reports its fundamental
frequency, duration, and the gap to the next beep. Eyeball the table against
the knob positions you set on the toy.

The firmware plays a PWM square wave whose pitch tracks the pitch pot, with
per-beep duration and inter-beep interval set by the other two pots. The
ranges are read at startup from main/audio_core.h (AUDIO_MIN_FREQ_HZ etc.),
the same header the firmware compiles, and printed with the results. A square wave's fundamental is its
strongest spectral component (harmonics fall off as 1/n), so an argmax over
the expected band recovers the pitch directly.

Usage:
    uv run bench_audio.py                  # record 5 s from default input
    uv run bench_audio.py -d 10            # record 10 s
    uv run bench_audio.py --list-devices   # show input devices and exit
    uv run bench_audio.py --device 2       # pick a specific input device

Acceptance check: set the pitch pot to min / mid / max and confirm the
reported fundamental lands near the minimum / midpoint / maximum of the
firmware pitch range. Repeat the min/mid/max sweep on the duration and
interval pots against the firmware ranges the script prints.
"""

import argparse
import re
import sys
from pathlib import Path

import numpy as np
import sounddevice as sd

AUDIO_CORE_H = Path(__file__).resolve().parent.parent / "main" / "audio_core.h"
RANGE_NAMES = (
    "MIN_FREQ_HZ",
    "MAX_FREQ_HZ",
    "MIN_DURATION_MS",
    "MAX_DURATION_MS",
    "MIN_INTERVAL_MS",
    "MAX_INTERVAL_MS",
)


def load_firmware_ranges(header: Path = AUDIO_CORE_H) -> dict[str, float]:
    """Read the parameter ranges from audio_core.h, the firmware's single
    source of truth, so the bench test checks what the firmware actually
    does rather than a hand-copied duplicate. Fails loudly if any bound is
    missing: a regex that matches nothing must not pass as a clean parse."""
    text = header.read_text()
    ranges = {}
    for name in RANGE_NAMES:
        m = re.search(
            rf"^\s*#define\s+AUDIO_{name}\s+([0-9]+(?:\.[0-9]*)?)f?\b",
            text,
            re.MULTILINE,
        )
        if m is None:
            raise RuntimeError(f"AUDIO_{name} not found in {header}")
        ranges[name] = float(m.group(1))
    return ranges


FIRMWARE = load_firmware_ranges()
MIN_FREQ_HZ = FIRMWARE["MIN_FREQ_HZ"]
MAX_FREQ_HZ = FIRMWARE["MAX_FREQ_HZ"]
# Search band padded slightly beyond the firmware range so a pot at the
# extreme still lands inside the window (80-2200 Hz for 100-2000 Hz).
BAND_LOW_HZ = MIN_FREQ_HZ * 0.8
BAND_HIGH_HZ = MAX_FREQ_HZ * 1.1

SAMPLE_RATE = 44_100
FRAME_MS = 10  # envelope analysis hop
ENVELOPE_THRESHOLD = 0.20  # fraction of peak RMS that counts as "tone on"
ABS_RMS_FLOOR = 0.01  # below this absolute RMS the capture is treated as silence
MIN_BEEP_MS = 30  # ignore on-segments shorter than this (noise)
MIN_GAP_MS = 30  # merge on-segments separated by less than this


def list_devices() -> None:
    print("Input devices:")
    for idx, dev in enumerate(sd.query_devices()):
        if dev["max_input_channels"] > 0:
            marker = " (default)" if idx == sd.default.device[0] else ""
            print(f"  [{idx}] {dev['name']}{marker}")


def record(seconds: float, device: int | None) -> np.ndarray:
    print(
        f"Recording {seconds:.1f}s from "
        f"{'default input' if device is None else f'device {device}'}... "
        "play with the knobs!",
        flush=True,
    )
    frames = int(seconds * SAMPLE_RATE)
    audio = sd.rec(
        frames, samplerate=SAMPLE_RATE, channels=1, dtype="float32", device=device
    )
    sd.wait()
    return audio.flatten()


def envelope_segments(audio: np.ndarray) -> list[tuple[int, int]]:
    """Return (start_sample, end_sample) for each tone-on segment."""
    hop = int(SAMPLE_RATE * FRAME_MS / 1000)
    n_frames = len(audio) // hop
    if n_frames == 0:
        return []

    rms = np.array(
        [np.sqrt(np.mean(audio[i * hop : (i + 1) * hop] ** 2)) for i in range(n_frames)]
    )
    if rms.max() < ABS_RMS_FLOOR:
        return []

    on = rms >= (ENVELOPE_THRESHOLD * rms.max())

    # Collect contiguous runs of "on" frames.
    runs: list[tuple[int, int]] = []
    start = None
    for i, flag in enumerate(on):
        if flag and start is None:
            start = i
        elif not flag and start is not None:
            runs.append((start, i))
            start = None
    if start is not None:
        runs.append((start, n_frames))

    # Merge runs separated by a sub-MIN_GAP_MS gap, then drop tiny runs.
    merged: list[tuple[int, int]] = []
    gap_frames = MIN_GAP_MS / FRAME_MS
    for run in runs:
        if merged and (run[0] - merged[-1][1]) < gap_frames:
            merged[-1] = (merged[-1][0], run[1])
        else:
            merged.append(run)

    min_frames = MIN_BEEP_MS / FRAME_MS
    segments = [(s * hop, e * hop) for s, e in merged if (e - s) >= min_frames]
    return segments


def fundamental_hz(segment: np.ndarray) -> float:
    """Estimate the fundamental frequency of one beep via FFT, with
    parabolic interpolation around the peak bin for sub-bin resolution."""
    if len(segment) < 64:
        return float("nan")

    windowed = segment * np.hanning(len(segment))
    spectrum = np.abs(np.fft.rfft(windowed))
    freqs = np.fft.rfftfreq(len(segment), 1 / SAMPLE_RATE)

    band = (freqs >= BAND_LOW_HZ) & (freqs <= BAND_HIGH_HZ)
    if not band.any():
        return float("nan")

    band_idx = np.where(band)[0]
    local_peak = band_idx[np.argmax(spectrum[band_idx])]

    # Parabolic interpolation on the log-magnitude around the peak bin.
    if 0 < local_peak < len(spectrum) - 1:
        a, b, c = (
            np.log(spectrum[local_peak - 1] + 1e-12),
            np.log(spectrum[local_peak] + 1e-12),
            np.log(spectrum[local_peak + 1] + 1e-12),
        )
        denom = a - 2 * b + c
        offset = 0.5 * (a - c) / denom if denom != 0 else 0.0
    else:
        offset = 0.0

    bin_width = freqs[1] - freqs[0]
    return float(freqs[local_peak] + offset * bin_width)


def analyze(audio: np.ndarray) -> None:
    segments = envelope_segments(audio)
    if not segments:
        print(
            "\nNo beeps detected. Is the toy running and the piezo near "
            "the mic? Try raising the volume or lowering ENVELOPE_THRESHOLD."
        )
        return

    print(f"\nDetected {len(segments)} beep(s):\n")
    print(
        f"  {'#':>3}  {'start (s)':>9}  {'pitch (Hz)':>10}  "
        f"{'duration (ms)':>13}  {'gap (ms)':>9}"
    )
    print(f"  {'-' * 3}  {'-' * 9}  {'-' * 10}  {'-' * 13}  {'-' * 9}")

    pitches = []
    for i, (start, end) in enumerate(segments):
        freq = fundamental_hz(audio[start:end])
        dur_ms = (end - start) / SAMPLE_RATE * 1000
        if i + 1 < len(segments):
            gap_ms = (segments[i + 1][0] - end) / SAMPLE_RATE * 1000
            gap_str = f"{gap_ms:9.0f}"
        else:
            gap_str = f"{'—':>9}"
        pitches.append(freq)
        print(
            f"  {i:>3}  {start / SAMPLE_RATE:>9.2f}  {freq:>10.0f}  "
            f"{dur_ms:>13.0f}  {gap_str}"
        )

    valid = [p for p in pitches if not np.isnan(p)]
    if valid:
        print(
            f"\n  pitch range: {min(valid):.0f}-{max(valid):.0f} Hz "
            f"(firmware: {MIN_FREQ_HZ:.0f}-{MAX_FREQ_HZ:.0f} Hz)"
        )
    print(
        f"  firmware duration: {FIRMWARE['MIN_DURATION_MS']:.0f}-"
        f"{FIRMWARE['MAX_DURATION_MS']:.0f} ms, interval: "
        f"{FIRMWARE['MIN_INTERVAL_MS']:.0f}-{FIRMWARE['MAX_INTERVAL_MS']:.0f} ms "
        f"(from {AUDIO_CORE_H.name})"
    )
    out_of_band = [p for p in valid if p < MIN_FREQ_HZ - 20 or p > MAX_FREQ_HZ + 20]
    if out_of_band:
        print(
            f"  WARNING: {len(out_of_band)} beep(s) outside the firmware "
            "range — possible harmonic mis-pick or noise."
        )


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Mic bench test for the ESP32 Kids Audio Toy."
    )
    parser.add_argument(
        "-d",
        "--duration",
        type=float,
        default=5.0,
        help="record length in seconds (default: 5)",
    )
    parser.add_argument(
        "--device",
        type=int,
        default=None,
        help="input device index (default: system default)",
    )
    parser.add_argument(
        "--list-devices", action="store_true", help="list input devices and exit"
    )
    args = parser.parse_args()

    if args.list_devices:
        list_devices()
        return 0

    try:
        audio = record(args.duration, args.device)
    except Exception as exc:  # noqa: BLE001 — surface any audio-stack error plainly
        print(f"Recording failed: {exc}", file=sys.stderr)
        print(
            "On macOS, grant terminal mic access in "
            "System Settings > Privacy & Security > Microphone.",
            file=sys.stderr,
        )
        return 1

    peak = float(np.abs(audio).max())
    print(f"Captured {len(audio) / SAMPLE_RATE:.1f}s, peak amplitude {peak:.3f}")
    if peak < 0.01:
        print("Signal is very quiet — move the mic closer to the piezo.")

    analyze(audio)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
