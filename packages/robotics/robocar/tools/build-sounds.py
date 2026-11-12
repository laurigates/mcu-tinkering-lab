# /// script
# requires-python = ">=3.11"
# dependencies = ["numpy"]
# ///
"""Turn Suno sound downloads into the firmware's embedded clips.

    uv run tools/build-sounds.py <raw dir> [--preview <dir>]

Reads sounds/manifest.toml. For each entry, loads <raw dir>/<suno_id>.wav,
cuts [start_s, end_s], mixes to mono, resamples 48 kHz -> 24 kHz, fades both
ends, peak-normalises to peak_dbfs and writes main/sounds/<tag>.pcm as raw
int16 little-endian: exactly what audio_player_write() takes, so the firmware
copies it into the ring without decoding.

--preview also writes <dir>/<tag>.wav (the same samples with a WAV header) so
the result can be heard on a workstation before flashing.
"""

import argparse
import sys
import tomllib
import wave
from pathlib import Path

import numpy as np

OUT_RATE = 24000  # AUDIO_SAMPLE_RATE_HZ in main/pin_config.h
FADE_IN_MS = 5  # removes the click of a cut that lands mid-waveform
PROJECT = Path(__file__).resolve().parent.parent


def load_wav(path: Path) -> tuple[int, np.ndarray]:
    """Return (rate, mono float samples in [-1, 1])."""
    with wave.open(str(path)) as w:
        if w.getsampwidth() != 2:
            sys.exit(f"{path}: expected 16-bit PCM, got {w.getsampwidth() * 8}-bit")
        rate, ch = w.getframerate(), w.getnchannels()
        x = np.frombuffer(w.readframes(w.getnframes()), dtype="<i2").astype(np.float64)
    return rate, x.reshape(-1, ch).mean(axis=1) / 32768.0


def resample(x: np.ndarray, rate: int) -> np.ndarray:
    """Integer-ratio downsample with a windowed-sinc low-pass ahead of it."""
    if rate == OUT_RATE:
        return x
    if rate % OUT_RATE:
        sys.exit(f"source rate {rate} Hz is not a multiple of {OUT_RATE} Hz")
    factor = rate // OUT_RATE
    taps = 129
    # Cycles/sample: 10.8 kHz at 48 kHz, under the 12 kHz output Nyquist.
    cutoff = 0.45 / factor
    n = np.arange(taps) - (taps - 1) / 2
    h = 2 * cutoff * np.sinc(2 * cutoff * n) * np.blackman(taps)
    h /= h.sum()
    return np.convolve(x, h, mode="same")[::factor]


def build(entry: dict, raw_dir: Path) -> np.ndarray:
    rate, x = load_wav(raw_dir / f"{entry['suno_id']}.wav")
    start, end = int(entry["start_s"] * rate), int(entry["end_s"] * rate)
    if not 0 <= start < end <= len(x):
        sys.exit(
            f"{entry['tag']}: trim {entry['start_s']}-{entry['end_s']} s outside the "
            f"{len(x) / rate:.2f} s source"
        )
    y = resample(x[start:end], rate)

    fade_in = int(OUT_RATE * FADE_IN_MS / 1000)
    fade_out = int(OUT_RATE * entry["fade_out_ms"] / 1000)
    y[:fade_in] *= np.linspace(0.0, 1.0, fade_in)
    y[len(y) - fade_out :] *= np.linspace(1.0, 0.0, fade_out)

    peak = np.abs(y).max()
    if peak == 0:
        sys.exit(f"{entry['tag']}: the trimmed slice is silent")
    y *= 10 ** (entry["peak_dbfs"] / 20) / peak
    return np.round(y * 32767).astype("<i2")


def main() -> None:
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    ap.add_argument(
        "raw_dir", type=Path, help="directory holding <suno_id>.wav downloads"
    )
    ap.add_argument(
        "--preview", type=Path, help="also write <tag>.wav here for listening"
    )
    args = ap.parse_args()

    manifest = tomllib.loads((PROJECT / "sounds" / "manifest.toml").read_text())
    out_dir = PROJECT / "main" / "sounds"
    out_dir.mkdir(exist_ok=True)
    if args.preview:
        args.preview.mkdir(parents=True, exist_ok=True)

    for entry in manifest["sound"]:
        pcm = build(entry, args.raw_dir)
        (out_dir / f"{entry['tag']}.pcm").write_bytes(pcm.tobytes())
        if args.preview:
            with wave.open(str(args.preview / f"{entry['tag']}.wav"), "wb") as w:
                w.setnchannels(1)
                w.setsampwidth(2)
                w.setframerate(OUT_RATE)
                w.writeframes(pcm.tobytes())
        print(
            f"{entry['tag']:8s} {len(pcm) / OUT_RATE * 1000:5.0f} ms {len(pcm) * 2:6d} bytes"
        )


if __name__ == "__main__":
    main()
