"""Audition retro-robot DSP chains on a rendered TTS clip, before writing firmware.

The style sweep (make-style-sweep.py) established what prompting can and cannot
do: it moves pace and timbre, and it does not touch the recording medium. What
is left is a filter question, and a filter question is cheap to answer on a
workstation — so this renders candidate chains over an existing WAV rather than
flashing a board per candidate.

The chains deliberately SPAN the retro-robot range rather than converging on
one, because the target is a taste call that cannot be made in the abstract:

    00-dry        the input, untouched (control)
    01-band       period medium only — no machine at all
    02-band-sat   medium plus the saturation of an overdriven period chain
    03-comb       machine only — resonance of a metal head, full bandwidth
    04-retro      medium + machine, the composite candidate
    05-ringlite   04 plus a hint of ring modulation
    06-dalek      full 45 Hz ring modulation, as the far end of the axis

C-3PO sits nearer 00-01 (performance carries it), Robby the Robot nearer 04,
the Dalek at 06. Listening across the span is the only way to locate Teuvo.

Everything here is written as plain per-sample float loops, NOT vectorised.
That is the point: this is a prototype of `voice_fx_core.c`, so the arithmetic
should port to C literally rather than being re-derived from a numpy
expression. Deliberate divergences from the briefing's reference code, each of
which is a bug there:

  * The comb trims its input by (1-g). A feedback comb has DC gain 1/(1-g) —
    5.6x at g=0.82 — so without the trim the "resonator" is a clipper.
  * Ring modulation is a wet/dry blend, not a bare multiply, so depth can be
    dialled toward zero. A bare multiply is only ever the 06 end of the axis.
  * No bitcrusher and no impulse-train drone. Quantisation noise is a 1980s
    artifact with no place in a 1950s medium, and pitch flatness is already
    better controlled from the prompt than by an envelope-triggered buzz.

Usage:
    python3 voice-fx-preview.py <input.wav> <out_dir>
"""

from __future__ import annotations

import array
import math
import random
import struct
import sys
import wave
from pathlib import Path

SAMPLE_RATE = 24000


# --------------------------------------------------------------------------
# Building blocks — one state struct + one per-sample step each, as they would
# be written in C.
# --------------------------------------------------------------------------


def biquad(kind: str, f0: float, q: float, fs: int) -> tuple[float, ...]:
    """RBJ audio-EQ-cookbook coefficients, normalised by a0."""
    w0 = 2.0 * math.pi * f0 / fs
    cw, sw = math.cos(w0), math.sin(w0)
    alpha = sw / (2.0 * q)
    if kind == "hp":
        b0, b1, b2 = (1 + cw) / 2, -(1 + cw), (1 + cw) / 2
    elif kind == "lp":
        b0, b1, b2 = (1 - cw) / 2, 1 - cw, (1 - cw) / 2
    else:
        raise ValueError(kind)
    a0, a1, a2 = 1 + alpha, -2 * cw, 1 - alpha
    return b0 / a0, b1 / a0, b2 / a0, a1 / a0, a2 / a0


def apply_biquad(x: list[float], coeffs: tuple[float, ...]) -> list[float]:
    b0, b1, b2, a1, a2 = coeffs
    x1 = x2 = y1 = y2 = 0.0
    out = []
    for s in x:
        y = b0 * s + b1 * x1 + b2 * x2 - a1 * y1 - a2 * y2
        x2, x1 = x1, s
        y2, y1 = y1, y
        out.append(y)
    return out


def bandpass(
    x: list[float], lo_hz: float, hi_hz: float, fs: int = SAMPLE_RATE
) -> list[float]:
    """Cascaded 2nd-order HP + LP.

    Optical and early magnetic sound ran roughly 100 Hz-5 kHz. The defaults
    used by the chains sit inside that, because the goal is 'played back
    through a period chain', not 'telephone'.
    """
    y = apply_biquad(x, biquad("hp", lo_hz, 0.707, fs))
    return apply_biquad(y, biquad("lp", hi_hz, 0.707, fs))


def saturate(x: list[float], drive: float) -> list[float]:
    """Soft clip, normalised so drive changes character and not level."""
    norm = math.tanh(drive)
    return [math.tanh(s * drive) / norm for s in x]


def comb(
    x: list[float], delay_ms: float, g: float, fs: int = SAMPLE_RATE
) -> list[float]:
    """Feedback comb: y[n] = (1-g)*x[n] + g*y[n-D].

    The (1-g) input trim is what keeps this a resonator. The briefing's version
    omits it and then recommends -3 dB of pre-attenuation, which is ~12 dB short
    at the feedback it asks for.
    """
    d = max(1, int(delay_ms / 1000.0 * fs))
    buf = [0.0] * d
    idx = 0
    out = []
    for s in x:
        y = (1.0 - g) * s + g * buf[idx]
        buf[idx] = y
        idx = (idx + 1) % d
        out.append(y)
    return out


def ringmod(
    x: list[float], carrier_hz: float, depth: float, fs: int = SAMPLE_RATE
) -> list[float]:
    """Wet/dry ring modulation. depth=0 is bypass, depth=1 is a bare multiply."""
    inc = 2.0 * math.pi * carrier_hz / fs
    phase = 0.0
    out = []
    for s in x:
        c = math.cos(phase)
        phase += inc
        if phase >= 2.0 * math.pi:
            phase -= 2.0 * math.pi
        out.append(s * ((1.0 - depth) + depth * c))
    return out


def hiss(x: list[float], level: float) -> list[float]:
    """A little broadband noise floor. Level is relative to full scale."""
    rng = random.Random(1950)  # fixed seed: two runs must be comparable
    return [s + rng.uniform(-level, level) for s in x]


# --------------------------------------------------------------------------
# I/O
# --------------------------------------------------------------------------


def read_wav(path: Path) -> list[float]:
    with wave.open(str(path), "rb") as w:
        if w.getsampwidth() != 2 or w.getnchannels() != 1:
            raise SystemExit(
                f"{path}: expected 16-bit mono, got "
                f"{w.getsampwidth() * 8}-bit {w.getnchannels()}ch"
            )
        if w.getframerate() != SAMPLE_RATE:
            raise SystemExit(
                f"{path}: expected {SAMPLE_RATE} Hz, got {w.getframerate()}"
            )
        raw = w.readframes(w.getnframes())
    return [s / 32768.0 for s in array.array("h", raw)]


def rms(x: list[float]) -> float:
    return math.sqrt(sum(s * s for s in x) / max(len(x), 1)) if x else 0.0


def write_wav(path: Path, x: list[float]) -> tuple[float, float]:
    """Write float samples as 16-bit mono. Returns (applied gain, peak before clip)."""
    peak = max((abs(s) for s in x), default=0.0)
    # Guard the cast, the same way audio_player_set_volume_pct() guards its own:
    # an int16 cast of an out-of-range float wraps to the opposite rail, which
    # is full-scale noise rather than distortion.
    g = min(1.0, 0.99 / peak) if peak > 0.99 else 1.0
    pcm = array.array(
        "h", (int(max(-32768, min(32767, round(s * g * 32767)))) for s in x)
    )
    data = pcm.tobytes()
    header = struct.pack(
        "<4sI4s4sIHHIIHH4sI",
        b"RIFF",
        36 + len(data),
        b"WAVE",
        b"fmt ",
        16,
        1,
        1,
        SAMPLE_RATE,
        SAMPLE_RATE * 2,
        2,
        16,
        b"data",
        len(data),
    )
    path.write_bytes(header + data)
    return g, peak


# --------------------------------------------------------------------------


def chains(dry: list[float]) -> list[tuple[str, list[float]]]:
    return [
        ("00-dry", dry),
        ("01-band", bandpass(dry, 250.0, 3800.0)),
        ("02-band-sat", hiss(saturate(bandpass(dry, 250.0, 3800.0), 1.8), 0.0015)),
        ("03-comb", comb(dry, 6.5, 0.78)),
        ("04-retro", saturate(comb(bandpass(dry, 250.0, 3800.0), 6.5, 0.72), 1.5)),
        (
            "05-ringlite",
            ringmod(
                saturate(comb(bandpass(dry, 250.0, 3800.0), 6.5, 0.72), 1.5),
                120.0,
                0.35,
            ),
        ),
        ("06-dalek", ringmod(bandpass(dry, 250.0, 3800.0), 45.0, 1.0)),
    ]


def retro(
    dry: list[float], lo=250.0, hi=3800.0, delay=6.5, fb=0.72, drive=1.5, band=True
) -> list[float]:
    """The `04-retro` chain with its constants exposed.

    Order is medium -> body -> overdrive, and it is not arbitrary: band-limiting
    after the comb would filter away the resonance the comb just added, and
    saturating before the comb would feed the delay line a signal that is
    already clipped, so the feedback compounds distortion instead of ringing.
    """
    x = bandpass(dry, lo, hi) if band else dry
    return saturate(comb(x, delay, fb), drive)


def chains_robby(dry: list[float]) -> list[tuple[str, list[float]]]:
    """One-variable-at-a-time sweep around `04-retro`.

    Every arm differs from 00-ref in exactly ONE parameter, so a difference you
    hear is attributable. The axes are chosen because they map onto physical
    intuitions you can judge without knowing the arithmetic:

      delay  -> how BIG the body sounds. 6.5 ms resonates near 154 Hz; halving
                the delay doubles that and shrinks the box.
      fb     -> how METALLIC. Low is a hint of enclosure, high is a struck bell.
      band   -> whether you are hearing Robby IN THE ROOM (no band limit) or a
                1956 FILM RECORDING of Robby (band limited). 04-retro silently
                assumed the latter; these two are separable and only you know
                which one Teuvo is.
      drive  -> how hard the amplifier inside him is being pushed.
    """
    return [
        ("00-ref", retro(dry)),
        ("01-body-small", retro(dry, delay=3.0)),
        ("02-body-large", retro(dry, delay=12.0)),
        ("03-metal-soft", retro(dry, fb=0.55)),
        ("04-metal-hard", retro(dry, fb=0.85)),
        ("05-in-the-room", retro(dry, band=False)),
        ("06-wide-band", retro(dry, lo=150.0, hi=6000.0)),
        ("07-drier", retro(dry, drive=1.0)),
    ]


CHAIN_SETS = {"span": chains, "robby": chains_robby}

# Single named chains, for applying ONE treatment across MANY clips — the
# inverse of a chain set. `in-the-room` is the selected Teuvo chain: comb body
# with no band limit, i.e. Robby standing in the room rather than on a 1956
# soundtrack.
PRESETS = {
    "in-the-room": lambda dry: retro(dry, band=False),
    "retro": lambda dry: retro(dry),
    "dry": lambda dry: dry,
}


def main() -> int:
    if len(sys.argv) not in (3, 4):
        print(
            "usage: voice-fx-preview.py <input.wav|input_dir> <out_dir> [name]\n"
            f"  file -> chain set: {', '.join(CHAIN_SETS)}\n"
            f"  dir  -> preset:    {', '.join(PRESETS)}",
            file=sys.stderr,
        )
        return 2
    src = Path(sys.argv[1]).resolve()
    out_dir = Path(sys.argv[2]).resolve()
    out_dir.mkdir(parents=True, exist_ok=True)

    if src.is_dir():
        # MANY clips, ONE chain: comparing voices, with the treatment fixed.
        name = sys.argv[3] if len(sys.argv) == 4 else "in-the-room"
        if name not in PRESETS:
            print(
                f"unknown preset {name!r}; have {', '.join(PRESETS)}", file=sys.stderr
            )
            return 2
        srcs = sorted(src.glob("*.wav"))
        if not srcs:
            # An empty glob renders nothing and otherwise exits 0, which reads
            # exactly like a completed sweep.
            print(f"{src}: no .wav files", file=sys.stderr)
            return 1
        items = [(f.stem, PRESETS[name](read_wav(f))) for f in srcs]
        # Match every voice to the loudest-neutral reference rather than to one
        # arbitrary voice: Gemini returns different voices at different levels,
        # and timbre cannot be judged across a level difference.
        target = sum(rms(y) for _, y in items) / len(items)
        print(f"[{name}] {len(items)} clips from {src.name}  mean rms {target:.4f}")
    else:
        # ONE clip, MANY chains: comparing treatments, with the voice fixed.
        name = sys.argv[3] if len(sys.argv) == 4 else "span"
        if name not in CHAIN_SETS:
            print(
                f"unknown chain set {name!r}; have {', '.join(CHAIN_SETS)}",
                file=sys.stderr,
            )
            return 2
        dry = read_wav(src)
        target = rms(dry)
        items = CHAIN_SETS[name](dry)
        print(
            f"[{name}] Input: {src.name}  {len(dry) / SAMPLE_RATE:.2f} s  rms {target:.4f}"
        )
    print()
    # Loudness-match every chain to the dry clip. Without this the A/B is
    # confounded: a bandpass removes energy and a comb adds it, and the louder
    # of two clips is judged the better one almost regardless of what else
    # changed. This is the control that makes the comparison about character.
    matched = []
    for name, y in items:
        r = rms(y)
        mgain = (target / r) if r > 0 else 1.0
        matched.append((name, [s * mgain for s in y], mgain))

    # ONE global headroom scale, applied to every chain including the dry.
    # Rescaling each chain individually to fit int16 would silently undo the
    # match just made: a comb and a ring modulator raise crest factor, so they
    # need the most attenuation and would arrive quietest — which is the exact
    # bias the match exists to remove. Measured before this was fixed:
    # 05-ringlite came out at x0.481, less than half the intended loudness.
    peaks = [max((abs(s) for s in y), default=0.0) for _, y, _ in matched]
    head = min(1.0, 0.99 / max(peaks)) if max(peaks) > 0.99 else 1.0

    print(f"{'chain':<14}{'rms gain':>10}{'peak':>8}{'final rms':>11}")
    for (name, y, mgain), peak in zip(matched, peaks):
        y = [s * head for s in y]
        write_wav(out_dir / f"{name}.wav", y)
        print(f"{name:<14}{mgain:10.3f}{peak:8.3f}{rms(y):11.4f}")
    print(f"\nglobal headroom scale: x{head:.3f} (applied to all, match preserved)")

    print()
    print(f"Listen in order:  open {out_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
