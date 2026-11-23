#!/usr/bin/env python3
"""Summarise a balancebot telemetry capture for tuning.

Usage: analyze_capture.py CAPTURE.csv [--wheel-diam M] [--steps-per-rev N]
                          [--maxrate SPS] [--max-accel SPS2]
       analyze_capture.py --self-test

For the RUN rows it reports what decided each step of the 2026-11-23 rev1
tuning session (README "Tuning loop"):

- angle mean and RMS, and the mean as a trim suggestion (the robot balancing
  on its own sits at its balance point on average, so the mean angle is what
  to add to `trim`, if the position held steady)
- the share of rows at the speed limit (maxrate) and at the acceleration
  clamp (MAX_ACCEL_SPS2): a large share means the gains ask for more than
  the motors are allowed, and `r` should go up
- the dominant swing frequency and amplitude from a spectrum (zero-crossing
  counts are inflated by sensor noise: they read 9.4 Hz for a 0.9 Hz swing),
  compared with the pendulum frequency sqrt(g / L_eff) when --l-eff is given;
  a rock near that frequency pointed at the filter alpha
- wheel-speed peak and position range, which do not pass through the IMU
  filter, so they show whether the robot moved less or only read calmer

Standard library only.
"""

import argparse
import cmath
import csv
import math
import sys
from pathlib import Path

RUN = 1
G = 9.80665


def read_capture(path: Path) -> list[dict]:
    with path.open(newline="") as f:
        return [{k: float(v) for k, v in row.items()} for row in csv.DictReader(f)]


def run_rows(rows: list[dict]) -> list[dict]:
    """The longest stretch of consecutive RUN rows."""
    best: list[dict] = []
    cur: list[dict] = []
    for r in rows:
        if int(r["state"]) == RUN:
            cur.append(r)
        else:
            cur = []
        if len(cur) > len(best):
            best = list(cur)
    return best


def dft_peak(t: list[float], xs: list[float], f_lo: float, f_hi: float, step: float):
    """(frequency, amplitude) of the largest single-bin DFT magnitude."""
    m = sum(xs) / len(xs)
    xs = [x - m for x in xs]
    best = (0.0, 0.0)
    f = f_lo
    while f <= f_hi:
        z = sum(
            x * cmath.exp(-2j * math.pi * f * (ti - t[0]))
            for x, ti in zip(xs, t, strict=True)
        )
        amp = abs(z) * 2 / len(xs)
        if amp > best[1]:
            best = (f, amp)
        f += step
    return best


def analyze(rows: list[dict], maxrate: float, max_accel: float) -> dict:
    seg = run_rows(rows)
    if len(seg) < 50:
        raise ValueError(
            f"only {len(seg)} consecutive RUN rows; capture with the robot armed"
        )
    t = [r["t_ms"] / 1000.0 for r in seg]
    ang = [r["angle_deg"] for r in seg]
    rate = [r["rate_sps"] for r in seg]
    pos = [r["pos_m"] for r in seg]
    n = len(seg)
    mean = sum(ang) / n
    rms = math.sqrt(sum((a - mean) ** 2 for a in ang) / n)
    acc = [
        (rate[i] - rate[i - 1]) / (t[i] - t[i - 1])
        for i in range(1, n)
        if t[i] > t[i - 1]
    ]
    f, amp = dft_peak(t, ang, 0.2, 25.0, 0.1)
    return {
        "rows": n,
        "duration_s": t[-1] - t[0],
        "angle_mean": mean,
        "angle_rms": rms,
        "angle_min": min(ang),
        "angle_max": max(ang),
        "rate_peak": max(abs(r) for r in rate),
        "at_maxrate": sum(1 for r in rate if abs(r) >= 0.999 * maxrate) / n,
        "accel_peak": max(abs(a) for a in acc) if acc else 0.0,
        "at_clamp": sum(1 for a in acc if abs(a) >= 0.95 * max_accel)
        / max(1, len(acc)),
        "swing_hz": f,
        "swing_deg": amp,
        "pos_range": max(pos) - min(pos),
    }


def report(a: dict, l_eff: float | None) -> None:
    print(f"RUN:       {a['rows']} rows, {a['duration_s']:.1f} s")
    print(
        f"angle:     mean {a['angle_mean']:+.2f} deg, RMS {a['angle_rms']:.2f}, "
        f"range {a['angle_min']:+.2f}..{a['angle_max']:+.2f}"
    )
    print(
        f"swing:     {a['swing_hz']:.1f} Hz, {a['swing_deg']:.2f} deg (spectral peak)"
    )
    if l_eff:
        fp = math.sqrt(G / l_eff) / (2 * math.pi)
        print(f"           pendulum frequency sqrt(g/L) = {fp:.2f} Hz")
    print(
        f"speed:     peak {a['rate_peak']:.0f} steps/s, {a['at_maxrate']:.1%} of rows at maxrate"
    )
    print(
        f"accel:     peak {a['accel_peak']:.0f} steps/s^2, {a['at_clamp']:.1%} at the clamp"
    )
    print(f"position:  range {100 * a['pos_range']:.1f} cm")
    print("hints:")
    if a["at_clamp"] > 0.05:
        print("  - often at the acceleration clamp: the gains ask too much; raise r")
    if a["at_maxrate"] > 0.05:
        print(
            "  - often at maxrate: raise maxrate if the motors keep up, else lower the gains"
        )
    if (
        l_eff
        and abs(a["swing_hz"] - math.sqrt(G / l_eff) / (2 * math.pi)) < 0.4
        and a["swing_deg"] > 0.3
    ):
        print("  - rocking near the pendulum frequency: check alpha (0.996 at 500 Hz)")
    if a["pos_range"] < 0.05 and abs(a["angle_mean"]) > 0.1:
        print(
            f"  - position held: add {a['angle_mean']:+.2f} to trim "
            "(just balancebot::cli trim, then set trim <sum>)"
        )
    if a["pos_range"] >= 0.05:
        print("  - position wandered: the mean angle is not a trim measurement")


def self_test() -> int:
    """A synthetic capture: 1.1 Hz, 0.9 deg swing about -0.4 deg, no saturation."""
    rows = []
    for i in range(2000):
        t = i * 0.01
        rows.append(
            {
                "t_ms": t * 1000.0,
                "state": 1.0,
                "angle_deg": -0.4
                + 0.9 * math.sin(2 * math.pi * 1.1 * t)
                + 0.05 * math.sin(97.0 * t),
                "rate_sps": 300.0 * math.sin(2 * math.pi * 1.1 * t + 1.0),
                "pos_m": 0.01 * math.sin(2 * math.pi * 0.2 * t),
            }
        )
    a = analyze(rows, 4000.0, 40000.0)
    ok = (
        abs(a["swing_hz"] - 1.1) < 0.051
        and abs(a["swing_deg"] - 0.9) < 0.05
        and abs(a["angle_mean"] + 0.4) < 0.01
        and a["at_clamp"] == 0.0
        and a["at_maxrate"] == 0.0
    )
    print(
        f"self-test: swing {a['swing_hz']:.2f} Hz {a['swing_deg']:.2f} deg, "
        f"mean {a['angle_mean']:+.3f}: {'PASS' if ok else 'FAIL'}"
    )
    return 0 if ok else 1


def main() -> int:
    ap = argparse.ArgumentParser(
        description="Summarise a balancebot capture for tuning."
    )
    ap.add_argument("capture", nargs="?", type=Path)
    ap.add_argument(
        "--maxrate", type=float, default=4000.0, help="steps/s, as set on the robot"
    )
    ap.add_argument("--max-accel", type=float, default=40000.0, help="MAX_ACCEL_SPS2")
    ap.add_argument(
        "--l-eff", type=float, help="l_eff_m, to compare the swing with sqrt(g/L)"
    )
    ap.add_argument("--self-test", action="store_true")
    args = ap.parse_args()
    if args.self_test:
        return self_test()
    if args.capture is None:
        ap.error("a capture CSV is required")
    try:
        a = analyze(read_capture(args.capture), args.maxrate, args.max_accel)
    except ValueError as e:
        print(f"ERROR: {e}", file=sys.stderr)
        return 1
    report(a, args.l_eff)
    return 0


if __name__ == "__main__":
    sys.exit(main())
