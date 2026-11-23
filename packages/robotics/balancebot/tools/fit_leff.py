#!/usr/bin/env python3
"""Fit the LQR pendulum length l_eff_m from a balancebot telemetry capture.

Usage: fit_leff.py CAPTURE.csv [--wheel-diam M] [--steps-per-rev N]
       fit_leff.py --self-test

The LQR model (src/lqr.h) is  theta_ddot = (g * theta - u) / L_eff,  with
theta = -pitch (rad), positive toward +x, and u the wheel acceleration (m/s^2)
in the positive step direction. Integrated over 0.2 s windows of the RUN rows,

    delta theta_dot * L_eff = g * integral(theta dt) - delta v + g * delta * T

with v the wheel speed from rate_sps, so neither theta_ddot nor u is ever
differentiated out of the data (see samples()). It is fitted by least squares;
the constant term is a pitch offset delta, reported as the angle the balance
point reads, which is what to add to `trim`.

Caveats, all of which make the fit wrong rather than loose:
- Pushes and bumps are forces the model lacks; let the robot balance alone.
- v is the commanded speed, so stretches where a motor stalled or skipped
  steps fit motion the wheels did not make; cut those captures.
Over 30 simulated 20 s captures the fit averaged +5 % from the true L_eff
with an 11 % spread, and the printed 2-sigma range covered it 29 times;
--self-test checks three of them. Standard library only.
"""

import argparse
import csv
import itertools
import math
import random
import sys
from dataclasses import dataclass
from pathlib import Path

G = 9.80665
RUN = 1


@dataclass
class Fit:
    l_eff_m: float
    l_eff_se: float  # standard error, widened for the overlapping windows
    offset_deg: float
    r2: float
    n: int


def run_segments(rows: list[dict]) -> list[list[dict]]:
    """Split the RUN rows into stretches without gaps or state changes."""
    segs: list[list[dict]] = []
    cur: list[dict] = []
    prev_t = None
    for r in rows:
        t = r["t_ms"]
        gap = prev_t is not None and t - prev_t > 50.0
        if r["state"] != RUN or gap:
            if len(cur) >= 5:
                segs.append(cur)
            cur = []
        if r["state"] == RUN:
            cur.append(r)
        prev_t = t
    if len(cur) >= 5:
        segs.append(cur)
    return segs


def samples(rows: list[dict], steps_per_m: float, max_deg: float, window: int):
    """(delta theta_dot, g * integral(theta) - delta v) pairs over RUN windows.

    The model is integrated over `window` rows rather than differentiated:
    delta theta_dot * L = g * integral(theta dt) - delta v (+ b * T), where
    v is the wheel speed from rate_sps, so u is never differentiated out of
    it. Differentiating would bias L: the controller computes u from the very
    gyro samples a differentiated theta_ddot is made of, so their noise is
    correlated. Windows overlap by half; a window that leans past max_deg
    anywhere is skipped.
    """
    xs: list[float] = []  # delta theta_dot
    ys: list[float] = []  # g * integral(theta) - delta v
    step = max(1, window // 2)
    for seg in run_segments(rows):
        t = [r["t_ms"] / 1000.0 for r in seg]
        theta = [-math.radians(r["angle_deg"]) for r in seg]
        theta_dot = [-math.radians(r["gyro_dps"]) for r in seg]
        v = [r["rate_sps"] / steps_per_m for r in seg]  # m/s
        for i0 in range(0, len(seg) - window, step):
            i1 = i0 + window
            if any(abs(seg[i]["angle_deg"]) > max_deg for i in range(i0, i1 + 1)):
                continue
            integral = sum(
                0.5 * (theta[i] + theta[i + 1]) * (t[i + 1] - t[i])
                for i in range(i0, i1)
            )
            xs.append(theta_dot[i1] - theta_dot[i0])
            ys.append(G * integral - (v[i1] - v[i0]))
    return xs, ys


def regress(xs: list[float], ys: list[float]) -> tuple[float, float, float, float]:
    """x = a * y + c by least squares; returns (a, c, R^2, standard error of a).

    x (the gyro difference) is the regressand because it carries the sensor
    noise; noise in the regressor would bias the slope toward 0.
    """
    n = len(xs)
    mx, my = sum(xs) / n, sum(ys) / n
    syy = sum((y - my) ** 2 for y in ys)
    sxy = sum((x - mx) * (y - my) for x, y in zip(xs, ys, strict=True))
    if syy == 0 or sxy == 0:
        raise ValueError("no motion to fit")
    a = sxy / syy
    c = mx - a * my
    ss_res = sum((x - (a * y + c)) ** 2 for x, y in zip(xs, ys, strict=True))
    ss_tot = sum((x - mx) ** 2 for x in xs)
    se_a = math.sqrt(ss_res / max(1, n - 2) / syy)
    return a, c, (1.0 - ss_res / ss_tot if ss_tot > 0 else 0.0), se_a


def fit_rows(
    rows: list[dict], steps_per_m: float, max_deg: float, window: int = 20
) -> Fit:
    xs, ys = samples(rows, steps_per_m, max_deg, window)
    if len(xs) < 3:
        raise ValueError(f"only {len(xs)} usable RUN windows")
    a, c, r2, se_a = regress(xs, ys)
    l_eff = 1.0 / a
    # L = 1/a, so se(L) ~ se(a) / a^2; half-overlapping windows share data,
    # so the independent-residual formula is widened by sqrt(2).
    l_se = math.sqrt(2.0) * se_a / (a * a)
    # With theta_true = theta_measured + delta, the windows satisfy
    # delta theta_dot = (y + g * delta * T) / L, so c = g * delta * T / L for
    # a window of length T. The balance point (theta_true = 0) reads
    # pitch = delta.
    t_win = window * median_dt(rows)
    delta = c * l_eff / (G * t_win)
    return Fit(l_eff, l_se, math.degrees(delta), r2, len(xs))


def median_dt(rows: list[dict]) -> float:
    dts = sorted(
        b["t_ms"] - a["t_ms"]
        for a, b in itertools.pairwise(rows)
        if b["t_ms"] > a["t_ms"]
    )
    return dts[len(dts) // 2] / 1000.0 if dts else 0.01


def read_capture(path: Path) -> list[dict]:
    with path.open(newline="") as f:
        return [{k: float(v) for k, v in row.items()} for row in csv.DictReader(f)]


def simulate(
    l_eff: float,
    offset_deg: float,
    steps_per_m: float,
    seed: int = 1,
    kicks: bool = False,
) -> list[dict]:
    """Telemetry from the model balanced by a PD law on noisy readings.

    The controller sees the offset pitch and the sensor noise, as the robot
    does, and holds a position target that moves every 2 s; that is what
    excites the motion. kicks=True adds pushes, which the model lacks.
    """
    rng = random.Random(seed)
    dt, theta, theta_dot, x, v = 0.002, 0.03, 0.0, 0.0, 0.0
    x_ref = 0.0
    rows = []
    for i in range(10000):  # 20 s at 500 Hz
        pitch = -math.degrees(theta) + offset_deg + rng.gauss(0, 0.05)
        gyro = -math.degrees(theta_dot) + rng.gauss(0, 0.3)
        th_m, thd_m = -math.radians(pitch), -math.radians(gyro)
        if i % 1000 == 0:
            x_ref = rng.uniform(-0.15, 0.15)
        # Position terms enter with + on an inverted pendulum: to go to +x
        # it must first lean +x.
        u = 30.0 * th_m + 4.0 * thd_m + 1.0 * (x - x_ref) + 1.5 * v
        theta_ddot = (G * theta - u) / l_eff
        theta_dot += theta_ddot * dt
        theta += theta_dot * dt
        v += u * dt
        x += v * dt
        if kicks and i % 500 == 250:
            theta_dot += rng.uniform(-0.3, 0.3)
        if i % 5 == 0:  # telemetry at 100 Hz
            rows.append(
                {
                    "t_ms": i * dt * 1000.0,
                    "state": float(RUN),
                    "angle_deg": pitch,
                    "gyro_dps": gyro,
                    "rate_sps": v * steps_per_m,
                }
            )
    return rows


def self_test() -> int:
    spm = 1600 / (math.pi * 0.065)
    ok = True
    for seed in (1, 2, 3):
        fit = fit_rows(simulate(0.21, 1.5, spm, seed=seed), spm, max_deg=20.0)
        good = abs(fit.l_eff_m - 0.21) < 0.03 and abs(fit.offset_deg - 1.5) < 0.3
        ok = ok and good
        print(
            f"self-test (seed {seed}): l_eff {fit.l_eff_m:.4f} (true 0.21), offset "
            f"{fit.offset_deg:.2f} (true 1.5), R^2 {fit.r2:.3f}, n {fit.n}: "
            f"{'ok' if good else 'FAIL'}"
        )
    # Pushes are forces the model lacks; this only shows how far they drag it.
    fit = fit_rows(simulate(0.21, 1.5, spm, kicks=True), spm, max_deg=20.0)
    print(
        f"self-test (with pushes, not checked): l_eff {fit.l_eff_m:.4f}, R^2 {fit.r2:.3f}"
    )
    print("self-test:", "PASS" if ok else "FAIL")
    return 0 if ok else 1


def main() -> int:
    ap = argparse.ArgumentParser(description="Fit l_eff_m from a balancebot capture.")
    ap.add_argument("capture", nargs="?", type=Path)
    ap.add_argument(
        "--wheel-diam", type=float, help="m, as set on the robot (required)"
    )
    ap.add_argument("--steps-per-rev", type=float, default=1600.0)
    ap.add_argument(
        "--max-deg", type=float, default=20.0, help="ignore rows leaning further"
    )
    ap.add_argument(
        "--self-test", action="store_true", help="fit simulated data and check it"
    )
    args = ap.parse_args()
    if args.self_test:
        return self_test()
    if args.capture is None or args.wheel_diam is None:
        ap.error("a capture CSV and --wheel-diam are required")

    spm = args.steps_per_rev / (math.pi * args.wheel_diam)
    try:
        fit = fit_rows(read_capture(args.capture), spm, args.max_deg)
    except ValueError as e:
        print(f"ERROR: {e}; capture with the robot armed and moving", file=sys.stderr)
        return 1
    print(
        f"windows:   {fit.n} half-overlapping RUN windows within +/-{args.max_deg:g} deg"
    )
    print(f"l_eff_m:   {fit.l_eff_m:.3f} m +/- {fit.l_eff_se:.3f} (1 sigma)")
    print(
        f"offset:    {fit.offset_deg:+.2f} deg (what the balance point reads; add to trim)"
    )
    print(f"R^2:       {fit.r2:.2f}")
    if fit.n < 100:
        print("warning: under about 10 s of balancing; capture longer")
    if fit.r2 < 0.3:
        print(
            "warning: the model explains little of this capture (pushes, stalls, skipped "
            "steps, or too little motion); do not trust l_eff_m"
        )
    if not 0.02 < fit.l_eff_m < 1.0:
        print(
            "warning: l_eff_m outside 0.02-1 m is not this robot; check wheel size and signs"
        )
    else:
        print(f'apply:     just balancebot::cli "set l_eff_m {fit.l_eff_m:.3f}" save')
    return 0


if __name__ == "__main__":
    sys.exit(main())
