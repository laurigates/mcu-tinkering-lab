#!/usr/bin/env python3
"""LQR gains, closed-loop poles and clamp lean across a range of r.

Usage (needs numpy and scipy; the recipe runs it through `uv run --with`):
    lqr_sweep.py --l-eff 0.21 --wheel-diam 0.065 [--q 400 1 25 4] [--r 0.04 1 10 50]

Uses tests/host/lqr_reference.py's discretisation and scipy's DARE solver,
which the host tests check src/lqr.c against, so the K printed for a given
setting matches what `set r` makes the robot solve. For each r it prints K,
the closed-loop poles as continuous-time equivalents (rad/s), and the lean
at which K[0] alone reaches the acceleration clamp. On the rev1 robot, r =
0.04 hit the clamp at 2.2 deg and jittered against it; past r ~ 10-50 the
angle poles stop at the mirror of the open-loop pole sqrt(g / L) and only
the position hold keeps weakening (README "Choosing r").
"""

import argparse
import math
import sys
from pathlib import Path

import numpy as np
from scipy.linalg import solve_discrete_are

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "tests" / "host"))
import lqr_reference as ref

G = ref.G


def main() -> int:
    ap = argparse.ArgumentParser(description="LQR gains across r.")
    ap.add_argument("--l-eff", type=float, required=True, help="l_eff_m")
    ap.add_argument("--wheel-diam", type=float, required=True, help="wheel_diam_m")
    ap.add_argument("--steps-per-rev", type=float, default=1600.0)
    ap.add_argument("--max-accel", type=float, default=40000.0, help="MAX_ACCEL_SPS2")
    ap.add_argument(
        "--q",
        type=float,
        nargs=4,
        default=[400.0, 1.0, 25.0, 4.0],
        metavar=("Q_THETA", "Q_DTHETA", "Q_X", "Q_DX"),
    )
    ap.add_argument(
        "--r", type=float, nargs="+", default=[0.04, 0.16, 1.0, 5.0, 10.0, 50.0, 200.0]
    )
    args = ap.parse_args()

    spm = args.steps_per_rev / (math.pi * args.wheel_diam)
    clamp = args.max_accel / spm
    a, b = ref.discretise(args.l_eff, ref.DT)
    q = np.diag(args.q)
    pole = math.sqrt(G / args.l_eff)
    print(
        f"L_eff {args.l_eff} m: open-loop pole sqrt(g/L) = {pole:.2f} rad/s; "
        f"stability needs |K0| > g = {G:.2f}; clamp {clamp:.2f} m/s^2"
    )
    print(
        f"{'r':>8}  {'K0':>8} {'K1':>7} {'K2':>8} {'K3':>8}  {'clamp at':>8}  poles (rad/s)"
    )
    for r in args.r:
        rr = np.array([[r]])
        p = solve_discrete_are(a, b, q, rr)
        k = np.linalg.solve(rr + b.T @ p @ b, b.T @ p @ a).ravel()
        poles = np.log(np.linalg.eigvals(a - b @ k[None, :]).astype(complex)) / ref.DT
        ps = ", ".join(
            f"{z.real:.2f}{z.imag:+.2f}j" if abs(z.imag) > 1e-6 else f"{z.real:.2f}"
            for z in sorted(poles, key=lambda z: (z.real, z.imag))
        )
        lean = math.degrees(clamp / abs(k[0]))
        print(
            f"{r:8g}  {k[0]:8.2f} {k[1]:7.2f} {k[2]:8.3f} {k[3]:8.3f}  {lean:7.1f}°  {ps}"
        )
    return 0


if __name__ == "__main__":
    sys.exit(main())
