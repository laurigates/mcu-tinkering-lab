#!/usr/bin/env python3
"""A steady push, then its release, on the balancebot's LQR model (#1279).

Usage (the gain solve needs numpy and scipy; `just balancebot::push-sim`
runs it through `uv run --with`):
    push_sim.py [--l-eff M] [--wheel-diam M] [--steps-per-rev N] [--r R]
                [--q-x Q ...] [--maxrate SPS ...] [--ramp S] [--tilt-cutoff DEG]
    push_sim.py --self-test

Point-mass wheeled inverted pendulum (lqr.h): theta_dd = (g*theta - u + a_p)/L,
x_dd = u, where a_p is a push acting at the centre of mass (force / mass).
u = -K s, clamped to MAX_ACCEL_SPS2, and the wheel speed is clamped to
maxrate (stepper_set_rate()'s slew and rate clamp). The push is held 3 s,
optionally building up over --ramp seconds, then released; the robot falls
if |theta| passes TILT_CUTOFF_DEG (30, config.h).

For each maxrate and q_x it prints the largest push (as a fraction of g)
survived, the steady lean at that push (a_p / g in the linear model), how
far the robot sits from its start while held there, and where the next push
up failed: during the push or after the release. K comes from
tests/host/lqr_reference.py's discretisation, which the host tests check
src/lqr.c against, so the first line's K should match what `stat` prints for
the same q and r.

--self-test checks the simulation against the linear model's steady state
with a fixed K (standard library only), and the gain solve against a K the
robot printed when scipy is importable.
"""

import argparse
import math
import sys
from pathlib import Path

G = 9.80665
DT = 1.0 / 500.0
PUSH_S = 3.0
RUN_S = 6.0
# The K `stat` printed for #1279: L_eff 0.21 m, 65 mm wheels, q [400 1 400 4], r 10.
ROBOT_K = (-33.658, -4.8507, -6.2135, -5.5327)
ROBOT_Q = (400.0, 1.0, 400.0, 4.0)
ROBOT_R = 10.0


def gain(l_eff: float, q: tuple[float, ...], r: float) -> list[float]:
    import numpy as np
    from scipy.linalg import solve_discrete_are

    sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "tests" / "host"))
    import lqr_reference as ref

    a, b = ref.discretise(l_eff, DT)
    rr = np.array([[r]])
    p = solve_discrete_are(a, b, np.diag(q), rr)
    return [float(v) for v in np.linalg.solve(rr + b.T @ p @ b, b.T @ p @ a).ravel()]


def run(k, a_push, *, l_eff, acc_max, vmax, ramp_s=0.0, cutoff_deg=30.0):
    """('ok' | 'push' | 'release', theta at the end of the push (rad), x then (m))."""
    th = thd = x = v = 0.0
    th_rel = x_rel = None
    cutoff = math.radians(cutoff_deg)
    for i in range(int(RUN_S / DT)):
        t = i * DT
        pushing = t < PUSH_S
        if not pushing and th_rel is None:
            th_rel, x_rel = th, x
        u = -(k[0] * th + k[1] * thd + k[2] * x + k[3] * v)
        u = max(-acc_max, min(acc_max, u))
        v_new = max(-vmax, min(vmax, v + u * DT))
        u_eff = (v_new - v) / DT
        ramp = min(1.0, t / ramp_s) if ramp_s > 0 else 1.0
        thdd = (G * th - u_eff + (a_push * ramp if pushing else 0.0)) / l_eff
        thd += thdd * DT
        th += thd * DT
        v = v_new
        x += v * DT
        if abs(th) > cutoff:
            return ("push" if pushing else "release"), th_rel or th, x_rel or x
    return "ok", th_rel, x_rel


def largest_push(k, **model) -> tuple[float, str, float]:
    """(largest push survived as a fraction of g, how the next failed, x held)."""
    best, fail, x_held = 0.0, "-", 0.0
    for pct in range(1, 60):
        res, _, x = run(k, pct / 100.0 * G, **model)
        if res != "ok":
            fail = res
            break
        best, x_held = pct / 100.0, x
    return best, fail, x_held


def self_test() -> int:
    ok = True

    def check(name: str, cond: bool, got: str) -> None:
        nonlocal ok
        ok = ok and cond
        print(f"self-test {name}: {got}: {'PASS' if cond else 'FAIL'}")

    spm = 1600.0 / (math.pi * 0.065)
    model = dict(l_eff=0.21, acc_max=40000.0 / spm, vmax=4000.0 / spm)
    # Held still under a push, theta_dd = 0 and u = 0, so theta = -a_p / g,
    # and u = 0 also needs K0*theta + K2*x = 0.
    a = 0.05 * G
    res, th, x = run(ROBOT_K, a, **model)
    th_want = -a / G
    x_want = -ROBOT_K[0] * th_want / ROBOT_K[2]
    check(
        "steady lean",
        res == "ok"
        and abs(th - th_want) < 0.02 * abs(th_want)
        and abs(x - x_want) < 0.02 * abs(x_want),
        f"{res}, theta {math.degrees(th):+.3f} deg (want {math.degrees(th_want):+.3f}), "
        f"x {x:+.3f} m (want {x_want:+.3f})",
    )
    res, _, _ = run(ROBOT_K, 0.5 * G, **model)
    check("half g falls", res != "ok", res)
    try:
        import numpy  # noqa: F401
        import scipy  # noqa: F401
    except ImportError:
        print("self-test gain: numpy/scipy not importable: SKIP")
    else:
        k = gain(0.21, ROBOT_Q, ROBOT_R)
        check(
            "gain",
            all(abs(a - b) < 1e-3 * abs(b) for a, b in zip(k, ROBOT_K, strict=True)),
            " ".join(f"{v:.5g}" for v in k),
        )
    return 0 if ok else 1


def main() -> int:
    ap = argparse.ArgumentParser(
        description="Steady push and release on the LQR model."
    )
    ap.add_argument("--l-eff", type=float, default=0.21, help="l_eff_m")
    ap.add_argument("--wheel-diam", type=float, default=0.065, help="wheel_diam_m")
    ap.add_argument("--steps-per-rev", type=float, default=1600.0)
    ap.add_argument("--max-accel", type=float, default=40000.0, help="MAX_ACCEL_SPS2")
    ap.add_argument("--q-theta", type=float, default=400.0)
    ap.add_argument("--q-dtheta", type=float, default=1.0)
    ap.add_argument("--q-dx", type=float, default=4.0)
    ap.add_argument("--r", type=float, default=ROBOT_R)
    ap.add_argument(
        "--q-x", type=float, nargs="+", default=[25.0, 100.0, 400.0, 1600.0]
    )
    ap.add_argument(
        "--maxrate", type=float, nargs="+", default=[4000.0, 5000.0, 6000.0]
    )
    ap.add_argument(
        "--ramp", type=float, default=0.0, help="s for the push to build up"
    )
    ap.add_argument("--tilt-cutoff", type=float, default=30.0, help="TILT_CUTOFF_DEG")
    ap.add_argument("--self-test", action="store_true")
    args = ap.parse_args()
    if args.self_test:
        return self_test()

    spm = args.steps_per_rev / (math.pi * args.wheel_diam)

    def q(q_x: float) -> tuple[float, ...]:
        return (args.q_theta, args.q_dtheta, q_x, args.q_dx)

    k_ref = gain(args.l_eff, q(ROBOT_Q[2]), args.r)
    print(
        f"K(q_x {ROBOT_Q[2]:g}, r {args.r:g}): " + " ".join(f"{v:.5g}" for v in k_ref),
        f" (robot, #1279: {' '.join(str(v) for v in ROBOT_K)})",
    )
    print("\n q_x  maxrate  largest push  steady lean   held at  next push up failed")
    for vmax_sps in args.maxrate:
        for q_x in args.q_x:
            best, fail, x_held = largest_push(
                gain(args.l_eff, q(q_x), args.r),
                l_eff=args.l_eff,
                acc_max=args.max_accel / spm,
                vmax=vmax_sps / spm,
                ramp_s=args.ramp,
                cutoff_deg=args.tilt_cutoff,
            )
            print(
                f"{q_x:5.0f}  {vmax_sps:6.0f}   {best * 100:4.0f}% of g    "
                f"{math.degrees(best):5.1f} deg  {abs(x_held):6.2f} m  {fail}"
            )
    return 0


if __name__ == "__main__":
    sys.exit(main())
