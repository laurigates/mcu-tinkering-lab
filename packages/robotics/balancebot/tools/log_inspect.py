#!/usr/bin/env python3
"""Look inside a balancebot run log or capture (#1279).

Usage:
    log_inspect.py timeline LOG.csv [--maxrate SPS]
    log_inspect.py terms LOG.csv --k K0 K1 K2 K3 --t0 S --t1 S [--wheel-diam M]
                   [--steps-per-rev N]
    log_inspect.py accel LOG.csv --t0 S --t1 S --steady S S [--l-eff M]
                   [--wheel-diam M] [--steps-per-rev N]
    log_inspect.py --self-test

Reads the CSV that `just balancebot::log-dump` or `capture` writes
(LOG_CSV_HEADER, src/log_record.h). Times are seconds from the first row.

timeline  A 1 s summary of the whole run (states, angle min/mean/max, peak
          |rate|, position, loop time, largest gap between rows), every row
          from 3 s before the first FAULT to 0.2 s after it, and the
          stretches at 99 % of maxrate or more.
terms     The LQR's commanded acceleration rebuilt term by term, every 5th
          row in [t0, t1]: u = -K . [theta, theta_dot, x, x_dot] with
          theta = -pitch, theta_dot = -gyro, x = pos_m (x_ref = 0, no drive)
          and x_dot = rate / steps-per-metre, as in balance_control.c and
          lqr.h. K is the one `stat` prints.
accel     The wheel acceleration the IMU implies against the one commanded,
          every 3rd row in [t0, t1]. From the pendulum model (lqr.h),
          theta_dd = (g*theta - u) / L, so the ground acceleration that
          explains the body's motion is a_imu = g*theta - L*theta_dd, with
          theta_dd a centred 5-row difference of -gyro. a_cmd is the same
          difference of the commanded wheel speed. a_imu above a_cmd means
          the wheels went slower than commanded. Small-angle only: trust it
          to about 15 deg. Then the gyro's high-frequency RMS (gyro minus
          its 9-row moving average) over the steady window, for the ~15 Hz
          wobble.

Standard library only. --self-test checks each against a synthetic log with
known answers.
"""

import argparse
import csv
import itertools
import math
import sys
from pathlib import Path

G = 9.80665
FAULT = 2


def read_log(path: Path) -> list[dict]:
    """Rows as floats, with "t" in seconds from the first row."""
    with path.open(newline="") as f:
        rows = [{k: float(v) for k, v in r.items()} for r in csv.DictReader(f)]
    if not rows:
        raise ValueError(f"{path}: no rows")
    t0 = rows[0]["t_ms"]
    for r in rows:
        r["t"] = (r["t_ms"] - t0) / 1000.0
    return rows


def steps_per_m(wheel_diam: float, steps_per_rev: float) -> float:
    return steps_per_rev / (math.pi * wheel_diam)


def theta(r: dict) -> float:
    """Model lean (rad, positive toward +x) from the logged pitch."""
    return -math.radians(r["angle_deg"])


def theta_dot(r: dict) -> float:
    return -math.radians(r["gyro_dps"])


# timeline ------------------------------------------------------------------


def windows(rows: list[dict], width: float = 1.0) -> list[dict]:
    out = []
    w = 0.0
    while w <= rows[-1]["t"]:
        win = [r for r in rows if w <= r["t"] < w + width]
        if win:
            gaps = [b["t_ms"] - a["t_ms"] for a, b in itertools.pairwise(win)] or [0.0]
            a = [r["angle_deg"] for r in win]
            out.append(
                {
                    "t": w,
                    "states": sorted({int(r["state"]) for r in win}),
                    "angle_min": min(a),
                    "angle_mean": sum(a) / len(a),
                    "angle_max": max(a),
                    "rate_max": max(abs(r["rate_sps"]) for r in win),
                    "pos_end": win[-1]["pos_m"],
                    "loop_max": max(r["loop_us"] for r in win),
                    "gap_max": max(gaps),
                }
            )
        w += width
    return out


def first_fault(rows: list[dict]) -> int | None:
    return next((i for i, r in enumerate(rows) if int(r["state"]) == FAULT), None)


def maxrate_stretches(rows: list[dict], maxrate: float) -> list[tuple[int, int]]:
    """(first, last) row index of each run of rows at >= 99 % of maxrate."""
    out = []
    start = None
    for i, r in enumerate(rows):
        at = abs(r["rate_sps"]) >= 0.99 * maxrate
        if at and start is None:
            start = i
        if start is not None and (not at or i == len(rows) - 1):
            out.append((start, i if at else i - 1))
            start = None
    return out


def cmd_timeline(rows: list[dict], maxrate: float) -> None:
    print(
        "== 1 s windows: t, state, angle min/mean/max, |rate| max, pos end, "
        "loop_us max, gap max (ms)"
    )
    for w in windows(rows):
        print(
            f"{w['t']:5.1f}  st {w['states']}  angle {w['angle_min']:+7.2f} "
            f"{w['angle_mean']:+7.2f} {w['angle_max']:+7.2f}  |rate| {w['rate_max']:6.0f}"
            f"  pos {w['pos_end']:+.3f}  loop {w['loop_max']:5.0f}  gap {w['gap_max']:.0f}"
        )
    fault = first_fault(rows)
    if fault is None:
        print("\n== no FAULT in this log")
    else:
        tf = rows[fault]["t"]
        print(f"\n== FAULT at t = {tf:.2f} s (row {fault}); rows from t-3 s to t+0.2 s")
        print("   t     st  angle    gyro   setpt    rate     pos")
        for r in rows:
            if tf - 3.0 <= r["t"] <= tf + 0.2:
                print(
                    f"{r['t']:6.2f}  {int(r['state'])}  {r['angle_deg']:+7.2f} "
                    f"{r['gyro_dps']:+7.1f} {r['setpoint_deg']:+6.2f} "
                    f"{r['rate_sps']:+7.0f}  {r['pos_m']:+.3f}"
                )
    print(f"\n== stretches with |rate| >= 99% of maxrate ({maxrate:.0f})")
    for s, e in maxrate_stretches(rows, maxrate):
        a, b = rows[s], rows[e]
        print(
            f"  t {a['t']:.2f}..{b['t']:.2f} ({b['t'] - a['t']:.2f} s), "
            f"angle {a['angle_deg']:+.2f} -> {b['angle_deg']:+.2f}, "
            f"sign {'+' if a['rate_sps'] > 0 else '-'}"
        )


# terms ---------------------------------------------------------------------


def lqr_terms(r: dict, k: list[float], spm: float) -> list[float]:
    """The four terms of u = -K s (m/s^2, positive toward +x)."""
    s = [theta(r), theta_dot(r), r["pos_m"], r["rate_sps"] / spm]
    return [-ki * si for ki, si in zip(k, s, strict=True)]


def cmd_terms(rows, k, spm, t0, t1) -> None:
    print(f"steps/m {spm:.0f}; u terms in m/s^2 (positive = toward +x)")
    print("   t    st  pitch   pos     v(m/s)  u_theta u_dtheta  u_x    u_v    u_sum")
    for i, r in enumerate(rows):
        if not (t0 <= r["t"] <= t1) or i % 5:
            continue
        u = lqr_terms(r, k, spm)
        print(
            f"{r['t']:6.2f}  {int(r['state'])}  {r['angle_deg']:+6.2f} {r['pos_m']:+6.3f} "
            f"{r['rate_sps'] / spm:+6.3f}  "
            + "  ".join(f"{x:+6.2f}" for x in u)
            + f"  {sum(u):+6.2f}"
        )


# accel ---------------------------------------------------------------------


def implied_accel(rows, i: int, spm: float, l_eff: float) -> tuple[float, float]:
    """(a_cmd, a_imu) at row i, m/s^2, from centred 5-row differences."""
    a, b = rows[i - 2], rows[i + 2]
    dt = b["t"] - a["t"]
    thdd = (theta_dot(b) - theta_dot(a)) / dt
    a_cmd = (b["rate_sps"] - a["rate_sps"]) / spm / dt
    return a_cmd, G * theta(rows[i]) - l_eff * thdd


def gyro_hf_rms(rows, s0: float, s1: float) -> float | None:
    hf = []
    for i in range(4, len(rows) - 4):
        if s0 <= rows[i]["t"] <= s1:
            avg = sum(r["gyro_dps"] for r in rows[i - 4 : i + 5]) / 9.0
            hf.append(rows[i]["gyro_dps"] - avg)
    return math.sqrt(sum(x * x for x in hf) / len(hf)) if hf else None


def cmd_accel(rows, spm, l_eff, t0, t1, s0, s1) -> None:
    print(
        "   t    pitch   v_cmd   a_cmd   a_imu   "
        "(m/s^2; a_imu > a_cmd = wheels slower than commanded)"
    )
    for i in range(2, len(rows) - 2):
        if not (t0 <= rows[i]["t"] <= t1) or i % 3:
            continue
        a_cmd, a_imu = implied_accel(rows, i, spm, l_eff)
        print(
            f"{rows[i]['t']:6.2f}  {rows[i]['angle_deg']:+6.2f}  "
            f"{rows[i]['rate_sps'] / spm:+6.3f}  {a_cmd:+6.2f}  {a_imu:+6.2f}"
        )
    rms = gyro_hf_rms(rows, s0, s1)
    if rms is not None:
        print(f"\ngyro high-frequency RMS over {s0}-{s1} s: {rms:.1f} deg/s")


# self-test -----------------------------------------------------------------


def _row(t: float, **kw) -> dict:
    r = {
        "t_ms": t * 1000.0,
        "t": t,
        "state": 1.0,
        "angle_deg": 0.0,
        "gyro_dps": 0.0,
        "setpoint_deg": 0.0,
        "rate_sps": 0.0,
        "loop_us": 50.0,
        "ctrl": 1.0,
        "pos_m": 0.0,
        "pack_v": 7.4,
    }
    r.update(kw)
    return r


def self_test() -> int:
    ok = True

    def check(name: str, cond: bool, got: str) -> None:
        nonlocal ok
        ok = ok and cond
        print(f"self-test {name}: {got}: {'PASS' if cond else 'FAIL'}")

    # timeline: 100 Hz, at +maxrate from 4.00 to 4.50 s, FAULT from 5.00 s,
    # one 30 ms gap in the 2 s window.
    rows = []
    for i in range(600):
        t = i * 0.01
        if 230 <= i < 233:
            continue
        rows.append(
            _row(
                t,
                state=2.0 if t >= 5.0 else 1.0,
                rate_sps=4000.0 if 400 <= i <= 450 else 100.0,
                angle_deg=-1.0 + 0.001 * i,
            )
        )
    f = first_fault(rows)
    stretches = maxrate_stretches(rows, 4000.0)
    win = windows(rows)
    check(
        "timeline fault", f is not None and abs(rows[f]["t"] - 5.0) < 1e-9, f"row {f}"
    )
    check(
        "timeline maxrate",
        len(stretches) == 1
        and abs(rows[stretches[0][0]]["t"] - 4.0) < 1e-9
        and abs(rows[stretches[0][1]]["t"] - 4.5) < 1e-9,
        f"{[(rows[s]['t'], rows[e]['t']) for s, e in stretches]}",
    )
    check(
        "timeline windows",
        len(win) == 6
        and abs(win[2]["gap_max"] - 40.0) < 1e-6
        and win[5]["states"] == [2],
        f"{len(win)} windows, gap {win[2]['gap_max']:.0f} ms",
    )

    # terms: pitch -2 deg, gyro +10 deg/s, pos 0.05 m, rate 800 steps/s.
    spm = steps_per_m(0.065, 1600.0)
    k = [-33.658, -4.8507, -6.2135, -5.5327]
    u = lqr_terms(
        _row(0.0, angle_deg=-2.0, gyro_dps=10.0, pos_m=0.05, rate_sps=800.0), k, spm
    )
    want = [
        33.658 * math.radians(2.0),
        4.8507 * -math.radians(10.0),
        6.2135 * 0.05,
        5.5327 * 800.0 / spm,
    ]
    check(
        "terms",
        all(abs(a - b) < 1e-9 for a, b in zip(u, want, strict=True)),
        " ".join(f"{x:+.4f}" for x in u),
    )

    # accel: theta = A sin(wt) gives a_imu = (g + L w^2) theta; the wheels
    # accelerate at a constant 0.5 m/s^2.
    amp, w, l_eff, acc = math.radians(3.0), 2 * math.pi * 1.0, 0.21, 0.5
    rows = [
        _row(
            i * 0.01,
            angle_deg=-math.degrees(amp * math.sin(w * i * 0.01)),
            gyro_dps=-math.degrees(amp * w * math.cos(w * i * 0.01)),
            rate_sps=acc * i * 0.01 * spm,
        )
        for i in range(200)
    ]
    i = 25  # theta at its peak
    a_cmd, a_imu = implied_accel(rows, i, spm, l_eff)
    a_want = (G + l_eff * w * w) * amp
    check(
        "accel",
        abs(a_cmd - acc) < 1e-6 and abs(a_imu - a_want) / a_want < 0.02,
        f"a_cmd {a_cmd:+.3f} a_imu {a_imu:+.3f} (want {a_want:+.3f})",
    )

    # gyro RMS: a 15 Hz, 10 deg/s wobble shows; a 0.5 Hz swing does not.
    def wobble(hz: float) -> list[dict]:
        return [
            _row(i * 0.01, gyro_dps=10.0 * math.sin(2 * math.pi * hz * i * 0.01))
            for i in range(300)
        ]

    hi, lo = gyro_hf_rms(wobble(15.0), 0.5, 2.5), gyro_hf_rms(wobble(0.5), 0.5, 2.5)
    check(
        "gyro rms",
        hi is not None and lo is not None and 6.0 < hi < 9.0 and lo < 0.2,
        f"15 Hz {hi:.2f}, 0.5 Hz {lo:.3f} deg/s",
    )
    return 0 if ok else 1


def main() -> int:
    ap = argparse.ArgumentParser(description="Look inside a balancebot run log.")
    ap.add_argument("--self-test", action="store_true")
    sub = ap.add_subparsers(dest="cmd")

    def common(p, model: bool) -> None:
        p.add_argument("log", type=Path)
        if model:
            p.add_argument(
                "--wheel-diam", type=float, default=0.065, help="wheel_diam_m"
            )
            p.add_argument("--steps-per-rev", type=float, default=1600.0)

    p = sub.add_parser("timeline", help="1 s summary, the fall, the maxrate stretches")
    common(p, False)
    p.add_argument("--maxrate", type=float, default=4000.0, help="steps/s, as set")
    p = sub.add_parser("terms", help="the LQR's acceleration, term by term")
    common(p, True)
    p.add_argument(
        "--k", type=float, nargs=4, required=True, metavar="K", help="from stat"
    )
    p.add_argument("--t0", type=float, required=True)
    p.add_argument("--t1", type=float, required=True)
    p = sub.add_parser("accel", help="IMU-implied against commanded acceleration")
    common(p, True)
    p.add_argument("--l-eff", type=float, default=0.21, help="l_eff_m")
    p.add_argument("--t0", type=float, required=True)
    p.add_argument("--t1", type=float, required=True)
    p.add_argument("--steady", type=float, nargs=2, metavar=("S0", "S1"), required=True)

    args = ap.parse_args()
    if args.self_test:
        return self_test()
    if args.cmd is None:
        ap.error("a subcommand is required")
    try:
        rows = read_log(args.log)
    except (OSError, ValueError, KeyError) as e:
        print(f"ERROR: {e}", file=sys.stderr)
        return 1
    if args.cmd == "timeline":
        cmd_timeline(rows, args.maxrate)
        return 0
    spm = steps_per_m(args.wheel_diam, args.steps_per_rev)
    if args.cmd == "terms":
        cmd_terms(rows, args.k, spm, args.t0, args.t1)
    else:
        cmd_accel(rows, spm, args.l_eff, args.t0, args.t1, *args.steady)
    return 0


if __name__ == "__main__":
    sys.exit(main())
