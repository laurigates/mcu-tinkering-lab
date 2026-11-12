"""In-host fit and fixture-sensitivity probe for entries/snapfit (revision)."""

import dataclasses
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import design as D
from build123d import Box, Pos, Rot

fx = D.fx


def vol(a, b):
    try:
        return (a & b).volume
    except Exception:
        return float("nan")


P = D.printed_parts()
base, yoke, cradle = P["base"], P["yoke"], P["cradle"]
head = D.HEAD_LOC * fx.head(screws=False)
j2 = D.HEAD_LOC * (
    Pos(0, fx.J2_CENTRE_YZ[0], fx.J2_CENTRE_YZ[1])
    * __import__("build123d").Box(
        fx.J2_KEEPOUT[0],
        fx.J2_KEEPOUT[1],
        fx.J2_KEEPOUT[2],
        align=(D.MAX, D.CEN, D.CEN),
    )
)
print(
    "cradle & head",
    round(vol(cradle, head), 2),
    " cradle & J2 keep-out",
    round(vol(cradle, j2), 2),
)


def run(label, spec):
    # servo located by its tabs: tab underside at the same place as the nominal
    dz = (spec.height - spec.tab_underside) - (D.SERVO.height - D.SERVO.tab_underside)
    pan_s = Pos(0, 0, D.Z_PAN_TOP + dz) * fx.servo(spec)
    pan_h = Pos(0, 0, D.Z_PAN_TOP + dz) * fx.horn("cross", spec)
    tl = Pos(0, dz, 0) * D.tilt_servo_loc()
    tilt_s = tl * fx.servo(spec)
    tilt_h = tl * Rot(0, 0, 180) * fx.horn("single", spec)
    r = {
        "base/pan-servo": vol(base, pan_s),
        "yoke/pan-servo": vol(yoke, pan_s),
        "yoke/pan-horn": vol(yoke, pan_h),
        "yoke/tilt-servo": vol(yoke, tilt_s),
        "cradle/tilt-horn": vol(cradle, tilt_h),
        "cradle/tilt-servo": vol(cradle, tilt_s),
    }
    # horn arm engagement: share of the horn's arms inside the pocket slabs
    from build123d import Box

    arm_p = pan_h - Pos(0, 0, D.Z_PAN_TOP + dz) * __import__("build123d").Cylinder(
        3.6, 10
    )
    slab = Pos(0, 0, D.YOKE_B) * Box(
        80, 80, D.YOKE_T - D.YOKE_B, align=(D.CEN, D.CEN, D.MIN)
    )
    eng_p = vol(arm_p, slab) / arm_p.volume
    arm_t = tilt_h - tl * __import__("build123d").Cylinder(3.6, 10)
    slab_t = Pos(0, -D.CHEEK_OUT_N, 0) * Box(
        80, D.CHEEK_T_N, 200, align=(D.CEN, D.MIN, D.CEN)
    )
    eng_t = vol(arm_t, slab_t) / arm_t.volume
    print(
        f"{label:28s}",
        {k: round(v, 1) for k, v in r.items()},
        f"pan arm engaged {eng_p:.2f}  tilt arm engaged {eng_t:.2f}",
    )


S = D.SERVO
run("nominal", S)
for dC in (-1.0, -0.5, 0.5, 1.0):
    run(
        f"C{dC:+} (C-F{dC:+})",
        dataclasses.replace(S, height=S.height + dC, overall=S.overall + dC),
    )
for dF in (1.0, 2.0):
    run(
        f"C,F{dF:+} (C-F same)",
        dataclasses.replace(
            S,
            height=S.height + dF,
            tab_underside=S.tab_underside + dF,
            overall=S.overall + dF,
        ),
    )
run(
    "width 12.2 length 22.8",
    dataclasses.replace(S, width=12.2, length=22.8, shaft_from_end=6.1),
)

# tilt servo restraint: push the servo 0.6 mm toward the open groove end (+y_s)
tl = D.tilt_servo_loc()
moved = tl * Pos(0, 0.6, 0) * fx.servo(S)
cx = S.shaft_from_end - S.length / 2

for name, x0, x1 in (
    ("lower tab", cx - S.tab_span / 2 - 1, S.shaft_from_end - S.length),
    ("upper tab", S.shaft_from_end, cx + S.tab_span / 2 + 1),
):
    zone = tl * (Pos((x0 + x1) / 2, 0, -5) * Box(x1 - x0, 40, 20))
    print(
        f"servo pushed +0.6 toward the open end, {name} meets nose:",
        round(vol(yoke & zone, moved), 2),
        "mm3",
    )
