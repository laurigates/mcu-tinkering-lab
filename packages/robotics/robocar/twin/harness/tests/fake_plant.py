"""A stand-in for ``python -m robocar_sim.plant``, speaking its stepper protocol.

It is not a model of the robot. The robot is a point on the x axis that moves
at ``duty / 4095 * 60`` cm/s while both wheels are ``forward`` (backward for
``reverse``) and stands still otherwise: no ramp, no coasting, no turning. The
arena is read as two numbers, the largest x of the ``floor`` polygon and, when
there is a wall, the x of its first point. The front rangefinder is 8 cm ahead
of the point and reads the distance to the wall in whole centimetres, or null
outside 2..400 or with no wall; the front floor sensor is 7 cm ahead. The
robot stops at the wall (``collided``) and is ``off_floor`` past the floor's
largest x. The rear sensors always read null and floor.

Like the real stepper it refuses a message with a missing or an extra key, with
one ``error`` line and exit status 1, which is how the harness tests show that
the twin's ``id`` and ``at_ms`` are not forwarded.

Test-only arguments: ``--exit-after N`` (write to stderr and exit 3 instead of
answering step N+1), ``--error-after N`` (answer step N+1 with an ``error``
line), ``--ready-twice`` (answer the first step with a second ``ready``),
``--skew`` (report every reading 1 ms late), ``--hang`` (never answer
``init``), ``--null-range`` (every range null), ``--bump-at N`` (reading N
alone reports ``collided``, at whatever pose it has), ``--log FILE`` (append
each message received to FILE).
"""

from __future__ import annotations

import argparse
import json
import sys
import time

FRONT_RANGE_X = 8.0
FRONT_FLOOR_X = 7.0
SPEED_CM_S = 60.0
INIT_KEYS = {"t", "seed", "arena", "start"}
START_KEYS = {"x_cm", "y_cm", "heading_deg"}
STEP_KEYS = {"t", "dt_ms", "left", "right"}
WHEEL_KEYS = {"drive", "duty"}


def out(obj: dict) -> None:
    sys.stdout.write(json.dumps(obj, separators=(",", ":")) + "\n")
    sys.stdout.flush()


def error(message: str) -> int:
    out({"t": "error", "message": message})
    return 1


def keys_problem(what: str, obj: object, want: set[str]) -> str | None:
    if not isinstance(obj, dict):
        return f"{what}: expected an object"
    if set(obj) != want:
        return f"{what}: expected exactly the keys {sorted(want)}, got {sorted(obj)}"
    return None


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--exit-after", type=int)
    ap.add_argument("--error-after", type=int)
    ap.add_argument("--ready-twice", action="store_true")
    ap.add_argument("--skew", action="store_true")
    ap.add_argument("--hang", action="store_true")
    ap.add_argument("--null-range", action="store_true")
    ap.add_argument("--bump-at", type=int)
    ap.add_argument("--log")
    args = ap.parse_args()

    def log(line: str) -> None:
        if args.log:
            with open(args.log, "a", encoding="utf-8") as fh:
                fh.write(line if line.endswith("\n") else line + "\n")

    first = sys.stdin.readline()
    log(first)
    if args.hang:
        time.sleep(3600)
    init = json.loads(first)
    problem = keys_problem("init", init, INIT_KEYS) or keys_problem(
        "init.start", init["start"], START_KEYS
    )
    if problem is None and init["t"] != "init":
        problem = f"expected init, got {init['t']!r}"
    if problem is None and "floor" not in init["arena"]:
        problem = "arena: missing 'floor'"
    if problem:
        return error(problem)
    arena = init["arena"]
    floor_max_x = max(p[0] for p in arena["floor"])
    walls = arena.get("walls") or []
    wall_x = walls[0]["from"][0] if walls else None
    x = float(init["start"]["x_cm"])
    at_ms = 0
    out({"t": "ready"})

    steps = 0
    for line in sys.stdin:
        log(line)
        if not line.strip():
            continue
        if args.exit_after is not None and steps >= args.exit_after:
            print("fake plant: giving up", file=sys.stderr)
            return 3
        if args.error_after is not None and steps >= args.error_after:
            return error("step.left.duty: expected 0..4095")
        if args.ready_twice and steps == 0:
            out({"t": "ready"})
            steps += 1
            continue
        step = json.loads(line)
        problem = (
            keys_problem("step", step, STEP_KEYS)
            or keys_problem("step.left", step["left"], WHEEL_KEYS)
            or keys_problem("step.right", step["right"], WHEEL_KEYS)
        )
        if problem is None and step["t"] != "step":
            problem = f"expected step, got {step['t']!r}"
        if problem:
            return error(problem)
        steps += 1
        dt_ms = step["dt_ms"]
        at_ms += dt_ms
        drives = {step["left"]["drive"], step["right"]["drive"]}
        duty = min(step["left"]["duty"], step["right"]["duty"])
        sign = {"forward": 1.0, "reverse": -1.0}.get(
            drives.pop() if len(drives) == 1 else "", 0.0
        )
        x += sign * duty / 4095 * SPEED_CM_S * dt_ms / 1000
        collided = False
        if wall_x is not None and x + FRONT_RANGE_X >= wall_x:
            x = wall_x - FRONT_RANGE_X
            collided = True
        if args.bump_at == steps:
            collided = True
        front = None
        if wall_x is not None and not args.null_range:
            cm = round(wall_x - (x + FRONT_RANGE_X))
            front = cm if 2 <= cm <= 400 else None
        out(
            {
                "t": "reading",
                "at_ms": at_ms + (1 if args.skew else 0),
                "pose": {"x_cm": round(x, 3), "y_cm": 0.0, "heading_deg": 0.0},
                "range_cm": {"front": front, "rear": None},
                "floor": {"front": x + FRONT_FLOOR_X <= floor_max_x, "rear": True},
                "collided": collided,
                "off_floor": x > floor_max_x,
            }
        )
    return 0


if __name__ == "__main__":
    sys.exit(main())
