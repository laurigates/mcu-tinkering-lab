"""Step the plant through scripted wheel commands and write the pose trace.

``python -m robocar_sim.plant_demo --out trace.jsonl`` runs the plant alone,
with no twin and no firmware: a fixed list of wheel commands in 33 ms steps.
The trace is one ``reading`` line per step, as the stepper writes them. With
``--plot`` and matplotlib importable (the ``viz`` extra) it also draws the
arena and the path; without matplotlib the plot is skipped and the trace is
still written.

The default arena is a walled room with one box and one cylinder. The script
drives at the far wall and holds full duty against it, so the trace shows the
front range closing, ``collided`` turning true and the pose holding at the
wall.
"""

from __future__ import annotations

import argparse
import importlib.util
import json
import sys
from pathlib import Path

from .arena import Arena, ArenaError, Box, Cylinder, load_arena, parse_arena
from .plant import Plant, PlantError, Pose, Reading, WheelCmd

TICK_MS = 33

# Centimetres and degrees, the arena schema of robocar_sim.arena.
DEMO_ARENA = {
    "floor": [[-60, -80], [220, -80], [220, 80], [-60, 80]],
    "walls": [
        {"from": [-60, -80], "to": [220, -80]},
        {"from": [220, -80], "to": [220, 80]},
        {"from": [220, 80], "to": [-60, 80]},
        {"from": [-60, 80], "to": [-60, -80]},
    ],
    "obstacles": [
        {"box": {"center": [120, -45], "size": [20, 30], "heading_deg": 20}},
        {"cylinder": {"center": [70, 50], "radius_cm": 8}},
    ],
}

# (duration in ms, left wheel, right wheel)
DEMO_SCRIPT: tuple[tuple[int, WheelCmd, WheelCmd], ...] = (
    (1980, WheelCmd("forward", 2457), WheelCmd("forward", 2457)),  # straight at 60 %
    (990, WheelCmd("forward", 2457), WheelCmd("forward", 1228)),  # arc to the right
    (990, WheelCmd("coast"), WheelCmd("coast")),  # roll
    (495, WheelCmd("reverse", 2457), WheelCmd("forward", 2457)),  # turn in place, left
    (495, WheelCmd("brake"), WheelCmd("brake")),  # stop
    (5940, WheelCmd("forward", 4095), WheelCmd("forward", 4095)),  # into the wall, held
    (990, WheelCmd("reverse", 2457), WheelCmd("reverse", 2457)),  # back off
    (495, WheelCmd("brake"), WheelCmd("brake")),
)


def run_demo(arena: Arena, seed: int, start: Pose) -> list[Reading]:
    """Run the script and return one reading per 33 ms step."""
    plant = Plant(arena, seed, start=start)
    readings = []
    for duration_ms, left, right in DEMO_SCRIPT:
        for _ in range(duration_ms // TICK_MS):
            readings.append(plant.step(TICK_MS, left, right))
    return readings


def write_trace(readings: list[Reading], path: Path) -> None:
    with open(path, "w", encoding="utf-8") as f:
        for reading in readings:
            f.write(json.dumps(reading.to_message(), separators=(",", ":")) + "\n")


def plot_available() -> bool:
    return importlib.util.find_spec("matplotlib") is not None


def write_plot(arena: Arena, readings: list[Reading], path: Path) -> None:
    """Draw the arena and the path to an image file. Needs matplotlib."""
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.patches import Circle, Polygon, Rectangle

    fig, ax = plt.subplots(figsize=(8, 6))
    ax.add_patch(Polygon(arena.floor, closed=True, facecolor="#f2f2f2", edgecolor="#999999"))
    for wall in arena.walls:
        ax.plot([wall.start[0], wall.end[0]], [wall.start[1], wall.end[1]], color="black", lw=2)
    for obstacle in arena.obstacles:
        if isinstance(obstacle, Box):
            ax.add_patch(
                Rectangle(
                    (
                        obstacle.center[0] - obstacle.size[0] / 2,
                        obstacle.center[1] - obstacle.size[1] / 2,
                    ),
                    obstacle.size[0],
                    obstacle.size[1],
                    angle=obstacle.heading_deg,
                    rotation_point="center",
                    facecolor="#b0b0b0",
                    edgecolor="black",
                )
            )
        elif isinstance(obstacle, Cylinder):
            ax.add_patch(
                Circle(obstacle.center, obstacle.radius_cm, facecolor="#b0b0b0", edgecolor="black")
            )
    xs = [r.pose.x_cm for r in readings]
    ys = [r.pose.y_cm for r in readings]
    ax.plot(xs, ys, color="tab:blue", lw=1.5, label="path (axle midpoint)")
    hit = [r for r in readings if r.collided]
    if hit:
        ax.scatter(
            [r.pose.x_cm for r in hit],
            [r.pose.y_cm for r in hit],
            color="tab:red",
            s=12,
            zorder=3,
            label="collided",
        )
    ax.scatter(xs[:1], ys[:1], color="tab:green", zorder=4, label="start")
    ax.set_aspect("equal")
    ax.autoscale_view()
    ax.set_xlabel("x (cm)")
    ax.set_ylabel("y (cm)")
    ax.set_title("robocar_sim.plant demo")
    ax.legend(loc="upper left")
    fig.savefig(path, dpi=120, bbox_inches="tight")
    plt.close(fig)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        prog="python -m robocar_sim.plant_demo", description=(__doc__ or "").split("\n\n")[0]
    )
    parser.add_argument("--arena", type=Path, help="arena YAML file (default: the built-in room)")
    parser.add_argument("--seed", type=int, default=0, help="seed for the rangefinder noise")
    parser.add_argument(
        "--out", type=Path, default=Path("plant_demo_trace.jsonl"), help="pose trace, JSON lines"
    )
    parser.add_argument(
        "--plot", type=Path, help="image of the arena and the path (needs matplotlib)"
    )
    args = parser.parse_args(argv)

    try:
        arena = load_arena(args.arena) if args.arena else parse_arena(DEMO_ARENA)
        readings = run_demo(arena, args.seed, Pose(0.0, 0.0, 0.0))
    except (ArenaError, PlantError, OSError) as e:
        print(f"error: {e}", file=sys.stderr)
        return 2
    write_trace(readings, args.out)
    last = readings[-1]
    print(
        f"{len(readings)} steps, {last.at_ms} ms, final pose "
        f"({last.pose.x_cm:.1f}, {last.pose.y_cm:.1f}) cm, {last.pose.heading_deg:.1f} deg, "
        f"{sum(r.collided for r in readings)} steps in contact: {args.out}"
    )
    if args.plot:
        if plot_available():
            write_plot(arena, readings, args.plot)
            print(f"plot: {args.plot}")
        else:
            print(
                "plot skipped: matplotlib is not installed (uv sync --extra viz)", file=sys.stderr
            )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
