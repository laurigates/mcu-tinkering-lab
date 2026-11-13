"""The standalone plant demo, ``python -m robocar_sim.plant_demo``."""

import json

from robocar_sim.arena import parse_arena
from robocar_sim.plant import Pose
from robocar_sim.plant_demo import DEMO_ARENA, DEMO_SCRIPT, TICK_MS, main, plot_available, run_demo

STEPS = sum(duration // TICK_MS for duration, _, _ in DEMO_SCRIPT)


def test_the_script_reaches_the_wall_and_backs_off():
    readings = run_demo(parse_arena(DEMO_ARENA), 0, Pose(0.0, 0.0, 0.0))

    assert len(readings) == STEPS
    assert readings[-1].at_ms == STEPS * TICK_MS
    assert any(r.collided for r in readings)
    assert not readings[-1].collided
    assert not any(r.off_floor for r in readings)


def test_main_writes_one_reading_per_step(tmp_path, capsys):
    trace = tmp_path / "trace.jsonl"

    status = main(["--out", str(trace), "--seed", "3"])

    assert status == 0
    rows = [json.loads(line) for line in trace.read_text().splitlines()]
    assert len(rows) == STEPS
    assert {row["t"] for row in rows} == {"reading"}
    assert [row["at_ms"] for row in rows[:3]] == [33, 66, 99]
    assert str(trace) in capsys.readouterr().out


def test_main_reads_an_arena_file(tmp_path):
    arena = tmp_path / "open.yaml"
    arena.write_text("floor:\n  - [-500, -500]\n  - [500, -500]\n  - [500, 500]\n  - [-500, 500]\n")
    trace = tmp_path / "trace.jsonl"

    assert main(["--arena", str(arena), "--out", str(trace)]) == 0

    rows = [json.loads(line) for line in trace.read_text().splitlines()]
    assert not any(row["collided"] for row in rows), "the open arena has no wall"


def test_main_reports_a_bad_arena(tmp_path, capsys):
    arena = tmp_path / "bad.yaml"
    arena.write_text("walls: []\n")

    assert main(["--arena", str(arena), "--out", str(tmp_path / "trace.jsonl")]) == 2

    assert "arena: missing key(s) floor" in capsys.readouterr().err


def test_the_plot_is_drawn_only_where_matplotlib_is_installed(tmp_path, capsys):
    plot = tmp_path / "path.png"

    status = main(["--out", str(tmp_path / "trace.jsonl"), "--plot", str(plot)])

    assert status == 0
    if plot_available():
        assert plot.stat().st_size > 0
    else:
        assert not plot.exists()
        assert "plot skipped" in capsys.readouterr().err
