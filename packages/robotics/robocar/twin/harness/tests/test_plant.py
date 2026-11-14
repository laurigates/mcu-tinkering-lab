"""The plant port against tests/fake_plant.py: no twin and no simulation."""

import json
import sys
import threading

import pytest
from robocar_twin.plant import (
    PhysicsPlant,
    PlantError,
    PlantReading,
    PlantRequest,
    PlantTimeout,
    ScriptedPlant,
    default_simulation_dir,
    plant_command,
    plant_for,
)
from robocar_twin.runner import default_twin_dir
from robocar_twin.scenario import load_scenario

ARENA = {"floor": [[-50, -60], [100, -60], [100, 60], [-50, 60]]}
WALLED = {**ARENA, "walls": [{"from": [100, -60], "to": [100, 60]}]}
START = {"x_cm": 0, "y_cm": 0, "heading_deg": 0}
FORWARD = {"drive": "forward", "duty": 4095}
COAST = {"drive": "coast", "duty": 0}


def request(tick, left=COAST, right=COAST):
    return PlantRequest(id=tick, at_ms=tick * 33, dt_ms=33, left=left, right=right)


def physics(cmd, *extra, arena=WALLED):
    return PhysicsPlant(7, arena, START, cmd=[*cmd, *extra])


def test_init_then_one_reading_per_step(fake_plant_cmd, tmp_path):
    log = tmp_path / "plant.log"
    plant = physics(fake_plant_cmd, "--log", str(log))
    plant.start(10)
    try:
        still = plant.step(request(0), 10)
        moved = plant.step(request(1, FORWARD, FORWARD), 10)
    finally:
        plant.close()
    assert still.at_ms == 33 and moved.at_ms == 66
    assert still.pose == {"x_cm": 0.0, "y_cm": 0.0, "heading_deg": 0.0}
    assert moved.pose["x_cm"] == 1.98  # 60 cm/s for 33 ms
    assert still.range_cm == {"front": 92, "rear": None}
    assert moved.floor == {"front": True, "rear": True}
    assert plant.steps == 2 and plant.returncode == 0
    # What the plant was sent: init with exactly its four keys, then steps
    # without the twin's id and at_ms. The fake refuses any other key.
    sent = [json.loads(line) for line in log.read_text().splitlines()]
    assert sent[0] == {"t": "init", "seed": 7, "arena": WALLED, "start": START}
    assert sent[1] == {"t": "step", "dt_ms": 33, "left": COAST, "right": COAST}
    assert sent[2] == {"t": "step", "dt_ms": 33, "left": FORWARD, "right": FORWARD}


def test_reading_becomes_the_twin_response_and_the_pose_state():
    reading = PlantReading(
        at_ms=66,
        pose={"x_cm": 1.98, "y_cm": 0.0, "heading_deg": 0.0},
        range_cm={"front": 90, "rear": None},
        floor={"front": True, "rear": False},
        collided=False,
        off_floor=True,
    )
    assert reading.to_protocol(1) == {
        "t": "plant_response",
        "id": 1,
        "range_cm": {"front": 90, "rear": None},
        "floor": {"front": True, "rear": False},
    }
    assert reading.pose_state() == {
        "x_cm": 1.98,
        "y_cm": 0.0,
        "heading_deg": 0.0,
        "collided": False,
        "off_floor": True,
    }


def test_a_plant_that_exits_fails_with_its_status_and_stderr(fake_plant_cmd):
    plant = physics(fake_plant_cmd, "--exit-after", "2")
    plant.start(10)
    plant.step(request(0), 10)
    plant.step(request(1), 10)
    with pytest.raises(PlantError) as info:
        plant.step(request(2), 10)
    assert "exited with status 3 before sending 'reading'" in str(info.value)
    assert "fake plant: giving up" in str(info.value)
    assert plant.returncode == 3  # reaped


def test_an_error_line_fails_with_the_plants_message(fake_plant_cmd):
    plant = physics(fake_plant_cmd, "--error-after", "1")
    plant.start(10)
    plant.step(request(0), 10)
    with pytest.raises(PlantError, match=r"reported an error: step\.left\.duty"):
        plant.step(request(1), 10)
    assert plant.returncode == 1


def test_a_bad_init_fails_at_start(fake_plant_cmd):
    plant = physics(fake_plant_cmd, arena={"walls": []})
    with pytest.raises(PlantError, match="reported an error: arena: missing 'floor'"):
        plant.start(10)
    assert plant.returncode == 1


@pytest.mark.parametrize(
    ("flag", "match"),
    [
        ("--ready-twice", "answered out of order: expected 'reading', got"),
        ("--skew", "its reading is at 34 ms, and plant_request 0 ends at 33 ms"),
    ],
)
def test_an_answer_out_of_order_fails(fake_plant_cmd, flag, match):
    plant = physics(fake_plant_cmd, flag)
    plant.start(10)
    with pytest.raises(PlantError, match=match):
        plant.step(request(0), 10)
    assert plant.returncode is not None


def test_a_plant_that_never_answers_times_out_and_is_killed(fake_plant_cmd):
    plant = physics(fake_plant_cmd, "--hang")
    with pytest.raises(PlantTimeout, match="no 'ready' within"):
        plant.start(0.5)
    assert plant.returncode is not None


def test_a_hung_plant_behind_a_wrapper_is_killed_with_it(fake_plant_cmd):
    # The default command is `uv run ... python -m robocar_sim.plant`, so the
    # stepper is a grandchild. It holds the stdout and stderr pipes: with only
    # the wrapper killed, close() waits on them for as long as the stepper
    # lives. start() runs in a thread so that this fails and does not block.
    wrapper = "import subprocess, sys; sys.exit(subprocess.call(sys.argv[1:]))"
    plant = physics([sys.executable, "-c", wrapper, *fake_plant_cmd], "--hang")
    raised = []

    def start():
        try:
            plant.start(0.5)
        except PlantError as exc:
            raised.append(exc)

    thread = threading.Thread(target=start, daemon=True)
    thread.start()
    thread.join(timeout=20)
    assert not thread.is_alive(), "close() is waiting on the stepper's pipes"
    assert isinstance(raised[0], PlantTimeout)
    assert plant.returncode is not None


def test_a_hung_plant_whose_wrapper_has_exited_is_killed(fake_plant_cmd):
    # The wrapper starts the stepper and exits at once, so close() finds the
    # direct child already gone while the stepper still holds the pipes.
    wrapper = "import subprocess, sys; subprocess.Popen(sys.argv[1:])"
    plant = physics([sys.executable, "-c", wrapper, *fake_plant_cmd], "--hang")
    raised = []

    def start():
        try:
            plant.start(0.5)
        except PlantError as exc:
            raised.append(exc)

    thread = threading.Thread(target=start, daemon=True)
    thread.start()
    thread.join(timeout=20)
    assert not thread.is_alive(), "close() is waiting on the stepper's pipes"
    assert isinstance(raised[0], PlantError)


def test_a_command_that_cannot_start(tmp_path):
    plant = PhysicsPlant(1, ARENA, START, cmd=[str(tmp_path / "nope")])
    with pytest.raises(PlantError, match="cannot start"):
        plant.start(5)
    plant.close()


def test_close_is_safe_before_start_and_twice(fake_plant_cmd):
    plant = physics(fake_plant_cmd)
    plant.close()
    plant.start(10)
    plant.close()
    plant.close()
    assert plant.returncode == 0


@pytest.mark.parametrize(
    "patch",
    [
        {"at_ms": 33.5},
        {"pose": {"x_cm": 0.0, "y_cm": 0.0}},
        {"pose": {"x_cm": "0", "y_cm": 0.0, "heading_deg": 0.0}},
        {"range_cm": {"front": "near", "rear": None}},
        {"floor": {"front": 1, "rear": True}},
        {"collided": 0},
        {"off_floor": None},
    ],
)
def test_a_malformed_reading_is_refused(patch):
    good = {
        "t": "reading",
        "at_ms": 33,
        "pose": {"x_cm": 0.0, "y_cm": 0.0, "heading_deg": 0.0},
        "range_cm": {"front": 38, "rear": None},
        "floor": {"front": True, "rear": True},
        "collided": False,
        "off_floor": False,
    }
    PlantReading.from_message(good)
    with pytest.raises((KeyError, TypeError, ValueError)):
        PlantReading.from_message({**good, **patch})


@pytest.mark.parametrize(
    "patch",
    [
        {"id": "7"},
        {"dt_ms": 33.0},
        {"left": {"drive": "ahead", "duty": 0}},
        {"right": {"drive": "coast", "duty": 0.5}},
        {"right": {"drive": "coast", "duty": True}},
        {"right": {"drive": "coast"}},
    ],
)
def test_a_malformed_plant_request_is_refused(patch):
    good = {
        "t": "plant_request",
        "id": 7,
        "at_ms": 231,
        "dt_ms": 33,
        "left": FORWARD,
        "right": COAST,
    }
    # The wheels differ, so one sent as the other does not compare equal.
    assert PlantRequest.from_record(good).to_step() == {
        "t": "step",
        "dt_ms": 33,
        "left": FORWARD,
        "right": COAST,
    }
    with pytest.raises((KeyError, TypeError, ValueError)):
        PlantRequest.from_record({**good, **patch})


def test_scripted_plant_has_nothing_to_step():
    plant = ScriptedPlant()
    plant.start(1)
    with pytest.raises(PlantError, match="the scenario's plant is scripted"):
        plant.step(request(0), 1)
    plant.close()


def test_default_command_runs_the_simulation_next_to_this_harness(monkeypatch):
    monkeypatch.delenv("ROBOCAR_PLANT_CMD", raising=False)
    sim = default_simulation_dir()
    assert sim == default_twin_dir().parent / "simulation"
    assert (sim / "src" / "robocar_sim" / "plant.py").is_file()
    assert plant_command() == [
        "uv",
        "run",
        "--project",
        str(sim),
        "--locked",
        "--quiet",
        "python",
        "-m",
        "robocar_sim.plant",
    ]


def test_plant_cmd_env_is_split_like_a_shell_line(monkeypatch):
    monkeypatch.setenv("ROBOCAR_PLANT_CMD", "python 'my plant.py' --seed 3")
    assert plant_command() == ["python", "my plant.py", "--seed", "3"]
    monkeypatch.setenv("ROBOCAR_PLANT_CMD", "")
    assert plant_command()[:2] == ["uv", "run"]


PHYSICS = """\
seed: 7
duration: 30s
world:
  - {{at: 0s, frame: {{scene: room}}}}
models: {{planner: inline}}
plant:
  physics:
    arena: {arena}
    start: {{x_cm: 1.5, y_cm: 0, heading_deg: 0}}
default: {{status: 200, call: stop}}
expect: [{{requests: {{max: 10}}}}]
"""


def test_plant_for_follows_the_scenario(
    write_scenario, write_arena, fake_plant_cmd, monkeypatch
):
    arena = write_arena("room", "floor: [[0, 0], [100, 0], [100, 100]]\n")
    block = PHYSICS.format(arena=arena).split("plant:")[1].split("default:")[0]
    scripted = load_scenario(
        write_scenario("s", PHYSICS.format(arena=arena).replace(block, " scripted\n"))
    )
    physical = load_scenario(write_scenario("p", PHYSICS.format(arena=arena)))
    assert physical.plant_kind == "physics"
    plant = plant_for(physical, cmd=fake_plant_cmd)
    assert isinstance(plant, PhysicsPlant)
    plant.start(10)
    try:
        # The scenario's seed, arena and start reach the plant.
        assert plant.step(request(0), 10).pose["x_cm"] == 1.5
    finally:
        plant.close()
    # Without `cmd` the command comes from the environment when the port is made.
    monkeypatch.setenv("ROBOCAR_PLANT_CMD", " ".join(fake_plant_cmd))
    from_env = plant_for(physical)
    from_env.start(10)
    from_env.close()
    assert from_env.returncode == 0
    assert scripted.plant_kind == "scripted"
    assert isinstance(plant_for(scripted), ScriptedPlant)
