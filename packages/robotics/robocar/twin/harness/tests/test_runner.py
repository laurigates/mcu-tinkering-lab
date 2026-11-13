import itertools
import json

import pytest
from robocar_twin.assertions import evaluate
from robocar_twin.cli import main as cli_main
from robocar_twin.plant import plant_for
from robocar_twin.runner import (
    TwinError,
    TwinTimeout,
    resolve_twin_bin,
    run_scenario,
    usage_tokens,
)
from robocar_twin.scenario import build_body, load_scenario

A4_LIKE = """\
seed: 7
duration: 60s
world:
  - {at: 0s, frame: {scene: kitchen-table, pose: C}}
  - {at: 5s, console: status}
  - {at: 16s, voice_turn: {dur: 2s, reply: "A blue cup."}}
  - {at: 25s, console: status}
models: {planner: inline}
responses:
  - {status: 200, latency: 4s, call: stop, speak: "Some strange noise.", tokens: 100}
  - {status: 200, latency: 1500ms, call: drive, speak: "The table looks tidy.",
     args: {heading_deg: 0, distance_cm: 20, speed_pct: 40}, tokens: {prompt: 40, output: 2}}
default: {status: 429, latency: 300ms}
expect:
  - suppressed: {reason: person_spoke, between: [15s, 20s], count: 1}
  - no_utterance: {source: planner, between: [15s, 29s]}
  - utterance: {source: voice_turn, after: "voice_turn[0]", within: 4s, count: 1}
  - utterance: {source: planner, between: [30s, 35s], count: 1, text_contains: tidy}
  - goals: {sequence: [stop, drive]}
  - requests: {count: 2}
"""


def test_runs_fake_twin_end_to_end(write_scenario, fake_twin_cmd):
    scn = load_scenario(write_scenario("a4-like", A4_LIKE))
    result = run_scenario(scn, fake_twin_cmd, timeout_s=20)
    assert result.trace[-1]["t"] == "done"
    assert result.request_count == 2
    assert result.tokens == 142
    # The harness logs each reply into the trace at the time the twin handles it.
    replies = [r for r in result.trace if r["t"] == "model_response"]
    assert [r["at_ms"] for r in replies] == [19000, 31500]
    outcomes = evaluate(scn, result.trace)
    assert all(o.passed for o in outcomes), [
        o.message for o in outcomes if not o.passed
    ]


def test_failing_assertion_against_real_run(write_scenario, fake_twin_cmd):
    body = A4_LIKE.replace(
        "no_utterance: {source: planner, between: [15s, 29s]}",
        "no_utterance: {source: planner, between: [15s, 40s]}",
    )
    scn = load_scenario(write_scenario("a4-fail", body))
    outcomes = evaluate(scn, run_scenario(scn, fake_twin_cmd, timeout_s=20).trace)
    bad = [o for o in outcomes if not o.passed]
    assert len(bad) == 1 and "tidy" in bad[0].message


def test_backoff_records(write_scenario, fake_twin_cmd):
    body = """\
seed: 1
duration: 300s
world:
  - {at: 0s, frame: {scene: hall}}
  - {at: 5s, every: 5s, until: 295s, console: status}
models: {planner: inline}
responses: [{status: 429}, {status: 429}, {status: -1}, {status: 429}]
default: {status: 200, call: stop}
expect:
  - backoff: {sequence: [1, 2, 4, 4]}
  - request_intervals: {between: [15s, 240s], intervals: [30s, 45s, 75s, 75s]}
  - force_stop: {reason: backend_error, count: 4}
  - requests: {between: [241s, 256s], count: 1}
"""
    scn = load_scenario(write_scenario("bo", body))
    outcomes = evaluate(scn, run_scenario(scn, fake_twin_cmd, timeout_s=20).trace)
    assert all(o.passed for o in outcomes), [
        o.message for o in outcomes if not o.passed
    ]


SIMPLE = """\
seed: 1
duration: 30s
world:
  - {at: 0s, frame: {scene: room}}
  - {at: 5s, console: status}
models: {planner: inline}
default: {status: 200, call: stop}
expect: [{requests: {count: 1}}]
"""


def _simple(write_scenario, **config):
    scn = load_scenario(write_scenario("simple", SIMPLE))
    scn.config.update(config)  # test-only keys for the fake twin
    return scn


def test_key_mismatch_is_an_error(write_scenario, fake_twin_cmd):
    with pytest.raises(TwinError, match="not sha256 of the canonical descriptor"):
        run_scenario(
            _simple(write_scenario, fake_bad_key=1), fake_twin_cmd, timeout_s=20
        )


def test_nonzero_exit(write_scenario, fake_twin_cmd):
    with pytest.raises(TwinError, match="exited with status 3"):
        run_scenario(_simple(write_scenario, fake_exit=3), fake_twin_cmd, timeout_s=20)


def test_timeout(write_scenario, fake_twin_cmd):
    with pytest.raises(TwinTimeout, match="timed out"):
        run_scenario(_simple(write_scenario, fake_hang=1), fake_twin_cmd, timeout_s=1)


def test_non_json_line(write_scenario, fake_twin_cmd):
    with pytest.raises(TwinError, match="not JSON"):
        run_scenario(
            _simple(write_scenario, fake_garbage=1), fake_twin_cmd, timeout_s=20
        )


def test_missing_cassette_names_key(write_scenario, fake_twin_cmd):
    body = SIMPLE.replace(
        "models: {planner: inline}\ndefault: {status: 200, call: stop}\n",
        'models: {planner: "replay:cassettes/none"}\n',
    )
    scn = load_scenario(write_scenario("replay-miss", body))
    with pytest.raises(
        TwinError, match=r"no cassette for request 1 at 15000 ms"
    ) as info:
        run_scenario(scn, fake_twin_cmd, timeout_s=20)
    assert '"scene":"room"' in str(info.value)


def test_missing_binary(write_scenario, tmp_path):
    scn = _simple(write_scenario)
    with pytest.raises(TwinError, match="cannot start twin"):
        run_scenario(scn, [str(tmp_path / "nope")], timeout_s=5)


def test_resolve_twin_bin(monkeypatch, tmp_path):
    monkeypatch.setenv("ROBOCAR_TWIN_BIN", "/x/twin")
    assert str(resolve_twin_bin(tmp_path)) == "/x/twin"
    monkeypatch.delenv("ROBOCAR_TWIN_BIN")
    assert resolve_twin_bin(tmp_path) == tmp_path / "build" / "robocar-twin"


def test_usage_tokens():
    assert usage_tokens(200, build_body(tokens=77)) == 77
    assert usage_tokens(429, build_body(tokens=77)) == 0
    assert usage_tokens(200, "not json") == 0
    assert usage_tokens(200, "{}") == 0


def test_cli_scorecard(write_scenario, fake_twin_cmd, tmp_path):
    good = write_scenario("a4-like", A4_LIKE)
    bad = write_scenario(
        "a4-bad", A4_LIKE.replace("requests: {count: 2}", "requests: {count: 5}")
    )
    out = tmp_path / "scorecard.json"
    code = cli_main(
        [
            "run",
            "--scenarios",
            str(good),
            str(bad),
            "--out",
            str(out),
            "--twin",
            " ".join(fake_twin_cmd),
            "--traces",
            str(tmp_path / "traces"),
        ]
    )
    assert code == 1
    card = json.loads(out.read_text())
    assert card["summary"] == {
        "scenarios": 2,
        "passed": 1,
        "failed": 1,
        "requests": 4,
        "tokens": 284,
    }
    by_id = {e["id"]: e for e in card["scenarios"]}
    assert by_id["a4-like"]["passed"] and by_id["a4-like"]["requests"] == 2
    assert "exactly 5, found 2" in by_id["a4-bad"]["failed"][0]["message"]
    assert (tmp_path / "traces" / "a4-like.jsonl").is_file()


def test_cli_check_reports_scenario_errors(write_scenario, capsys):
    p = write_scenario("broken", SIMPLE.replace("seed: 1", "seed: x"))
    assert cli_main(["check", "--scenarios", str(p)]) == 2
    assert "seed" in capsys.readouterr().err


EXECUTOR_LIKE = """\
seed: 3
duration: 60s
config: {head: true, cliff: true}
world:
  - {at: 0s, frame: {scene: hall}}
  - {at: 0s, range: 120}
  - {at: 0s, floor: {front: true, rear: true}}
  - {at: 5s, console: status}
  - {at: 20s, range: 8}
  - {at: 21s, range: {cm: 9, sensor: rear}}
  - {at: 22s, range_fail: {sensor: rear}}
  - {at: 25s, range: 120}
models: {planner: inline}
default: {status: 200, latency: 1s, call: drive,
          args: {heading_deg: 0, distance_cm: 50, speed_pct: 60}}
expect:
  - wheels_at: {at: 15s, left: coast, right: coast}
  - wheels: {left: forward, right: forward, between: [16s, 16s], count: 1}
  - wheels: {left: brake, after: "range[1]", within: 297ms, count: 1}
  - wheels_at: {at: 24.999s, left: brake, left_duty: 0}
  - wheels_at: {at: "range[3]", left: forward, left_duty: [2000, 4095]}
  - head_at: {at: 30s, pan: 0, tilt: [-1, 1], mode: gaze, settled: false}
  - head: {count: 1}
"""


def test_executor_records_pass_through_the_runner(write_scenario, fake_twin_cmd):
    scn = load_scenario(write_scenario("executor-like", EXECUTOR_LIKE))
    lines = scn.protocol_lines()
    assert lines[0]["head"] is True and lines[0]["cliff"] is True
    assert lines[1] == {"t": "frame", "at_ms": 0, "scene": "hall"}
    result = run_scenario(scn, fake_twin_cmd, timeout_s=20)
    kinds = [r["t"] for r in result.trace]
    assert kinds.count("wheels") == 3 and kinds.count("head") == 1
    outcomes = evaluate(scn, result.trace)
    assert all(o.passed for o in outcomes), [
        o.message for o in outcomes if not o.passed
    ]


def test_state_assertion_fails_against_a_real_run(write_scenario, fake_twin_cmd):
    body = EXECUTOR_LIKE.replace(
        "wheels_at: {at: 24.999s, left: brake, left_duty: 0}",
        "wheels_at: {at: 24.999s, left: forward, left_duty: 0}",
    )
    scn = load_scenario(write_scenario("executor-fail", body))
    outcomes = evaluate(scn, run_scenario(scn, fake_twin_cmd, timeout_s=20).trace)
    bad = [o for o in outcomes if not o.passed]
    assert len(bad) == 1
    assert "left is 'brake', want 'forward'" in bad[0].message


WALL_ARENA = """\
floor: [[-50, -60], [100, -60], [100, 60], [-50, 60]]
walls:
  - {from: [100, -60], to: [100, 60]}
"""
TABLE_ARENA = "floor: [[-40, -40], [60, -40], [60, 40], [-40, 40]]\n"

# The fake twin drives at duty 2457 from the first tick after the goal (16 s)
# and the fake plant moves 60 cm/s at full duty: 36 cm/s, 1.188 cm per tick.
PHYSICS_LIKE = """\
seed: 3
duration: 30s
world:
  - {at: 0s, frame: {scene: hall}}
  - {at: 5s, console: status}
models: {planner: inline}
plant:
  physics:
    arena: arenas/room.yaml
    start: {x_cm: 0, y_cm: 0, heading_deg: 0}
default: {status: 200, latency: 1s, call: drive,
          args: {heading_deg: 0, distance_cm: 50, speed_pct: 60}}
expect:
  - requests: {count: 1}
  - pose_at: {at: 16s, x_cm: 0, collided: false, off_floor: false}
  - pose: {x_cm: [0.5, 100], min: 10}
  - wheels: {left: forward, right: forward, between: [16.005s, 16.005s], count: 1}
  - wheels: {left: brake, right: brake, count: 1}
  - pose: {collided: true, count: 0}
  - pose_at: {at: 30s, x_cm: [70, 91]}
"""


def _physics(
    write_scenario, write_arena, arena=WALL_ARENA, body=PHYSICS_LIKE, name="phys"
):
    write_arena("room", arena)
    return load_scenario(write_scenario(name, body))


def test_physics_run_relays_the_plant_and_traces_the_pose(
    write_scenario, write_arena, fake_twin_cmd, fake_plant_cmd
):
    scn = _physics(write_scenario, write_arena)
    plant = plant_for(scn, cmd=fake_plant_cmd)
    result = run_scenario(scn, fake_twin_cmd, plant=plant, timeout_s=30)
    kinds = {r["t"] for r in result.trace}
    # The per-tick exchange is not in the trace.
    assert "plant_request" not in kinds and "plant_response" not in kinds
    assert result.plant == "physics"
    assert result.plant_steps == plant.steps == 30000 // 33 + 1
    assert plant.returncode == 0  # reaped
    poses = [r for r in result.trace if r["t"] == "pose"]
    # The first reading is recorded, then only changes: nothing while parked.
    assert poses[0] == {
        "t": "pose",
        "at_ms": 33,
        "x_cm": 0.0,
        "y_cm": 0.0,
        "heading_deg": 0.0,
        "collided": False,
        "off_floor": False,
    }
    # The first tick after the goal is 16.005 s (485 x 33); its reading is the
    # world one tick later.
    assert poses[1]["at_ms"] == 16005 + 33
    assert all(a["x_cm"] < b["x_cm"] for a, b in itertools.pairwise(poses))
    # The plant's range reached the twin: it braked before the wall.
    assert poses[-1]["x_cm"] < 92 and not poses[-1]["collided"]
    assert len(poses) < result.plant_steps
    # Records stay in time order with the pose records merged in.
    times = [r["at_ms"] for r in result.trace if r["t"] in ("pose", "wheels")]
    assert times == sorted(times)
    outcomes = evaluate(scn, result.trace)
    assert all(o.passed for o in outcomes), [
        o.message for o in outcomes if not o.passed
    ]


def test_pose_is_traced_when_only_a_flag_changes(
    write_scenario, write_arena, fake_twin_cmd, fake_plant_cmd
):
    # Reading 10 (330 ms) alone reports a contact while the robot is parked at
    # the start pose: the flag sets and clears with x, y and heading unchanged.
    scn = _physics(write_scenario, write_arena)
    plant = plant_for(scn, cmd=[*fake_plant_cmd, "--bump-at", "10"])
    result = run_scenario(scn, fake_twin_cmd, plant=plant, timeout_s=30)
    parked = [r for r in result.trace if r["t"] == "pose" and r["at_ms"] < 16000]
    assert [(r["at_ms"], r["x_cm"], r["collided"]) for r in parked] == [
        (33, 0.0, False),
        (330, 0.0, True),
        (363, 0.0, False),
    ]


def test_physics_latency_window_ticks_exchange_before_the_reply(
    write_scenario, write_arena, fake_twin_cmd, fake_plant_cmd
):
    scn = _physics(write_scenario, write_arena)
    result = run_scenario(
        scn, fake_twin_cmd, plant=plant_for(scn, cmd=fake_plant_cmd), timeout_s=30
    )
    # 15 s .. 16 s is the reply's latency. Had its 30 ticks not exchanged, the
    # plant's clock would trail the twin's and the run would have failed as out
    # of order; and the goal is handled after them, so the wheels start at the
    # first tick past 16 s.
    goal = next(r for r in result.trace if r["t"] == "goal")
    first_wheels = next(r for r in result.trace if r["t"] == "wheels")
    assert goal["at_ms"] == 16000 and first_wheels["at_ms"] == 16005
    assert result.trace.index(goal) < result.trace.index(first_wheels)


def test_physics_floor_reading_reaches_the_twin(
    write_scenario, write_arena, fake_twin_cmd, fake_plant_cmd
):
    body = PHYSICS_LIKE.replace("  - pose_at: {at: 30s, x_cm: [70, 91]}\n", "").replace(
        "  - pose: {collided: true, count: 0}\n",
        "  - pose: {off_floor: true, count: 0}\n  - pose_at: {at: 30s, x_cm: [40, 60]}\n",
    )
    scn = _physics(write_scenario, write_arena, TABLE_ARENA, body, "table")
    result = run_scenario(
        scn, fake_twin_cmd, plant=plant_for(scn, cmd=fake_plant_cmd), timeout_s=30
    )
    outcomes = evaluate(scn, result.trace)
    assert all(o.passed for o in outcomes), [
        o.message for o in outcomes if not o.passed
    ]


def test_physics_assertions_fail_when_the_range_is_lost(
    write_scenario, write_arena, fake_twin_cmd, fake_plant_cmd
):
    scn = _physics(write_scenario, write_arena)
    plant = plant_for(scn, cmd=[*fake_plant_cmd, "--null-range"])
    result = run_scenario(scn, fake_twin_cmd, plant=plant, timeout_s=30)
    bad = {o.name: o for o in evaluate(scn, result.trace) if not o.passed}
    assert "pose(collided=True)" in bad["pose"].message
    assert "wheels" in bad  # it never braked


@pytest.mark.parametrize(
    ("flag", "match"),
    [
        (["--exit-after", "5"], "exited with status 3"),
        (["--error-after", "5"], r"reported an error: step\.left\.duty"),
        (["--ready-twice"], "answered out of order"),
        (["--skew"], "answered out of order"),
    ],
)
def test_plant_failure_fails_the_scenario(
    write_scenario, write_arena, fake_twin_cmd, fake_plant_cmd, flag, match
):
    scn = _physics(write_scenario, write_arena)
    plant = plant_for(scn, cmd=[*fake_plant_cmd, *flag])
    with pytest.raises(TwinError, match=match) as info:
        run_scenario(scn, fake_twin_cmd, plant=plant, timeout_s=30)
    assert str(info.value).startswith("phys: plant: ")
    assert "plant stderr:" in str(info.value)
    if flag[0] == "--exit-after":
        assert "fake plant: giving up" in str(info.value)
    assert plant.returncode is not None  # the child is reaped


def test_plant_that_cannot_init_fails_before_the_twin_starts(
    write_scenario, write_arena, fake_plant_cmd, tmp_path
):
    scn = _physics(write_scenario, write_arena, "walls: []\n")
    plant = plant_for(scn, cmd=fake_plant_cmd)
    # The twin command does not exist: reaching it would be a different error.
    with pytest.raises(TwinError, match="phys: plant: the plant reported an error"):
        run_scenario(scn, [str(tmp_path / "nope")], plant=plant, timeout_s=30)
    assert plant.returncode == 1


def test_hanging_plant_is_a_timeout(
    write_scenario, write_arena, fake_twin_cmd, fake_plant_cmd
):
    scn = _physics(write_scenario, write_arena)
    plant = plant_for(scn, cmd=[*fake_plant_cmd, "--hang"])
    with pytest.raises(TwinTimeout, match="no 'ready' within"):
        run_scenario(scn, fake_twin_cmd, plant=plant, timeout_s=1)
    assert plant.returncode is not None


def test_plant_is_reaped_when_the_twin_fails(
    write_scenario, write_arena, fake_twin_cmd, fake_plant_cmd
):
    scn = _physics(write_scenario, write_arena)
    scn.config.update(fake_exit=3)
    plant = plant_for(scn, cmd=fake_plant_cmd)
    with pytest.raises(TwinError, match="exited with status 3"):
        run_scenario(scn, fake_twin_cmd, plant=plant, timeout_s=30)
    assert plant.returncode == 0


def test_malformed_plant_request_is_an_error(
    write_scenario, write_arena, fake_twin_cmd, fake_plant_cmd
):
    scn = _physics(write_scenario, write_arena)
    scn.config.update(fake_bad_plant_request=1)
    plant = plant_for(scn, cmd=fake_plant_cmd)
    with pytest.raises(TwinError, match="malformed plant_request"):
        run_scenario(scn, fake_twin_cmd, plant=plant, timeout_s=30)
    assert plant.returncode == 0


def test_plant_request_in_a_scripted_scenario_is_an_error(
    write_scenario, fake_twin_cmd
):
    # The config line of a scripted scenario never asks for a plant; a twin
    # that sends a plant_request anyway is refused, not answered.
    scn = _simple(write_scenario, plant="external")
    with pytest.raises(TwinError, match="the scenario's plant is scripted"):
        run_scenario(scn, fake_twin_cmd, timeout_s=20)


def test_scripted_run_has_no_pose_and_no_plant(write_scenario, fake_twin_cmd):
    scn = load_scenario(write_scenario("executor-like", EXECUTOR_LIKE))
    result = run_scenario(scn, fake_twin_cmd, timeout_s=20)
    assert result.plant == "scripted" and result.plant_steps == 0
    assert not [r for r in result.trace if r["t"] == "pose"]


def test_cli_scorecard_runs_the_plant_named_by_the_environment(
    write_scenario, write_arena, fake_twin_cmd, fake_plant_cmd, tmp_path, monkeypatch
):
    phys = _physics(write_scenario, write_arena).path
    scripted = write_scenario("a4-like", A4_LIKE)
    monkeypatch.setenv("ROBOCAR_PLANT_CMD", " ".join(fake_plant_cmd))
    out = tmp_path / "scorecard.json"
    traces = tmp_path / "traces"
    args = ["run", "--scenarios", str(phys), str(scripted), "--out", str(out)]
    args += ["--twin", " ".join(fake_twin_cmd), "--traces", str(traces)]
    assert cli_main(args) == 0
    by_id = {e["id"]: e for e in json.loads(out.read_text())["scenarios"]}
    assert by_id["phys"]["plant"] == "physics"
    assert by_id["phys"]["plant_steps"] == 910
    assert by_id["a4-like"]["plant"] == "scripted"
    assert "plant_steps" not in by_id["a4-like"]
    written = (traces / "phys.jsonl").read_text()
    assert '"t": "pose"' in written and "plant_request" not in written
    # A plant command that fails is a failed entry, not a crash.
    monkeypatch.setenv("ROBOCAR_PLANT_CMD", "false")
    assert cli_main(args) == 1
    by_id = {e["id"]: e for e in json.loads(out.read_text())["scenarios"]}
    assert not by_id["phys"]["passed"] and "plant" in by_id["phys"]["error"]
    assert by_id["a4-like"]["passed"]
