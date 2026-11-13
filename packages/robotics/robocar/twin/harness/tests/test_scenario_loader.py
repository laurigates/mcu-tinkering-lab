import json

import pytest
from robocar_twin.scenario import (
    ScenarioError,
    build_body,
    load_scenario,
    load_scenarios,
)

MINIMAL = """\
seed: 1
duration: 60s
world:
  - {at: 0s, frame: {scene: room}}
models: {planner: inline}
default: {status: 200, call: stop}
expect:
  - requests: {max: 10}
"""


def test_minimal_loads_and_defaults(write_scenario):
    s = load_scenario(write_scenario("minimal", MINIMAL))
    assert s.duration_ms == 60000
    assert s.world[0].to_protocol() == {
        "t": "frame",
        "at_ms": 0,
        "scene": "room",
        "pose": "C",
    }
    cfg = s.protocol_lines()[0]
    assert cfg == {
        "t": "config",
        "seed": 1,
        "duration_ms": 60000,
        "planner_period_ms": 15000,
    }
    assert s.default_response is not None
    assert s.default_response.status == 200
    assert s.default_response.latency_ms == 1000


def test_every_kind_and_repeat(write_scenario):
    body = """\
seed: 1
duration: 30s
config: {planner_period_ms: 5000, plan_budget_max_requests: 3, cliff: true}
world:
  - {at: 0s, link: {wifi: true, key: false}}
  - {at: 1s, range: 80}
  - {at: 1500ms, range: {cm: 40}}
  - {at: 1600ms, range: {cm: 9, sensor: rear}}
  - {at: 2s, range_fail: {}}
  - {at: 2100ms, range_fail: {sensor: rear}}
  - {at: 2200ms, floor: {front: false, rear: true}}
  - {at: 3s, sound: {peak_db: 20, dur: 300ms}}
  - {at: 4s, voice_turn: {dur: 2s, reply: hi}}
  - {at: 5s, every: 10s, until: 25s, console: status}
  - {at: 6s, console: {line: listen}}
  - {at: 7s, frame: {scene: a, pose: L}}
models: {planner: inline}
responses: [{status: 429, latency: 200ms}]
expect: [{requests: {max: 9}}]
"""
    s = load_scenario(write_scenario("kinds", body))
    lines = s.protocol_lines()
    assert lines[0]["planner_period_ms"] == 5000
    assert lines[0]["plan_budget_max_requests"] == 3
    assert lines[0]["cliff"] is True and "head" not in lines[0]
    assert [e.to_protocol() for e in s.world if e.kind in ("range", "range_fail")] == [
        {"t": "range", "at_ms": 1000, "cm": 80, "sensor": "front"},
        {"t": "range", "at_ms": 1500, "cm": 40, "sensor": "front"},
        {"t": "range", "at_ms": 1600, "cm": 9, "sensor": "rear"},
        {"t": "range_fail", "at_ms": 2000, "sensor": "front"},
        {"t": "range_fail", "at_ms": 2100, "sensor": "rear"},
    ]
    floor = next(e for e in s.world if e.kind == "floor")
    assert floor.to_protocol() == {
        "t": "floor",
        "at_ms": 2200,
        "front": False,
        "rear": True,
    }
    kinds = [(e.kind, e.at_ms) for e in s.world]
    assert ("console", 5000) in kinds and ("console", 15000) in kinds
    assert ("console", 25000) in kinds
    assert [e.at_ms for e in s.world] == sorted(e.at_ms for e in s.world)
    sound = next(e for e in s.world if e.kind == "sound")
    assert sound.to_protocol() == {
        "t": "sound",
        "at_ms": 3000,
        "peak_db": 20,
        "dur_ms": 300,
    }
    vt = next(e for e in s.world if e.kind == "voice_turn")
    assert vt.fields == {"dur_ms": 2000, "reply": "hi"}
    assert s.responses[0].body == ""


@pytest.mark.parametrize(
    ("patch", "match"),
    [
        (
            (
                "world:\n  - {at: 0s, frame: {scene: room}}",
                "world:\n  - {at: 0s, voice_trun: {}}",
            ),
            "unknown key",
        ),
        (("{at: 0s, frame: {scene: room}}", "{frame: {scene: room}}"), "missing 'at'"),
        (("{at: 0s, frame", "{at: 15, frame"), "unit"),
        (("{scene: room}", "{scene: room, pose: X}"), "pose"),
        (("{at: 0s, frame", "{at: 90s, frame"), "past the duration"),
        (("requests: {max: 10}", "requests: {}"), "count, min or max"),
        (("requests: {max: 10}", "utterances: {}"), "unknown assertion"),
        (("requests: {max: 10}", "utterance: {source: robot}"), "expected one of"),
        (
            ("requests: {max: 10}", "utterance: {after: 'voice_turn[0]', within: 2s}"),
            "refers to voice_turn event 0",
        ),
        (
            ("requests: {max: 10}", "requests: {between: [5s], max: 1}"),
            r"\[start, end\]",
        ),
        (("models: {planner: inline}", "models: {planner: gemini}"), "replay:"),
        (("seed: 1", "seed: one"), "integer"),
        (
            (
                "default: {status: 200, call: stop}",
                "default: {status: 200, call: jump}",
            ),
            "call",
        ),
        (
            ("default: {status: 200, call: stop}", "default: {status: 429, speak: hi}"),
            "status 200",
        ),
        (
            (
                "default: {status: 200, call: stop}",
                "default: {status: 200, note: {kind: rant, text: x}}",
            ),
            "note.kind",
        ),
        (
            (
                "default: {status: 200, call: stop}",
                "default: {status: 200, note: {kind: difficulty}}",
            ),
            "text",
        ),
        (
            ("{at: 0s, frame", "{at: 0s, range: 80, frame"),
            "exactly one event kind",
        ),
        # ADR-036: a reflex is scripted as the obstacle that causes it.
        (("{at: 0s, frame: {scene: room}}", "{at: 0s, reflex: {}}"), "were removed"),
        # The bounds ultrasonic_core_decode() accepts.
        (("{at: 0s, frame: {scene: room}}", "{at: 0s, range: 1}"), r"2\.\.400"),
        (("{at: 0s, frame: {scene: room}}", "{at: 0s, range: {cm: 401}}"), r"2\.\.400"),
        (("{at: 0s, frame: {scene: room}}", "{at: 0s, range: {cm: 2.5}}"), "integer"),
        (
            (
                "{at: 0s, frame: {scene: room}}",
                "{at: 0s, range: {cm: 80, sensor: side}}",
            ),
            "sensor",
        ),
        (
            ("{at: 0s, frame: {scene: room}}", "{at: 0s, range_fail: {sensor: top}}"),
            "sensor",
        ),
        (("{at: 0s, frame: {scene: room}}", "{at: 0s, range_fail: 3}"), "mapping"),
        (
            (
                "{at: 0s, frame: {scene: room}}",
                "{at: 0s, floor: {front: true, rear: true}}",
            ),
            "config.cliff",
        ),
        (("seed: 1", "seed: 1\nconfig: {head: 1}"), "true/false"),
        (("seed: 1", "seed: 1\nconfig: {plant: external}"), "unknown key"),
    ],
)
def test_validation_errors(write_scenario, patch, match):
    old, new = patch
    assert old in MINIMAL
    path = write_scenario("bad", MINIMAL.replace(old, new))
    with pytest.raises(ScenarioError, match=match) as info:
        load_scenario(path)
    assert str(path) in str(info.value)


def test_id_must_match_filename(write_scenario, tmp_path):
    p = write_scenario("one", MINIMAL)
    other = p.with_name("two.yaml")
    other.write_text(p.read_text())
    with pytest.raises(ScenarioError, match="must match the file name"):
        load_scenario(other)


def test_inline_needs_replies(write_scenario):
    body = MINIMAL.replace("default: {status: 200, call: stop}\n", "")
    with pytest.raises(ScenarioError, match="neither 'responses' nor 'default'"):
        load_scenario(write_scenario("noreply", body))


def test_duplicate_ids(write_scenario, tmp_path):
    p = write_scenario("dup", MINIMAL)
    with pytest.raises(ScenarioError, match="duplicate"):
        load_scenarios([p, p])


def test_build_body_shape_matches_gemini_parse():
    body = json.loads(
        build_body("track", {"box_2d": [1, 2, 3, 4], "max_speed_pct": 40}, "Hi")
    )
    parts = body["candidates"][0]["content"]["parts"]
    assert parts[0] == {
        "functionCall": {
            "name": "track",
            "args": {"box_2d": [1, 2, 3, 4], "max_speed_pct": 40},
        }
    }
    assert parts[1] == {"functionCall": {"name": "speak", "args": {"text": "Hi"}}}
    assert body["usageMetadata"]["totalTokenCount"] == 1230
    assert json.loads(build_body(tokens=500))["usageMetadata"]["totalTokenCount"] == 500


def test_voice_turn_record_window(write_scenario):
    turn = "{at: 0s, frame: {scene: room}}"
    assert turn in MINIMAL
    ok = MINIMAL.replace(
        turn, turn + "\n  - {at: 4s, voice_turn: {dur: 3s, record: 1s, reply: hi}}"
    )
    s = load_scenario(write_scenario("rec", ok))
    vt = next(e for e in s.world if e.kind == "voice_turn")
    assert vt.fields == {"dur_ms": 3000, "reply": "hi", "record_ms": 1000}
    bad = MINIMAL.replace(
        turn, turn + "\n  - {at: 4s, voice_turn: {dur: 3s, record: 4s, reply: hi}}"
    )
    with pytest.raises(ScenarioError, match="record"):
        load_scenario(write_scenario("rec_bad", bad))


def test_build_body_carries_a_note_call():
    body = json.loads(build_body(note={"kind": "suggestion", "text": "Slow down."}))
    parts = body["candidates"][0]["content"]["parts"]
    assert parts[-1] == {
        "functionCall": {
            "name": "note",
            "args": {"kind": "suggestion", "text": "Slow down."},
        }
    }


def test_range_bounds_are_inclusive(write_scenario):
    body = MINIMAL.replace(
        "{at: 0s, frame: {scene: room}}",
        "{at: 0s, frame: {scene: room}}\n  - {at: 1s, range: 2}\n  - {at: 2s, range: 400}",
    )
    events = load_scenario(write_scenario("bounds", body)).world
    assert [e.fields["cm"] for e in events if e.kind == "range"] == [2, 400]


def test_head_config_sends_frames_without_a_pose(write_scenario):
    body = MINIMAL.replace("seed: 1", "seed: 1\nconfig: {head: true}")
    s = load_scenario(write_scenario("head", body))
    assert s.protocol_lines()[0]["head"] is True
    assert s.world[0].to_protocol() == {"t": "frame", "at_ms": 0, "scene": "room"}


def test_head_config_refuses_a_scripted_pose(write_scenario):
    body = MINIMAL.replace("seed: 1", "seed: 1\nconfig: {head: true}").replace(
        "{scene: room}", "{scene: room, pose: C}"
    )
    with pytest.raises(ScenarioError, match=r"not allowed with config\.head"):
        load_scenario(write_scenario("head-pose", body))
    # head: false is the default written out: the pose works as before.
    off = body.replace("head: true", "head: false")
    s = load_scenario(write_scenario("head-off", off))
    assert s.world[0].fields == {"scene": "room", "pose": "C"}
    assert s.protocol_lines()[0]["head"] is False


def test_floor_events_need_boolean_fields(write_scenario):
    body = MINIMAL.replace("seed: 1", "seed: 1\nconfig: {cliff: true}").replace(
        "{at: 0s, frame: {scene: room}}",
        "{at: 0s, frame: {scene: room}}\n  - {at: 1s, floor: {front: 1, rear: true}}",
    )
    with pytest.raises(ScenarioError, match=r"floor\.front"):
        load_scenario(write_scenario("floor-bad", body))
    with pytest.raises(ScenarioError, match="missing key"):
        load_scenario(
            write_scenario(
                "floor-half", body.replace("front: 1, rear: true", "rear: true")
            )
        )
