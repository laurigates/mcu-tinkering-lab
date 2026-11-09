import json
from pathlib import Path

import pytest

from robocar_glitchtip_bridge.event import (
    Frame,
    InvalidEvent,
    build_sha,
    parse_payload,
    route,
    to_sentry_event,
)

FIXTURES = Path(__file__).parents[2] / "fixtures"


def fixture(name: str) -> dict:
    return parse_payload((FIXTURES / name).read_bytes())


def test_fixtures_parse_and_route():
    assert route(fixture("reset-brownout.json")) == "robocar"
    assert route(fixture("crash.json")) == "robocar"
    assert route(fixture("planner-note.json")) == "robocar-ai"


@pytest.mark.parametrize(
    "patch",
    [
        {"event_id": "short"},
        {"event_id": "0123456789ABCDEF0123456789ABCDEF"},
        {"source": "camera"},
        {"level": "loud"},
        {"release": ""},
        {"message": None},
    ],
)
def test_rejects_invalid_fields(patch):
    event = fixture("reset-brownout.json") | patch
    with pytest.raises(InvalidEvent):
        parse_payload(json.dumps(event).encode())


@pytest.mark.parametrize("raw", [b"not json", b"[1, 2]", b"\xff\xfe"])
def test_rejects_non_objects(raw):
    with pytest.raises(InvalidEvent):
        parse_payload(raw)


def test_zero_timestamp_uses_receive_time():
    out = to_sentry_event(fixture("reset-brownout.json"), received_at=1234.5)
    assert out["timestamp"] == 1234.5
    assert out["extra"]["robot_timestamp_missing"] is True


def test_robot_timestamp_kept_when_set():
    # The robot runs no SNTP, so its own events carry 0; a future clock would not.
    event = {**fixture("planner-note.json"), "timestamp": 1762700000}
    out = to_sentry_event(event, received_at=1234.5)
    assert out["timestamp"] == 1762700000.0
    assert "robot_timestamp_missing" not in out["extra"]


def test_reset_event_shape():
    out = to_sentry_event(fixture("reset-brownout.json"), received_at=0)
    assert out["release"] == "robocar-unified@0.0.0-fixture"
    assert out["message"] == {"formatted": "reset: BROWNOUT"}
    assert out["fingerprint"] == ["reset", "BROWNOUT"]
    assert out["tags"] == {
        "reset_reason": "BROWNOUT",
        "build_sha": "fixture",
        "source": "firmware",
    }
    assert out["breadcrumbs"]["values"][0]["message"].startswith("I (4100) motor")
    assert "exception" not in out
    assert "symbolicated" not in out["tags"]


def test_frames_reversed_and_unsymbolicated_without_elf():
    out = to_sentry_event(fixture("crash.json"), received_at=0)
    value = out["exception"]["values"][0]
    assert value["type"] == "LoadProhibited"
    addrs = [f["instruction_addr"] for f in value["stacktrace"]["frames"]]
    assert addrs == ["0x0000002c", "0x0000000a"]  # outermost first, as Sentry expects
    assert out["tags"]["symbolicated"] == "false"


def test_frames_symbolicated_when_resolver_hits():
    table = {0x0A: Frame("add", "tiny.c", 3), 0x2C: Frame("entry", "tiny.c", 8)}
    out = to_sentry_event(fixture("crash.json"), received_at=0, symbolicate=table.get)
    frames = out["exception"]["values"][0]["stacktrace"]["frames"]
    assert [(f["function"], f["lineno"]) for f in frames] == [("entry", 8), ("add", 3)]
    assert out["tags"]["symbolicated"] == "true"


def test_bad_address_left_raw():
    event = fixture("crash.json")
    event["exception"]["frames"] = [{"instruction_addr": "garbage"}]
    out = to_sentry_event(
        event, received_at=0, symbolicate=lambda _a: Frame("x", "y", 1)
    )
    assert out["exception"]["values"][0]["stacktrace"]["frames"] == [
        {"instruction_addr": "garbage", "in_app": True}
    ]
    assert out["tags"]["symbolicated"] == "false"


def test_build_sha():
    assert build_sha(fixture("crash.json")) == "fixture"
    assert build_sha({"tags": {}}) is None
    assert build_sha({}) is None
