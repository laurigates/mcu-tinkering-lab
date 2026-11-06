import hashlib
import json

import pytest

from robocar_twin.ports import (
    InlinePort,
    ModelPortError,
    ModelRequest,
    ModelResponse,
    RecordPort,
    ReplayPort,
    canonical_descriptor,
    descriptor_key,
    port_for,
    write_cassette,
)
from robocar_twin.scenario import InlineResponse, load_scenario

DESC = {
    "speak_allowed": True,
    "kind": "plan",
    "scene": "kitchen",
    "pose": "C",
    "gate": "V",
}


def make_req(i=1, desc=DESC):
    return ModelRequest(
        id=i, at_ms=15000, kind="plan", descriptor=desc, key=descriptor_key(desc)
    )


def test_canonical_form_is_sorted_compact_utf8():
    assert canonical_descriptor({"b": 1, "a": "ä"}) == '{"a":"ä","b":1}'
    expected = hashlib.sha256('{"a":"ä","b":1}'.encode()).hexdigest()
    assert descriptor_key({"b": 1, "a": "ä"}) == expected


def test_replay_hit_and_miss(tmp_path):
    r = make_req()
    write_cassette(tmp_path, r, ModelResponse(200, 900, '{"x":1}'))
    port = ReplayPort(tmp_path)
    assert port.respond(r) == ModelResponse(200, 900, '{"x":1}')
    other = make_req(2, {**DESC, "scene": "hallway"})
    with pytest.raises(ModelPortError) as info:
        port.respond(other)
    msg = str(info.value)
    assert other.key in msg and '"scene":"hallway"' in msg


def test_replay_rejects_incomplete_cassette(tmp_path):
    r = make_req()
    (tmp_path / f"{r.key}.json").write_text(json.dumps({"status": 200}))
    with pytest.raises(ModelPortError, match="missing 'latency_ms'"):
        ReplayPort(tmp_path).respond(r)


def test_inline_order_then_default_then_exhausted():
    a = InlineResponse(429, 100, "")
    b = InlineResponse(200, 200, "{}")
    port = InlinePort([a], b)
    assert port.respond(make_req(1)).status == 429
    assert port.respond(make_req(2)).status == 200
    assert port.respond(make_req(3)).status == 200
    bare = InlinePort([a])
    bare.respond(make_req(1))
    with pytest.raises(ModelPortError, match="exhausted"):
        bare.respond(make_req(2))


def test_record_port_needs_key(tmp_path, monkeypatch):
    monkeypatch.delenv("GEMINI_API_KEY", raising=False)
    with pytest.raises(ModelPortError, match="GEMINI_API_KEY"):
        RecordPort(tmp_path)


def test_record_port_writes_cassette_without_network(tmp_path):
    calls = []

    def transport(url, data, headers):
        calls.append((url, json.loads(data), headers))
        return 200, '{"candidates":[]}'

    ticks = iter([1.0, 1.25])
    port = RecordPort(
        tmp_path,
        api_key="test-key",
        request_builder=lambda r: {
            "contents": [{"parts": [{"text": r.descriptor["scene"]}]}]
        },
        transport=transport,
        max_requests=1,
        clock=lambda: next(ticks),
    )
    r = make_req()
    resp = port.respond(r)
    assert resp == ModelResponse(200, 250, '{"candidates":[]}')
    assert calls[0][0].endswith("gemini-robotics-er-2-preview:generateContent")
    assert calls[0][2]["x-goog-api-key"] == "test-key"
    # The cassette it wrote replays.
    assert ReplayPort(tmp_path).respond(r) == resp
    with pytest.raises(ModelPortError, match="spend guard"):
        port.respond(make_req(2))


def test_record_port_default_builder_explains_phase1(tmp_path):
    port = RecordPort(tmp_path, api_key="k", transport=lambda *a: (200, ""))
    with pytest.raises(ModelPortError, match="gemini_request"):
        port.respond(make_req())


def test_port_for_resolves_replay_dir_against_twin_dir(write_scenario):
    p = write_scenario(
        "rp",
        """\
seed: 1
duration: 10s
world: [{at: 0s, frame: {scene: r}}]
models: {planner: "replay:cassettes/rp"}
expect: [{requests: {max: 1}}]
""",
    )
    port = port_for(load_scenario(p))
    assert isinstance(port, ReplayPort)
    assert port.directory == p.resolve().parent.parent / "cassettes" / "rp"
