import json

import pytest
from robocar_twin.assertions import evaluate
from robocar_twin.cli import main as cli_main
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
