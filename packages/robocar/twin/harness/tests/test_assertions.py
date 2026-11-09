import pytest
import yaml

from robocar_twin.assertions import evaluate
from robocar_twin.scenario import ScenarioError, load_scenario


@pytest.fixture
def scenario_with(write_scenario):
    def _make(*expect, duration="30m", world=None):
        doc = {
            "seed": 1,
            "duration": duration,
            "world": world
            or [
                {"at": "0s", "frame": {"scene": "room"}},
                {"at": "16s", "voice_turn": {"dur": "2s", "reply": "hello"}},
            ],
            "models": {"planner": "inline"},
            "default": {"status": 200},
            "expect": list(expect),
        }
        return load_scenario(write_scenario("t", yaml.safe_dump(doc)))

    return _make


def req(at):
    return {"t": "model_request", "at_ms": at, "id": at, "kind": "plan"}


def utt(at, source, text="x"):
    return {"t": "utterance", "at_ms": at, "source": source, "text": text}


def one(scn, trace):
    (o,) = evaluate(scn, trace)
    return o


def test_no_utterance_cites_offender(scenario_with):
    scn = scenario_with(
        {"no_utterance": {"source": "planner", "between": ["15s", "30s"]}}
    )
    trace = [
        utt(10000, "planner"),
        utt(19000, "planner", "strange noise"),
        utt(18000, "voice_turn"),
    ]
    o = one(scn, trace)
    assert not o.passed
    assert "[1]" in o.message and "strange noise" in o.message
    assert one(scn, [utt(10000, "planner"), utt(18000, "voice_turn")]).passed


def test_utterance_after_reference_window(scenario_with):
    scn = scenario_with(
        {
            "utterance": {
                "source": "voice_turn",
                "after": "voice_turn[0]",
                "within": "4s",
            }
        }
    )
    assert one(scn, [utt(18000, "voice_turn")]).passed
    o = one(scn, [utt(21000, "voice_turn")])
    assert not o.passed and "[16s, 20s]" in o.message
    assert "all 'utterance' records" in o.message and "[0]" in o.message


def test_utterance_count_and_text(scenario_with):
    scn = scenario_with({"utterance": {"count": 1, "text_contains": "TIDY"}})
    assert one(scn, [utt(1, "planner", "tidy table")]).passed
    assert not one(scn, [utt(1, "planner", "tidy"), utt(2, "planner", "tidy")]).passed


def test_requests_bounds(scenario_with):
    scn = scenario_with(
        {"requests": {"between": ["0s", "60s"], "max": 2}},
        {"requests": {"min": 1}},
        {"requests": {"count": 0, "between": ["61s", "30m"]}},
    )
    out = evaluate(scn, [req(15000), req(30000), req(45000)])
    assert [o.passed for o in out] == [False, True, True]
    assert "at most 2, found 3" in out[0].message


def ladder_times(start=15000, period=15000, mults=(1, 2, 4, 8, 20)):
    times = [start]
    for m in mults:
        times.append(times[-1] + m * period)
    return times


def test_request_ladder_pass_and_fail(scenario_with):
    scn = scenario_with({"request_ladder": {"between": ["15s", "30m"]}})
    good = [req(t) for t in ladder_times()]
    assert one(scn, good).passed
    # One keep-alive too early: the third interval is 50 s, not 60 s.
    bad = [req(t) for t in [15000, 30000, 60000, 110000, 230000, 530000]]
    o = one(scn, bad)
    assert not o.passed and "interval 2 is 50s, want 60s" in o.message
    # An extra request after the top rung breaks "then none".
    o = one(scn, [*good, req(900000)])
    assert not o.passed and "then no further request" in o.message


def test_request_ladder_scales_with_period(write_scenario):
    doc = {
        "seed": 1,
        "duration": "30m",
        "config": {"planner_period_ms": 10000},
        "world": [{"at": "0s", "frame": {"scene": "r"}}],
        "models": {"planner": "inline"},
        "default": {"status": 200},
        "expect": [{"request_ladder": {}}],
    }
    scn = load_scenario(write_scenario("p", yaml.safe_dump(doc)))
    assert one(scn, [req(t) for t in ladder_times(0, 10000)]).passed


def test_request_intervals_inexact(scenario_with):
    scn = scenario_with(
        {"request_intervals": {"intervals": ["30s", "45s"], "exact": False}}
    )
    assert one(scn, [req(15000), req(45000), req(90000), req(105000)]).passed


def test_backoff_and_goals_sequences(scenario_with):
    scn = scenario_with(
        {"backoff": {"sequence": [1, 2, 4, 4]}},
        {"goals": {"sequence": ["stop", "drive"]}},
    )
    trace = [
        {"t": "backoff", "at_ms": t, "periods": p}
        for t, p in [(1, 1), (2, 2), (3, 4), (4, 4)]
    ] + [
        {"t": "goal", "at_ms": 5, "kind": "stop"},
        {"t": "goal", "at_ms": 6, "kind": "drive"},
    ]
    assert all(o.passed for o in evaluate(scn, trace))
    o = evaluate(scn, trace[:3])[0]
    assert not o.passed and "[1, 2, 4]" in o.message


def test_announce_force_stop_suppressed(scenario_with):
    scn = scenario_with(
        {"announce": {"what": "dormant", "count": 1}},
        {"force_stop": {"reason": "budget"}},
        {"suppressed": {"reason": "person_spoke", "count": 1}},
    )
    trace = [
        {"t": "announce", "at_ms": 0, "what": "dormant"},
        {"t": "announce", "at_ms": 9, "what": "dormant"},
        {"t": "force_stop", "at_ms": 1, "reason": "camera"},
        {"t": "suppressed", "at_ms": 2, "reason": "person_spoke", "text": "noise"},
    ]
    out = evaluate(scn, trace)
    assert [o.passed for o in out] == [False, False, True]
    assert "exactly 1, found 2" in out[0].message
    assert "camera" in out[1].message  # cites the non-matching records it saw


@pytest.mark.parametrize(
    ("bad", "match"),
    [
        ({"backoff": {"sequence": [0]}}, "positive"),
        ({"goals": {"sequence": ["fly"]}}, "unknown goal kind"),
        ({"announce": {}}, "missing argument"),
        ({"utterance": {"within": "2s"}}, "needs 'after'"),
        (
            {"utterance": {"between": ["1s", "2s"], "after": "1s", "within": "1s"}},
            "use 'between'",
        ),
        ({"requests": {"count": -1}}, "non-negative"),
        ({"request_intervals": {"intervals": []}}, "non-empty"),
        ({"utterance": {"bogus": 1}}, "unknown argument"),
        ({"utterance": {}, "requests": {"max": 1}}, "one-key"),
    ],
)
def test_expectation_validation(scenario_with, bad, match):
    with pytest.raises(ScenarioError, match=match):
        scenario_with(bad)


def test_model_request_filters_on_descriptor_fields(scenario_with):
    scn = scenario_with(
        {
            "model_request": {
                "between": ["15s", "15s"],
                "speak_allowed": True,
                "count": 1,
            }
        }
    )

    def mreq(at, allowed):
        return {**req(at), "descriptor": {"gate": "V", "speak_allowed": allowed}}

    assert one(scn, [mreq(15000, True)]).passed
    o = one(scn, [mreq(15000, False)])
    assert not o.passed and "speak_allowed" in o.message


def test_cycle_filters_on_spent_requests(scenario_with):
    scn = scenario_with({"cycle": {"requested": True, "spent_req": 4, "count": 1}})

    def cyc(at, spent, requested=True):
        return {"t": "cycle", "at_ms": at, "requested": requested, "spent_req": spent}

    assert one(scn, [cyc(1000, 3), cyc(2000, 4), cyc(3000, 4, False)]).passed
    assert not one(scn, [cyc(1000, 3), cyc(2000, 3)]).passed


def test_speak_not_offered_is_counted(scenario_with):
    scn = scenario_with({"speak_not_offered": {"count": 0}})
    flagged = {"t": "speak_not_offered", "at_ms": 30000, "text": "hi"}
    assert one(scn, []).passed
    assert not one(scn, [flagged]).passed


def test_note_offered_filters_on_trigger(scenario_with):
    scn = scenario_with({"note_offered": {"trigger": "reflex", "count": 1}})
    offered = {"t": "note_offered", "at_ms": 30000, "trigger": "reflex"}
    other = {"t": "note_offered", "at_ms": 60000, "trigger": "reflect"}
    assert one(scn, [offered, other]).passed
    assert not one(scn, [other]).passed


def test_note_and_note_not_offered_are_counted(scenario_with):
    scn = scenario_with(
        {"note": {"kind": "difficulty", "count": 1}}, {"note_not_offered": {"count": 0}}
    )
    note = {"t": "note", "at_ms": 31000, "trigger": "reflex", "kind": "difficulty"}
    flagged = {"t": "note_not_offered", "at_ms": 46000, "text": "x"}
    assert all(o.passed for o in evaluate(scn, [note]))
    assert not all(o.passed for o in evaluate(scn, [note, flagged]))


def test_note_trigger_is_validated(scenario_with):
    with pytest.raises(ScenarioError, match="trigger"):
        scenario_with({"note_offered": {"trigger": "boredom"}})
