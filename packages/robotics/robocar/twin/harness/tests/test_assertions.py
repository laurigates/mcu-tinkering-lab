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


def test_cycle_filters_on_pose(scenario_with):
    scn = scenario_with(
        {"cycle": {"pose": "-", "between": ["0s", "0s"], "count": 1}},
        {"cycle": {"pose": "C", "count": 1}},
    )

    def cyc(at, pose):
        return {"t": "cycle", "at_ms": at, "requested": False, "pose": pose}

    assert all(o.passed for o in evaluate(scn, [cyc(0, "-"), cyc(15000, "C")]))
    assert not any(o.passed for o in evaluate(scn, [cyc(0, "L"), cyc(15000, "L")]))
    with pytest.raises(ScenarioError, match="expected one of"):
        scenario_with({"cycle": {"pose": "X"}})


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


def wheels(at, left, right=None, duty=None):
    """A `wheels` record; `left`/`right` are (drive, duty) or a drive name."""

    def side(v):
        drive, d = v if isinstance(v, tuple) else (v, duty)
        if d is None:
            d = 2457 if drive in ("forward", "reverse") else 0
        return {"drive": drive, "duty": d}

    return {
        "t": "wheels",
        "at_ms": at,
        "left": side(left),
        "right": side(right if right is not None else left),
    }


def head(at, pan, tilt=0, mode="gaze", settled=False):
    return {
        "t": "head",
        "at_ms": at,
        "pan_deg": pan,
        "tilt_deg": tilt,
        "mode": mode,
        "settled": settled,
    }


RAMP = [
    wheels(16005, ("forward", 546)),
    wheels(16038, ("forward", 996)),
    wheels(16071, ("forward", 2457)),
    wheels(20262, "brake"),
    wheels(21252, ("forward", 546)),
]


def test_wheels_counts_matching_change_records(scenario_with):
    scn = scenario_with(
        {"wheels": {"left": "brake", "right": "brake", "between": ["20s", "20.3s"]}},
        {"wheels": {"left": "forward", "count": 4}},
        {"wheels": {"left": "reverse", "count": 0}},
    )
    assert all(o.passed for o in evaluate(scn, RAMP))


def test_wheels_numeric_bounds_are_inclusive(scenario_with):
    scn = scenario_with(
        {"wheels": {"left_duty": [546, 996], "count": 3}},
        {"wheels": {"left_duty": [547, 995], "count": 0}},
        {"wheels": {"right_duty": 2457, "count": 1}},
        {"wheels": {"left": "forward", "left_duty": [1, 4095], "count": 4}},
    )
    assert all(o.passed for o in evaluate(scn, RAMP)), [
        o.message for o in evaluate(scn, RAMP) if not o.passed
    ]


def test_wheels_failure_cites_the_records_in_the_window(scenario_with):
    scn = scenario_with(
        {"wheels": {"left": "brake", "after": "voice_turn[0]", "within": "297ms"}}
    )
    o = one(scn, RAMP)
    assert not o.passed
    assert "wheels(left='brake') in [16s, 16.297s]" in o.message
    assert "expected at least 1, found 0" in o.message
    # The three ramp records inside the window are cited, the later brake is not.
    assert "[0]" in o.message and "[2]" in o.message and "20262" not in o.message
    empty = one(scn, [wheels(40000, "brake")])
    assert "all 'wheels' records" in empty.message and "40000" in empty.message


def test_wheels_at_is_the_last_record_at_or_before(scenario_with):
    scn = scenario_with(
        {"wheels_at": {"at": "16.07s", "left": "forward", "left_duty": 996}},
        {"wheels_at": {"at": "16.071s", "left_duty": 2457}},
        {"wheels_at": {"at": "20.262s", "left": "brake", "right": "brake"}},
        {"wheels_at": {"at": "21s", "left": "brake", "left_duty": 0}},
        {"wheels_at": {"at": "30s", "left": "forward", "right": "forward"}},
    )
    assert all(o.passed for o in evaluate(scn, RAMP)), [
        o.message for o in evaluate(scn, RAMP) if not o.passed
    ]


def test_wheels_at_before_any_record_is_the_boot_state(scenario_with):
    scn = scenario_with(
        {"wheels_at": {"at": "16s", "left": "coast", "right": "coast", "left_duty": 0}}
    )
    assert one(scn, RAMP).passed
    assert one(scn, []).passed
    moving = scenario_with({"wheels_at": {"at": "16s", "left": "forward"}})
    o = one(moving, RAMP)
    assert not o.passed
    assert "left is 'coast', want 'forward'" in o.message
    assert "before the first record" in o.message


def test_wheels_at_failure_names_each_mismatch_and_the_record(scenario_with):
    scn = scenario_with(
        {"wheels_at": {"at": "20.5s", "left": "forward", "right_duty": [100, 200]}}
    )
    o = one(scn, RAMP)
    assert not o.passed
    assert "wheels at 20.5s" in o.message
    assert "left is 'brake', want 'forward'" in o.message
    assert "right_duty is 0, want [100, 200]" in o.message
    assert "[3]" in o.message and "20262" in o.message


def test_wheels_at_takes_an_event_reference(scenario_with):
    scn = scenario_with({"wheels_at": {"at": "voice_turn[0]", "left": "coast"}})
    assert one(scn, RAMP).passed


def test_wheels_sides_are_filtered_separately(scenario_with):
    scn = scenario_with(
        {"wheels": {"left": "forward", "right": "reverse", "count": 1}},
        {"wheels_at": {"at": "10s", "left": "forward", "right": "reverse"}},
        {"wheels_at": {"at": "10s", "left_duty": 100, "right_duty": 200}},
    )
    spin = [wheels(5000, ("forward", 100), ("reverse", 200))]
    assert all(o.passed for o in evaluate(scn, spin))
    assert not any(o.passed for o in evaluate(scn, [wheels(5000, "forward")]))


GAZE = [
    head(0, 0, settled=False),
    head(396, 0, settled=True),
    head(5016, -2),
    head(5643, -40),
    head(6039, -40, settled=True),
    head(9000, 0, tilt=-20, settled=True),
]


def test_head_filters_and_bounds(scenario_with):
    scn = scenario_with(
        {"head": {"settled": True, "pan": [-39, -1], "count": 0}},
        {"head": {"settled": True, "pan": -40, "count": 1}},
        {"head": {"settled": True, "tilt": [-20, -20], "count": 1}},
        {"head": {"mode": "gaze", "min": 6}},
        {"head": {"mode": "centre", "count": 0}},
        {"head": {"settled": False, "between": ["5s", "6s"], "count": 2}},
    )
    assert all(o.passed for o in evaluate(scn, GAZE)), [
        o.message for o in evaluate(scn, GAZE) if not o.passed
    ]
    off_pose = [*GAZE, head(12000, -30, settled=True)]
    bad = [o for o in evaluate(scn, off_pose) if not o.passed]
    assert len(bad) == 1 and "-30" in bad[0].message


def test_head_at_looks_up_the_state(scenario_with):
    scn = scenario_with(
        {"head_at": {"at": "5.5s", "pan": [-39, -1], "settled": False}},
        {"head_at": {"at": "8s", "pan": -40, "tilt": 0, "settled": True}},
        {"head_at": {"at": "9s", "tilt": -20, "mode": "gaze"}},
    )
    assert all(o.passed for o in evaluate(scn, GAZE))
    wrong = scenario_with({"head_at": {"at": "8s", "pan": [-3, 3]}})
    o = one(wrong, GAZE)
    assert not o.passed and "pan is -40, want [-3, 3]" in o.message
    assert "[4]" in o.message


def test_head_at_without_a_record_fails(scenario_with):
    # No initial state for the head: it is traced from tick 0 or not at all.
    scn = scenario_with({"head_at": {"at": "5s", "pan": 0}})
    o = one(scn, [])
    assert not o.passed and "no 'head' record at or before then" in o.message
    late = one(scn, [head(6000, 0)])
    assert not late.passed and "6000" in late.message


def test_settled_filter_does_not_match_a_number(scenario_with):
    scn = scenario_with({"head": {"settled": True, "count": 1}})
    assert not one(scn, [{**head(1000, 0), "settled": "true"}]).passed


@pytest.mark.parametrize(
    ("bad", "match"),
    [
        ({"wheels": {"left": "backward"}}, "expected one of"),
        ({"wheels": {"left_duty": [10]}}, "lo, hi"),
        ({"wheels": {"left_duty": [20, 10]}}, "lo <= hi"),
        ({"wheels": {"left_duty": "fast"}}, "number"),
        ({"wheels": {"left_duty": True}}, "number"),
        ({"wheels": {"duty": 5}}, "unknown argument"),
        ({"wheels": {"left": "brake", "count": -1}}, "non-negative"),
        ({"wheels_at": {"left": "brake"}}, "missing argument"),
        ({"wheels_at": {"at": "5s"}}, "at least one of"),
        ({"wheels_at": {"at": 5, "left": "brake"}}, "unit"),
        ({"wheels_at": {"at": "5s", "left": "brake", "count": 1}}, "unknown argument"),
        ({"wheels_at": {"at": "range[0]", "left": "brake"}}, "refers to range event 0"),
        ({"head": {"mode": "stare"}}, "expected one of"),
        ({"head": {"settled": 1}}, "true/false"),
        ({"head": {"pan": [0, "x"]}}, "lo, hi"),
        ({"head_at": {"at": "5s", "tilt": [1, 0]}}, "lo <= hi"),
        ({"pose_at": {"at": "5s", "x_cm": 0}}, "need plant: physics"),
        ({"pose": {"collided": True, "count": 0}}, "need plant: physics"),
        ({"gait_at": {"at": "5s"}}, "unknown assertion"),
    ],
)
def test_state_assertion_validation(scenario_with, bad, match):
    with pytest.raises(ScenarioError, match=match):
        scenario_with(bad)


@pytest.fixture
def physics_scenario_with(write_scenario, write_arena):
    def _make(*expect):
        arena = write_arena("room", "floor: [[0, 0], [100, 0], [100, 100]]\n")
        doc = {
            "seed": 1,
            "duration": "60s",
            "world": [{"at": "0s", "frame": {"scene": "room"}}],
            "models": {"planner": "inline"},
            "plant": {
                "physics": {
                    "arena": arena,
                    "start": {"x_cm": 0, "y_cm": 0, "heading_deg": 0},
                }
            },
            "default": {"status": 200},
            "expect": list(expect),
        }
        return load_scenario(write_scenario("p", yaml.safe_dump(doc)))

    return _make


def pose(at, x, y=0.0, heading=0.0, collided=False, off_floor=False):
    return {
        "t": "pose",
        "at_ms": at,
        "x_cm": x,
        "y_cm": y,
        "heading_deg": heading,
        "collided": collided,
        "off_floor": off_floor,
    }


PATH = [
    pose(33, 0.0),
    pose(16038, 0.086),
    pose(20000, 61.5, y=-0.25, heading=-1.5),
    pose(20033, 62.004, collided=True),
    pose(30000, 62.004, off_floor=True),
]


def test_pose_filters_and_bounds(physics_scenario_with):
    scn = physics_scenario_with(
        {"pose": {"collided": True, "count": 1}},
        {"pose": {"off_floor": True, "between": ["0s", "29.999s"], "count": 0}},
        {"pose": {"x_cm": [61, 62.004], "count": 3}},
        {"pose": {"x_cm": [61.6, 62.003], "count": 0}},
        {"pose": {"y_cm": [-1, -0.1], "heading_deg": [-2, -1], "count": 1}},
        {"pose": {"collided": False, "off_floor": False, "min": 3}},
    )
    assert all(o.passed for o in evaluate(scn, PATH)), [
        o.message for o in evaluate(scn, PATH) if not o.passed
    ]


def test_pose_at_looks_up_the_state(physics_scenario_with):
    scn = physics_scenario_with(
        {"pose_at": {"at": "15s", "x_cm": 0, "y_cm": 0, "heading_deg": 0}},
        {"pose_at": {"at": "20.032s", "x_cm": [50, 70], "collided": False}},
        {"pose_at": {"at": "25s", "collided": True, "off_floor": False}},
        {"pose_at": {"at": "60s", "off_floor": True}},
    )
    assert all(o.passed for o in evaluate(scn, PATH))
    wrong = physics_scenario_with({"pose_at": {"at": "25s", "collided": False}})
    o = one(wrong, PATH)
    assert not o.passed
    assert "pose at 25s: collided is True, want False" in o.message
    assert "[3]" in o.message and "20033" in o.message


def test_pose_at_before_the_first_reading_fails(physics_scenario_with):
    # The first pose record is the first reading, at 33 ms; there is no state
    # before it.
    scn = physics_scenario_with({"pose_at": {"at": "32ms", "x_cm": 0}})
    o = one(scn, PATH)
    assert not o.passed and "no 'pose' record at or before then" in o.message


@pytest.mark.parametrize(
    ("bad", "match"),
    [
        ({"pose": {"collided": "yes"}}, "true/false"),
        ({"pose": {"x_cm": [5, 1]}}, "lo <= hi"),
        ({"pose": {"x": 5}}, "unknown argument"),
        ({"pose_at": {"at": "5s"}}, "at least one of"),
        ({"pose_at": {"x_cm": 5}}, "missing argument"),
        ({"pose_at": {"at": "5s", "off_floor": 0}}, "true/false"),
    ],
)
def test_pose_assertion_validation(physics_scenario_with, bad, match):
    with pytest.raises(ScenarioError, match=match):
        physics_scenario_with(bad)
