"""Tests for ``stub_planner`` — the deterministic goal emitter.

Exercises the contract documented in ``src/stub_planner.py``:

* goals are emitted in the declared order
* the same goal is returned for repeated calls inside one period (like the
  MCU executor polling faster than the 1 Hz planner)
* the sequence wraps cyclically after the last entry
* ``reset()`` re-anchors the sequence to the next ``next_goal`` call
* invalid constructor args raise ``ValueError``
"""

import pytest

from stub_planner import DEFAULT_SEQUENCE, Goal, GoalKind, StubPlanner


def test_default_sequence_order():
    planner = StubPlanner(period_s=1.0)

    # Tick at t=0, 1, 2, 3 — one per period, one per sequence entry
    kinds = [planner.next_goal(now=float(i)).kind for i in range(len(DEFAULT_SEQUENCE))]
    expected = [g.kind for g in DEFAULT_SEQUENCE]
    assert kinds == expected
    assert kinds == [GoalKind.DRIVE, GoalKind.ROTATE, GoalKind.TRACK, GoalKind.STOP]


def test_drive_goal_shape_matches_mcu_contract():
    """Field names must match the C goal_t.drive struct in goal_state.h."""
    planner = StubPlanner(period_s=1.0)
    goal = planner.next_goal(now=0.0)
    assert goal.kind == GoalKind.DRIVE
    assert set(goal.params.keys()) == {"heading_deg", "distance_cm", "speed_pct"}


def test_track_goal_shape_matches_mcu_contract():
    """Field names must match the C goal_t.track struct in goal_state.h."""
    planner = StubPlanner(period_s=1.0)
    planner.next_goal(now=0.0)
    planner.next_goal(now=1.0)
    track = planner.next_goal(now=2.0)
    assert track.kind == GoalKind.TRACK
    assert set(track.params.keys()) == {"ymin", "xmin", "ymax", "xmax", "max_speed_pct"}


def test_rotate_goal_shape_matches_mcu_contract():
    planner = StubPlanner(period_s=1.0)
    planner.next_goal(now=0.0)
    rotate = planner.next_goal(now=1.0)
    assert rotate.kind == GoalKind.ROTATE
    assert set(rotate.params.keys()) == {"angle_deg"}


def test_stable_within_period():
    """Executor polls 30× per second; planner updates 1× per second. The same
    goal must be returned for all polls inside one period."""
    planner = StubPlanner(period_s=1.0)
    first = planner.next_goal(now=0.0)
    for dt in (0.01, 0.1, 0.5, 0.99):
        assert planner.next_goal(now=dt) == first


def test_advances_on_period_boundary():
    planner = StubPlanner(period_s=1.0)
    g0 = planner.next_goal(now=0.0)
    g1 = planner.next_goal(now=1.0)
    assert g0 != g1
    assert g0.kind == DEFAULT_SEQUENCE[0].kind
    assert g1.kind == DEFAULT_SEQUENCE[1].kind


def test_sequence_wraps_cyclically():
    planner = StubPlanner(period_s=1.0)
    n = len(DEFAULT_SEQUENCE)
    first_loop = [planner.next_goal(now=float(i)).kind for i in range(n)]
    second_loop = [planner.next_goal(now=float(n + i)).kind for i in range(n)]
    assert first_loop == second_loop


def test_reset_reanchors_sequence():
    planner = StubPlanner(period_s=1.0)
    planner.next_goal(now=10.0)
    planner.next_goal(now=11.0)  # would land on index 1
    planner.reset()
    # After reset, the first call anchors at its own "now" → index 0
    after = planner.next_goal(now=50.0)
    assert after.kind == DEFAULT_SEQUENCE[0].kind


def test_custom_sequence():
    custom = (
        Goal(kind=GoalKind.STOP, params={}),
        Goal(kind=GoalKind.DRIVE, params={"heading_deg": 0, "distance_cm": 10, "speed_pct": 30}),
    )
    planner = StubPlanner(period_s=0.5, sequence=custom)
    assert planner.next_goal(now=0.0).kind == GoalKind.STOP
    assert planner.next_goal(now=0.5).kind == GoalKind.DRIVE
    assert planner.next_goal(now=1.0).kind == GoalKind.STOP  # wrap


def test_custom_period():
    planner = StubPlanner(period_s=2.0)
    assert planner.next_goal(now=0.0).kind == DEFAULT_SEQUENCE[0].kind
    # at t=1.0, still inside first period
    assert planner.next_goal(now=1.0).kind == DEFAULT_SEQUENCE[0].kind
    assert planner.next_goal(now=2.0).kind == DEFAULT_SEQUENCE[1].kind


def test_invalid_period_rejected():
    with pytest.raises(ValueError):
        StubPlanner(period_s=0.0)
    with pytest.raises(ValueError):
        StubPlanner(period_s=-0.1)


def test_empty_sequence_rejected():
    with pytest.raises(ValueError):
        StubPlanner(period_s=1.0, sequence=())
