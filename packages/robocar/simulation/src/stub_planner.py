"""
Stub planner for hierarchical-AI-controller sim/dev runs.

The real planner on the robot (robocar-unified/main/planner_task.c) calls
Gemini Robotics-ER 1.6 at ~1 Hz and writes structured goals into a shared
goal state that the 30 Hz reactive executor drains. For sim regression runs
and offline development we do not want to pay that API cost nor hit Gemini
rate limits — this module emits a deterministic round-robin sequence of the
same goal shapes so the executor can be exercised end-to-end.

The goal shapes here mirror the C ``goal_t`` union in
``packages/robocar/unified/main/goal_state.h``:

  drive:   {heading_deg, distance_cm, speed_pct}
  track:   {ymin, xmin, ymax, xmax, max_speed_pct}   # Gemini ER 1.6 0..1000 units
  rotate:  {angle_deg}
  stop:    {}

Usage::

    planner = StubPlanner(period_s=1.0)
    while sim.running:
        goal = planner.next_goal(now=sim.clock())
        executor.consume(goal)
        sim.step()
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import StrEnum
from typing import Any


class GoalKind(StrEnum):
    """Mirrors goal_kind_t in goal_state.h."""

    NONE = "none"
    STOP = "stop"
    DRIVE = "drive"
    TRACK = "track"
    ROTATE = "rotate"


@dataclass(frozen=True)
class Goal:
    """Shape-compatible with the C ``goal_t`` union.

    ``params`` is a plain dict so the sim's executor port can pattern-match on
    ``kind`` and look up the fields it expects without depending on this
    module's classes.
    """

    kind: GoalKind
    params: dict[str, Any]


DEFAULT_SEQUENCE: tuple[Goal, ...] = (
    Goal(
        kind=GoalKind.DRIVE,
        params={"heading_deg": 0, "distance_cm": 100, "speed_pct": 60},
    ),
    Goal(
        kind=GoalKind.ROTATE,
        params={"angle_deg": 90},
    ),
    Goal(
        kind=GoalKind.TRACK,
        params={
            "ymin": 400,
            "xmin": 400,
            "ymax": 600,
            "xmax": 600,
            "max_speed_pct": 50,
        },
    ),
    Goal(kind=GoalKind.STOP, params={}),
)


class StubPlanner:
    """Deterministic cyclic planner emitting one goal per ``period_s``.

    The planner advances on a monotonic "now" value the caller supplies (seconds
    since epoch-of-choice). Calling :meth:`next_goal` multiple times inside one
    period returns the same goal — matching the MCU contract where the executor
    polls faster than the planner updates.
    """

    def __init__(
        self,
        period_s: float = 1.0,
        sequence: tuple[Goal, ...] = DEFAULT_SEQUENCE,
    ):
        if period_s <= 0:
            raise ValueError("period_s must be positive")
        if not sequence:
            raise ValueError("sequence must contain at least one goal")
        self._period_s = float(period_s)
        self._sequence = sequence
        self._start: float | None = None

    @property
    def period_s(self) -> float:
        return self._period_s

    @property
    def sequence(self) -> tuple[Goal, ...]:
        return self._sequence

    def reset(self) -> None:
        """Forget the anchor time — next :meth:`next_goal` reopens the sequence."""
        self._start = None

    def next_goal(self, now: float) -> Goal:
        """Return the goal the real planner *would* have published by ``now``.

        ``now`` is seconds in any monotonic frame; the first call anchors the
        sequence, and index ``(now - start) // period_s`` (modulo sequence
        length) selects the goal.
        """
        if self._start is None:
            self._start = now
        elapsed = max(0.0, now - self._start)
        idx = int(elapsed // self._period_s) % len(self._sequence)
        return self._sequence[idx]
