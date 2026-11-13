"""Deterministic checks over a twin trace (PRD-017 "D" oracle).

Each expectation in a scenario is a one-key mapping, ``{name: {args}}``. Times
in ``between``/``after`` are durations or ``<kind>[i]`` references to world
events. A window is ``between: [start, end]`` (inclusive), or
``after: X, within: D`` meaning [X, X + D], or the whole run when neither is
given. Every failure message lists the trace records it is about, with their
index in the trace, so a reader can find them in the JSONL.

``wheels`` and ``head`` records report a state when it changes, so they get two
assertions each: ``wheels`` counts the change records in a window that match
its filters, and ``wheels_at`` checks the state in force at one time (the last
record at or before it). Numeric filters take ``[lo, hi]`` (inclusive) or one
number. A new state record is one ``_StateKind`` entry in ``_STATE_KINDS``.
"""

from __future__ import annotations

import itertools
import json
from collections.abc import Callable
from dataclasses import dataclass
from typing import Any

from .scenario import Scenario, ScenarioError
from .timeparse import format_ms

MAX_CITED = 8
UTTERANCE_SOURCES = ("planner", "voice_turn")
SUPPRESS_REASONS = ("repeat", "person_spoke")
ANNOUNCE_WHAT = ("dormant", "budget", "waking")
NOTE_TRIGGERS = ("reflex", "blind", "track_stuck", "repeats", "reflect")
NOTE_KIND_NAMES = ("difficulty", "behaviour", "suggestion")
FORCE_STOP_REASONS = ("camera", "dormant", "budget", "no_link", "backend_error")
GOAL_KINDS = ("stop", "drive", "track", "rotate", "none")
CYCLE_POSES = ("C", "L", "R", "F", "-")  # scene_pose_letter() in main/scene_change.c
WHEEL_DRIVES = ("forward", "reverse", "coast", "brake")  # twin_exec.c decode_wheel()
# reactive_head_mode_name() in main/reactive_controller.c
HEAD_MODES = ("unavailable", "centre", "aim", "turn", "lease", "gaze")
LADDER_MULTIPLIERS = (1, 2, 4, 8, 20)  # k_ladder in plan_activity.c

_WINDOW = {"between", "within", "after"}
_COUNTS = {"count", "min", "max"}


@dataclass(frozen=True)
class Outcome:
    name: str
    spec: dict[str, Any]
    passed: bool
    message: str = ""

    def to_json(self) -> dict[str, Any]:
        return {
            "assertion": {self.name: self.spec},
            "passed": self.passed,
            "message": self.message,
        }


Indexed = list[tuple[int, dict[str, Any]]]


def _cite(records: Indexed, label: str = "records") -> str:
    if not records:
        return f"  ({label}: none)"
    lines = [
        f"  [{i}] {json.dumps(r, ensure_ascii=False)}" for i, r in records[:MAX_CITED]
    ]
    if len(records) > MAX_CITED:
        lines.append(f"  ... and {len(records) - MAX_CITED} more")
    return f"  {label}:\n" + "\n".join(lines)


def _window(scenario: Scenario, args: dict[str, Any], where: str) -> tuple[int, int]:
    if "between" in args:
        if "within" in args or "after" in args:
            raise ScenarioError(
                f"{scenario.path}: {where}: use 'between' or 'after'+'within'"
            )
        b = args["between"]
        if not isinstance(b, list) or len(b) != 2:
            raise ScenarioError(
                f"{scenario.path}: {where}.between: expected [start, end]"
            )
        lo = scenario.resolve_time(b[0], where=f"{where}.between[0]")
        hi = scenario.resolve_time(b[1], where=f"{where}.between[1]")
        if hi < lo:
            raise ScenarioError(f"{scenario.path}: {where}.between: end before start")
        return lo, hi
    if "within" in args or "after" in args:
        if "within" not in args or "after" not in args:
            raise ScenarioError(
                f"{scenario.path}: {where}: 'within' needs 'after' and vice versa"
            )
        lo = scenario.resolve_time(args["after"], where=f"{where}.after")
        return lo, lo + scenario.resolve_time(args["within"], where=f"{where}.within")
    return 0, scenario.duration_ms


def _select(
    trace: list[dict[str, Any]], t: str, lo: int, hi: int, **match: Any
) -> tuple[Indexed, Indexed]:
    """(records of type t matching every ``match`` in [lo, hi], all records of type t)."""
    every = [(i, r) for i, r in enumerate(trace) if r.get("t") == t]
    hits = []
    for i, r in every:
        at = r.get("at_ms")
        if not isinstance(at, int) or not lo <= at <= hi:
            continue
        ok = True
        for k, v in match.items():
            if v is None:
                continue
            if k == "text_contains":
                ok = ok and v.lower() in str(r.get("text", "")).lower()
            elif k in r:
                ok = ok and r[k] == v
            else:
                # model_request carries its gates inside the descriptor.
                ok = ok and (r.get("descriptor") or {}).get(k) == v
        if ok:
            hits.append((i, r))
    return hits, every


def _check_count(args: dict[str, Any], n: int, default_min: int | None) -> str | None:
    if "count" in args and n != args["count"]:
        return f"expected exactly {args['count']}, found {n}"
    if "min" in args and n < args["min"]:
        return f"expected at least {args['min']}, found {n}"
    if "max" in args and n > args["max"]:
        return f"expected at most {args['max']}, found {n}"
    if not (_COUNTS & set(args)) and default_min is not None and n < default_min:
        return f"expected at least {default_min}, found {n}"
    return None


def _counted(
    t: str, filters: tuple[str, ...], default_min: int | None
) -> Callable[[Scenario, list[dict[str, Any]], dict[str, Any], str], str | None]:
    def check(
        scn: Scenario, trace: list[dict[str, Any]], args: dict[str, Any], where: str
    ):
        lo, hi = _window(scn, args, where)
        match = {f: args.get(f) for f in filters}
        hits, every = _select(trace, t, lo, hi, **match)
        err = _check_count(args, len(hits), default_min)
        if err is None:
            return None
        desc = ", ".join(f"{k}={v!r}" for k, v in match.items() if v is not None)
        head = f"{t}({desc}) in [{format_ms(lo)}, {format_ms(hi)}]: {err}"
        cited = hits if hits else every
        return head + "\n" + _cite(cited, "matching" if hits else f"all '{t}' records")

    return check


def _no_utterance(
    scn: Scenario, trace: list[dict[str, Any]], args: dict[str, Any], where: str
):
    return _counted("utterance", ("source", "text_contains"), None)(
        scn, trace, {**args, "count": 0}, where
    )


def _request_times(
    scn: Scenario, trace: list[dict[str, Any]], args: dict[str, Any], where: str
) -> tuple[Indexed, int, int]:
    lo, hi = _window(scn, args, where)
    hits, _ = _select(trace, "model_request", lo, hi)
    return hits, lo, hi


def _requests(
    scn: Scenario, trace: list[dict[str, Any]], args: dict[str, Any], where: str
):
    hits, lo, hi = _request_times(scn, trace, args, where)
    err = _check_count(args, len(hits), None)
    if err is None:
        return None
    return f"model_request in [{format_ms(lo)}, {format_ms(hi)}]: {err}\n" + _cite(hits)


def _period_ms(scn: Scenario) -> int:
    return scn.protocol_lines()[0]["planner_period_ms"]


def _intervals_check(
    scn: Scenario,
    trace: list[dict[str, Any]],
    args: dict[str, Any],
    where: str,
    expected: list,
):
    tol = scn.resolve_time(args.get("tolerance", "1s"), where=f"{where}.tolerance")
    exact = args.get("exact", True)
    hits, lo, hi = _request_times(scn, trace, args, where)
    times = [r["at_ms"] for _, r in hits]
    gaps = [b - a for a, b in itertools.pairwise(times)]
    shown = "[" + ", ".join(format_ms(g) for g in gaps) + "]"
    want = "[" + ", ".join(format_ms(g) for g in expected) + "]"
    prefix = f"request intervals in [{format_ms(lo)}, {format_ms(hi)}] were {shown}, want {want}"
    if len(gaps) < len(expected) or (exact and len(gaps) != len(expected)):
        extra = " then no further request" if exact else ""
        return f"{prefix}{extra}: {len(times)} requests\n" + _cite(hits)
    for k, (got, exp) in enumerate(zip(gaps, expected, strict=False)):
        if abs(got - exp) > tol:
            return (
                f"{prefix}: interval {k} is {format_ms(got)}, want {format_ms(exp)} "
                f"± {format_ms(tol)}\n" + _cite(hits[k : k + 2])
            )
    return None


def _request_intervals(scn: Scenario, trace, args: dict[str, Any], where: str):
    expected = [
        scn.resolve_time(v, where=f"{where}.intervals[{i}]")
        for i, v in enumerate(args["intervals"])
    ]
    return _intervals_check(scn, trace, args, where, expected)


def _request_ladder(scn: Scenario, trace, args: dict[str, Any], where: str):
    period = _period_ms(scn)
    return _intervals_check(
        scn, trace, args, where, [m * period for m in LADDER_MULTIPLIERS]
    )


def _sequence(t: str, field: str):
    def check(
        scn: Scenario, trace: list[dict[str, Any]], args: dict[str, Any], where: str
    ):
        lo, hi = _window(scn, args, where)
        hits, _ = _select(trace, t, lo, hi)
        got = [r.get(field) for _, r in hits]
        if got == list(args["sequence"]):
            return None
        return (
            f"{t}.{field} sequence in [{format_ms(lo)}, {format_ms(hi)}] was {got}, "
            f"want {list(args['sequence'])}\n" + _cite(hits)
        )

    return check


@dataclass(frozen=True)
class _StateKind:
    """A trace record that reports a state each time it changes.

    ``exact`` maps a filter name to the field's path in the record and its
    allowed values (``bool`` for a true/false field); ``numeric`` maps a filter
    name to the path of a number. ``initial`` is the state before the first
    record, for a kind whose starting state is not traced.
    """

    t: str
    exact: dict[str, tuple[tuple[str, ...], tuple[str, ...] | type]]
    numeric: dict[str, tuple[str, ...]]
    initial: dict[str, Any] | None = None

    @property
    def filters(self) -> set[str]:
        return set(self.exact) | set(self.numeric)

    def path(self, name: str) -> tuple[str, ...]:
        return self.exact[name][0] if name in self.exact else self.numeric[name]


_STATE_KINDS: dict[str, _StateKind] = {
    "wheels": _StateKind(
        "wheels",
        exact={
            "left": (("left", "drive"), WHEEL_DRIVES),
            "right": (("right", "drive"), WHEEL_DRIVES),
        },
        numeric={"left_duty": ("left", "duty"), "right_duty": ("right", "duty")},
        # What motor_controller_init() leaves; the twin does not trace it
        # (ADR-036 section 1 "Trace").
        initial={
            "left": {"drive": "coast", "duty": 0},
            "right": {"drive": "coast", "duty": 0},
        },
    ),
    # Traced from tick 0 when config.head is set, so there is no initial state.
    "head": _StateKind(
        "head",
        exact={"mode": (("mode",), HEAD_MODES), "settled": (("settled",), bool)},
        numeric={"pan": ("pan_deg",), "tilt": ("tilt_deg",)},
    ),
}


def _is_number(v: object) -> bool:
    return isinstance(v, int | float) and not isinstance(v, bool)


def _bounds(want: Any) -> tuple[float, float]:
    """A numeric filter as (lo, hi): ``[lo, hi]`` or one number."""
    if isinstance(want, list):
        return want[0], want[1]
    return want, want


def _field_at(rec: dict[str, Any], path: tuple[str, ...]) -> Any:
    cur: Any = rec
    for key in path:
        if not isinstance(cur, dict):
            return None
        cur = cur.get(key)
    return cur


def _state_mismatches(
    kind: _StateKind, rec: dict[str, Any], filters: dict[str, Any]
) -> list[str]:
    """One line per filter ``rec`` does not satisfy; empty when it matches."""
    out = []
    for name, want in filters.items():
        got = _field_at(rec, kind.path(name))
        if name in kind.numeric:
            lo, hi = _bounds(want)
            if not _is_number(got) or not lo <= got <= hi:
                shown = f"{lo:g}" if lo == hi else f"[{lo:g}, {hi:g}]"
                out.append(f"{name} is {got!r}, want {shown}")
        elif got != want:
            out.append(f"{name} is {got!r}, want {want!r}")
    return out


def _describe(filters: dict[str, Any]) -> str:
    return ", ".join(f"{k}={v!r}" for k, v in filters.items())


def _state_changes(kind: _StateKind):
    """Count the ``kind`` records in the window that match every filter."""

    def check(
        scn: Scenario, trace: list[dict[str, Any]], args: dict[str, Any], where: str
    ):
        lo, hi = _window(scn, args, where)
        filters = {k: args[k] for k in args if k in kind.filters}
        every = [(i, r) for i, r in enumerate(trace) if r.get("t") == kind.t]
        hits = [
            (i, r)
            for i, r in every
            if isinstance(r.get("at_ms"), int)
            and lo <= r["at_ms"] <= hi
            and not _state_mismatches(kind, r, filters)
        ]
        err = _check_count(args, len(hits), 1)
        if err is None:
            return None
        head = f"{kind.t}({_describe(filters)}) in [{format_ms(lo)}, {format_ms(hi)}]: {err}"
        in_window = [(i, r) for i, r in every if lo <= r.get("at_ms", -1) <= hi]
        if hits:
            return head + "\n" + _cite(hits, "matching")
        if in_window:
            return head + "\n" + _cite(in_window, f"'{kind.t}' records in the window")
        return head + "\n" + _cite(every, f"all '{kind.t}' records")

    return check


def _state_at(kind: _StateKind):
    """Check the state in force at ``at``: the last ``kind`` record at or before it."""

    def check(
        scn: Scenario, trace: list[dict[str, Any]], args: dict[str, Any], where: str
    ):
        at = scn.resolve_time(args["at"], where=f"{where}.at")
        filters = {k: args[k] for k in args if k in kind.filters}
        every = [(i, r) for i, r in enumerate(trace) if r.get("t") == kind.t]
        before = [
            (i, r)
            for i, r in every
            if isinstance(r.get("at_ms"), int) and r["at_ms"] <= at
        ]
        head = f"{kind.t} at {format_ms(at)}"
        if before:
            # The twin writes these in time order; the latest wins a tie.
            index, state = max(before, key=lambda ir: (ir[1]["at_ms"], ir[0]))
            cited = _cite([(index, state)], "state from")
        elif kind.initial is not None:
            state = kind.initial
            cited = (
                f"  no '{kind.t}' record at or before then; the state is the one "
                f"before the first record: {json.dumps(state)}"
            )
        else:
            return f"{head}: no '{kind.t}' record at or before then\n" + _cite(
                every, f"all '{kind.t}' records"
            )
        bad = _state_mismatches(kind, state, filters)
        if not bad:
            return None
        return f"{head}: " + "; ".join(bad) + "\n" + cited

    return check


@dataclass(frozen=True)
class _Spec:
    keys: set[str]
    required: set[str]
    check: Callable[[Scenario, list[dict[str, Any]], dict[str, Any], str], str | None]
    enums: dict[str, tuple] | None = None


REGISTRY: dict[str, _Spec] = {
    "utterance": _Spec(
        _WINDOW | _COUNTS | {"source", "text_contains"},
        set(),
        _counted("utterance", ("source", "text_contains"), 1),
        {"source": UTTERANCE_SOURCES},
    ),
    "no_utterance": _Spec(
        _WINDOW | {"source", "text_contains"},
        set(),
        _no_utterance,
        {"source": UTTERANCE_SOURCES},
    ),
    "suppressed": _Spec(
        _WINDOW | _COUNTS | {"reason", "text_contains"},
        set(),
        _counted("suppressed", ("reason", "text_contains"), 1),
        {"reason": SUPPRESS_REASONS},
    ),
    "requests": _Spec(_WINDOW | _COUNTS, set(), _requests),
    "request_intervals": _Spec(
        _WINDOW | {"intervals", "tolerance", "exact"}, {"intervals"}, _request_intervals
    ),
    "request_ladder": _Spec(_WINDOW | {"tolerance", "exact"}, set(), _request_ladder),
    "announce": _Spec(
        _WINDOW | _COUNTS | {"what"},
        {"what"},
        _counted("announce", ("what",), 1),
        {"what": ANNOUNCE_WHAT},
    ),
    "force_stop": _Spec(
        _WINDOW | _COUNTS | {"reason"},
        set(),
        _counted("force_stop", ("reason",), 1),
        {"reason": FORCE_STOP_REASONS},
    ),
    "backoff": _Spec(
        _WINDOW | {"sequence"}, {"sequence"}, _sequence("backoff", "periods")
    ),
    "goal": _Spec(
        _WINDOW | _COUNTS | {"kind"},
        set(),
        _counted("goal", ("kind",), 1),
        {"kind": GOAL_KINDS},
    ),
    "goals": _Spec(_WINDOW | {"sequence"}, {"sequence"}, _sequence("goal", "kind")),
    # A reply carried `speak` when the request did not offer it; the twin
    # dropped the line (twin_main.c backend_plan()).
    "speak_not_offered": _Spec(
        _WINDOW | _COUNTS | {"text_contains"},
        set(),
        _counted("speak_not_offered", ("text_contains",), 1),
    ),
    # The note gate (note_gate.h): when the `note` tool was offered, what the
    # model wrote with it, and a note in a reply that did not offer it.
    "note_offered": _Spec(
        _WINDOW | _COUNTS | {"trigger"},
        set(),
        _counted("note_offered", ("trigger",), 1),
        {"trigger": NOTE_TRIGGERS},
    ),
    "note": _Spec(
        _WINDOW | _COUNTS | {"trigger", "kind", "text_contains"},
        set(),
        _counted("note", ("trigger", "kind", "text_contains"), 1),
        {"trigger": NOTE_TRIGGERS, "kind": NOTE_KIND_NAMES},
    ),
    "note_not_offered": _Spec(
        _WINDOW | _COUNTS | {"text_contains"},
        set(),
        _counted("note_not_offered", ("text_contains",), 1),
    ),
    "model_request": _Spec(
        _WINDOW | _COUNTS | {"gate", "speak_allowed", "scene", "pose", "note"},
        set(),
        _counted(
            "model_request", ("gate", "speak_allowed", "scene", "pose", "note"), 1
        ),
    ),
    # `pose` is the dwell pose of the frame the cycle captured: C, L, R, F, or
    # "-" for a frame taken off every pose or before the head settled.
    "cycle": _Spec(
        _WINDOW
        | _COUNTS
        | {"cause", "requested", "gate", "pose", "spent_req", "spent_tok"},
        set(),
        _counted(
            "cycle", ("cause", "requested", "gate", "pose", "spent_req", "spent_tok"), 1
        ),
        {"pose": CYCLE_POSES},
    ),
}

# Each state kind gets `<name>` (change records in a window) and `<name>_at`
# (the state at one time). Maps the assertion name to its kind.
_STATE_ASSERTIONS: dict[str, _StateKind] = {}
for _name, _kind in _STATE_KINDS.items():
    REGISTRY[_name] = _Spec(
        _WINDOW | _COUNTS | _kind.filters, set(), _state_changes(_kind)
    )
    REGISTRY[f"{_name}_at"] = _Spec({"at"} | _kind.filters, {"at"}, _state_at(_kind))
    _STATE_ASSERTIONS[_name] = _STATE_ASSERTIONS[f"{_name}_at"] = _kind


def _split(scenario: Scenario, item: object, where: str) -> tuple[str, dict[str, Any]]:
    if not isinstance(item, dict) or len(item) != 1:
        raise ScenarioError(
            f"{scenario.path}: {where}: expected a one-key mapping like "
            f"{{utterance: {{...}}}}, got {item!r}"
        )
    ((name, args),) = item.items()
    if name not in REGISTRY:
        raise ScenarioError(
            f"{scenario.path}: {where}: unknown assertion {name!r}; known: {sorted(REGISTRY)}"
        )
    if args is None:
        args = {}
    if not isinstance(args, dict):
        raise ScenarioError(
            f"{scenario.path}: {where}.{name}: expected a mapping of arguments"
        )
    return name, args


def _validate_state_filters(
    scenario: Scenario, kind: _StateKind, args: dict[str, Any], w: str
) -> None:
    for name in kind.filters & set(args):
        v = args[name]
        if name in kind.numeric:
            ok = _is_number(v) or (
                isinstance(v, list)
                and len(v) == 2
                and all(_is_number(x) for x in v)
                and v[0] <= v[1]
            )
            if not ok:
                raise ScenarioError(
                    f"{scenario.path}: {w}.{name}: expected a number or [lo, hi] "
                    f"with lo <= hi, got {v!r}"
                )
            continue
        allowed = kind.exact[name][1]
        if allowed is bool:
            if not isinstance(v, bool):
                raise ScenarioError(
                    f"{scenario.path}: {w}.{name}: expected true/false, got {v!r}"
                )
        elif v not in allowed:
            raise ScenarioError(
                f"{scenario.path}: {w}.{name}: expected one of {allowed}, got {v!r}"
            )


def validate_expectation(scenario: Scenario, item: object, where: str) -> None:
    """Check names, argument keys, enum values, counts and times. Raises ScenarioError."""
    name, args = _split(scenario, item, where)
    spec = REGISTRY[name]
    w = f"{where}.{name}"
    unknown = set(args) - spec.keys
    if unknown:
        raise ScenarioError(
            f"{scenario.path}: {w}: unknown argument(s) {sorted(unknown)}; "
            f"allowed {sorted(spec.keys)}"
        )
    missing = spec.required - set(args)
    if missing:
        raise ScenarioError(
            f"{scenario.path}: {w}: missing argument(s) {sorted(missing)}"
        )
    if name == "requests" and not _COUNTS & set(args):
        raise ScenarioError(f"{scenario.path}: {w}: give count, min or max")
    for k in _COUNTS & set(args):
        v = args[k]
        if isinstance(v, bool) or not isinstance(v, int) or v < 0:
            raise ScenarioError(
                f"{scenario.path}: {w}.{k}: expected a non-negative integer"
            )
    for k, allowed in (spec.enums or {}).items():
        if k in args and args[k] not in allowed:
            raise ScenarioError(
                f"{scenario.path}: {w}.{k}: expected one of {allowed}, got {args[k]!r}"
            )
    if "sequence" in args and not isinstance(args["sequence"], list):
        raise ScenarioError(f"{scenario.path}: {w}.sequence: expected a list")
    if name == "goals":
        bad = [k for k in args["sequence"] if k not in GOAL_KINDS]
        if bad:
            raise ScenarioError(
                f"{scenario.path}: {w}.sequence: unknown goal kind(s) {bad}"
            )
    if name == "backoff" and not all(
        isinstance(p, int) and not isinstance(p, bool) and p > 0
        for p in args["sequence"]
    ):
        raise ScenarioError(
            f"{scenario.path}: {w}.sequence: expected positive integers"
        )
    if name == "request_intervals":
        if not isinstance(args["intervals"], list) or not args["intervals"]:
            raise ScenarioError(
                f"{scenario.path}: {w}.intervals: expected a non-empty list"
            )
        for i, v in enumerate(args["intervals"]):
            scenario.resolve_time(v, where=f"{w}.intervals[{i}]")
    if "exact" in args and not isinstance(args["exact"], bool):
        raise ScenarioError(f"{scenario.path}: {w}.exact: expected true/false")
    if "tolerance" in args:
        scenario.resolve_time(args["tolerance"], where=f"{w}.tolerance")
    if name in _STATE_ASSERTIONS:
        kind = _STATE_ASSERTIONS[name]
        _validate_state_filters(scenario, kind, args, w)
        if "at" in spec.required:
            scenario.resolve_time(args["at"], where=f"{w}.at")
            if not kind.filters & set(args):
                raise ScenarioError(
                    f"{scenario.path}: {w}: give at least one of {sorted(kind.filters)}"
                )
    _window(scenario, args, w)


def evaluate(scenario: Scenario, trace: list[dict[str, Any]]) -> list[Outcome]:
    """Run every expectation of ``scenario`` against ``trace``."""
    outcomes = []
    for i, item in enumerate(scenario.expect):
        name, args = _split(scenario, item, f"expect[{i}]")
        msg = REGISTRY[name].check(scenario, trace, args, f"expect[{i}].{name}")
        outcomes.append(Outcome(name, args, msg is None, msg or ""))
    return outcomes
