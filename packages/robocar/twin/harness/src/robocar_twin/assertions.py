"""Deterministic checks over a twin trace (PRD-017 "D" oracle).

Each expectation in a scenario is a one-key mapping, ``{name: {args}}``. Times
in ``between``/``after`` are durations or ``<kind>[i]`` references to world
events. A window is ``between: [start, end]`` (inclusive), or
``after: X, within: D`` meaning [X, X + D], or the whole run when neither is
given. Every failure message lists the trace records it is about, with their
index in the trace, so a reader can find them in the JSONL.
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
    "cycle": _Spec(
        _WINDOW | _COUNTS | {"cause", "requested", "gate", "spent_req", "spent_tok"},
        set(),
        _counted("cycle", ("cause", "requested", "gate", "spent_req", "spent_tok"), 1),
    ),
}


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
    _window(scenario, args, w)


def evaluate(scenario: Scenario, trace: list[dict[str, Any]]) -> list[Outcome]:
    """Run every expectation of ``scenario`` against ``trace``."""
    outcomes = []
    for i, item in enumerate(scenario.expect):
        name, args = _split(scenario, item, f"expect[{i}]")
        msg = REGISTRY[name].check(scenario, trace, args, f"expect[{i}].{name}")
        outcomes.append(Outcome(name, args, msg is None, msg or ""))
    return outcomes
