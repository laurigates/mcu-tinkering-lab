"""Scenario files (scenarios/*.yaml): load, validate, and turn into protocol lines.

Validation fails fast with the scenario path and the offending field, because a
scenario that loads with a typo (``voice_trun:``) would otherwise run as a
scenario with one event fewer and pass for the wrong reason.
"""

from __future__ import annotations

import json
import re
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import yaml

from .timeparse import TimeParseError, parse_ms

POSES = ("C", "L", "R", "F")
EVENT_KINDS = ("frame", "range", "range_fail", "sound", "voice_turn", "console", "link")
CONFIG_KEYS = (
    "planner_period_ms",
    "plan_budget_max_requests",
    "plan_budget_max_tokens",
)
MOTION_CALLS = ("stop", "drive", "track", "rotate")
DEFAULT_PLANNER_PERIOD_MS = 15000
DEFAULT_PROMPT_TOKENS = 1200
DEFAULT_OUTPUT_TOKENS = 30

_TOP_KEYS = {
    "id",
    "stories",
    "description",
    "persona",
    "seed",
    "duration",
    "config",
    "world",
    "models",
    "responses",
    "default",
    "expect",
    "rubric",
}
_REF = re.compile(r"^([a-z_]+)\[(\d+)\]$")


class ScenarioError(ValueError):
    """A scenario file that does not validate."""


@dataclass(frozen=True)
class WorldEvent:
    """One timed world event, already expanded and in protocol form."""

    at_ms: int
    kind: str
    fields: dict[str, Any]

    def to_protocol(self) -> dict[str, Any]:
        return {"t": self.kind, "at_ms": self.at_ms, **self.fields}


@dataclass(frozen=True)
class InlineResponse:
    """A scripted model reply: what the twin receives as model_response."""

    status: int
    latency_ms: int
    body: str


@dataclass
class Scenario:
    id: str
    path: Path
    stories: list[str]
    seed: int
    duration_ms: int
    config: dict[str, int]
    world: list[WorldEvent]
    planner_binding: str
    responses: list[InlineResponse] = field(default_factory=list)
    default_response: InlineResponse | None = None
    expect: list[dict[str, Any]] = field(default_factory=list)
    persona: str | None = None
    rubric: list[str] = field(default_factory=list)

    @property
    def twin_dir(self) -> Path:
        """The twin directory: the parent of the directory holding the scenario."""
        return self.path.resolve().parent.parent

    def protocol_lines(self) -> list[dict[str, Any]]:
        """Everything written to the twin's stdin before ``{"t":"run"}``."""
        cfg: dict[str, Any] = {
            "t": "config",
            "seed": self.seed,
            "duration_ms": self.duration_ms,
            "planner_period_ms": DEFAULT_PLANNER_PERIOD_MS,
        }
        cfg.update(self.config)
        return [cfg, *(e.to_protocol() for e in self.world)]

    def resolve_time(self, value: object, *, where: str) -> int:
        """A time in an assertion: a duration, or ``<kind>[i]`` naming a world event."""
        if isinstance(value, str):
            m = _REF.match(value.strip())
            if m:
                kind, idx = m.group(1), int(m.group(2))
                matching = [e for e in self.world if e.kind == kind]
                if idx >= len(matching):
                    raise ScenarioError(
                        f"{self.path}: {where}: {value!r} refers to {kind} event {idx}, "
                        f"but the world has {len(matching)}"
                    )
                return matching[idx].at_ms
        try:
            return parse_ms(value, field=where)
        except TimeParseError as exc:
            raise ScenarioError(f"{self.path}: {exc}") from exc


def _fail(path: Path, msg: str) -> ScenarioError:
    return ScenarioError(f"{path}: {msg}")


def _ms(path: Path, value: object, where: str) -> int:
    try:
        return parse_ms(value, field=where)
    except TimeParseError as exc:
        raise _fail(path, str(exc)) from exc


def _require_keys(
    path: Path, obj: object, where: str, required: set[str], optional: set[str]
):
    if not isinstance(obj, dict):
        raise _fail(path, f"{where}: expected a mapping, got {obj!r}")
    unknown = set(obj) - required - optional
    if unknown:
        raise _fail(
            path,
            f"{where}: unknown key(s) {sorted(unknown)}; allowed: {sorted(required | optional)}",
        )
    missing = required - set(obj)
    if missing:
        raise _fail(path, f"{where}: missing key(s) {sorted(missing)}")
    return obj


def _int(path: Path, value: object, where: str) -> int:
    if isinstance(value, bool) or not isinstance(value, int):
        raise _fail(path, f"{where}: expected an integer, got {value!r}")
    return value


def _event_fields(path: Path, kind: str, spec: object, where: str) -> dict[str, Any]:
    if kind == "frame":
        d = _require_keys(path, spec, where, {"scene"}, {"pose"})
        if not isinstance(d["scene"], str) or not d["scene"]:
            raise _fail(path, f"{where}.scene: expected a non-empty string")
        pose = d.get("pose", "C")
        if pose not in POSES:
            raise _fail(path, f"{where}.pose: expected one of {POSES}, got {pose!r}")
        return {"scene": d["scene"], "pose": pose}
    if kind == "range":
        if isinstance(spec, dict):
            spec = _require_keys(path, spec, where, {"cm"}, set())["cm"]
        cm = _int(path, spec, f"{where}.cm")
        if cm < 0:
            raise _fail(path, f"{where}.cm: must be >= 0")
        return {"cm": cm}
    if kind == "range_fail":
        if spec not in (None, True, {}):
            raise _fail(path, f"{where}: range_fail takes no fields, got {spec!r}")
        return {}
    if kind == "sound":
        d = _require_keys(path, spec, where, {"peak_db", "dur"}, set())
        return {
            "peak_db": _int(path, d["peak_db"], f"{where}.peak_db"),
            "dur_ms": _ms(path, d["dur"], f"{where}.dur"),
        }
    if kind == "voice_turn":
        d = _require_keys(path, spec, where, {"dur", "reply"}, {"record"})
        if not isinstance(d["reply"], str) or not d["reply"]:
            raise _fail(path, f"{where}.reply: expected a non-empty string")
        dur = _ms(path, d["dur"], f"{where}.dur")
        out: dict[str, Any] = {"dur_ms": dur, "reply": d["reply"]}
        if "record" in d:
            # How long record_clip() holds the mic; the whole turn when omitted.
            record = _ms(path, d["record"], f"{where}.record")
            if record > dur:
                raise _fail(path, f"{where}.record: longer than the turn ({dur} ms)")
            out["record_ms"] = record
        return out
    if kind == "console":
        if isinstance(spec, dict):
            spec = _require_keys(path, spec, where, {"line"}, set())["line"]
        if not isinstance(spec, str) or not spec:
            raise _fail(
                path, f"{where}: console expects a non-empty line, got {spec!r}"
            )
        return {"line": spec}
    if kind == "link":
        d = _require_keys(path, spec, where, {"wifi", "key"}, set())
        for k in ("wifi", "key"):
            if not isinstance(d[k], bool):
                raise _fail(path, f"{where}.{k}: expected true/false, got {d[k]!r}")
        return {"wifi": d["wifi"], "key": d["key"]}
    raise AssertionError(kind)  # unreachable: callers check EVENT_KINDS


def _parse_world(path: Path, raw: object, duration_ms: int) -> list[WorldEvent]:
    if not isinstance(raw, list) or not raw:
        raise _fail(path, "world: expected a non-empty list of events")
    events: list[WorldEvent] = []
    for i, item in enumerate(raw):
        where = f"world[{i}]"
        if not isinstance(item, dict):
            raise _fail(path, f"{where}: expected a mapping, got {item!r}")
        kinds = [k for k in item if k in EVENT_KINDS]
        unknown = set(item) - set(EVENT_KINDS) - {"at", "every", "until"}
        if unknown:
            raise _fail(
                path,
                f"{where}: unknown key(s) {sorted(unknown)}; event kinds: {EVENT_KINDS}",
            )
        if len(kinds) != 1:
            raise _fail(
                path, f"{where}: expected exactly one event kind, found {kinds}"
            )
        if "at" not in item:
            raise _fail(path, f"{where}: missing 'at'")
        kind = kinds[0]
        at = _ms(path, item["at"], f"{where}.at")
        fields = _event_fields(path, kind, item[kind], f"{where}.{kind}")
        times = [at]
        if "every" in item or "until" in item:
            if "every" not in item or "until" not in item:
                raise _fail(path, f"{where}: 'every' and 'until' go together")
            every = _ms(path, item["every"], f"{where}.every")
            until = _ms(path, item["until"], f"{where}.until")
            if every <= 0 or until < at:
                raise _fail(path, f"{where}: need every > 0 and until >= at")
            times = list(range(at, until + 1, every))
        for t in times:
            if t > duration_ms:
                raise _fail(
                    path, f"{where}: at {t} ms is past the duration {duration_ms} ms"
                )
            events.append(WorldEvent(t, kind, fields))
    # Stable sort: events at the same instant keep their file order.
    return sorted(events, key=lambda e: e.at_ms)


def build_body(
    call: str = "stop",
    args: dict[str, Any] | None = None,
    speak: str | None = None,
    tokens: dict[str, int] | int | None = None,
) -> str:
    """A Gemini generateContent body in the shape gemini_parse_response() reads.

    Motion and speech are parallel ``functionCall`` parts, as the model sends
    them (gemini_parse.c scans every part; test_gemini_parse.c
    test_parse_speak_alongside_motion).
    """
    parts: list[dict[str, Any]] = [
        {"functionCall": {"name": call, "args": dict(args or {})}}
    ]
    if speak is not None:
        parts.append({"functionCall": {"name": "speak", "args": {"text": speak}}})
    if isinstance(tokens, int):
        prompt, output = tokens, 0
    elif isinstance(tokens, dict):
        prompt, output = tokens.get("prompt", 0), tokens.get("output", 0)
    else:
        prompt, output = DEFAULT_PROMPT_TOKENS, DEFAULT_OUTPUT_TOKENS
    body = {
        "candidates": [
            {"content": {"role": "model", "parts": parts}, "finishReason": "STOP"}
        ],
        "usageMetadata": {
            "promptTokenCount": prompt,
            "candidatesTokenCount": output,
            "totalTokenCount": prompt + output,
        },
    }
    return json.dumps(body, separators=(",", ":"))


def _parse_response(path: Path, raw: object, where: str) -> InlineResponse:
    d = _require_keys(
        path,
        raw,
        where,
        {"status"},
        {"latency", "body", "call", "args", "speak", "tokens"},
    )
    status = _int(path, d["status"], f"{where}.status")
    if status not in (200, 429, 500, -1) and not 100 <= status <= 599:
        raise _fail(
            path, f"{where}.status: expected an HTTP status or -1, got {status}"
        )
    latency = _ms(path, d.get("latency", "1s"), f"{where}.latency")
    shorthand = {"call", "args", "speak", "tokens"} & set(d)
    if "body" in d and shorthand:
        raise _fail(
            path, f"{where}: give either 'body' or {sorted(shorthand)}, not both"
        )
    if "body" in d:
        body = d["body"]
        if isinstance(body, dict):
            body = json.dumps(body, separators=(",", ":"))
        if not isinstance(body, str):
            raise _fail(path, f"{where}.body: expected a JSON string or mapping")
    elif status == 200:
        call = d.get("call", "stop")
        if call not in MOTION_CALLS:
            raise _fail(
                path, f"{where}.call: expected one of {MOTION_CALLS}, got {call!r}"
            )
        args = d.get("args") or {}
        if not isinstance(args, dict):
            raise _fail(path, f"{where}.args: expected a mapping")
        speak = d.get("speak")
        if speak is not None and (not isinstance(speak, str) or not speak):
            raise _fail(path, f"{where}.speak: expected a non-empty string")
        tokens = d.get("tokens")
        if tokens is not None and not isinstance(tokens, int | dict):
            raise _fail(path, f"{where}.tokens: expected an int or {{prompt, output}}")
        body = build_body(call, args, speak, tokens)
    else:
        if shorthand:
            raise _fail(
                path, f"{where}: {sorted(shorthand)} only make sense with status 200"
            )
        body = ""
    return InlineResponse(status=status, latency_ms=latency, body=body)


def load_scenario(path: str | Path) -> Scenario:
    """Load and validate one scenario file."""
    from .assertions import validate_expectation  # local import: assertions imports us

    path = Path(path)
    try:
        raw = yaml.safe_load(path.read_text(encoding="utf-8"))
    except yaml.YAMLError as exc:
        raise _fail(path, f"invalid YAML: {exc}") from exc
    if not isinstance(raw, dict):
        raise _fail(path, "top level must be a mapping")
    unknown = set(raw) - _TOP_KEYS
    if unknown:
        raise _fail(
            path,
            f"unknown top-level key(s) {sorted(unknown)}; allowed {sorted(_TOP_KEYS)}",
        )
    for key in ("id", "seed", "duration", "world", "models", "expect"):
        if key not in raw:
            raise _fail(path, f"missing required key '{key}'")
    sid = raw["id"]
    if not isinstance(sid, str) or not re.fullmatch(r"[a-z0-9][a-z0-9-]*", sid):
        raise _fail(path, f"id: expected a kebab-case string, got {sid!r}")
    if sid != path.stem:
        raise _fail(path, f"id {sid!r} must match the file name {path.stem!r}")
    stories = raw.get("stories", [])
    if not isinstance(stories, list) or not all(isinstance(s, str) for s in stories):
        raise _fail(path, "stories: expected a list of story IDs")
    seed = _int(path, raw["seed"], "seed")
    duration_ms = _ms(path, raw["duration"], "duration")
    if duration_ms <= 0:
        raise _fail(path, "duration: must be > 0")

    config_raw = raw.get("config") or {}
    _require_keys(path, config_raw, "config", set(), set(CONFIG_KEYS))
    config = {k: _int(path, v, f"config.{k}") for k, v in config_raw.items()}

    world = _parse_world(path, raw["world"], duration_ms)

    models = _require_keys(path, raw["models"], "models", {"planner"}, set())
    binding = models["planner"]
    if not isinstance(binding, str) or not (
        binding == "inline" or (binding.startswith("replay:") and len(binding) > 7)
    ):
        raise _fail(
            path,
            f"models.planner: expected 'inline' or 'replay:<dir>', got {binding!r}",
        )

    responses_raw = raw.get("responses", [])
    if not isinstance(responses_raw, list):
        raise _fail(path, "responses: expected a list")
    responses = [
        _parse_response(path, r, f"responses[{i}]") for i, r in enumerate(responses_raw)
    ]
    default = (
        _parse_response(path, raw["default"], "default")
        if raw.get("default") is not None
        else None
    )
    if binding == "inline" and not responses and default is None:
        raise _fail(
            path,
            "models.planner is 'inline' but neither 'responses' nor 'default' given",
        )
    if binding != "inline" and (responses or default is not None):
        raise _fail(
            path, "'responses'/'default' only apply when models.planner is 'inline'"
        )

    expect = raw["expect"]
    if not isinstance(expect, list) or not expect:
        raise _fail(path, "expect: expected a non-empty list of assertions")

    rubric = raw.get("rubric", [])
    if not isinstance(rubric, list):
        raise _fail(path, "rubric: expected a list")
    persona = raw.get("persona")
    if persona is not None and not isinstance(persona, str):
        raise _fail(path, "persona: expected a string")

    scenario = Scenario(
        id=sid,
        path=path,
        stories=list(stories),
        seed=seed,
        duration_ms=duration_ms,
        config=config,
        world=world,
        planner_binding=binding,
        responses=responses,
        default_response=default,
        expect=list(expect),
        persona=persona,
        rubric=list(rubric),
    )
    for i, item in enumerate(expect):
        validate_expectation(scenario, item, f"expect[{i}]")
    return scenario


def load_scenarios(paths: list[str | Path]) -> list[Scenario]:
    """Load several scenarios; refuse duplicate ids."""
    out: list[Scenario] = []
    seen: dict[str, Path] = {}
    for p in paths:
        s = load_scenario(p)
        if s.id in seen:
            raise ScenarioError(
                f"{p}: duplicate scenario id {s.id!r} (also {seen[s.id]})"
            )
        seen[s.id] = s.path
        out.append(s)
    return out
