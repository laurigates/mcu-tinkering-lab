"""Pure event handling: validate a robot event, route it, turn it into a Sentry event.

No I/O here. The robot's wire format is documented in ADR-033 and the README;
`to_sentry_event` is the only place that knows both it and the Sentry schema.
"""

from __future__ import annotations

import json
import re
from collections.abc import Callable
from dataclasses import dataclass
from typing import Any

PROJECT_BY_SOURCE = {"firmware": "robocar", "planner": "robocar-ai"}
LEVELS = {"fatal", "error", "warning", "info", "debug"}
_EVENT_ID = re.compile(r"^[0-9a-f]{32}$")


class InvalidEvent(ValueError):
    """The payload is not a robot event the bridge can forward."""


@dataclass(frozen=True)
class Frame:
    """A resolved code location for one instruction address."""

    function: str | None
    filename: str | None
    lineno: int | None


Symbolicator = Callable[[int], Frame | None]


def parse_payload(raw: bytes) -> dict[str, Any]:
    """Decode an MQTT payload and check the fields every event must carry."""
    try:
        event = json.loads(raw)
    except (UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise InvalidEvent(f"not JSON: {exc}") from exc
    if not isinstance(event, dict):
        raise InvalidEvent("payload is not a JSON object")
    event_id = event.get("event_id")
    if not isinstance(event_id, str) or not _EVENT_ID.match(event_id):
        raise InvalidEvent(
            f"event_id must be 32 lowercase hex characters, got {event_id!r}"
        )
    if event.get("source") not in PROJECT_BY_SOURCE:
        raise InvalidEvent(f"unknown source {event.get('source')!r}")
    if event.get("level") not in LEVELS:
        raise InvalidEvent(f"unknown level {event.get('level')!r}")
    for field in ("release", "message"):
        if not isinstance(event.get(field), str) or not event[field]:
            raise InvalidEvent(f"missing {field}")
    return event


def route(event: dict[str, Any]) -> str:
    """The GlitchTip project an event belongs to, from its `source`."""
    return PROJECT_BY_SOURCE[event["source"]]


def build_sha(event: dict[str, Any]) -> str | None:
    """The firmware build SHA the event's addresses belong to, if it says."""
    sha = (event.get("tags") or {}).get("build_sha")
    return sha if isinstance(sha, str) and sha else None


def _frame(
    raw: dict[str, Any], symbolicate: Symbolicator | None
) -> tuple[dict[str, Any], bool]:
    addr_text = str(raw.get("instruction_addr", ""))
    out: dict[str, Any] = {"instruction_addr": addr_text, "in_app": True}
    try:
        addr = int(addr_text, 16)
    except ValueError:
        return out, False
    resolved = symbolicate(addr) if symbolicate else None
    if resolved is None:
        return out, False
    if resolved.function:
        out["function"] = resolved.function
    if resolved.filename:
        out["filename"] = resolved.filename
    if resolved.lineno is not None:
        out["lineno"] = resolved.lineno
    return out, True


def _exception(
    raw: dict[str, Any], symbolicate: Symbolicator | None
) -> tuple[dict[str, Any], bool]:
    """Sentry `exception` interface; frames reversed (robot sends innermost first)."""
    frames_and_hits = [_frame(f, symbolicate) for f in raw.get("frames") or []]
    frames = [f for f, _ in reversed(frames_and_hits)]
    value: dict[str, Any] = {
        "type": str(raw.get("type") or "Crash"),
        "value": str(raw.get("value") or ""),
        "mechanism": {"type": "coredump", "handled": False},
    }
    if frames:
        value["stacktrace"] = {"frames": frames}
    resolved_any = any(hit for _, hit in frames_and_hits)
    return {"values": [value]}, resolved_any


def to_sentry_event(
    event: dict[str, Any],
    received_at: float,
    symbolicate: Symbolicator | None = None,
) -> dict[str, Any]:
    """Map a validated robot event to the Sentry event schema.

    `timestamp` 0 or absent means the robot had no wall-clock time (no SNTP
    yet), so the bridge's receive time stands in. `symbolicate` resolves crash
    addresses; pass None when no ELF matches the event's build SHA.
    """
    timestamp = event.get("timestamp") or 0
    tags = {str(k): str(v) for k, v in (event.get("tags") or {}).items()}
    tags["source"] = event["source"]
    out: dict[str, Any] = {
        "event_id": event["event_id"],
        "timestamp": float(timestamp) if timestamp else received_at,
        "level": event["level"],
        "platform": "native",
        "release": event["release"],
        "message": {"formatted": event["message"]},
        "tags": tags,
        "extra": dict(event.get("extra") or {}),
    }
    if not timestamp:
        out["extra"]["robot_timestamp_missing"] = True
    if event.get("fingerprint"):
        out["fingerprint"] = [str(f) for f in event["fingerprint"]]
    crumbs = [
        {
            "type": "default",
            "category": str(c.get("category") or "log"),
            "message": str(c.get("message") or ""),
            "level": str(c.get("level") or "info"),
        }
        for c in event.get("breadcrumbs") or []
        if isinstance(c, dict)
    ]
    if crumbs:
        out["breadcrumbs"] = {"values": crumbs}
    if isinstance(event.get("exception"), dict):
        out["exception"], resolved = _exception(event["exception"], symbolicate)
        tags["symbolicated"] = "true" if resolved else "false"
    return out
