"""Model ports: who answers the twin's ``model_request`` lines.

The request descriptor is a phase-1 proxy for the request body (ADR-030 §5):
until ``gemini_request.c`` is extracted, the twin hashes a canonical JSON
descriptor (kind, frame scene, pose, gate verdict, speak_allowed) instead of the
body the firmware would send. Cassettes recorded against it are re-recorded
when the real body lands.
"""

from __future__ import annotations

import hashlib
import json
import os
import urllib.error
import urllib.request
from collections.abc import Callable
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Protocol

from .scenario import InlineResponse, Scenario


class ModelPortError(RuntimeError):
    """The port cannot answer a request (missing cassette, no reply left, no key)."""


@dataclass(frozen=True)
class ModelRequest:
    id: int
    at_ms: int
    kind: str
    descriptor: dict[str, Any]
    key: str


@dataclass(frozen=True)
class ModelResponse:
    status: int
    latency_ms: int
    body: str

    def to_protocol(self, request_id: int) -> dict[str, Any]:
        return {
            "t": "model_response",
            "id": request_id,
            "status": self.status,
            "latency_ms": self.latency_ms,
            "body": self.body,
        }


def canonical_descriptor(descriptor: dict[str, Any]) -> str:
    """The canonical form the key hashes: sorted keys, no whitespace, UTF-8."""
    return json.dumps(
        descriptor, sort_keys=True, separators=(",", ":"), ensure_ascii=False
    )


def descriptor_key(descriptor: dict[str, Any]) -> str:
    return hashlib.sha256(canonical_descriptor(descriptor).encode("utf-8")).hexdigest()


class ModelPort(Protocol):
    name: str

    def respond(self, request: ModelRequest) -> ModelResponse: ...


def _read_cassette(path: Path) -> ModelResponse:
    data = json.loads(path.read_text(encoding="utf-8"))
    for key in ("status", "latency_ms", "body"):
        if key not in data:
            raise ModelPortError(f"cassette {path} is missing '{key}'")
    return ModelResponse(
        int(data["status"]), int(data["latency_ms"]), str(data["body"])
    )


def write_cassette(
    directory: Path, request: ModelRequest, response: ModelResponse
) -> Path:
    """Write ``<dir>/<key>.json``; the descriptor is stored beside the reply for review."""
    directory.mkdir(parents=True, exist_ok=True)
    path = directory / f"{request.key}.json"
    payload = {
        "descriptor": request.descriptor,
        "status": response.status,
        "latency_ms": response.latency_ms,
        "body": response.body,
    }
    path.write_text(
        json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    return path


class ReplayPort:
    """Answers from ``<dir>/<key>.json``. A missing cassette is a hard error."""

    name = "replay"

    def __init__(self, directory: Path):
        self.directory = Path(directory)

    def respond(self, request: ModelRequest) -> ModelResponse:
        path = self.directory / f"{request.key}.json"
        if not path.is_file():
            raise ModelPortError(
                f"no cassette for request {request.id} at {request.at_ms} ms: expected {path}\n"
                f"  key:        {request.key}\n"
                f"  descriptor: {canonical_descriptor(request.descriptor)}\n"
                "  A changed prompt or gate changes the key; re-record with RecordPort."
            )
        return _read_cassette(path)


class InlinePort:
    """Scenario-scripted replies, served in order; then ``default`` for every request."""

    name = "inline"

    def __init__(
        self, responses: list[InlineResponse], default: InlineResponse | None = None
    ):
        self._responses = list(responses)
        self._default = default
        self._next = 0

    def respond(self, request: ModelRequest) -> ModelResponse:
        if self._next < len(self._responses):
            r = self._responses[self._next]
            self._next += 1
        elif self._default is not None:
            r = self._default
        else:
            raise ModelPortError(
                f"inline replies exhausted: request {request.id} at {request.at_ms} ms is "
                f"number {self._next + 1}, the scenario scripts {len(self._responses)} and no "
                f"default (descriptor {canonical_descriptor(request.descriptor)})"
            )
        return ModelResponse(r.status, r.latency_ms, r.body)


Transport = Callable[[str, bytes, dict[str, str]], tuple[int, str]]
RequestBuilder = Callable[[ModelRequest], dict[str, Any]]

GEMINI_ENDPOINT = (
    "https://generativelanguage.googleapis.com/v1beta/models/{model}:generateContent"
)


def _urllib_transport(
    url: str, data: bytes, headers: dict[str, str]
) -> tuple[int, str]:
    req = urllib.request.Request(url, data=data, headers=headers, method="POST")
    try:
        with urllib.request.urlopen(req, timeout=60) as resp:
            return resp.status, resp.read().decode("utf-8")
    except urllib.error.HTTPError as exc:
        return exc.code, exc.read().decode("utf-8", errors="replace")
    except (urllib.error.URLError, TimeoutError):
        return -1, ""


def _no_builder(request: ModelRequest) -> dict[str, Any]:
    raise ModelPortError(
        "RecordPort needs a request builder: in phase 1 the twin sends only a descriptor, "
        "not the request body (gemini_request.c is not extracted yet, ADR-030 §2)"
    )


class RecordPort:
    """Live Gemini behind GEMINI_API_KEY; writes every reply as a cassette.

    Skeleton for phase 3. The model, the request builder and the transport are
    injected, so tests never touch the network. ``max_requests`` is the per-run
    spend guard (PRD-017 H-6).
    """

    name = "record"

    def __init__(
        self,
        directory: Path,
        *,
        model: str = "gemini-robotics-er-2-preview",  # GEMINI_MODEL in gemini_backend.c
        request_builder: RequestBuilder = _no_builder,
        transport: Transport = _urllib_transport,
        api_key: str | None = None,
        max_requests: int = 50,
        clock: Callable[[], float] | None = None,
    ):
        key = api_key if api_key is not None else os.environ.get("GEMINI_API_KEY")
        if not key:
            raise ModelPortError("RecordPort needs GEMINI_API_KEY (live Gemini calls)")
        self.directory = Path(directory)
        self._api_key = key
        self._model = model
        self._build = request_builder
        self._transport = transport
        self._max = max_requests
        self._count = 0
        if clock is None:
            import time

            clock = time.monotonic
        self._clock = clock

    def respond(self, request: ModelRequest) -> ModelResponse:
        if self._count >= self._max:
            raise ModelPortError(
                f"RecordPort spend guard: {self._max} requests already made"
            )
        body = json.dumps(self._build(request)).encode("utf-8")
        url = GEMINI_ENDPOINT.format(model=self._model)
        headers = {"Content-Type": "application/json", "x-goog-api-key": self._api_key}
        self._count += 1
        start = self._clock()
        status, text = self._transport(url, body, headers)
        latency = round((self._clock() - start) * 1000)
        response = ModelResponse(status, latency, text if status == 200 else "")
        write_cassette(self.directory, request, response)
        return response


def port_for(scenario: Scenario) -> ModelPort:
    """The port a scenario's ``models.planner`` binding names."""
    binding = scenario.planner_binding
    if binding == "inline":
        return InlinePort(scenario.responses, scenario.default_response)
    directory = Path(binding.removeprefix("replay:"))
    if not directory.is_absolute():
        directory = scenario.twin_dir / directory
    return ReplayPort(directory)
