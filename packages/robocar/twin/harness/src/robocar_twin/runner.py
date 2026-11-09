"""Drive one twin process through one scenario over the JSONL protocol.

stdin (harness -> twin): ``config``, every world event, ``{"t":"run"}``, then one
``model_response`` per ``model_request`` the twin emits. stdout (twin ->
harness): trace records, ``model_request`` lines (after which the twin blocks
on stdin), and ``done``. stderr is drained and kept for error messages.
"""

from __future__ import annotations

import json
import os
import queue
import subprocess
import threading
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

from .ports import ModelPort, ModelPortError, ModelRequest, descriptor_key, port_for
from .scenario import Scenario

DEFAULT_TIMEOUT_S = 60.0
TWIN_BIN_ENV = "ROBOCAR_TWIN_BIN"
_EOF = object()


class TwinError(RuntimeError):
    """The twin run did not complete as the protocol requires."""


class TwinTimeout(TwinError):
    pass


def default_twin_dir() -> Path:
    """packages/robocar/twin, from this file's location in harness/src/robocar_twin."""
    return Path(__file__).resolve().parents[3]


def resolve_twin_bin(twin_dir: Path | None = None) -> Path:
    """``$ROBOCAR_TWIN_BIN``, else ``<twin dir>/build/robocar-twin``."""
    env = os.environ.get(TWIN_BIN_ENV)
    if env:
        return Path(env)
    return (twin_dir or default_twin_dir()) / "build" / "robocar-twin"


@dataclass
class RunResult:
    scenario_id: str
    trace: list[dict[str, Any]]
    stderr: str = ""
    wall_s: float = 0.0
    requests: list[dict[str, Any]] = field(default_factory=list)

    @property
    def request_count(self) -> int:
        return len(self.requests)

    @property
    def tokens(self) -> int:
        return sum(r.get("tokens", 0) for r in self.requests)


def usage_tokens(status: int, body: str) -> int:
    """totalTokenCount from a reply's usageMetadata, as plan_budget charges it; 0 if none."""
    if status != 200 or not body:
        return 0
    try:
        usage = json.loads(body).get("usageMetadata") or {}
    except (json.JSONDecodeError, AttributeError):
        return 0
    total = usage.get("totalTokenCount")
    return total if isinstance(total, int) and not isinstance(total, bool) else 0


def _tail(trace: list[dict[str, Any]], n: int = 5) -> str:
    return "\n".join("  " + json.dumps(r) for r in trace[-n:]) or "  (no records)"


def run_scenario(
    scenario: Scenario,
    twin_cmd: list[str] | None = None,
    *,
    port: ModelPort | None = None,
    timeout_s: float = DEFAULT_TIMEOUT_S,
    strict_keys: bool = True,
) -> RunResult:
    """Run ``scenario`` through the twin and return the collected trace.

    Raises TwinError on a protocol violation, a model-port failure, a non-zero
    exit or a missing ``done``; TwinTimeout when the wall-clock budget runs out.
    ``strict_keys`` checks every request's key against sha256 of the canonical
    descriptor, so a twin and harness that disagree on canonical form fail here
    rather than as a cassette that is never found.
    """
    cmd = twin_cmd or [str(resolve_twin_bin(scenario.twin_dir))]
    port = port or port_for(scenario)
    start = time.monotonic()
    deadline = start + timeout_s
    try:
        proc = subprocess.Popen(
            cmd,
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            encoding="utf-8",
            bufsize=1,
        )
    except OSError as exc:
        raise TwinError(f"{scenario.id}: cannot start twin {cmd}: {exc}") from exc

    lines: queue.Queue[object] = queue.Queue()
    stderr_chunks: list[str] = []

    def pump_stdout() -> None:
        assert proc.stdout is not None
        for line in proc.stdout:
            lines.put(line)
        lines.put(_EOF)

    def pump_stderr() -> None:
        assert proc.stderr is not None
        for line in proc.stderr:
            stderr_chunks.append(line)

    threads = [
        threading.Thread(target=pump_stdout, daemon=True),
        threading.Thread(target=pump_stderr, daemon=True),
    ]
    for t in threads:
        t.start()

    trace: list[dict[str, Any]] = []
    requests: list[dict[str, Any]] = []
    done = False

    def cleanup() -> None:
        if proc.poll() is None:
            proc.kill()
        proc.wait()
        for t in threads:
            t.join(timeout=1)
        for f in (proc.stdin, proc.stdout, proc.stderr):
            if f is not None:
                try:
                    f.close()
                except OSError:
                    pass

    def fail(msg: str, exc_type: type[TwinError] = TwinError) -> TwinError:
        cleanup()
        return exc_type(
            f"{scenario.id}: {msg}\nlast trace records:\n{_tail(trace)}\n"
            f"twin stderr:\n{''.join(stderr_chunks)[-2000:]}"
        )

    def send(obj: dict[str, Any]) -> None:
        assert proc.stdin is not None
        try:
            proc.stdin.write(
                json.dumps(obj, separators=(",", ":"), ensure_ascii=False) + "\n"
            )
            proc.stdin.flush()
        except (BrokenPipeError, OSError) as exc:
            raise fail(
                f"twin closed stdin ({exc}) while sending {obj.get('t')}"
            ) from exc

    for obj in scenario.protocol_lines():
        send(obj)
    send({"t": "run"})

    while True:
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            raise fail(f"timed out after {timeout_s:g} s wall clock", TwinTimeout)
        try:
            item = lines.get(timeout=remaining)
        except queue.Empty:
            raise fail(
                f"timed out after {timeout_s:g} s wall clock", TwinTimeout
            ) from None
        if item is _EOF:
            break
        assert isinstance(item, str)
        text = item.strip()
        if not text:
            continue
        try:
            rec = json.loads(text)
        except json.JSONDecodeError as exc:
            raise fail(f"twin wrote a line that is not JSON: {text[:200]!r}") from exc
        if not isinstance(rec, dict) or "t" not in rec:
            raise fail(f"twin record has no 't' field: {text[:200]!r}")
        if rec["t"] != "done" and "at_ms" not in rec:
            raise fail(f"twin record has no 'at_ms': {text[:200]!r}")
        trace.append(rec)

        if rec["t"] == "model_request":
            try:
                req = ModelRequest(
                    id=int(rec["id"]),
                    at_ms=int(rec["at_ms"]),
                    kind=str(rec["kind"]),
                    descriptor=dict(rec["descriptor"]),
                    key=str(rec["key"]),
                )
            except (KeyError, TypeError, ValueError) as exc:
                raise fail(f"malformed model_request ({exc}): {text[:200]!r}") from exc
            if strict_keys and descriptor_key(req.descriptor) != req.key:
                raise fail(
                    f"request {req.id}: key {req.key} is not sha256 of the canonical "
                    f"descriptor (expected {descriptor_key(req.descriptor)}); the twin and "
                    "harness disagree on canonical form (sorted keys, no whitespace, UTF-8)"
                )
            try:
                resp = port.respond(req)
            except ModelPortError as exc:
                raise fail(str(exc)) from exc
            send(resp.to_protocol(req.id))
            tokens = usage_tokens(resp.status, resp.body)
            summary = {
                "t": "model_response",
                "origin": "harness",
                "id": req.id,
                "at_ms": req.at_ms + resp.latency_ms,
                "status": resp.status,
                "latency_ms": resp.latency_ms,
                "tokens": tokens,
            }
            trace.append(summary)
            requests.append({**summary, "request_at_ms": req.at_ms, "key": req.key})
        elif rec["t"] == "done":
            done = True

    if proc.stdin is not None:
        try:
            proc.stdin.close()
        except OSError:
            pass
    try:
        code = proc.wait(timeout=max(0.1, deadline - time.monotonic()))
    except subprocess.TimeoutExpired:
        raise fail("twin did not exit after closing stdout", TwinTimeout) from None
    cleanup()
    if code != 0:
        raise TwinError(
            f"{scenario.id}: twin exited with status {code}\nlast trace records:\n"
            f"{_tail(trace)}\ntwin stderr:\n{''.join(stderr_chunks)[-2000:]}"
        )
    if not done:
        raise TwinError(
            f"{scenario.id}: twin exited without a 'done' record\n{_tail(trace)}"
        )
    return RunResult(
        scenario_id=scenario.id,
        trace=trace,
        stderr="".join(stderr_chunks),
        wall_s=time.monotonic() - start,
        requests=requests,
    )
