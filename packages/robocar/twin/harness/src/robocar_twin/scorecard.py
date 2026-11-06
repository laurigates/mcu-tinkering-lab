"""Run a set of scenarios and summarise them as a scorecard (PRD-017 H-2, H-3)."""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any

from .assertions import evaluate
from .runner import DEFAULT_TIMEOUT_S, TwinError, run_scenario
from .scenario import Scenario

SCORECARD_VERSION = 1


def score_scenario(
    scenario: Scenario,
    twin_cmd: list[str] | None = None,
    *,
    timeout_s: float = DEFAULT_TIMEOUT_S,
    trace_dir: Path | None = None,
) -> dict[str, Any]:
    """One scenario's scorecard entry. A twin error is a failed entry, not an exception."""
    entry: dict[str, Any] = {
        "id": scenario.id,
        "stories": scenario.stories,
        "model": scenario.planner_binding,
    }
    try:
        result = run_scenario(scenario, twin_cmd, timeout_s=timeout_s)
    except TwinError as exc:
        entry.update(passed=False, error=str(exc), failed=[], requests=0, tokens=0)
        return entry
    outcomes = evaluate(scenario, result.trace)
    failed = [o.to_json() for o in outcomes if not o.passed]
    latencies = sorted(r["latency_ms"] for r in result.requests)
    entry.update(
        passed=not failed,
        failed=failed,
        assertions=len(outcomes),
        requests=result.request_count,
        tokens=result.tokens,
        latency_ms_p50=latencies[len(latencies) // 2] if latencies else None,
        wall_s=round(result.wall_s, 3),
    )
    if trace_dir is not None:
        trace_dir.mkdir(parents=True, exist_ok=True)
        path = trace_dir / f"{scenario.id}.jsonl"
        path.write_text(
            "".join(json.dumps(r) + "\n" for r in result.trace), encoding="utf-8"
        )
        entry["trace"] = str(path)
    return entry


def build_scorecard(
    scenarios: list[Scenario],
    twin_cmd: list[str] | None = None,
    *,
    timeout_s: float = DEFAULT_TIMEOUT_S,
    trace_dir: Path | None = None,
) -> dict[str, Any]:
    entries = [
        score_scenario(s, twin_cmd, timeout_s=timeout_s, trace_dir=trace_dir)
        for s in scenarios
    ]
    passed = sum(1 for e in entries if e["passed"])
    return {
        "version": SCORECARD_VERSION,
        "summary": {
            "scenarios": len(entries),
            "passed": passed,
            "failed": len(entries) - passed,
            "requests": sum(e["requests"] for e in entries),
            "tokens": sum(e["tokens"] for e in entries),
        },
        "scenarios": entries,
    }
