"""Command line: ``python -m robocar_twin run --scenarios ... --out scorecard.json``."""

from __future__ import annotations

import argparse
import json
import shlex
import sys
from pathlib import Path

from .runner import DEFAULT_TIMEOUT_S
from .scenario import ScenarioError, load_scenarios
from .scorecard import build_scorecard


def _parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(prog="robocar_twin", description=__doc__)
    sub = p.add_subparsers(dest="command", required=True)
    run = sub.add_parser(
        "run", help="run scenarios through the twin and write a scorecard"
    )
    run.add_argument(
        "--scenarios", nargs="+", required=True, type=Path, help="scenario files"
    )
    run.add_argument("--out", type=Path, help="scorecard JSON path (default: stdout)")
    run.add_argument(
        "--twin",
        help="twin command (default: $ROBOCAR_TWIN_BIN or <twin dir>/build/robocar-twin)",
    )
    run.add_argument(
        "--timeout", type=float, default=DEFAULT_TIMEOUT_S, help="seconds per run"
    )
    run.add_argument(
        "--traces", type=Path, help="directory to write each trace as JSONL"
    )
    check = sub.add_parser("check", help="validate scenario files without running them")
    check.add_argument("--scenarios", nargs="+", required=True, type=Path)
    return p


def main(argv: list[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    try:
        scenarios = load_scenarios(args.scenarios)
    except ScenarioError as exc:
        print(f"scenario error: {exc}", file=sys.stderr)
        return 2
    if args.command == "check":
        print(f"{len(scenarios)} scenario(s) valid")
        return 0
    twin_cmd = shlex.split(args.twin) if args.twin else None
    card = build_scorecard(
        scenarios, twin_cmd, timeout_s=args.timeout, trace_dir=args.traces
    )
    text = json.dumps(card, indent=2) + "\n"
    if args.out:
        args.out.parent.mkdir(parents=True, exist_ok=True)
        args.out.write_text(text, encoding="utf-8")
    else:
        sys.stdout.write(text)
    for e in card["scenarios"]:
        status = "PASS" if e["passed"] else "FAIL"
        print(
            f"{status} {e['id']} ({e['requests']} req, {e['tokens']} tok)",
            file=sys.stderr,
        )
        if e.get("error"):
            print("  " + e["error"].replace("\n", "\n  "), file=sys.stderr)
        for f in e["failed"]:
            print("  " + f["message"].replace("\n", "\n  "), file=sys.stderr)
    s = card["summary"]
    print(f"{s['passed']}/{s['scenarios']} scenarios passed", file=sys.stderr)
    return 0 if s["failed"] == 0 else 1
