#!/usr/bin/env python3
"""Pick the uv test suites a change touches, for test.yml's Python Suites job.

`.github/python-suites.json` lists each suite: a `name`, the `run` command
(from the repo root), the `paths` globs that select it, the uv project `dir`
whose lock the job asserts with `uv sync --locked` (left out for a suite with
no uv project, such as the make-only component host suites), and optional
`apt` packages. This prints the job's matrix for a list of changed files:

    python3 tools/python-suites.py --changed-json '["tools/cadstd/fits.py"]'
    python3 tools/python-suites.py --all
    python3 tools/python-suites.py --changed-json - < changed.json

It writes `matrix=<json>` and `any=true|false` to $GITHUB_OUTPUT when that is
set, else to stdout. Globs: `**` crosses path segments, `*` and `?` stay
inside one. A change to the CI definition itself (ALWAYS) selects every
suite. Stdlib only; tests in tools/test_python_suites.py.
"""

from __future__ import annotations

import argparse
import json
import os
import re
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent
MANIFEST = REPO_ROOT / ".github" / "python-suites.json"

# A change to any of these can change how every suite runs.
ALWAYS = (
    ".github/python-suites.json",
    ".github/actions/setup-tools/**",
    ".github/workflows/test.yml",
    "tools/python-suites.py",
)
REQUIRED = {"name": str, "run": str, "paths": list}
OPTIONAL = {"dir": str, "apt": str}


def glob_regex(pattern: str) -> re.Pattern[str]:
    out, i = [], 0
    while i < len(pattern):
        if pattern.startswith("**/", i):
            out.append("(?:.*/)?")
            i += 3
        elif pattern.startswith("**", i):
            out.append(".*")
            i += 2
        elif pattern[i] == "*":
            out.append("[^/]*")
            i += 1
        elif pattern[i] == "?":
            out.append("[^/]")
            i += 1
        else:
            out.append(re.escape(pattern[i]))
            i += 1
    return re.compile("".join(out), re.S)


def load_manifest(path: Path = MANIFEST) -> list[dict]:
    suites = json.loads(Path(path).read_text())
    if not isinstance(suites, list):
        raise ValueError(f"{path}: want a list of suites")
    seen = set()
    for n, s in enumerate(suites):
        where = f"{path}: suite {n}"
        if not isinstance(s, dict):
            raise ValueError(f"{where}: want an object")
        unknown = set(s) - set(REQUIRED) - set(OPTIONAL)
        if unknown:
            raise ValueError(f"{where}: unknown field(s) {sorted(unknown)}")
        for key, kind in {**REQUIRED, **OPTIONAL}.items():
            if key in REQUIRED and key not in s:
                raise ValueError(f"{where}: missing {key!r}")
            if key in s and not isinstance(s[key], kind):
                raise ValueError(f"{where}: {key!r} must be a {kind.__name__}")
        if not s["paths"] or not all(isinstance(p, str) and p for p in s["paths"]):
            raise ValueError(f"{where}: 'paths' must be a non-empty list of globs")
        if s["name"] in seen:
            raise ValueError(f"{where}: duplicate name {s['name']!r}")
        seen.add(s["name"])
    return suites


def _matches(globs, files) -> bool:
    regexes = [glob_regex(g) for g in globs]
    return any(r.fullmatch(f) for r in regexes for f in files)


def select(suites: list[dict], changed: list[str] | None) -> list[dict]:
    """The suites `changed` touches; every suite when `changed` is None."""
    if changed is None or _matches(ALWAYS, changed):
        return list(suites)
    return [s for s in suites if _matches(s["paths"], changed)]


def matrix(suites: list[dict]) -> dict:
    return {
        "include": [
            {
                "name": s["name"],
                "dir": s.get("dir", ""),
                "run": s["run"],
                "apt": s.get("apt", ""),
            }
            for s in suites
        ]
    }


def main(argv: list[str] | None = None, github_output: Path | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    group = ap.add_mutually_exclusive_group(required=True)
    group.add_argument("--all", action="store_true", help="select every suite")
    group.add_argument(
        "--changed-json", help="JSON array of changed paths; - reads stdin"
    )
    ap.add_argument("--manifest", type=Path, default=MANIFEST)
    args = ap.parse_args(argv)

    raw = sys.stdin.read() if args.changed_json == "-" else args.changed_json
    changed = None if args.all else json.loads(raw)
    if changed is not None and not (
        isinstance(changed, list) and all(isinstance(f, str) for f in changed)
    ):
        ap.error("--changed-json must be a JSON array of strings")
    picked = select(load_manifest(args.manifest), changed)
    lines = [
        f"matrix={json.dumps(matrix(picked), separators=(',', ':'))}",
        f"any={'true' if picked else 'false'}",
    ]
    for s in picked:
        print(f"selected: {s['name']}", file=sys.stderr)
    target = github_output or (
        Path(os.environ["GITHUB_OUTPUT"]) if os.environ.get("GITHUB_OUTPUT") else None
    )
    if target:
        with open(target, "a") as fh:
            fh.write("\n".join(lines) + "\n")
    else:
        print("\n".join(lines))
    return 0


if __name__ == "__main__":
    sys.exit(main())
