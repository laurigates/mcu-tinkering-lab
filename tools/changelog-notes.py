#!/usr/bin/env python3
"""Print one version's user-facing changelog entries as a JSON array of strings.

    changelog-notes.py <CHANGELOG.md> <version> [--max-items N] [--max-chars N]

generate-flasher-manifests.sh calls this for every project and writes the
result into its OTA manifest as "notes", a top-level field ESP Web Tools
ignores (as it does "buildSha"). robocar-unified reads it to say what an update
changes before installing it.

Only the Features, Bug Fixes and Performance Improvements subsections of the
version's section are kept: documentation, CI and dependency entries say
nothing to someone listening to the robot. Each entry loses its commit link and
markdown, becoming "scope: summary".

The caps bound what the firmware has to hold: at most --max-items entries
(default 8), each at most --max-chars characters (default 160, ending in "…"
when cut). A missing changelog or version section prints [] and exits 0, so a
release never fails over notes; the firmware falls back to a template line.
"""

from __future__ import annotations

import argparse
import json
import re
import sys
from pathlib import Path

KEPT_SUBSECTIONS = ("features", "bug fixes", "performance improvements")


def version_section(text: str, version: str) -> list[str]:
    """Lines of the `## [version]` section, up to the next `## ` heading."""
    head = re.compile(r"^##\s+\[?" + re.escape(version) + r"(?:\]|\s|$)")
    lines = text.splitlines()
    for i, line in enumerate(lines):
        if head.match(line):
            out = []
            for following in lines[i + 1 :]:
                if following.startswith("## "):
                    break
                out.append(following)
            return out
    return []


def clean(item: str) -> str:
    """'* **scope:** summary ([abc](url))' -> 'scope: summary'."""
    s = item.strip()[1:].strip()  # drop the bullet
    s = re.sub(r"\s*\(\[[0-9a-f]{6,40}\]\([^)]*\)\)\s*$", "", s)  # trailing commit link
    s = re.sub(r"\*\*([^*]+?):\*\*\s*", r"\1: ", s, count=1)  # **scope:** prefix
    s = re.sub(r"\[([^\]]+)\]\([^)]*\)", r"\1", s)  # other links -> their text
    s = s.replace("**", "").replace("`", "")
    return re.sub(r"\s+", " ", s).strip()


def truncate(s: str, max_chars: int) -> str:
    if len(s) <= max_chars:
        return s
    return s[: max_chars - 1].rstrip() + "…"


def notes(text: str, version: str, max_items: int, max_chars: int) -> list[str]:
    out: list[str] = []
    keep = False
    for line in version_section(text, version):
        if line.startswith("### "):
            keep = line[4:].strip().lower() in KEPT_SUBSECTIONS
            continue
        if keep and line.lstrip().startswith("* "):
            entry = clean(line)
            if entry:
                out.append(truncate(entry, max_chars))
        if len(out) >= max_items:
            break
    return out


def main(argv: list[str] | None = None) -> int:
    p = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    p.add_argument("changelog", type=Path)
    p.add_argument("version")
    p.add_argument("--max-items", type=int, default=8)
    p.add_argument("--max-chars", type=int, default=160)
    args = p.parse_args(argv)
    try:
        text = args.changelog.read_text(encoding="utf-8")
    except FileNotFoundError:
        text = ""
    result = notes(text, args.version, args.max_items, args.max_chars)
    json.dump(result, sys.stdout, ensure_ascii=False)
    sys.stdout.write("\n")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
