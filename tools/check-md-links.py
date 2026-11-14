#!/usr/bin/env python3
"""Fail on a relative Markdown link whose target is not in git (issue #1147).

Every relative link target in git-tracked Markdown must be a tracked file, or
a directory holding tracked files. Checked: inline links and images
`[t](target "title")`, `[t](<target with spaces>)`, and reference definitions
`[id]: target`. Skipped: fenced code blocks, inline code spans, URLs with a
scheme, `//host`, `#anchor`-only targets, and targets holding a placeholder
(`<`, `{`, `...`, `$(`). A target starting with `/` resolves from the repo
root, any other from the linking file's directory. Anchors are not checked,
only the path.

Targets resolve against `git ls-files`, so an untracked or ignored target is
broken here as it would be in a CI checkout. A link breaks when its target
moves, which changes no Markdown file, so the pre-commit hook runs on every
commit rather than on `.md` changes.

Run: python3 tools/check-md-links.py   (the check-md-links pre-commit hook)
Exit: 0 when every link resolves; 1 when one does not; 2 when nothing was
checked, since an empty scan is not a pass.
"""

from __future__ import annotations

import argparse
import fnmatch
import re
import subprocess
import sys
from collections.abc import Callable
from pathlib import Path, PurePosixPath
from urllib.parse import unquote

REPO_ROOT = Path(__file__).resolve().parent.parent

# {glob on the linking file's repo-relative path: why its links are not checked}
EXCLUDES = {
    "tools/kicad/testdata/*": (
        "frozen, hash-pinned copy of the balancebot inputs the kicad tests build"
        " from (tools/kicad/testfixture.py); its links name files the copy leaves out"
    ),
}
# Not an exclude: the two example links in
# packages/networking/it-troubleshooter/.claude/rules/document-management.md
# sit inside inline code spans, which the scan skips like any code.

FENCE = re.compile(r"^\s{0,3}(`{3,}|~{3,})(.*)$")
INLINE_CODE = re.compile(r"(`+)(?:(?!\1).)+?\1")
# [text](target) / ![alt](target); text may hold one level of [] (badges).
INLINE = re.compile(
    r"!?\[((?:[^\[\]]|\[[^\[\]]*\])*)\]\(\s*(<[^>]*>|[^)\s]+)"
    r"(?:\s+(?:\"[^\"]*\"|'[^']*'))?\s*\)"
)
# A definition line holds the target and at most a title; `[4]: SLA M3 hole
# 4.1-4.2), ...` in running prose is not one.
REFDEF = re.compile(
    r"^\s{0,3}\[[^\]]+\]:\s*(<[^>]*>|\S+)\s*(?:\"[^\"]*\"|'[^']*'|\([^)]*\))?\s*$"
)
SCHEME = re.compile(r"^[A-Za-z][A-Za-z0-9+.\-]*:")
PLACEHOLDER = re.compile(r"[<>{}]|\.\.\.|\$\(")


def targets(text: str):
    """(line number, raw target) for every link outside code."""
    fence = None
    for n, line in enumerate(text.splitlines(), 1):
        m = FENCE.match(line)
        if fence is None and m:
            fence = m.group(1)
            continue
        if fence is not None:
            closes = m and m.group(1)[0] == fence[0] and len(m.group(1)) >= len(fence)
            if closes and not m.group(2).strip():
                fence = None
            continue
        line = INLINE_CODE.sub(lambda mm: " " * len(mm.group(0)), line)
        for mm in INLINE.finditer(line):
            # A badge: the image inside the link text is a link too.
            for inner in INLINE.finditer(mm.group(1)):
                yield n, inner.group(2)
            yield n, mm.group(2)
        mm = REFDEF.match(line)
        if mm:
            yield n, mm.group(1)


def resolve(doc: str, raw: str) -> str | None:
    """Repo-relative POSIX path the target names, or None when it names no local
    path. A target that climbs above the root resolves to `<outside repo>/...`."""
    t = raw.strip()
    if t.startswith("<") and t.endswith(">"):
        t = t[1:-1]
    if not t or t.startswith("#") or t.startswith("//") or SCHEME.match(t):
        return None
    if PLACEHOLDER.search(t):
        return None
    t = unquote(t.split("#", 1)[0].split("?", 1)[0])
    if not t:
        return None
    base = PurePosixPath("") if t.startswith("/") else PurePosixPath(doc).parent
    parts: list[str] = []
    for p in (base / t.lstrip("/")).parts:
        if p == ".":
            continue
        if p == "..":
            if not parts:
                return "<outside repo>/" + t
            parts.pop()
        else:
            parts.append(p)
    return "/".join(parts) or "."


def git_ls(root: Path) -> list[str]:
    out = subprocess.run(
        ["git", "ls-files", "-z"],
        cwd=root,
        check=True,
        capture_output=True,
        text=True,
    ).stdout
    return [p for p in out.split("\0") if p]


def excluded(doc: str, excludes=EXCLUDES) -> bool:
    return any(fnmatch.fnmatch(doc, g) for g in excludes)


def scan(
    tracked: set[str], read: Callable[[str], str], excludes=EXCLUDES
) -> tuple[int, int, list[str]]:
    """(docs scanned, links checked, one `doc:line: target -> path` per broken
    link) for the Markdown among `tracked`, each read through `read`."""
    dirs = {"."}
    for f in tracked:
        p = PurePosixPath(f).parent
        while str(p) != ".":
            dirs.add(str(p))
            p = p.parent
    docs = sorted(
        d
        for d in tracked
        if d.endswith((".md", ".markdown")) and not excluded(d, excludes)
    )
    checked, broken = 0, []
    for doc in docs:
        for line, raw in targets(read(doc)):
            path = resolve(doc, raw)
            if path is None:
                continue
            checked += 1
            if path not in tracked and path.rstrip("/") not in dirs:
                broken.append(f"{doc}:{line}: {raw} -> {path}")
    return len(docs), checked, broken


def check(root: Path, excludes=EXCLUDES) -> tuple[int, int, list[str]]:
    """`scan` over the files git tracks under `root`."""
    return scan(
        set(git_ls(root)),
        lambda doc: (root / doc).read_text(encoding="utf-8", errors="replace"),
        excludes,
    )


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--root", type=Path, default=REPO_ROOT)
    args = ap.parse_args(argv)
    docs, checked, broken = check(args.root.resolve())
    for b in broken:
        print(b, file=sys.stderr)
    summary = f"docs={docs} links_checked={checked} broken={len(broken)}"
    if not checked:
        print(
            f"{summary}: no relative link found, so nothing was checked",
            file=sys.stderr,
        )
        return 2
    if broken:
        print(
            f"{summary}. Fix the path, or drop the link if its target is gone.",
            file=sys.stderr,
        )
        return 1
    print(summary)
    return 0


if __name__ == "__main__":
    sys.exit(main())
