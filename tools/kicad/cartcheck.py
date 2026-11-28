"""A Claude Code PreToolUse hook: before a PCB goes into a fab cart, say which
export the cart line must match, and whether that export is stale.

On 2026-10-23 JLCPCB built an upload left in the cart from ten days earlier
(`kicad.provenance`). `carrier-fab` now names every output and the silkscreen
after the revision and commit (`r<N> <sha7> <pcb8>`), but only a check of the
cart line against the hand-off row catches a wrong line, and that check is a
step an agent can skip. This hook puts it in front of the agent at the moment
it acts on a JLCPCB page or the JLCONE app (`.claude/settings.json`):

- it reads the tool call from stdin and stays silent unless it touches JLCPCB
  or JLCONE and an upload, the cart, a quote or checkout;
- it lists each carrier's export in `output/`: the zip name the cart line
  must carry, the silkscreen text the Gerber viewer must show, and what makes
  it unfit to order: a `-dirty` export, a board or revision that changed since
  the export, or a revision the order ledger says needs a bump
  (`kicad.revision`);
- it answers with `additionalContext`, never a block, once per session per set
  of exports, and again on every save-to-cart or checkout step.

    echo '{"tool_name": ..., "tool_input": ...}' | PYTHONPATH=tools python3 -m kicad.cartcheck

Stdlib only; a failure prints nothing and exits 0, so the hook never stops a
tool call.
"""

from __future__ import annotations

import contextlib
import hashlib
import json
import os
import re
import sys
import tempfile
from pathlib import Path
from typing import Any

from . import paths, revision
from .errors import KicadError
from .paths import REPO_ROOT

_FAB = re.compile(r"jlcpcb|jlcone|jlc\.com", re.IGNORECASE)
_ACTION = re.compile(
    r"cart|upload|gerber|quote|checkout|order|cdp\.mjs|cart\.js", re.IGNORECASE
)
# Steps that commit a line to the cart or pay for it: nudge every time.
_COMMIT = re.compile(
    r"save[ _-]?to[ _-]?cart|add[ _-]?to[ _-]?cart|checkout|place[ _-]?order|\bpay\b",
    re.IGNORECASE,
)


def relevant(tool_input: Any) -> tuple[bool, bool]:
    """(touches a fab cart, commits to it) for one tool call's input."""
    text = json.dumps(tool_input, ensure_ascii=False)
    if not (_FAB.search(text) and _ACTION.search(text)):
        return False, False
    return True, bool(_COMMIT.search(text))


def export_lines(
    project: Path, repo_root: Path = REPO_ROOT
) -> tuple[list[str], list[str]]:
    """(what the cart line must match, what is wrong) for one carrier's export."""
    cfg, pcb, _ledger = revision.files(project)
    rel = _rel(project, repo_root)
    rows = revision.handoff_rows(project)
    if not rows:
        return [], []
    lines, problems = [], []
    sha = hashlib.sha256(pcb.read_bytes()).hexdigest() if pcb.is_file() else ""
    rev = revision.number(cfg)
    ledger_errors = revision.check(project)
    for row in rows:
        stem = Path(row.get("zip", "")).stem
        lines.append(
            f"- {cfg.name}: cart line `{stem}_Y<n>`, silkscreen `{row.get('silk_text')}`, "
            f"zip sha256 {row.get('zip_sha256', '')[:12]}..."
        )
        if row.get("dirty") != "no":
            problems.append(f"{stem}: a -dirty export; never order it")
        if row.get("pcb_sha256") != sha:
            problems.append(
                f"{stem}: exported from board {row.get('pcb_sha256', '')[:8]}, the "
                f"committed board is now {sha[:8]}; re-run `just <module>::carrier-fab` "
                f"for {rel}"
            )
        if row.get("revision") != str(rev):
            problems.append(
                f"{stem}: revision {row.get('revision')}, board.toml says {rev}"
            )
    problems += ledger_errors
    return lines, problems


def summary(projects: list[Path], repo_root: Path = REPO_ROOT) -> str:
    """The nudge text, or "" when no carrier has an export."""
    lines, problems = [], []
    for project in projects:
        try:
            if not paths.load_board_config(project).has_board:
                continue
            got, bad = export_lines(project, repo_root)
        except KicadError as e:
            got, bad = [], [str(e)]
        lines += got
        problems += bad
    if not lines and not problems:
        return ""
    out = [
        "PCB fab cart check (kicad.cartcheck, jlcpcb-order skill section 3 step 8): "
        "before saving to the cart or checking out, compare EVERY PCB line in the "
        "cart, not only the one uploaded now. Its file name must be the zip stem "
        "below plus JLCPCB's _Y<n>, and its Gerber viewer silkscreen must read the "
        "text below character for character (r<N> revision, commit, board hash). "
        "Delete any other PCB line, or tell Lauri before paying.",
        "Exports in output/:",
        *(lines or ["- none"]),
    ]
    if problems:
        out += ["NOT ORDERABLE as it stands:", *(f"- {p}" for p in problems)]
    out.append(
        "After payment: `PYTHONPATH=tools python3 -m kicad.revision record "
        "<project> --vendor JLCPCB --order <order number>` and commit orders.toml."
    )
    return "\n".join(out)


def should_nudge(state_dir: Path, session: str, text: str, commits: bool) -> bool:
    """Once per session per summary; every time for a commit step."""
    if commits:
        return True
    key = re.sub(r"[^A-Za-z0-9_-]", "_", session or "nosession")[:80]
    marker = state_dir / f"claude-fab-cart-{key}"
    digest = hashlib.sha256(text.encode()).hexdigest()
    try:
        if marker.read_text(encoding="utf-8") == digest:
            return False
    except OSError:
        pass
    with contextlib.suppress(OSError):
        marker.write_text(digest, encoding="utf-8")
    return True


def hook(
    event: dict[str, Any],
    projects: list[Path] | None = None,
    repo_root: Path = REPO_ROOT,
    state_dir: Path | None = None,
) -> dict[str, Any] | None:
    """The hook's JSON answer for one PreToolUse event, or None for silence."""
    touches, commits = relevant(event.get("tool_input", {}))
    if not touches:
        return None
    chosen = paths.tracked_projects(repo_root) if projects is None else projects
    text = summary(chosen, repo_root)
    if not text:
        return None
    where = state_dir or Path(os.environ.get("TMPDIR") or tempfile.gettempdir())
    if not should_nudge(where, str(event.get("session_id", "")), text, commits):
        return None
    return {
        "hookSpecificOutput": {
            "hookEventName": "PreToolUse",
            "additionalContext": text,
        }
    }


def _rel(path: Path, repo_root: Path) -> str:
    try:
        return path.resolve().relative_to(repo_root.resolve()).as_posix()
    except ValueError:
        return str(path)


def main(argv: list[str] | None = None, repo_root: Path = REPO_ROOT) -> int:
    del argv  # the event comes on stdin
    try:
        answer = hook(json.load(sys.stdin), repo_root=repo_root)
    except Exception:
        return 0
    if answer:
        print(json.dumps(answer))
    return 0


if __name__ == "__main__":
    sys.exit(main())
