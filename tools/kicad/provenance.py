"""Which commit a set of fab files came from, on the copper and in the names.

JLCPCB built a stale upload still in the cart (balancebot, 2026-10-23): the
board carried nothing that named its commit, the zip was called
`<name>-fab.zip` whatever it held, and the hand-off CSV recorded the commit of
the zip on disk, not of the file in the cart. `kicad.fab` therefore:

- refuses to export from a tree with uncommitted changes to anything that feeds
  the board (`FEEDS`), unless `$CARRIER_FAB_ALLOW_DIRTY=1` or `--allow-dirty`
  says otherwise, and then marks every file name `-dirty` and the silk text
  with `*` in place of its space;
- writes `<sha7> <pcb8>` (the commit, and the first 8 hex digits of the check
  stamp's `board_sha256`) on F.SilkS at board.toml's `[fab] provenance_at`, on
  a copy of the board used only for plotting: the committed `.kicad_pcb` and
  its stamp are never written. A dirty run writes `<sha7>*<pcb8>`: the same 16
  characters, and the same width, since KiCad's stroke font gives `*` the
  advance of a space, so a position that clears its neighbours clean clears
  them dirty (`-dirty` there grew the text into J3's pin labels);
- names the zip, BOM, CPL and hand-off row after that commit
  (`<name>-fab-<sha7>.zip`), so the cart line's file name says it too;
- writes the hand-off row: the commit and the zip's full sha256, which the
  `jlcpcb-order` skill's hand-off CSV copies.

Stdlib only; `kicad.fab` does the kicad-cli half.
"""

from __future__ import annotations

import csv
import hashlib
import json
import os
import subprocess
import uuid
from dataclasses import dataclass
from pathlib import Path
from typing import Any

from . import paths
from .errors import KicadError

SHA_LEN = 7
PCB_HASH_LEN = 8
DIRTY = "-dirty"
SILK_SEP = " "
SILK_DIRTY = "*"  # replaces SILK_SEP: same advance in KiCad's stroke font
SILK_LEN = SHA_LEN + 1 + PCB_HASH_LEN
ALLOW_DIRTY_ENV = "CARRIER_FAB_ALLOW_DIRTY"
DEFAULT_SIZE_MM = 1.0
FAB_KEYS = frozenset({"provenance_at", "provenance_size"})
# Repo-relative paths every carrier's files are generated or checked from,
# besides the project's own hardware.toml and hardware/ directory.
SHARED_FEEDS = (
    "tools/kicad",
    "tools/kicad.just",
    "tools/hardware",
    "docs/reference/parts",
    "docs/reference/blocks",
    "docs/reference/boards",
    "docs/reference/jlcpcb-rotations.toml",
)
HANDOFF_FIELDS = (
    "board",
    "git_sha",
    "dirty",
    "silk_text",
    "pcb_sha256",
    "zip",
    "zip_sha256",
    "bom",
    "cpl",
)


@dataclass(frozen=True)
class GitState:
    sha: str  # the full HEAD commit
    dirty: tuple[str, ...]  # repo-relative paths in FEEDS with uncommitted changes


@dataclass(frozen=True)
class Names:
    zip: str
    bom: str
    cpl: str
    handoff: str


def feeds(project: Path, repo_root: Path) -> list[str]:
    """Repo-relative pathspecs whose uncommitted changes make a fab run dirty."""
    try:
        rel = project.resolve().relative_to(repo_root.resolve()).as_posix()
    except ValueError:
        raise KicadError(f"{project}: not inside the repo {repo_root}") from None
    return [f"{rel}/hardware.toml", f"{rel}/hardware", *SHARED_FEEDS]


def dirty_paths(porcelain: str) -> list[str]:
    """Paths in `git status --porcelain=v1 -z` output; a rename gives both."""
    out = []
    fields = porcelain.split("\0")
    i = 0
    while i < len(fields):
        entry = fields[i]
        i += 1
        if len(entry) < 4:
            continue
        status, path = entry[:2], entry[3:]
        out.append(path)
        if "R" in status or "C" in status:
            out.append(fields[i])  # the source path follows as its own field
            i += 1
    return out


def _git(repo_root: Path, *args: str) -> str:
    done = subprocess.run(["git", *args], cwd=repo_root, capture_output=True, text=True)
    if done.returncode != 0:
        raise KicadError(f"git {' '.join(args[:2])}: {done.stderr.strip()}")
    return done.stdout


def git_state(project: Path, repo_root: Path = paths.REPO_ROOT) -> GitState:
    """HEAD, and which of the project's feeds differ from it (untracked included)."""
    sha = _git(repo_root, "rev-parse", "--verify", "HEAD").strip()
    status = _git(
        repo_root,
        "status",
        "--porcelain=v1",
        "-z",
        "--untracked-files=all",
        "--",
        *feeds(project, repo_root),
    )
    return GitState(sha=sha, dirty=tuple(dirty_paths(status)))


def allow_dirty_from_env(environ: dict[str, str] | None = None) -> bool:
    return (os.environ if environ is None else environ).get(ALLOW_DIRTY_ENV) == "1"


def require_clean(state: GitState, allow_dirty: bool) -> bool:
    """Whether the run is dirty; raises when it is and that is not allowed."""
    if not state.dirty:
        return False
    if allow_dirty:
        return True
    shown = "\n  ".join(state.dirty[:20])
    more = f"\n  ... and {len(state.dirty) - 20} more" if len(state.dirty) > 20 else ""
    raise KicadError(
        "uncommitted changes to files that feed the board; the copper and the file "
        f"names would claim commit {state.sha[:SHA_LEN]} for a board it does not "
        f"hold:\n  {shown}{more}\ncommit them (or set {ALLOW_DIRTY_ENV}=1 to mark "
        f"every output '{DIRTY}')"
    )


def tag(sha: str, dirty: bool) -> str:
    """The commit as the file names carry it."""
    if len(sha) < SHA_LEN:
        raise KicadError(f"git SHA {sha!r} is shorter than {SHA_LEN} characters")
    return sha[:SHA_LEN] + (DIRTY if dirty else "")


def silk_text(sha: str, dirty: bool, board_sha256: str) -> str:
    """The provenance text: the commit, then the stamp's board hash prefix,
    joined by a space, or by `*` when the tree is dirty. Always `SILK_LEN`
    characters, so a dirty run takes no more silkscreen than a clean one."""
    if len(sha) < SHA_LEN or len(board_sha256) < PCB_HASH_LEN:
        raise KicadError(
            f"provenance text needs {SHA_LEN} hex digits of commit and "
            f"{PCB_HASH_LEN} of board hash, got {sha!r} and {board_sha256!r}"
        )
    sep = SILK_DIRTY if dirty else SILK_SEP
    return f"{sha[:SHA_LEN]}{sep}{board_sha256[:PCB_HASH_LEN]}"


def names(name: str, run_tag: str) -> Names:
    return Names(
        zip=f"{name}-fab-{run_tag}.zip",
        bom=f"{name}-bom-jlc-{run_tag}.csv",
        cpl=f"{name}-cpl-jlc-{run_tag}.csv",
        handoff=f"{name}-fab-{run_tag}.csv",
    )


def spec(cfg: paths.BoardConfig) -> tuple[list[float], float]:
    """board.toml's [fab] provenance position (origin-relative mm) and text size."""
    fab = cfg.data.get("fab")
    if not isinstance(fab, dict) or "provenance_at" not in fab:
        raise KicadError(
            f"{cfg.path}: [fab] provenance_at = [x, y] is required: where the commit "
            "goes on F.SilkS (beside the revision text or the title)"
        )
    at = fab["provenance_at"]
    size = fab.get("provenance_size", DEFAULT_SIZE_MM)
    if not _numbers(at, 2) or not _positive(size):
        raise KicadError(
            f"{cfg.path}: [fab] provenance_at must be [x, y] and provenance_size > 0"
        )
    return [float(v) for v in at], float(size)


def validate(where: str, data: dict[str, Any], size: Any, floor: float) -> list[str]:
    """boardcheck's view of [fab] (its keys are checked with the other tables'):
    the point inside the outline, the size at or above the silkscreen floor."""
    fab = data.get("fab")
    if not isinstance(fab, dict):
        return []
    errors = []
    at = fab.get("provenance_at")
    if not _numbers(at, 2):
        errors.append(f"{where}: [fab] provenance_at must be [x, y] numbers")
    elif size is not None and not (0 <= at[0] <= size[0] and 0 <= at[1] <= size[1]):
        errors.append(f"{where}: [fab] provenance_at {at} is not inside [outline]")
    text_size = fab.get("provenance_size", DEFAULT_SIZE_MM)
    if not _positive(text_size):
        errors.append(f"{where}: [fab] provenance_size must be > 0")
    elif text_size < floor:
        errors.append(
            f"{where}: [fab] provenance_size is {text_size} mm; the floor is {floor} mm"
        )
    return errors


def gr_text(text: str, at: tuple[float, float], size: float) -> str:
    """A KiCad 10 board text on F.SilkS, at absolute board coordinates."""
    if '"' in text or "\\" in text or "\n" in text:
        raise KicadError(f"provenance text {text!r}: quote, backslash or newline")
    thickness = max(0.16, size * 0.15)
    ident = uuid.uuid5(uuid.NAMESPACE_URL, f"carrier-fab-provenance:{text}")
    return (
        f'\t(gr_text "{text}"\n'
        f"\t\t(at {_fmt(at[0])} {_fmt(at[1])} 0)\n"
        '\t\t(layer "F.SilkS")\n'
        f'\t\t(uuid "{ident}")\n'
        "\t\t(effects\n"
        "\t\t\t(font\n"
        f"\t\t\t\t(size {_fmt(size)} {_fmt(size)})\n"
        f"\t\t\t\t(thickness {_fmt(thickness)})\n"
        "\t\t\t)\n"
        "\t\t)\n"
        "\t)\n"
    )


def add_text(board_text: str, text: str, at: tuple[float, float], size: float) -> str:
    """`board_text` with the provenance text added as the board's last item."""
    body = board_text.rstrip()
    if not body.startswith("(kicad_pcb") or not body.endswith(")"):
        raise KicadError("not a .kicad_pcb: no (kicad_pcb ...) form to add text to")
    return body[:-1] + gr_text(text, at, size) + ")\n"


def absolute(cfg: paths.BoardConfig, at: list[float]) -> tuple[float, float]:
    """An origin-relative board.toml point in board coordinates."""
    origin = cfg.data.get("outline", {}).get("origin")
    if not _numbers(origin, 2):
        raise KicadError(f"{cfg.path}: [outline] origin must be [x, y]")
    return (origin[0] + at[0], origin[1] + at[1])


def sha256(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def stamped_board_sha256(stamp_path: Path, pcb: bytes) -> str:
    """The stamp's board_sha256, which must be the committed board's."""
    try:
        stamp = json.loads(stamp_path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as e:
        raise KicadError(f"{stamp_path}: {e}") from None
    stamped = stamp.get("board_sha256") if isinstance(stamp, dict) else None
    if stamped != sha256(pcb):
        raise KicadError(
            f"{stamp_path.name}: board_sha256 does not match the .kicad_pcb; run "
            "carrier-check, which writes the stamp, and commit both"
        )
    return stamped


def handoff_row(
    board: str,
    state: GitState,
    dirty: bool,
    text: str,
    pcb_sha256: str,
    zip_path: Path,
    bom: Path,
    cpl: Path,
    repo_root: Path = paths.REPO_ROOT,
) -> dict[str, str]:
    """One hand-off CSV row: the commit and the zip's full sha256."""
    return {
        "board": board,
        "git_sha": state.sha,
        "dirty": "yes" if dirty else "no",
        "silk_text": text,
        "pcb_sha256": pcb_sha256,
        "zip": _rel(zip_path, repo_root),
        "zip_sha256": sha256(zip_path.read_bytes()),
        "bom": _rel(bom, repo_root),
        "cpl": _rel(cpl, repo_root),
    }


def write_handoff(path: Path, row: dict[str, str]) -> None:
    with open(path, "w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=HANDOFF_FIELDS)
        w.writeheader()
        w.writerow(row)


def _rel(path: Path, repo_root: Path) -> str:
    try:
        return path.resolve().relative_to(repo_root.resolve()).as_posix()
    except ValueError:
        return str(path)


def _numbers(v: Any, n: int) -> bool:
    return (
        isinstance(v, list)
        and len(v) == n
        and all(isinstance(x, (int, float)) and not isinstance(x, bool) for x in v)
    )


def _positive(v: Any) -> bool:
    return isinstance(v, (int, float)) and not isinstance(v, bool) and v > 0


def _fmt(v: float) -> str:
    return f"{v:.4f}".rstrip("0").rstrip(".")
