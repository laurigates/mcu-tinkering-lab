"""Where a project's carrier lives, and which projects have one.

A carrier project is a project directory with a git-tracked
`hardware/carrier/board.toml`; the KiCad project sits beside it in
`hardware/carrier/<[project] name>/`. A perfboard project has a git-tracked
`hardware/perfboard/perfboard.toml` (`kicad.perfboard`).

Every per-project CLI in tools/kicad takes project directories as arguments
and, given none, runs on every tracked carrier project. It prints how many it
found and **fails when that is zero** unless given `--allow-none`: a renamed or
not-yet-added board.toml must not turn every gate into a silent pass.
"""

from __future__ import annotations

import argparse
import re
import subprocess
import tomllib
from dataclasses import dataclass
from pathlib import Path
from typing import Any

from .errors import KicadError

REPO_ROOT = Path(__file__).resolve().parents[2]
BOARD_TOML = "board.toml"
# The git-tracked file that makes a directory a project, relative to it.
CARRIER_MARKER = f"hardware/carrier/{BOARD_TOML}"

# board.toml's tables. Their contents are validated where they are
# read; this rejects a misspelt table name, which would otherwise be ignored.
BOARD_TABLES = frozenset(
    {"project", "outline", "placement", "silk", "net_classes", "signal", "pour"}
)
PROJECT_KEYS = frozenset({"name", "paper"})
_NAME = re.compile(r"[A-Za-z0-9][A-Za-z0-9_.-]*")


@dataclass(frozen=True)
class BoardConfig:
    """A project's hardware/carrier/board.toml.

    `name` is the KiCad project directory and file stem; `data` is the whole
    file, for the modules that read its other tables.
    """

    path: Path
    name: str
    paper: str
    data: dict[str, Any]


def carrier_dir(project: Path) -> Path:
    return project / "hardware" / "carrier"


def load_board_config(project: Path) -> BoardConfig:
    path = carrier_dir(project) / BOARD_TOML
    try:
        data = tomllib.loads(path.read_text(encoding="utf-8"))
    except FileNotFoundError:
        raise KicadError(f"{path}: no board.toml") from None
    except tomllib.TOMLDecodeError as e:
        raise KicadError(f"{path}: {e}") from None
    unknown = sorted(set(data) - BOARD_TABLES)
    if unknown:
        raise KicadError(
            f"{path}: unknown table(s) {unknown}; allowed: {sorted(BOARD_TABLES)}"
        )
    proj = data.get("project")
    if not isinstance(proj, dict):
        raise KicadError(f"{path}: [project] is required")
    unknown = sorted(set(proj) - PROJECT_KEYS)
    if unknown:
        raise KicadError(
            f"{path}: [project]: unknown key(s) {unknown}; allowed: {sorted(PROJECT_KEYS)}"
        )
    name = proj.get("name")
    if not isinstance(name, str) or not _NAME.fullmatch(name):
        raise KicadError(f"{path}: [project] name must be a file stem, got {name!r}")
    paper = proj.get("paper", "A3")
    if not isinstance(paper, str) or not paper:
        raise KicadError(f"{path}: [project] paper must be a string, got {paper!r}")
    return BoardConfig(path=path, name=name, paper=paper, data=data)


def kicad_project_dir(project: Path) -> Path:
    return carrier_dir(project) / load_board_config(project).name


def tracked_projects(
    repo_root: Path = REPO_ROOT, marker: str = CARRIER_MARKER
) -> list[Path]:
    """Projects with a git-tracked `marker` (default hardware/carrier/board.toml), sorted.

    Tracked only (`--cached`): a marker file that is not added yet is not a
    project, and a CLI run on no projects fails rather than passes.
    """
    depth = len(Path(marker).parts)
    listed = subprocess.run(
        [
            "git",
            "ls-files",
            "-z",
            "--cached",
            "--",
            f":(glob)packages/**/{marker}",
        ],
        cwd=repo_root,
        capture_output=True,
        text=True,
    )
    if listed.returncode != 0:
        raise KicadError(
            f"{repo_root}: cannot list carrier projects; not inside a git work tree? "
            f"({listed.stderr.strip()})"
        )
    found = set()
    for rel in listed.stdout.split("\0"):
        if not rel:
            continue
        path = repo_root / rel
        # --cached lists a tracked file that was deleted from the working tree.
        if path.is_file():
            found.add(path.parents[depth - 1])
    return sorted(found)


def add_project_args(parser: argparse.ArgumentParser) -> None:
    """The project arguments every per-project CLI takes."""
    parser.add_argument(
        "projects",
        nargs="*",
        type=Path,
        help="project directories (default: every git-tracked carrier project)",
    )
    parser.add_argument(
        "--allow-none",
        action="store_true",
        help="exit 0 instead of 1 when there is no project to process",
    )


def select_projects(
    projects: list[Path],
    allow_none: bool,
    repo_root: Path = REPO_ROOT,
    marker: str = CARRIER_MARKER,
) -> list[Path]:
    """The projects to process; prints `N project(s): ...`.

    With no `projects`, every project with a git-tracked `marker`. Raises
    KicadError when there are none and `allow_none` is false.
    """
    chosen = list(projects) or tracked_projects(repo_root, marker)
    shown = " ".join(_display(p, repo_root) for p in chosen)
    print(f"{len(chosen)} project(s): {shown}".rstrip())
    if not chosen and not allow_none:
        raise KicadError(
            f"0 project(s): no git-tracked packages/**/{marker} "
            f"under {repo_root}; pass a project directory, or --allow-none"
        )
    return chosen


def _display(path: Path, repo_root: Path) -> str:
    try:
        return path.resolve().relative_to(repo_root.resolve()).as_posix()
    except ValueError:
        return str(path)
