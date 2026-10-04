"""Markdown emitted from the join, injected between marker comments (#461).

A project's Markdown files restate its pin table by hand — WIRING.md's master
GPIO table and the per-peripheral signal tables. This module regenerates those
tables from `join()` and writes them between markers, leaving every line
outside a marker block exactly as it was:

    <!-- BEGIN GENERATED: pin-table -->
    ...regenerated...
    <!-- END GENERATED -->

Only the tables and the power diagram are generated. The prose around them —
power budgets, trade-offs, the flashing section — is judgment, not a
restatement, and stays hand-written.

Blocks:

    pin-table         every GPIO pad on the board's header, in silkscreen order
    signals:<part>    the nets landing on one [parts.<part>], in sidecar order
    power-diagram     [[rails]], the MCU's nets and [[outputs]] as Mermaid (#646)

Usage, from the repo root:

    PYTHONPATH=tools python3 -m hardware.docs [--check] [<project_dir> ...]

With no project, every git-tracked `packages/**/hardware.toml` is processed.
Within a project, every Markdown file git would track is scanned, at any depth
(#653) — a block in `docs/<x>.md` is checked like one in WIRING.md. A
subdirectory with its own hardware.toml is a separate project and is skipped.
`--check` writes nothing and exits 1 with a diff if any block is stale — the
same regenerate-and-compare shape as the build-guide and schematic guards.
"""

from __future__ import annotations

import argparse
import difflib
import re
import subprocess
import sys
from collections.abc import Callable
from pathlib import Path

from .board import BoardPin
from .errors import HardwareError
from .model import MCU, REPO_ROOT, SIDECAR, HardwareModel, join

END = "<!-- END GENERATED -->"
NOTICE = (
    "<!-- Generated from hardware.toml, main/pin_config.h and the board reference"
    " by `just hardware::gen` — edit those, not this block. -->"
)
_BEGIN = re.compile(r"^<!-- BEGIN GENERATED: (\S+) -->$")
# Anything that looks like a marker but does not parse is an error rather than
# prose: a typo would otherwise leave a hand-edited table that nothing checks.
_MARKERISH = re.compile(r"^<!--\s*(BEGIN|END)\s+GENERATED\b")
_DASH = "—"


def BEGIN(name: str) -> str:  # noqa: N802 — reads as the marker it builds
    return f"<!-- BEGIN GENERATED: {name} -->"


def _cell(text: str) -> str:
    return text.replace("|", "\\|")


def _row(cells: list[str]) -> str:
    return "| " + " | ".join(_cell(c) for c in cells) + " |"


def _table(header: list[str], rows: list[list[str]]) -> str:
    lines = [
        _row(header),
        "|" + "|".join("-" * (len(h) + 2) for h in header) + "|",
        *(_row(r) for r in rows),
    ]
    return "\n".join(lines) + "\n"


def _silkscreen_key(pin: BoardPin) -> tuple[str, int, str]:
    """D0, D1, …, D10 — numeric within a prefix, not the board table's order."""
    m = re.fullmatch(r"(\D*)(\d+)(.*)", pin.name)
    if not m:
        return (pin.name, -1, "")
    return (m.group(1), int(m.group(2)), m.group(3))


def _pin_cell(model: HardwareModel, role: str) -> str:
    pad = model.pin_for(role)
    gpio = f"GPIO{model.roles[role]}"
    return f"{gpio} ({pad.name})" if pad else gpio


def pin_table(model: HardwareModel) -> str:
    """Every GPIO pad on the header: what drives it and what it is wired to."""
    by_gpio: dict[int, list[str]] = {}
    for role, gpio in model.roles.items():
        by_gpio.setdefault(gpio, []).append(role)
    undrawn = {u.role: u.why for u in model.undrawn}

    rows = []
    for pad in sorted(
        (p for p in model.board.pins if p.gpio is not None), key=_silkscreen_key
    ):
        roles = by_gpio.get(pad.gpio, [])
        if not roles:
            rows.append([pad.name, f"GPIO{pad.gpio}", _DASH, "*unassigned*", ""])
            continue
        for role in roles:
            nets = [n for n in model.nets if n.role == role]
            wired = (
                ", ".join(f"{model.parts[n.part].name} {n.pin}" for n in nets) or _DASH
            )
            notes = [n.note for n in nets if n.note]
            if role in undrawn:
                notes.append(undrawn[role])
            rows.append(
                [pad.name, f"GPIO{pad.gpio}", f"`{role}`", wired, "; ".join(notes)]
            )
    return _table(["Pin", "GPIO", "Macro", "Wired to", "Notes"], rows)


def signals_table(model: HardwareModel, part: str) -> str:
    """The nets landing on one part, in the order hardware.toml lists them."""
    if part not in model.parts:
        raise HardwareError(
            f"signals:{part}: part {part!r} is not declared under [parts]"
        )
    nets = [n for n in model.nets if n.part == part]
    if not nets:
        raise HardwareError(f"signals:{part}: part {part!r} has no nets")
    rows = [[n.pin, _pin_cell(model, n.role), n.note] for n in nets]
    return _table(["Signal", "Pin", "Function"], rows)


def _label(text: str) -> str:
    """Mermaid label text: entity codes for the characters that end a label."""
    return text.replace('"', "#quot;").replace("|", "#124;")


# Words Mermaid's flowchart grammar reserves; one used as a node id fails the
# whole chart, which the string-only --check cannot see.
_MERMAID_RESERVED = {
    "end",
    "graph",
    "flowchart",
    "subgraph",
    "class",
    "classDef",
    "click",
    "style",
    "linkStyle",
    "direction",
}


def power_diagram(model: HardwareModel) -> str:
    """The supply topology and the MCU's signal nets as one Mermaid graph (#646).

    Solid edges are rails (`[[rails]]`), labelled with the rail and the load
    pin; dotted edges are the MCU's signal nets, one per part, each pin with
    its GPIO and header pad; unlabelled edges are `[[outputs]]`. A part on
    none of the three is left out.
    """
    order: list[str] = []

    def node(key: str) -> str:
        if key in _MERMAID_RESERVED:
            raise HardwareError(
                f"part id {key!r} is a Mermaid keyword and cannot be a node in the "
                "power diagram; rename the part"
            )
        if key not in order:
            order.append(key)
        return key

    edges: list[str] = []
    for rail in model.rails:
        source = node(rail.source.part)
        for load in rail.loads:
            label = _label(f"{rail.name} → {load.pin}")
            edges.append(f'{source} -->|"{label}"| {node(load.part)}')
    by_part: dict[str, list[str]] = {}
    for net in model.nets:
        by_part.setdefault(net.part, []).append(
            _label(f"{_pin_cell(model, net.role)} → {net.pin}")
        )
    for part, pins in by_part.items():
        edges.append(f'{node(MCU)} -.->|"{"<br/>".join(pins)}"| {node(part)}')
    for output in model.outputs:
        source = node(output.part)
        edges.extend(f"{source} --> {node(load)}" for load in output.loads)

    def title(key: str) -> str:
        if key == MCU:
            return _label(model.mcu)
        part = model.parts[key]
        return _label(part.name + (f"<br/>{part.note}" if part.note else ""))

    lines = [
        "```mermaid",
        "graph TD",
        *(f'    {key}["{title(key)}"]' for key in order),
        *(f"    {edge}" for edge in edges),
        "```",
    ]
    return "\n".join(lines) + "\n"


def render_block(model: HardwareModel, name: str) -> str:
    if name == "pin-table":
        return pin_table(model)
    if name == "power-diagram":
        return power_diagram(model)
    kind, colon, arg = name.partition(":")
    if kind == "signals" and colon and arg:
        return signals_table(model, arg)
    raise HardwareError(
        f"unknown generated block {name!r}; known: 'pin-table', 'power-diagram', "
        "'signals:<part>'"
    )


def inject(text: str, render: Callable[[str], str], where: str) -> str:
    """Replace every marker block's body with `render(name)`; nothing else moves."""
    out: list[str] = []
    open_name: str | None = None
    open_line = 0
    for i, line in enumerate(text.splitlines(keepends=True), start=1):
        bare = line.rstrip("\r\n")
        begin = _BEGIN.match(bare)
        if begin:
            if open_name is not None:
                raise HardwareError(
                    f"{where}:{i}: BEGIN GENERATED inside block {open_name!r} "
                    f"(opened on line {open_line})"
                )
            open_name, open_line = begin.group(1), i
            out.append(line)
            continue
        if bare == END:
            if open_name is None:
                raise HardwareError(f"{where}:{i}: END GENERATED with no open block")
            out.append(f"{NOTICE}\n\n{render(open_name)}\n")
            out.append(line)
            open_name = None
            continue
        if _MARKERISH.match(bare):
            raise HardwareError(
                f"{where}:{i}: malformed marker {bare!r}; expected "
                f"{BEGIN('<block>')!r} or {END!r}"
            )
        if open_name is None:
            out.append(line)
    if open_name is not None:
        raise HardwareError(f"{where}:{open_line}: block {open_name!r} is never closed")
    return "".join(out)


def _markdown(project_dir: Path) -> list[Path]:
    """Every Markdown file git would track under the project, at any depth (#653).

    Read through `git ls-files` so `build/` and `managed_components/` stay out,
    and with `--others --exclude-standard` so a doc not committed yet is still
    checked. A subdirectory carrying its own hardware.toml is a separate
    project and is left to its own run.
    """
    listed = subprocess.run(
        [
            "git",
            "ls-files",
            "-z",
            "--cached",
            "--others",
            "--exclude-standard",
            "--",
            ":(glob)**/*.md",
        ],
        cwd=project_dir,
        capture_output=True,
        text=True,
    )
    if listed.returncode != 0:
        raise HardwareError(
            f"{project_dir}: cannot list Markdown files — not inside a git work "
            f"tree? ({listed.stderr.strip()})"
        )
    paths = []
    for rel in listed.stdout.split("\0"):
        if not rel:
            continue
        path = project_dir / rel
        nested = any(
            (project_dir / d / SIDECAR).is_file()
            for d in Path(rel).parents
            if d != Path(".")
        )
        # --cached lists a tracked file that was deleted from the working tree.
        if path.is_file() and not nested:
            paths.append(path)
    return paths


def _docs(project_dir: Path) -> list[Path]:
    """The project's Markdown files, at any depth, that carry at least one block."""
    return sorted(
        p
        for p in _markdown(project_dir)
        if any(_BEGIN.match(ln) for ln in p.read_text(encoding="utf-8").splitlines())
    )


def regenerate(
    project_dir: Path, repo_root: Path = REPO_ROOT
) -> dict[Path, tuple[str, str]]:
    """Path -> (current text, regenerated text) for every doc with blocks."""
    model = join(project_dir, repo_root=repo_root)
    result = {}
    for path in _docs(model.project_dir):
        old = path.read_text(encoding="utf-8")
        result[path] = (
            old,
            inject(old, lambda name: render_block(model, name), str(path)),
        )
    return result


def check_project(
    project_dir: Path, repo_root: Path = REPO_ROOT
) -> list[tuple[Path, str]]:
    """(path, unified diff) for every doc whose blocks are stale; [] when clean."""
    drift = []
    for path, (old, new) in regenerate(project_dir, repo_root).items():
        if old != new:
            diff = "".join(
                difflib.unified_diff(
                    old.splitlines(keepends=True),
                    new.splitlines(keepends=True),
                    f"{path} (committed)",
                    f"{path} (regenerated)",
                )
            )
            drift.append((path, diff))
    return drift


def _tracked_projects(repo_root: Path) -> list[Path]:
    listed = subprocess.run(
        [
            "git",
            "ls-files",
            "-z",
            "--cached",
            "--others",
            "--exclude-standard",
            f":(glob)packages/**/{SIDECAR}",
        ],
        cwd=repo_root,
        check=True,
        capture_output=True,
        text=True,
    ).stdout.split("\0")
    return [repo_root / Path(p).parent for p in listed if p]


def main(argv: list[str] | None = None, repo_root: Path = REPO_ROOT) -> int:
    parser = argparse.ArgumentParser(
        prog="hardware.docs", description=__doc__.split("\n")[0]
    )
    parser.add_argument(
        "--check", action="store_true", help="write nothing; exit 1 on drift"
    )
    parser.add_argument("projects", nargs="*", type=Path)
    args = parser.parse_args(argv)
    projects = args.projects or _tracked_projects(repo_root)

    stale = 0
    for project in projects:
        if args.check:
            for path, diff in check_project(project, repo_root):
                stale += 1
                print(f"STALE: {path}\n{diff}")
            continue
        for path, (old, new) in regenerate(project, repo_root).items():
            if old != new:
                path.write_text(new, encoding="utf-8")
                print(f"Wrote {path}")
            else:
                print(f"Up to date: {path}")
    if stale:
        print(
            f"{stale} document(s) have stale generated blocks. Run `just hardware::gen` "
            "and commit the result; edit hardware.toml or the header, never the block."
        )
        return 1
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except HardwareError as e:
        print(f"error: {e}", file=sys.stderr)
        sys.exit(2)
