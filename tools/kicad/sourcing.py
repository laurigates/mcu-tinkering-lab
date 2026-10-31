"""Where each part on a carrier comes from (.claude/rules/part-sourcing.md).

    PYTHONPATH=tools python3 -m kicad.sourcing [project ...] [--allow-none]

Every part in a carrier's BOM needs a route onto the board:

- JLCPCB places it: an LCSC number (`lcsc`), on the hardware.toml instance or
  its registry entry, as the schematic already carries it;
- it is bought or owned for hand assembly: `buy`, in the same two places, an
  AliExpress listing (`aliexpress:<id>`), loose LCSC parts (`lcsc:C<n>`) or
  stock on hand (`owned:<what>`), per `hardware.model.BUY`.

The two are separate facts: an LCSC number says JLCPCB can place the part, not
that anyone sells it loose. A part with neither fails. A part with only an
LCSC number is listed, because a board that is hand-assembled must leave it on
the PCBA BOM or order it from LCSC. A part outside the BOM (mounting holes,
fiducials) needs nothing. For a socketed module, `buy` names what is soldered
to the board, the headers, not the module fitted later.
"""

from __future__ import annotations

import argparse
import sys
from dataclasses import dataclass
from pathlib import Path

from hardware import CARRIER, HardwareError, join

from . import netlist, paths, registry
from .errors import KicadError
from .paths import REPO_ROOT


@dataclass(frozen=True)
class Row:
    ref: str
    value: str
    in_bom: bool
    lcsc: str  # "" when JLCPCB cannot place it
    buy: str  # "" when no hand-assembly source is recorded


@dataclass(frozen=True)
class Report:
    unsourced: tuple[Row, ...]  # in the BOM with neither route: fails
    jlc_only: tuple[Row, ...]  # JLCPCB must place these on a hand-built board


def classify(rows: list[Row]) -> Report:
    bom = [r for r in rows if r.in_bom]
    return Report(
        unsourced=tuple(r for r in bom if not r.lcsc and not r.buy),
        jlc_only=tuple(r for r in bom if r.lcsc and not r.buy),
    )


def rows(project: Path, repo_root: Path = REPO_ROOT) -> list[Row]:
    """One row per carrier component, `buy` from the instance, else its entry."""
    try:
        model = join(project, repo_root, view=CARRIER)
    except HardwareError as e:
        raise KicadError(str(e)) from None
    out = []
    for c in netlist.build(project, repo_root).comps:
        instance = model.parts.get(c.part_key)
        buy = (instance.buy if instance else "") or registry.load(
            c.registry, repo_root
        ).buy
        out.append(Row(c.ref, c.value, c.in_bom, c.lcsc, buy))
    return out


def render(report: Report) -> str:
    lines = []
    for title, group in (
        ("no source (needs lcsc or buy)", report.unsourced),
        ("JLCPCB must place these on a hand-built board", report.jlc_only),
    ):
        lines.append(f"  {title}: {len(group)}")
        lines.extend(f"    {r.ref}  {r.value}  {r.lcsc}".rstrip() for r in group)
    return "\n".join(lines)


def main(argv: list[str] | None = None, repo_root: Path = REPO_ROOT) -> int:
    parser = argparse.ArgumentParser(
        prog="kicad.sourcing", description=__doc__.split("\n")[0]
    )
    paths.add_project_args(parser)
    args = parser.parse_args(argv)
    failed = False
    try:
        for project in paths.select_projects(args.projects, args.allow_none, repo_root):
            report = classify(rows(project, repo_root))
            print(f"{project}:")
            print(render(report))
            failed = failed or bool(report.unsourced)
    except KicadError as e:
        print(f"FAIL: {e}")
        return 1
    if failed:
        print("FAIL: a part has no source; add `buy` or `lcsc` (part-sourcing.md)")
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
