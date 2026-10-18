"""Board area a carrier's parts take: each footprint's courtyard, summed.

    PYTHONPATH=tools python3 -m kicad.area [project ...] [--allow-none]

KiCad-free: it reads the carrier netlist and the vendored footprints, so it
works on a carrier that has a schematic and no board yet. Each part counts its
F.CrtYd bounding box, the area nothing else may overlap. For a breakout socket
that box is the module's outline, the board area the module covers; for a bare
chip it is the package and its pads. The sum is a lower bound on the board: it
leaves out routing, keepouts and the gaps a placement leaves.

Parts are grouped by what they connect to, not by where a board.toml puts them,
so a carrier with no placement is measured the same way:

- `head`: the MCU socket and the cable's head connector, on every head board
  (with no `[[cables]]` entry the MCU is on the main board);
- `mechanical`: parts with no pins on any net (mounting holes);
- `main`: everything else.

Given two projects, it also lists the main-board parts each has that the other
does not, matched by reference and footprint. That is the comparison between
the socketed and the integrated robocar carriers:

    PYTHONPATH=tools python3 -m kicad.area packages/robocar/unified \\
        packages/robocar/unified-integrated
"""

from __future__ import annotations

import argparse
import sys
from dataclasses import dataclass
from pathlib import Path

from hardware import MCU

from . import footprints, library, netlist, paths
from .errors import KicadError
from .paths import REPO_ROOT

MAIN, HEAD, MECHANICAL = "main", "head", "mechanical"
GROUPS = (MAIN, HEAD, MECHANICAL)


@dataclass(frozen=True)
class Part:
    ref: str
    value: str
    footprint: str
    group: str
    width: float  # courtyard bounding box, mm
    height: float
    smd: bool  # the footprint's (attr smd)
    pads: int  # numbered pads

    @property
    def area(self) -> float:
        return self.width * self.height


def group_of(comp: netlist.Component, on_nets: set[str], cable: bool = True) -> str:
    """`head` for an MCU copy or a head connector, `mechanical` for a part on no net.

    Without a cable (`cable` false) the MCU sits on the main board.
    """
    key = comp.part_key
    head = key == MCU or key.startswith(f"{MCU}.") or key.startswith(netlist.HEAD)
    if head and cable:
        return HEAD
    if comp.ref not in on_nets:
        return MECHANICAL
    return MAIN


def measure(
    project: Path, repo_root: Path = REPO_ROOT, vendored: Path = library.VENDORED
) -> list[Part]:
    """Every part of `project`'s carrier with its group and courtyard, by natural ref."""
    nl = netlist.build(project, repo_root, vendored)
    return parts(nl, vendored)


def parts(nl: netlist.Netlist, vendored: Path = library.VENDORED) -> list[Part]:
    on_nets = {ref for members in nl.nets.values() for ref, _ in members}
    on_nets |= {ref for ref, _ in nl.no_connect}
    cable = any(c.part_key.startswith(netlist.HEAD) for c in nl.comps)
    out = []
    for c in nl.comps:
        fp = footprints.parse(library.footprint_file(c.footprint, vendored=vendored))
        x0, y0, x1, y1 = fp.courtyard
        out.append(
            Part(
                ref=c.ref,
                value=c.value,
                footprint=c.footprint,
                group=group_of(c, on_nets, cable),
                width=x1 - x0,
                height=y1 - y0,
                smd="smd" in fp.attr.split(),
                pads=len(fp.pads),
            )
        )
    return out


def summary(found: list[Part]) -> list[str]:
    """One line per group: part count, courtyard area, SMD and through-hole counts."""
    lines = []
    for group in GROUPS:
        members = [p for p in found if p.group == group]
        if not members:
            continue
        area = sum(p.area for p in members)
        smd = sum(p.smd for p in members)
        lines.append(
            f"  {group:10s} {len(members):3d} parts {area:9.1f} mm²  "
            f"({smd} SMD, {len(members) - smd} through-hole, "
            f"{sum(p.pads for p in members)} pads)"
        )
    return lines


def compare(a: list[Part], b: list[Part], name_a: str, name_b: str) -> list[str]:
    """The main-board parts only one side has, matched by (ref, footprint)."""
    main_a = {(p.ref, p.footprint): p for p in a if p.group == MAIN}
    main_b = {(p.ref, p.footprint): p for p in b if p.group == MAIN}
    only_a = [p for k, p in main_a.items() if k not in main_b]
    only_b = [p for k, p in main_b.items() if k not in main_a]
    shared = [p for k, p in main_a.items() if k in main_b]
    lines = [
        f"main board: {len(shared)} parts in both, "
        f"{sum(p.area for p in shared):.1f} mm²"
    ]
    for name, side in ((name_a, only_a), (name_b, only_b)):
        lines.append(
            f"only in {name}: {len(side)} parts, {sum(p.area for p in side):.1f} mm²"
        )
        lines += [_row(p) for p in side]
    total_a = sum(p.area for p in main_a.values())
    total_b = sum(p.area for p in main_b.values())
    if total_a:
        lines.append(
            f"main board courtyards: {total_a:.1f} -> {total_b:.1f} mm² "
            f"({(total_b - total_a) / total_a:+.0%})"
        )
    return lines


def _row(p: Part) -> str:
    kind = "SMD" if p.smd else "THT"
    return (
        f"    {p.ref:5s} {p.width:5.1f} x {p.height:5.1f} = {p.area:7.1f} mm²  "
        f"{kind}  {p.value}  ({p.footprint})"
    )


def main(argv: list[str] | None = None, repo_root: Path = REPO_ROOT) -> int:
    parser = argparse.ArgumentParser(
        prog="kicad.area", description=__doc__.split("\n")[0]
    )
    paths.add_project_args(parser)
    parser.add_argument(
        "--parts", action="store_true", help="list every part with its courtyard"
    )
    args = parser.parse_args(argv)
    vendored = repo_root / library.VENDORED.relative_to(REPO_ROOT)
    try:
        chosen = paths.select_projects(args.projects, args.allow_none, repo_root)
        measured = []
        for project in chosen:
            found = measure(project, repo_root, vendored)
            name = paths._display(project, repo_root)
            measured.append((name, found))
            print(f"{name}: courtyard bounding boxes")
            for line in summary(found):
                print(line)
            if args.parts:
                for p in found:
                    print(_row(p) + f"  [{p.group}]")
        if len(measured) == 2:
            (name_a, a), (name_b, b) = measured
            for line in compare(a, b, name_a, name_b):
                print(line)
    except KicadError as e:
        print(f"FAIL: {e}")
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
