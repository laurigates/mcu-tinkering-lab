"""Lay a carrier out on a perfboard: perfboard.svg and perfboard.md.

    PYTHONPATH=tools python3 -m kicad.perfboard [project ...] [--check] [--allow-none]

With no project, every project with a git-tracked
`hardware/perfboard/perfboard.toml`. `--check` writes nothing and fails when
the committed files differ from what would be written. Either way nothing is
written, and the run fails, when the layout breaks a check: a pad off the
board or two in one hole, overlapping bodies, a net split or two joined, or a
route that jumps, passes over a hole or shares a channel. KiCad-free.
"""

from __future__ import annotations

import argparse
import sys
from dataclasses import dataclass
from pathlib import Path

from .. import boardcheck, library, netlist, paths, registry
from ..errors import KicadError
from ..paths import REPO_ROOT
from . import config, place, render_svg, route, wires, wirelist

MARKER = f"hardware/perfboard/{config.FILENAME}"
OUTPUTS = ("perfboard.svg", "perfboard.md")


@dataclass(frozen=True)
class Layout:
    outputs: dict[Path, str]  # path -> text, in OUTPUTS order
    parts: int
    wires: int
    nets: int


def heavy_nets(
    project: Path, cfg: config.PerfboardConfig, nl: netlist.Netlist
) -> frozenset[str]:
    """The nets of the carrier board.toml net class `[wires] heavy_class` names."""
    board = paths.load_board_config(project)
    for name, nets in boardcheck.expand_classes(board, nl):
        if name == cfg.heavy_class:
            return frozenset(nets)
    names = [c.get("name") for c in board.data.get("net_classes", [])]
    raise KicadError(
        f"{cfg.path} [wires] heavy_class {cfg.heavy_class!r} is not a [[net_classes]] "
        f"name in {board.path} ({names})"
    )


def generate(
    project: Path, repo_root: Path = REPO_ROOT, vendored: Path = library.VENDORED
) -> Layout:
    """The perfboard files of `project`; raises KicadError, one line per failed check."""
    cfg = config.load(config.perfboard_dir(project) / config.FILENAME)
    nl = netlist.build(project, repo_root, vendored)
    heavy = heavy_nets(project, cfg, nl)
    skip = cfg.skip | place.pinless(nl)
    specs = {c.registry: registry.load(c.registry, repo_root) for c in nl.comps}
    parts, raised, tall = place.build_parts(
        nl,
        cfg,
        specs,
        lambda fpid: library.footprint_file(
            fpid, vendored_only=True, vendored=vendored
        ),
        skip,
    )
    nets = place.nets_of(nl, skip)
    board = cfg.board
    errors = place.check_placement(parts, board, raised, tall)
    planned = wires.plan_wires(parts, nets)
    errors += wires.check_wires(parts, nets, planned)
    errors += config.check_steps(cfg, nets)
    if errors:
        raise KicadError("\n".join(errors))
    numbered = wires.number_wires(nets, planned, cfg.steps)
    routes = route.route_wires(numbered, len(cfg.steps), board)
    errors = route.check_routes(numbered, routes, len(cfg.steps))
    if errors:
        raise KicadError("\n".join(errors))
    out_dir = config.perfboard_dir(project)
    texts = (
        render_svg.svg(parts, numbered, routes, board, cfg.steps, raised, heavy),
        wirelist.wire_list(parts, numbered, routes, cfg, heavy),
    )
    return Layout(
        outputs={out_dir / name: t for name, t in zip(OUTPUTS, texts, strict=True)},
        parts=len(parts),
        wires=len(planned),
        nets=len(nets),
    )


def stale(layout: Layout) -> list[str]:
    """Names of the outputs whose committed text differs (or that are missing)."""
    return [
        path.name
        for path, text in layout.outputs.items()
        if not path.is_file() or path.read_text(encoding="utf-8") != text
    ]


def main(argv: list[str] | None = None, repo_root: Path = REPO_ROOT) -> int:
    parser = argparse.ArgumentParser(
        prog="kicad.perfboard", description=__doc__.split("\n")[0]
    )
    paths.add_project_args(parser)
    parser.add_argument(
        "--check", action="store_true", help="fail if the committed outputs differ"
    )
    args = parser.parse_args(argv)
    failed = False
    try:
        projects = paths.select_projects(
            args.projects, args.allow_none, repo_root, MARKER
        )
    except KicadError as e:
        print(f"FAIL: {e}")
        return 1
    for project in projects:
        try:
            layout = generate(project, repo_root)
        except KicadError as e:
            for line in str(e).splitlines():
                print(f"FAIL: {line}")
            failed = True
            continue
        summary = (
            f"{layout.parts} parts, {layout.wires} wires, {layout.nets} nets connected"
        )
        if args.check:
            names = stale(layout)
            if names:
                print(
                    f"FAIL: {paths._display(project, repo_root)}: stale: {names}; "
                    f"run just {Path(project).resolve().name}::perfboard"
                )
                failed = True
            else:
                print(f"perfboard up to date: {summary}")
            continue
        for path, text in layout.outputs.items():
            path.write_text(text, encoding="utf-8")
        print(f"wrote {summary}")
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
