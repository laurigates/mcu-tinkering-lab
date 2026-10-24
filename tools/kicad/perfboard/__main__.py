"""Lay a carrier out on a perfboard: its drawing, 1:1 print, wire list, checklist and step guide.

    PYTHONPATH=tools python3 -m kicad.perfboard [project ...] [--check] [--allow-none]

With no project, every project with a git-tracked
`hardware/perfboard/perfboard.toml`. `--check` writes nothing and fails when
the committed files differ from what would be written. Either way nothing is
written, and the run fails, when the layout breaks a check: a pad off the
board or two in one hole, overlapping bodies, a net split or two joined, or a
route that jumps, lies over a pad or joint, shares a segment, or overfills a hole. KiCad-free.
"""

from __future__ import annotations

import argparse
import sys
from dataclasses import dataclass
from pathlib import Path

from .. import boardcheck, library, netlist, paths, registry
from ..errors import KicadError
from ..paths import REPO_ROOT
from . import (
    config,
    continuity,
    place,
    render_guide,
    render_print,
    render_svg,
    route,
    wirelist,
    wires,
)

MARKER = config.MARKER
OUTPUTS = ("perfboard.svg", "perfboard-print.svg", "perfboard.md", "continuity.md")
# The step guide goes to docs/, beside the build guide, where its PDF is compiled.
GUIDE_DIR = "docs"


@dataclass(frozen=True)
class Layout:
    outputs: dict[Path, str]  # path -> text, in OUTPUTS order, then the step guide
    parts: int
    wires: int
    nets: int


def heavy_nets(
    project: Path,
    cfg: config.PerfboardConfig,
    nl: netlist.Netlist,
    repo_root: Path = REPO_ROOT,
) -> frozenset[str]:
    """The nets of the carrier board.toml net class `[wires] heavy_class` names."""
    board = paths.load_board_config(project)
    for name, nets in boardcheck.expand_classes(board, nl):
        if name == cfg.heavy_class:
            return frozenset(nets)
    names = [c.get("name") for c in board.data.get("net_classes", [])]
    raise KicadError(
        f"{cfg.path} [wires] heavy_class {cfg.heavy_class!r} is not a [[net_classes]] "
        f"name in {paths._display(board.path, repo_root)} ({names})"
    )


def generate(
    project: Path, repo_root: Path = REPO_ROOT, vendored: Path = library.VENDORED
) -> Layout:
    """The perfboard files of `project`; raises KicadError, one line per failed check."""
    toml = config.perfboard_dir(project) / config.FILENAME
    # Errors name the file repo-relative, as the other kicad CLIs do.
    cfg = config.load(toml, Path(paths._display(toml, repo_root)))
    nl = netlist.build(project, repo_root, vendored)
    heavy = heavy_nets(project, cfg, nl, repo_root)
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
    errors += config.check_steps(cfg, nets)
    errors += config.check_isolated(cfg, nets)
    errors += config.check_buses(cfg, nets)
    if errors:
        raise KicadError("\n".join(errors))
    planned = route.plan(parts, nets, cfg.steps, board, cfg.buses, cfg.conductors)
    numbered, routes = wires.number_wires(nets, planned, cfg.steps)
    errors = wires.check_wires(parts, nets, planned)
    errors += route.check_routes(numbered, routes, parts, nets, cfg.conductors)
    if errors:
        raise KicadError("\n".join(errors))
    out_dir = config.perfboard_dir(project)
    texts = (
        render_svg.svg(parts, numbered, routes, board, cfg.steps, raised, heavy),
        render_print.svg(parts, numbered, routes, board, raised, heavy, cfg.title),
        wirelist.wire_list(parts, numbered, routes, cfg, heavy),
        continuity.checklist(parts, nets, cfg),
    )
    outputs = {out_dir / name: t for name, t in zip(OUTPUTS, texts, strict=True)}
    guide_dir = project / GUIDE_DIR
    outputs[guide_dir / render_guide.FILENAME] = render_guide.guide(
        parts, nets, numbered, routes, cfg, heavy, guide_dir, out_dir / OUTPUTS[0]
    )
    return Layout(
        outputs=outputs,
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
                    f"run just {paths.just_module(project, repo_root)}::perfboard"
                )
                failed = True
            else:
                print(f"perfboard up to date: {summary}")
            continue
        for path, text in layout.outputs.items():
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text(text, encoding="utf-8")
        print(f"wrote {summary}")
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
