"""The board input: what the pcbnew scripts in tools/kicad/pcb/ place and route.

    PYTHONPATH=tools python3 -m kicad.boardinput [project ...] [--allow-none]

KiCad's bundled Python is 3.9, with no `tomllib` and no access to the join, so
the pcbnew scripts read one JSON file instead of hardware.toml, the parts
registry and board.toml. This writes it to
`<project>/hardware/carrier/output/board-input.json` (git-ignored):

- `pcb`, `out_dir`: the KiCad project's `.kicad_pcb` and its `output/`
  (route_board's DSN and session), absolute;
- `origin`, `size`: board.toml `[outline]`;
- `components`: per netlist component, in reference order, its value,
  footprint id, vendored footprint directory, the path of its symbol in the
  generated schematic (`/<uuid5 sym/REF>`), the footprint's own UUID
  (`uuid5 fp/REF`, see `footprint_uuid`), its custom fields (LCSC) and its
  board.toml `[placement]`;
- `nets`: every netlist net, plus one single-pad net per no-connect pin named
  as KiCad names it (`unconnected-(<ref>-<pin name>-Pad<n>)`); without those,
  DRC schematic parity fails with "Pad missing net given by schematic". Sorted
  by name, which is the order of KiCad's own netlist export;
- `silk`: board.toml `[silk]`, defaults filled in;
- `classes`: board.toml `[[net_classes]]` with `nets` expanded to net names
  (per pattern in order, matches in name order); `signal`: as written;
- `pour`: board.toml `[pour]` with its net(s) as `regions`, `[{net, rect}]`,
  each with its own `layers` and `priority` where board.toml sets them;
  a single-net `[pour]` is one region with `rect` null, the whole board;
- `tracks`: board.toml `[[tracks]]`, `[{name, net, layer, width_mm, points}]`,
  points relative to the origin; pre-routed tracks, laid down locked;
- `keepouts`: board.toml `[[keepouts]]`, `[{name, rect}]`, rects relative to
  the origin; rule areas with no copper (tracks, vias, pads, pour);
- `cutouts`: board.toml `[[cutouts]]`, `[{name, center, diameter}]`, centres
  relative to the origin; holes milled through the board, drawn on Edge.Cuts;
- `freerouting_jar`: $FREEROUTING_JAR, else ~/.cache/freerouting/freerouting-2.5.0.jar.

board.toml is validated against the netlist first (`boardcheck.validate_config`,
the check CI runs), so the scripts can trust every reference and net name.
"""

from __future__ import annotations

import argparse
import json
import os
import sys
import uuid
from pathlib import Path
from typing import Any

from . import boardcheck, library, netlist, paths, schematic
from .errors import KicadError
from .paths import REPO_ROOT

OUTPUT = "output"
FILENAME = "board-input.json"
FREEROUTING_JAR = "~/.cache/freerouting/freerouting-2.5.0.jar"
SILK_DEFAULTS: dict[str, Any] = {
    "hidden_refs": [],
    "ref_positions": {},
    "ref_sizes": {},
    "rects": [],
    "texts": [],
    "cut_lines": [],
}


def input_path(project: Path) -> Path:
    return paths.carrier_dir(project) / OUTPUT / FILENAME


def footprint_uuid(ns: uuid.UUID, ref: str) -> str:
    """The UUID build_board.py gives footprint `ref`: uuid5 under the carrier's namespace.

    KiCad saves a board's footprints in UUID order and its Specctra DSN export
    follows it, so a UUID KiCad draws afresh reorders every placement run (#763).
    """
    return str(uuid.uuid5(ns, f"fp/{ref}"))


def assemble(
    cfg: paths.BoardConfig,
    nl: netlist.Netlist,
    ns: uuid.UUID,
    kicad_dir: Path,
    vendored: Path = library.VENDORED,
) -> dict[str, Any]:
    """The board input for one project, without `freerouting_jar`.

    Raises KicadError listing every board.toml problem.
    """
    errors = boardcheck.validate_config(cfg, nl)
    if errors:
        raise KicadError("\n".join(errors))
    data = cfg.data
    placement = data["placement"]
    components = [
        {
            "ref": c.ref,
            "value": c.value,
            "footprint": c.footprint,
            "fp_dir": str(
                library.footprint_file(c.footprint, vendored=vendored).parent
            ),
            "path": boardcheck.symbol_path(ns, c.ref),
            "uuid": footprint_uuid(ns, c.ref),
            "fields": [list(f) for f in c.fields],
            "at": list(placement[c.ref]),
        }
        for c in nl.comps
    ]
    nets: dict[str, list[list[str]]] = {
        name: [list(n) for n in nodes] for name, nodes in nl.nets.items()
    }
    for name, node in netlist.unconnected(nl, vendored).items():
        if name in nets:
            raise KicadError(f"{cfg.path}: net {name!r} is also a no-connect net")
        nets[name] = [list(node)]
    classes = [
        {
            "name": name,
            "nets": members,
            "track_um": cls["track_um"],
            "clearance_um": cls["clearance_um"],
            "via_um": list(cls["via_um"]),
        }
        for (name, members), cls in zip(
            boardcheck.expand_classes(cfg, nl), data["net_classes"], strict=True
        )
    ]
    pcb = kicad_dir / f"{cfg.name}.kicad_pcb"
    return {
        "pcb": str(pcb.resolve()),
        "out_dir": str((kicad_dir / OUTPUT).resolve()),
        "origin": list(data["outline"]["origin"]),
        "size": list(data["outline"]["size"]),
        "components": components,
        "nets": [{"name": n, "nodes": nets[n]} for n in sorted(nets)],
        "silk": {**SILK_DEFAULTS, **data.get("silk", {})},
        "classes": classes,
        "signal": dict(data["signal"]),
        "pour": {
            **{k: v for k, v in data["pour"].items() if k not in ("net", "regions")},
            "regions": boardcheck.pour_region_specs(data["pour"]),
        },
        "tracks": [dict(tr) for tr in data.get("tracks", [])],
        "fanout": boardcheck.fanout_spec(data),
        "keepouts": [dict(k) for k in data.get("keepouts", [])],
        "cutouts": [dict(c) for c in data.get("cutouts", [])],
    }


def build(
    project: Path, repo_root: Path = REPO_ROOT, vendored: Path = library.VENDORED
) -> dict[str, Any]:
    """`assemble` for a project directory, without `freerouting_jar`."""
    cfg = paths.load_board_config(project)
    nl = netlist.build(project, repo_root, vendored)
    ns = schematic.namespace(schematic.project_key(project, repo_root))
    return assemble(cfg, nl, ns, paths.carrier_dir(project) / cfg.name, vendored)


def freerouting_jar() -> str:
    return os.path.expanduser(os.environ.get("FREEROUTING_JAR", FREEROUTING_JAR))


def main(argv: list[str] | None = None, repo_root: Path = REPO_ROOT) -> int:
    parser = argparse.ArgumentParser(
        prog="kicad.boardinput", description=__doc__.split("\n")[0]
    )
    paths.add_project_args(parser)
    args = parser.parse_args(argv)
    vendored = repo_root / library.VENDORED.relative_to(REPO_ROOT)
    try:
        for project in paths.select_projects(args.projects, args.allow_none, repo_root):
            spec = build(project, repo_root, vendored)
            spec["freerouting_jar"] = freerouting_jar()
            out = input_path(project)
            out.parent.mkdir(parents=True, exist_ok=True)
            out.write_text(json.dumps(spec, indent=1) + "\n", encoding="utf-8")
            print(
                f"wrote {out}: {len(spec['components'])} components, {len(spec['nets'])} nets"
            )
    except KicadError as e:
        print(f"FAIL: {e}")
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
