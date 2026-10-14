"""Netlist parity: the committed schematic, as KiCad reads it, against the join.

    PYTHONPATH=tools python3 -m kicad.parity [project ...] [--allow-none]

Local only: it runs `kicad-cli sch export netlist --format kicadxml` on the
project's committed `.kicad_sch` and compares that export with
`netlist.build()`. Equal means:

- the same reference designators;
- per reference, the same value, footprint, symbol (`libsource lib:part`),
  LCSC field, and BOM membership (`exclude_from_bom`);
- every named net (not `unconnected-*`) with the same name and the same
  (ref, pin) members;
- the same no-connect pins under the same `unconnected-(...)` names
  (`netlist.nc_net_name`).

Order is not compared: KiCad decides it. Power symbols (`#FLG`, `#PWR`) are
not components and carry no node of their own here.
"""

from __future__ import annotations

import argparse
import subprocess
import sys
import tempfile
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Any

from . import library, netlist, paths
from .errors import KicadError
from .paths import REPO_ROOT

UNCONNECTED = "unconnected-"


def normalise(root: ET.Element) -> dict[str, Any]:
    """A kicadxml export as plain data, in `netlist.as_dict`'s shape."""
    comps: dict[str, dict[str, Any]] = {}
    for c in root.iter("comp"):
        ref = c.get("ref", "")
        if ref.startswith("#"):
            continue
        src = c.find("libsource")
        if src is None:
            raise KicadError(f"netlist export: component {ref} has no <libsource>")
        fields = {f.get("name"): f.text or "" for f in c.iter("field")}
        props = {p.get("name") for p in c.iter("property")}
        comps[ref] = {
            "footprint": c.findtext("footprint") or "",
            "in_bom": "exclude_from_bom" not in props,
            "lcsc": fields.get("LCSC", ""),
            "lib_id": f"{src.get('lib')}:{src.get('part')}",
            "value": c.findtext("value") or "",
        }
    nets: dict[str, list[list[str]]] = {}
    unconnected: dict[str, list[str]] = {}
    found = root.find("nets")
    if found is None:
        raise KicadError("netlist export: no <nets>")
    for net in found:
        name = net.get("name", "")
        nodes = sorted(
            {
                (n.get("ref", ""), n.get("pin", ""))
                for n in net.iter("node")
                if not n.get("ref", "").startswith("#")
            },
            key=netlist.node_key,
        )
        if name.startswith(UNCONNECTED):
            if len(nodes) != 1:
                raise KicadError(
                    f"netlist export: {name} has {len(nodes)} nodes, expected 1"
                )
            unconnected[name] = list(nodes[0])
        elif nodes:
            nets[name] = [list(n) for n in nodes]
    return {
        "components": dict(
            sorted(comps.items(), key=lambda kv: netlist.ref_key(kv[0]))
        ),
        "nets": dict(sorted(nets.items())),
        "unconnected": dict(sorted(unconnected.items())),
    }


def diff(
    want: dict[str, Any], got: dict[str, Any], a: str = "join", b: str = "schematic"
) -> list[str]:
    """Differences between two normalised netlists; [] when equal."""
    errors = []
    wc, gc = want["components"], got["components"]
    for ref in sorted(set(wc) | set(gc), key=netlist.ref_key):
        if ref not in gc:
            errors.append(f"component {ref}: in the {a}, not in the {b}")
        elif ref not in wc:
            errors.append(f"component {ref}: in the {b}, not in the {a}")
        else:
            for field in sorted(set(wc[ref]) | set(gc[ref])):
                x, y = wc[ref].get(field), gc[ref].get(field)
                if x != y:
                    errors.append(f"component {ref} {field}: {a} {x!r}, {b} {y!r}")
    wn, gn = want["nets"], got["nets"]
    for name in sorted(set(wn) | set(gn)):
        if name not in gn:
            errors.append(
                f"net {name}: in the {a} ({_nodes(wn[name])}), not in the {b}"
            )
        elif name not in wn:
            errors.append(
                f"net {name}: in the {b} ({_nodes(gn[name])}), not in the {a}"
            )
        elif _set(wn[name]) != _set(gn[name]):
            errors.append(f"net {name}: {a} {_nodes(wn[name])}, {b} {_nodes(gn[name])}")
    wu, gu = _pairs(want["unconnected"]), _pairs(got["unconnected"])
    if wu != gu:
        errors.append(
            f"no-connects differ: only in the {a} {sorted(wu - gu)}, "
            f"only in the {b} {sorted(gu - wu)}"
        )
    return errors


def _set(nodes: list[list[str]]) -> set[tuple[str, ...]]:
    return {tuple(n) for n in nodes}


def _pairs(unconnected: dict[str, list[str]]) -> set[tuple[str, str, str]]:
    return {(name, *node) for name, node in unconnected.items()}


def _nodes(nodes: list[list[str]]) -> str:
    return " ".join(f"{r}.{p}" for r, p in nodes)


def compare(
    nl: netlist.Netlist, root: ET.Element, vendored: Path = library.VENDORED
) -> list[str]:
    """Differences between the join's netlist and a kicadxml export of a schematic."""
    return diff(netlist.as_dict(nl, vendored), normalise(root))


def kicad_cli() -> str:
    from .vendor import kicad_cli as find

    cli = find()
    if cli is None:
        raise KicadError("kicad-cli not found: parity is a local check (KiCad 10.0.6)")
    return cli


def export(schematic: Path) -> ET.Element:
    """`kicad-cli sch export netlist --format kicadxml` of `schematic`, parsed."""
    if not schematic.is_file():
        raise KicadError(f"{schematic}: no schematic")
    with tempfile.TemporaryDirectory() as tmp:
        out = Path(tmp) / "netlist.xml"
        run = subprocess.run(
            [
                kicad_cli(),
                "sch",
                "export",
                "netlist",
                "--format",
                "kicadxml",
                "-o",
                str(out),
                str(schematic),
            ],
            capture_output=True,
            text=True,
        )
        if run.returncode != 0 or not out.is_file():
            raise KicadError(
                f"{schematic}: kicad-cli netlist export failed (exit {run.returncode}): "
                f"{(run.stderr or run.stdout).strip()}"
            )
        return ET.parse(out).getroot()


def schematic_path(project: Path) -> Path:
    cfg = paths.load_board_config(project)
    return paths.carrier_dir(project) / cfg.name / f"{cfg.name}.kicad_sch"


def main(argv: list[str] | None = None, repo_root: Path = REPO_ROOT) -> int:
    parser = argparse.ArgumentParser(
        prog="kicad.parity", description=__doc__.split("\n")[0]
    )
    paths.add_project_args(parser)
    args = parser.parse_args(argv)
    failed = False
    try:
        for project in paths.select_projects(args.projects, args.allow_none, repo_root):
            nl = netlist.build(project, repo_root)
            sch = schematic_path(project)
            errors = compare(nl, export(sch))
            for e in errors:
                print(f"FAIL: {e}")
            shown = sch.resolve().relative_to(repo_root.resolve()).as_posix()
            if errors:
                failed = True
                print(
                    f"PARITY FAIL: {shown}: {len(errors)} difference(s) from the join"
                )
            else:
                print(
                    f"OK: {shown}: 0 differences ({len(nl.comps)} components, "
                    f"{len(nl.nets)} nets, {len(nl.no_connect)} no-connects)"
                )
    except KicadError as e:
        print(f"FAIL: {e}")
        return 1
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
