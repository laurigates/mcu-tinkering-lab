"""KiCad-free board parity: the committed `.kicad_pcb` against the join's netlist.

    PYTHONPATH=tools python3 -m kicad.boardcheck [project ...] [--relink] [--allow-none]

Reads each top-level `(footprint "<fpid>"` block of the committed board (its
Reference, Value and LCSC properties, its `(path ...)`, and every pad's
`(net "<name>")`; KiCad 10 writes pad nets by name) and fails when:

- a reference is on the board and not in the netlist, or the reverse, or twice;
- a footprint id, value or LCSC number differs from the netlist's;
- a footprint's path is not `/<uuid5(ns, "sym/<ref>")>`, the UUID of its
  symbol in the generated schematic;
- a pad's net differs from its pin's net, a no-connect pin's pad is not on
  KiCad's `unconnected-(...)` net for it, or a pad the netlist has no net for
  is on a named net (unnumbered pads, mounting holes and shells, are skipped).

It also validates board.toml against the netlist (placement, silkscreen, net
classes, pour), so a bad board.toml fails CI without KiCad.

Tracks are not read: copper connectivity is DRC's job, locally, and the check
stamp (`kicad.stamp`) pins the board bytes DRC passed.

`--relink` rewrites each footprint's `(path ...)` to its uuid5 symbol UUID,
keyed by its Reference, and changes nothing else; DRC matches footprints to
symbols by reference, so this keeps the GUI's "Update PCB from Schematic" and
cross-probing consistent without re-placing or re-routing.
"""

from __future__ import annotations

import argparse
import fnmatch
import re
import sys
import uuid
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

from . import library, netlist, paths, schematic, sexpr
from .errors import KicadError
from .paths import REPO_ROOT

_PROP = re.compile(r'\(property "((?:[^"\\]|\\.)*)" "((?:[^"\\]|\\.)*)"')
_PATH = re.compile(r'^\t\t\(path "((?:[^"\\]|\\.)*)"\)$', re.M)


def hint(module: str) -> str:
    return (
        f"re-place/route locally with just {module}::carrier-board, then "
        f"just {module}::carrier-check; never hand-edit the .kicad_pcb"
    )


@dataclass(frozen=True)
class BoardFootprint:
    fpid: str
    ref: str
    value: str
    lcsc: str
    path: str | None
    pads: dict[str, set[str]] = field(
        default_factory=dict
    )  # number -> nets ("" = none)


def footprint_blocks(board_text: str) -> list[tuple[int, int, str]]:
    """(start, end, block) of each top-level footprint of a board."""
    out = []
    for start, block in sexpr.top_level_blocks(board_text, "\t"):
        if sexpr.head(block)[0] == "footprint":
            out.append((start, start + len(block), block))
    return out


def parse_footprint(block: str) -> BoardFootprint:
    fpid = sexpr.head(block)[1] or ""
    props: dict[str, str] = {}
    path = None
    pads: dict[str, set[str]] = {}
    for c in sexpr.children(block):
        kw, name = sexpr.head(c)
        if kw == "property":
            m = _PROP.match(c.strip())
            if m:
                props.setdefault(sexpr.unquote(m.group(1)), sexpr.unquote(m.group(2)))
        elif kw == "path":
            path = name
        elif kw == "pad":
            net = sexpr.child(c, "net")
            pads.setdefault(name or "", set()).add(
                (sexpr.head(net)[1] or "") if net is not None else ""
            )
    if "Reference" not in props:
        raise KicadError(f"footprint {fpid!r}: no Reference property")
    return BoardFootprint(
        fpid=fpid,
        ref=props["Reference"],
        value=props.get("Value", ""),
        lcsc=props.get("LCSC", ""),
        path=path,
        pads=pads,
    )


def symbol_path(ns: uuid.UUID, ref: str) -> str:
    return f"/{uuid.uuid5(ns, f'sym/{ref}')}"


def check(
    board_text: str,
    nl: netlist.Netlist,
    ns: uuid.UUID,
    vendored: Path = library.VENDORED,
) -> list[str]:
    """Differences between a board's footprints and the netlist; [] when equal."""
    module = nl.project.name
    tail = f"; {hint(module)}"
    want_net = {node: name for name, nodes in nl.nets.items() for node in nodes}
    nc_net = {node: name for name, node in netlist.unconnected(nl, vendored).items()}
    comps = {c.ref: c for c in nl.comps}
    errors = []
    seen: set[str] = set()
    for _, _, block in footprint_blocks(board_text):
        fp = parse_footprint(block)
        ref = fp.ref
        if ref in seen:
            errors.append(f"{ref}: on the board twice{tail}")
            continue
        seen.add(ref)
        comp = comps.get(ref)
        if comp is None:
            errors.append(f"{ref}: on the board, not in the netlist{tail}")
            continue
        if fp.fpid != comp.footprint:
            errors.append(
                f"{ref}: footprint {fp.fpid!r}, netlist says {comp.footprint!r}{tail}"
            )
        if fp.value != comp.value:
            errors.append(
                f"{ref}: value {fp.value!r}, netlist says {comp.value!r}{tail}"
            )
        if fp.lcsc != comp.lcsc:
            errors.append(f"{ref}: LCSC {fp.lcsc!r}, netlist says {comp.lcsc!r}{tail}")
        want_path = symbol_path(ns, ref)
        if fp.path != want_path:
            errors.append(
                f"{ref}: path {fp.path!r}, the schematic symbol is {want_path!r}{tail}"
            )
        for num in sorted(fp.pads, key=_pad_key):
            if not num:
                continue  # mounting pads, connector shells
            node = (ref, num)
            for got in sorted(fp.pads[num]):
                if node in want_net:
                    if got != want_net[node]:
                        errors.append(
                            f"{ref}.{num}: pad on {got!r}, netlist says {want_net[node]!r}{tail}"
                        )
                elif node in nc_net:
                    if got != nc_net[node]:
                        errors.append(
                            f"{ref}.{num}: no-connect pad on {got!r}, KiCad names its net "
                            f"{nc_net[node]!r}{tail}"
                        )
                elif got:
                    errors.append(
                        f"{ref}.{num}: pad on {got!r}, netlist has no net{tail}"
                    )
    for ref in comps:
        if ref not in seen:
            errors.append(f"{ref}: in the netlist, not on the board{tail}")
    return errors


def relink(board_text: str, ns: uuid.UUID) -> tuple[str, int]:
    """The board with every footprint path set to its symbol's uuid5; (text, changed)."""
    out = []
    pos = 0
    changed = 0
    for start, end, block in footprint_blocks(board_text):
        fp = parse_footprint(block)
        found = _PATH.findall(block)
        if len(found) != 1:
            raise KicadError(f"{fp.ref}: {len(found)} (path ...) lines, expected 1")
        new = symbol_path(ns, fp.ref)
        if found[0] != new:
            block = block.replace(f'\t\t(path "{found[0]}")', f'\t\t(path "{new}")', 1)
            changed += 1
        out += [board_text[pos:start], block]
        pos = end
    out.append(board_text[pos:])
    return "".join(out), changed


# board.toml (ADR-027). Validated here, against the netlist, so CI rejects a bad
# layout file without KiCad; the board scripts read the same tables.
TABLE_KEYS = {
    "outline": {"origin", "size"},
    "silk": {"hidden_refs", "ref_positions", "ref_sizes", "rects", "texts"},
    "net_classes": {"name", "nets", "track_um", "clearance_um", "via_um"},
    "signal": {"track_um", "clearance_um", "via_um"},
    "pour": {
        "net",
        "layers",
        "clearance_mm",
        "min_thickness_mm",
        "thermal_gap_mm",
        "spoke_mm",
    },
}
TEXT_KEYS = {"text", "at", "size"}
REQUIRED = ("outline", "placement", "net_classes", "signal", "pour")


def _numbers(v: Any, n: int) -> bool:
    return (
        isinstance(v, list)
        and len(v) == n
        and all(isinstance(x, (int, float)) and not isinstance(x, bool) for x in v)
    )


def _positive(v: Any) -> bool:
    return isinstance(v, (int, float)) and not isinstance(v, bool) and v > 0


def expand_classes(
    cfg: paths.BoardConfig, nl: netlist.Netlist
) -> list[tuple[str, list[str]]]:
    """(class name, its nets) per `[[net_classes]]`, in file order.

    Each class lists, per pattern in order, the netlist's matching nets in name
    order, without repeats. Raises when a pattern matches nothing or a net is in
    two classes.
    """
    owner: dict[str, str] = {}
    out = []
    names = list(nl.nets)
    for cls in cfg.data.get("net_classes", []):
        nets: list[str] = []
        for pattern in cls.get("nets", []):
            hits = [n for n in names if fnmatch.fnmatchcase(n, pattern)]
            if not hits:
                raise KicadError(
                    f"{cfg.path} [[net_classes]] {cls.get('name')!r}: {pattern!r} matches no net"
                )
            for n in hits:
                if n in owner and owner[n] != cls.get("name"):
                    raise KicadError(
                        f"{cfg.path}: net {n!r} is in classes {owner[n]!r} and {cls.get('name')!r}"
                    )
                owner[n] = cls.get("name")
                if n not in nets:
                    nets.append(n)
        out.append((cls.get("name"), nets))
    return out


def validate_config(cfg: paths.BoardConfig, nl: netlist.Netlist) -> list[str]:
    """board.toml's layout tables against the netlist; [] when valid."""
    where = str(cfg.path)
    data = cfg.data
    errors = []
    for table in REQUIRED:
        if table not in data:
            errors.append(f"{where}: [{table}] is required")
    for table, allowed in TABLE_KEYS.items():
        entries = data.get(table, [] if table == "net_classes" else {})
        for entry in entries if isinstance(entries, list) else [entries]:
            if not isinstance(entry, dict):
                errors.append(f"{where}: [{table}] must be a table")
                continue
            unknown = sorted(set(entry) - allowed)
            if unknown:
                errors.append(
                    f"{where}: [{table}]: unknown key(s) {unknown}; allowed: {sorted(allowed)}"
                )
    if errors:
        return errors
    refs = {c.ref for c in nl.comps}
    outline = data["outline"]
    if not _numbers(outline.get("origin"), 2) or not _numbers(outline.get("size"), 2):
        errors.append(f"{where}: [outline] origin and size must be [x, y] numbers")
    placement = data["placement"]
    for ref, at in placement.items():
        if not _numbers(at, 3):
            errors.append(
                f"{where}: [placement] {ref} must be [x_mm, y_mm, rotation_deg]"
            )
    for ref in sorted(refs - set(placement), key=netlist.ref_key):
        errors.append(f"{where}: [placement] has no entry for {ref}")
    for ref in sorted(set(placement) - refs):
        errors.append(f"{where}: [placement] {ref} is not in the netlist")
    silk = data.get("silk", {})
    for ref in silk.get("hidden_refs", []):
        if ref not in refs:
            errors.append(f"{where}: [silk] hidden_refs: {ref!r} is not in the netlist")
    for key, n in (("ref_positions", 2), ("ref_sizes", 0)):
        for ref, v in silk.get(key, {}).items():
            if ref not in refs:
                errors.append(f"{where}: [silk] {key}: {ref!r} is not in the netlist")
            elif not (_numbers(v, n) if n else _positive(v)):
                errors.append(f"{where}: [silk] {key}: {ref} has a bad value {v!r}")
    for rect in silk.get("rects", []):
        if not _numbers(rect, 4):
            errors.append(f"{where}: [silk] rects: {rect!r} must be [x0, y0, x1, y1]")
    for text in silk.get("texts", []):
        if (
            not isinstance(text, dict)
            or set(text) != TEXT_KEYS
            or not isinstance(text["text"], str)
            or not _numbers(text["at"], 2)
            or not _positive(text["size"])
        ):
            errors.append(
                f"{where}: [silk] texts: {text!r} must be {{text, at = [x, y], size}}"
            )
    classes = data["net_classes"]
    for cls in [*classes, data["signal"]]:
        name = cls.get("name", "signal")
        if cls is not data["signal"] and (
            not isinstance(name, str)
            or not isinstance(cls.get("nets"), list)
            or not cls["nets"]
        ):
            errors.append(
                f"{where}: [[net_classes]] needs a name and a non-empty nets list"
            )
        if not all(_positive(cls.get(k)) for k in ("track_um", "clearance_um")):
            errors.append(
                f"{where}: net class {name!r}: track_um and clearance_um must be > 0"
            )
        if not _numbers(cls.get("via_um"), 2):
            errors.append(
                f"{where}: net class {name!r}: via_um must be [diameter, drill]"
            )
    names = [c.get("name") for c in classes]
    if len(set(names)) != len(names):
        errors.append(f"{where}: [[net_classes]] names repeat: {names}")
    if errors:
        return errors
    try:
        expand_classes(cfg, nl)
    except KicadError as e:
        errors.append(str(e))
    pour = data["pour"]
    if pour.get("net") not in nl.nets:
        errors.append(f"{where}: [pour] net {pour.get('net')!r} is not in the netlist")
    if not isinstance(pour.get("layers"), list) or not pour["layers"]:
        errors.append(f"{where}: [pour] layers must be a non-empty list")
    for k in ("clearance_mm", "min_thickness_mm", "thermal_gap_mm", "spoke_mm"):
        if not _positive(pour.get(k)):
            errors.append(f"{where}: [pour] {k} must be > 0")
    return errors


def board_path(project: Path) -> Path:
    cfg = paths.load_board_config(project)
    return paths.carrier_dir(project) / cfg.name / f"{cfg.name}.kicad_pcb"


def _pad_key(n: str) -> tuple[int, int | str]:
    return (0, int(n)) if n.isdigit() else (1, n)


def main(argv: list[str] | None = None, repo_root: Path = REPO_ROOT) -> int:
    parser = argparse.ArgumentParser(
        prog="kicad.boardcheck", description=__doc__.split("\n")[0]
    )
    paths.add_project_args(parser)
    parser.add_argument(
        "--relink",
        action="store_true",
        help="rewrite the footprint paths to the schematic's symbol UUIDs first",
    )
    args = parser.parse_args(argv)
    vendored = repo_root / library.VENDORED.relative_to(REPO_ROOT)
    failed = False
    try:
        for project in paths.select_projects(args.projects, args.allow_none, repo_root):
            cfg = paths.load_board_config(project)
            nl = netlist.build(project, repo_root, vendored)
            ns = schematic.namespace(schematic.project_key(project, repo_root))
            pcb = board_path(project)
            if not pcb.is_file():
                raise KicadError(f"{pcb}: no board; {hint(project.resolve().name)}")
            text = pcb.read_text(encoding="utf-8")
            if args.relink:
                text, n = relink(text, ns)
                if n:
                    pcb.write_text(text, encoding="utf-8")
                print(f"relinked {n} footprint path(s) in {pcb.name}")
            errors = validate_config(cfg, nl) + check(text, nl, ns, vendored)
            for e in errors:
                print(f"FAIL: {e}")
            if errors:
                failed = True
                print(
                    f"BOARD FAIL: {pcb.name}: {len(errors)} difference(s) from the join"
                )
            else:
                print(
                    f"OK: {pcb.name}: {len(nl.comps)} footprints match the join "
                    "(refs, footprints, values, LCSC, paths, pad nets)"
                )
    except KicadError as e:
        print(f"FAIL: {e}")
        return 1
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
