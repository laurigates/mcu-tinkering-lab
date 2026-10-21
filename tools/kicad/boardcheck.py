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
classes, pour, keepouts, cutouts), so a bad board.toml fails CI without KiCad.

Copper connectivity is DRC's job, locally, and the check stamp (`kicad.stamp`)
pins the board bytes DRC passed. Copper is read for one thing: no track, arc,
via or pour fill may enter a board.toml `[[keepouts]]` rectangle (the copper-free
band a carrier is sawn apart along, #750). The pcbnew scripts also give KiCad
and Freerouting the keepouts as rule areas; this check holds without trusting
either.

`--relink` rewrites each footprint's `(path ...)` to its uuid5 symbol UUID,
keyed by its Reference, and changes nothing else; DRC matches footprints to
symbols by reference, so this keeps the GUI's "Update PCB from Schematic" and
cross-probing consistent without re-placing or re-routing.
"""

from __future__ import annotations

import argparse
import fnmatch
import functools
import math
import re
import sys
import uuid
from collections.abc import Callable
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

from . import library, netlist, panel, paths, schematic, sexpr
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
    *,
    module: str | Callable[[], str],
    skip: frozenset[str] = frozenset(),
) -> list[str]:
    """Differences between a board's footprints and the netlist; [] when equal.

    `module` is the project's just module (`paths.just_module`), for the hint,
    or a function returning it, called only when there is a difference.
    `skip` names board-only footprints checked elsewhere (the panel's tabs).
    """
    want_net = {node: name for name, nodes in nl.nets.items() for node in nodes}
    nc_net = {node: name for name, node in netlist.unconnected(nl, vendored).items()}
    comps = {c.ref: c for c in nl.comps}
    errors = []
    seen: set[str] = set()
    for _, _, block in footprint_blocks(board_text):
        fp = parse_footprint(block)
        ref = fp.ref
        if ref in skip:
            continue
        if ref in seen:
            errors.append(f"{ref}: on the board twice")
            continue
        seen.add(ref)
        comp = comps.get(ref)
        if comp is None:
            errors.append(f"{ref}: on the board, not in the netlist")
            continue
        if fp.fpid != comp.footprint:
            errors.append(
                f"{ref}: footprint {fp.fpid!r}, netlist says {comp.footprint!r}"
            )
        if fp.value != comp.value:
            errors.append(f"{ref}: value {fp.value!r}, netlist says {comp.value!r}")
        if fp.lcsc != comp.lcsc:
            errors.append(f"{ref}: LCSC {fp.lcsc!r}, netlist says {comp.lcsc!r}")
        want_path = symbol_path(ns, ref)
        if fp.path != want_path:
            errors.append(
                f"{ref}: path {fp.path!r}, the schematic symbol is {want_path!r}"
            )
        for num in sorted(fp.pads, key=_pad_key):
            if not num:
                continue  # mounting pads, connector shells
            node = (ref, num)
            for got in sorted(fp.pads[num]):
                if node in want_net:
                    if got != want_net[node]:
                        errors.append(
                            f"{ref}.{num}: pad on {got!r}, netlist says {want_net[node]!r}"
                        )
                elif node in nc_net:
                    if got != nc_net[node]:
                        errors.append(
                            f"{ref}.{num}: no-connect pad on {got!r}, KiCad names its net "
                            f"{nc_net[node]!r}"
                        )
                elif got:
                    errors.append(f"{ref}.{num}: pad on {got!r}, netlist has no net")
    for ref in comps:
        if ref not in seen:
            errors.append(f"{ref}: in the netlist, not on the board")
    if not errors:
        return []
    tail = f"; {hint(module() if callable(module) else module)}"
    return [e + tail for e in errors]


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
    "silk": {
        "hidden_refs",
        "ref_positions",
        "ref_sizes",
        "rects",
        "texts",
        "cut_lines",
    },
    "net_classes": {"name", "nets", "track_um", "clearance_um", "via_um"},
    "signal": {"track_um", "clearance_um", "via_um"},
    "keepouts": {"name", "rect"},
    "route_keepouts": {"name", "layer", "rect"},
    "tracks": {"name", "net", "layer", "width_mm", "points"},
    "fanout": {
        "nets",
        "via_um",
        "track_um",
        "reach_mm",
        "in_pad",
        "exempt",
        "required",
        "own",
    },
    "cutouts": {"name", "center", "diameter"},
    "pour": {
        "net",
        "regions",
        "layers",
        "clearance_mm",
        "min_thickness_mm",
        "thermal_gap_mm",
        "spoke_mm",
    },
}
TEXT_KEYS = {"text", "at", "size"}
REGION_KEYS = {"net", "rect", "layers", "priority"}
# How far a pour stops short of a [silk] cut line: the hand saw's kerf and the
# board's flex while sawing must not reach copper (the integrated carrier's
# pre-fab review found GND fill at 0.0 mm).
CUT_CLEARANCE_MM = 1.0
# JLCPCB's legible silkscreen is 1.0 mm tall; 0.8 mm is the floor where a
# label does not fit at 1.0 (build_board.py keeps the stroke at 0.16 mm or
# more at any size).
SILK_MIN_MM = 0.8
LIST_TABLES = ("net_classes", "keepouts", "route_keepouts", "cutouts", "tracks")
REQUIRED = ("outline", "placement", "net_classes", "signal", "pour")


def _numbers(v: Any, n: int) -> bool:
    return (
        isinstance(v, list)
        and len(v) == n
        and all(isinstance(x, (int, float)) and not isinstance(x, bool) for x in v)
    )


def _positive(v: Any) -> bool:
    return isinstance(v, (int, float)) and not isinstance(v, bool) and v > 0


def _rect(v: Any) -> bool:
    """[x0, y0, x1, y1] numbers with x0 < x1 and y0 < y1."""
    return _numbers(v, 4) and v[0] < v[2] and v[1] < v[3]


def _inside(rect: list[float], size: list[float]) -> bool:
    """Whether `rect` (or a line's two ends) lies in the outline, origin-relative."""
    return all(0 <= x <= size[0] for x in rect[0::2]) and all(
        0 <= y <= size[1] for y in rect[1::2]
    )


def _overlap(a: list[float], b: list[float]) -> bool:
    """Whether two rectangles share interior (touching edges do not count)."""
    return a[0] < b[2] and b[0] < a[2] and a[1] < b[3] and b[1] < a[3]


def pour_regions(pour: dict[str, Any]) -> list[tuple[str, list[float] | None]]:
    """(net, rect) per pour region; a single-net [pour] is (net, None): the whole board."""
    return [(r["net"], r["rect"]) for r in pour_region_specs(pour)]


def pour_region_specs(pour: dict[str, Any]) -> list[dict[str, Any]]:
    """Each pour region as {net, rect} plus its own `layers` and `priority` if set.

    A single-net [pour] is one region with rect None, the whole board.
    """
    if "regions" not in pour:
        return [{"net": pour["net"], "rect": None}]
    out = []
    for r in pour["regions"]:
        spec = {"net": r["net"], "rect": list(r["rect"])}
        for key in ("layers", "priority"):
            if key in r:
                spec[key] = list(r[key]) if key == "layers" else r[key]
        out.append(spec)
    return out


def _validate_keepouts(where: str, data: dict[str, Any], size: Any) -> list[str]:
    errors = []
    names = []
    for k in data.get("keepouts", []):
        name = k.get("name")
        if not isinstance(name, str) or not name:
            errors.append(f"{where}: [keepouts] needs a name for each rectangle")
            continue
        names.append(name)
        if not _rect(k.get("rect")):
            errors.append(
                f"{where}: [keepouts] {name!r}: rect must be [x0, y0, x1, y1], "
                "x0 < x1 and y0 < y1"
            )
        elif size is not None and not _inside(k["rect"], size):
            errors.append(
                f"{where}: [keepouts] {name!r}: rect {k['rect']} is not inside [outline]"
            )
    if len(set(names)) != len(names):
        errors.append(f"{where}: [keepouts] names repeat: {names}")
    return errors


def _validate_route_keepouts(where: str, data: dict[str, Any], size: Any) -> list[str]:
    """[[route_keepouts]]: rectangles Freerouting may not route through, on one
    copper layer. Unlike [[keepouts]] they put no rule area on the board, so
    footprints and the pour may still go there (route_board.py, dsn.py)."""
    errors = []
    names = []
    for k in data.get("route_keepouts", []):
        name = k.get("name")
        if not isinstance(name, str) or not name:
            errors.append(f"{where}: [route_keepouts] needs a name for each rectangle")
            continue
        names.append(name)
        if k.get("layer") not in ("F.Cu", "B.Cu"):
            errors.append(
                f"{where}: [route_keepouts] {name!r}: layer must be F.Cu or B.Cu"
            )
        if not _rect(k.get("rect")):
            errors.append(
                f"{where}: [route_keepouts] {name!r}: rect must be [x0, y0, x1, y1], "
                "x0 < x1 and y0 < y1"
            )
        elif size is not None and not _inside(k["rect"], size):
            errors.append(
                f"{where}: [route_keepouts] {name!r}: rect {k['rect']} is not inside [outline]"
            )
    if len(set(names)) != len(names):
        errors.append(f"{where}: [route_keepouts] names repeat: {names}")
    return errors


def _validate_panel(where: str, data: dict[str, Any]) -> list[str]:
    """[panel] (#809): its values, then the slots and tabs it makes."""
    errors = panel.validate(where, data)
    if not errors:
        try:
            panel.build(data)
        except ValueError as e:
            errors = [f"{where}: [panel] {line}" for line in str(e).splitlines()]
    return errors


def _validate_cutouts(where: str, data: dict[str, Any], size: Any) -> list[str]:
    """[[cutouts]]: named circles, each wholly inside the outline."""
    errors = []
    names = []
    for c in data.get("cutouts", []):
        name = c.get("name")
        if not isinstance(name, str) or not name:
            errors.append(f"{where}: [cutouts] needs a name for each circle")
            continue
        names.append(name)
        center, diameter = c.get("center"), c.get("diameter")
        if not _numbers(center, 2) or not _positive(diameter):
            errors.append(
                f"{where}: [cutouts] {name!r}: needs center = [x, y] and diameter > 0"
            )
            continue
        # Strictly inside: a circle touching the outline would open it.
        r = diameter / 2
        if size is not None and not all(
            r < c < extent - r for c, extent in zip(center, size, strict=True)
        ):
            errors.append(
                f"{where}: [cutouts] {name!r}: circle at {center}, diameter "
                f"{diameter}, is not inside [outline]"
            )
    if len(set(names)) != len(names):
        errors.append(f"{where}: [cutouts] names repeat: {names}")
    return errors


def _validate_pour(
    where: str, pour: dict[str, Any], nl: netlist.Netlist, size: Any
) -> list[str]:
    errors = []
    if "net" in pour and "regions" in pour:
        return [f"{where}: [pour] takes net or regions, not both"]
    if "net" not in pour and "regions" not in pour:
        return [f"{where}: [pour] needs net or regions"]
    if "net" in pour:
        if pour["net"] not in nl.nets:
            errors.append(f"{where}: [pour] net {pour['net']!r} is not in the netlist")
        return errors
    regions = pour["regions"]
    if not isinstance(regions, list) or not regions:
        return [f"{where}: [pour] regions must be a non-empty list"]
    board_layers = pour.get("layers", [])
    placed = []  # (rect, layers, priority)
    for r in regions:
        if not isinstance(r, dict):
            errors.append(f"{where}: [pour] regions: {r!r} must be {{net, rect}}")
            continue
        unknown = sorted(set(r) - REGION_KEYS)
        if unknown:
            errors.append(f"{where}: [pour] regions: unknown key(s) {unknown}")
            continue
        if r.get("net") not in nl.nets:
            errors.append(
                f"{where}: [pour] regions: net {r.get('net')!r} is not in the netlist"
            )
        rect = r.get("rect")
        if not _rect(rect):
            errors.append(
                f"{where}: [pour] regions: rect {rect!r} must be [x0, y0, x1, y1], "
                "x0 < x1 and y0 < y1"
            )
        elif size is not None and not _inside(rect, size):
            errors.append(
                f"{where}: [pour] regions: rect {rect} is not inside [outline]"
            )
            continue
        layers = r.get("layers", board_layers)
        if not isinstance(layers, list) or not layers:
            errors.append(f"{where}: [pour] regions: layers must be a non-empty list")
            continue
        stray = [layer for layer in layers if layer not in board_layers]
        if stray:
            errors.append(
                f"{where}: [pour] regions: layers {stray} are not in [pour] layers"
            )
            continue
        priority = r.get("priority")
        if priority is not None and (
            not isinstance(priority, int) or isinstance(priority, bool) or priority < 0
        ):
            errors.append(
                f"{where}: [pour] regions: priority must be an integer >= 0, "
                f"got {priority!r}"
            )
            continue
        if _rect(rect):
            placed.append((rect, layers, priority))
    # Two regions on a shared layer may overlap only when both carry different
    # priorities: KiCad fills the higher one first and the lower around it.
    for i, (a, la, pa) in enumerate(placed):
        for b, lb, pb in placed[i + 1 :]:
            shared = [layer for layer in la if layer in lb]
            ranked = pa is not None and pb is not None and pa != pb
            if shared and not ranked and _overlap(a, b):
                errors.append(
                    f"{where}: [pour] regions: {a} and {b} overlap on {shared[0]}"
                )
    return errors


TRACK_LAYERS = ("F.Cu", "B.Cu")


def _validate_tracks(
    where: str, data: dict[str, Any], nl: netlist.Netlist, size: Any
) -> list[str]:
    """[[tracks]]: pre-routed tracks, each a polyline on one copper layer.

    build_board.py lays them down locked, Freerouting routes around them, and
    the zone fill joins them. A track may not enter a keepout.
    """
    errors = []
    for tr in data.get("tracks", []):
        name = tr.get("name")
        at = f"{where}: [[tracks]] {name!r}"
        if not isinstance(name, str) or not name:
            errors.append(f"{where}: [[tracks]] needs a name")
            continue
        if tr.get("net") not in nl.nets:
            errors.append(f"{at}: net {tr.get('net')!r} is not in the netlist")
        if tr.get("layer") not in TRACK_LAYERS:
            errors.append(f"{at}: layer must be F.Cu or B.Cu, got {tr.get('layer')!r}")
        width = tr.get("width_mm")
        if not _positive(width):
            errors.append(f"{at}: width_mm must be > 0")
            continue
        pts = tr.get("points")
        if (
            not isinstance(pts, list)
            or len(pts) < 2
            or not all(_numbers(p, 2) for p in pts)
        ):
            errors.append(f"{at}: points must be two or more [x, y]")
            continue
        outside = [p for p in pts if size is not None and not _inside(p + p, size)]
        if outside:
            errors.append(f"{at}: point {outside[0]} is not inside [outline]")
            continue
        for a, b in zip(pts, pts[1:]):
            for k in data.get("keepouts", []):
                if _segment_rect(tuple(a), tuple(b), tuple(k["rect"])) < width / 2:
                    errors.append(f"{at}: {a} to {b} enters keepout {k['name']!r}")
    return errors


def _validate_cut_clearance(where: str, data: dict[str, Any], size: Any) -> list[str]:
    """Each [pour] region stops CUT_CLEARANCE_MM short of every [silk] cut line."""
    lines = [
        tuple(line)
        for line in data.get("silk", {}).get("cut_lines", [])
        if _numbers(line, 4)
    ]
    pour = data.get("pour", {})
    if not lines or not isinstance(pour, dict) or not _numbers(size, 2):
        return []
    errors = []
    try:
        regions = pour_region_specs(pour)
    except (KeyError, TypeError):
        return []  # _validate_pour reports a malformed [pour]
    for region in regions:
        rect = region.get("rect")
        box = [0.0, 0.0, size[0], size[1]] if rect is None else rect
        if not _numbers(box, 4):
            continue
        for line in lines:
            d = _segment_rect(line[:2], line[2:], tuple(box))
            if d < CUT_CLEARANCE_MM - 1e-9:
                errors.append(
                    f"{where}: [pour] regions: {rect!r} is {d:.1f} mm from cut line "
                    f"{list(line)!r}; keep {CUT_CLEARANCE_MM} mm"
                )
    return errors


FANOUT_DEFAULTS = {
    "track_um": 300,
    "reach_mm": 2.0,
    "in_pad": [],
    "exempt": [],
    "required": [],
    "own": [],
}


def fanout_spec(data: dict[str, Any]) -> dict[str, Any] | None:
    """[fanout] with its defaults filled in, or None when the table is absent."""
    fan = data.get("fanout")
    if fan is None:
        return None
    return {**FANOUT_DEFAULTS, **fan}


def _validate_fanout(
    where: str, data: dict[str, Any], nl: netlist.Netlist
) -> list[str]:
    """[fanout]: route_board.py puts a via beside every SMD pad of these nets.

    A poured net's SMD pads reach the other layer's pour only through a via;
    Freerouting adds none where a track joins the pads on one layer, and a
    pour on that layer can leave them on an island. The vias go in after
    routing, where there is room: a `required` pad (on a power path) fails the
    route without one, any other keeps its routed track. `in_pad` pads (an
    exposed pad) take their via inside the pad; `exempt` pads take none. An
    `own` pad takes a via of its own even where a chain of fixed tracks already
    reaches one (a regulator's GND pin, whose fixed track to its input
    capacitor would otherwise satisfy it).
    """
    fan = data.get("fanout")
    if fan is None:
        return []
    at = f"{where}: [fanout]"
    errors = []
    nets = fan.get("nets")
    if not isinstance(nets, list) or not nets:
        return [f"{at} nets must be a non-empty list"]
    for net in nets:
        if net not in nl.nets:
            errors.append(f"{at} net {net!r} is not in the netlist")
    via = fan.get("via_um")
    if not (_numbers(via, 2) and via[0] > via[1] > 0):
        errors.append(
            f"{at} via_um must be [diameter, drill] with diameter > drill > 0"
        )
    spec = fanout_spec(data)
    for key in ("track_um", "reach_mm"):
        if not _positive(spec[key]):
            errors.append(f"{at} {key} must be > 0")
    pads = {f"{ref}:{pin}" for net in nets for ref, pin in nl.nets.get(net, ())}
    for key in ("in_pad", "exempt", "required", "own"):
        for pad in spec[key]:
            if pad not in pads:
                errors.append(f"{at} {key} {pad!r} is not a pad of a [fanout] net")
    return errors


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
        entries = data.get(table, [] if table in LIST_TABLES else {})
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
    size = None
    if not _numbers(outline.get("origin"), 2) or not _numbers(outline.get("size"), 2):
        errors.append(f"{where}: [outline] origin and size must be [x, y] numbers")
    else:
        size = outline["size"]
    placement = data["placement"]
    for ref, at in placement.items():
        side_ok = isinstance(at, list) and (len(at) == 3 or at[3:] == ["back"])
        if not side_ok or not _numbers(at[:3], 3):
            errors.append(
                f"{where}: [placement] {ref} must be [x_mm, y_mm, rotation_deg]"
                ' or [x_mm, y_mm, rotation_deg, "back"]'
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
    for line in silk.get("cut_lines", []):
        if not _numbers(line, 4):
            errors.append(
                f"{where}: [silk] cut_lines: {line!r} must be [x0, y0, x1, y1]"
            )
        elif line[:2] == line[2:]:
            errors.append(f"{where}: [silk] cut_lines: {line!r} has no length")
        elif size is not None and not _inside(line, size):
            errors.append(
                f"{where}: [silk] cut_lines: {line!r} is not inside [outline]"
            )
    for key in ("ref_sizes",):
        for ref, v in silk.get(key, {}).items():
            if _positive(v) and v < SILK_MIN_MM:
                errors.append(
                    f"{where}: [silk] {key}: {ref} is {v} mm tall; the floor is {SILK_MIN_MM} mm"
                )
    for text in silk.get("texts", []):
        if isinstance(text, dict) and _positive(text.get("size")):
            if text["size"] < SILK_MIN_MM:
                errors.append(
                    f"{where}: [silk] texts: {text.get('text')!r} is {text['size']} mm "
                    f"tall; the floor is {SILK_MIN_MM} mm"
                )
    errors += _validate_cut_clearance(where, data, size)
    errors += _validate_keepouts(where, data, size)
    errors += _validate_route_keepouts(where, data, size)
    errors += _validate_panel(where, data)
    errors += _validate_cutouts(where, data, size)
    errors += _validate_tracks(where, data, nl, size)
    errors += _validate_fanout(where, data, nl)
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
    errors += _validate_pour(where, pour, nl, size)
    if not isinstance(pour.get("layers"), list) or not pour["layers"]:
        errors.append(f"{where}: [pour] layers must be a non-empty list")
    for k in ("clearance_mm", "min_thickness_mm", "thermal_gap_mm", "spoke_mm"):
        if not _positive(pour.get(k)):
            errors.append(f"{where}: [pour] {k} must be > 0")
    return errors


Point = tuple[float, float]


def _point_in_rect(p: Point, r: tuple[float, float, float, float]) -> bool:
    return r[0] <= p[0] <= r[2] and r[1] <= p[1] <= r[3]


def _clip(a: Point, b: Point, r: tuple[float, float, float, float]) -> bool:
    """Whether segment a-b meets rectangle r (Liang-Barsky)."""
    dx, dy = b[0] - a[0], b[1] - a[1]
    t0, t1 = 0.0, 1.0
    for p, q in (
        (-dx, a[0] - r[0]),
        (dx, r[2] - a[0]),
        (-dy, a[1] - r[1]),
        (dy, r[3] - a[1]),
    ):
        if p == 0:
            if q < 0:
                return False
            continue
        t = q / p
        if p < 0:
            t0 = max(t0, t)
        else:
            t1 = min(t1, t)
        if t0 > t1:
            return False
    return True


def _point_segment(p: Point, a: Point, b: Point) -> float:
    dx, dy = b[0] - a[0], b[1] - a[1]
    n = dx * dx + dy * dy
    t = (
        0.0
        if n == 0
        else max(0.0, min(1.0, ((p[0] - a[0]) * dx + (p[1] - a[1]) * dy) / n))
    )
    return math.hypot(p[0] - a[0] - t * dx, p[1] - a[1] - t * dy)


def _point_rect(p: Point, r: tuple[float, float, float, float]) -> float:
    dx = max(r[0] - p[0], 0.0, p[0] - r[2])
    dy = max(r[1] - p[1], 0.0, p[1] - r[3])
    return math.hypot(dx, dy)


def _segment_rect(a: Point, b: Point, r: tuple[float, float, float, float]) -> float:
    """Distance from segment a-b to rectangle r; 0 when they meet."""
    if _clip(a, b, r):
        return 0.0
    corners = [(r[0], r[1]), (r[2], r[1]), (r[2], r[3]), (r[0], r[3])]
    return min(
        _point_rect(a, r),
        _point_rect(b, r),
        *(_point_segment(c, a, b) for c in corners),
    )


def _in_polygon(p: Point, poly: list[Point]) -> bool:
    inside = False
    for (x1, y1), (x2, y2) in zip(poly, poly[1:] + poly[:1], strict=True):
        if (y1 > p[1]) != (y2 > p[1]):
            if p[0] < x1 + (p[1] - y1) * (x2 - x1) / (y2 - y1):
                inside = not inside
    return inside


def _arc_points(s: Point, m: Point, e: Point, n: int = 32) -> list[Point]:
    """The arc s -> m -> e as n + 1 points along it."""
    ax, ay = s
    bx, by = m
    cx, cy = e
    d = 2 * (ax * (by - cy) + bx * (cy - ay) + cx * (ay - by))
    if abs(d) < 1e-12:
        return [s, m, e]
    ux = (
        (ax * ax + ay * ay) * (by - cy)
        + (bx * bx + by * by) * (cy - ay)
        + (cx * cx + cy * cy) * (ay - by)
    ) / d
    uy = (
        (ax * ax + ay * ay) * (cx - bx)
        + (bx * bx + by * by) * (ax - cx)
        + (cx * cx + cy * cy) * (bx - ax)
    ) / d
    radius = math.hypot(ax - ux, ay - uy)

    def ang(p: Point) -> float:
        return math.atan2(p[1] - uy, p[0] - ux)

    a0, am, a1 = ang(s), ang(m), ang(e)
    sweep = (a1 - a0) % (2 * math.pi)
    if (am - a0) % (2 * math.pi) > sweep:  # m is not on the counter-clockwise way
        sweep -= 2 * math.pi
    return [
        (
            ux + radius * math.cos(a0 + sweep * i / n),
            uy + radius * math.sin(a0 + sweep * i / n),
        )
        for i in range(n + 1)
    ]


def _xy(block: str, keyword: str) -> Point:
    c = sexpr.child(block, keyword)
    if c is None:
        raise KicadError(f"{sexpr.head(block)[0]} without ({keyword} ...)")
    x, y = (float(v) for v in sexpr.atoms(c)[:2])
    return x, y


def _number(block: str, keyword: str) -> float:
    c = sexpr.child(block, keyword)
    if c is None:
        raise KicadError(f"{sexpr.head(block)[0]} without ({keyword} ...)")
    return float(sexpr.atoms(c)[0])


def _net(block: str) -> str:
    c = sexpr.child(block, "net")
    return (sexpr.head(c)[1] or "") if c is not None else ""


# Copper may touch a keepout's edge (KiCad fills a zone up to a rule area's
# edge), not enter it: each keepout is shrunk by this much before the test.
KEEPOUT_TOLERANCE = 0.001


def keepout_copper(
    board_text: str, keepouts: list[dict[str, Any]], origin: list[float] | tuple
) -> list[str]:
    """Every track, arc, via and pour fill that enters a keepout; [] when none.

    `keepouts` are board.toml `[[keepouts]]` (`rect` relative to `origin`, the
    outline's top-left corner, as is every position in a message).
    """
    if not keepouts:
        return []
    ox, oy = origin
    t = KEEPOUT_TOLERANCE
    boxes = [
        (
            k["name"],
            (
                ox + k["rect"][0] + t,
                oy + k["rect"][1] + t,
                ox + k["rect"][2] - t,
                oy + k["rect"][3] - t,
            ),
        )
        for k in keepouts
    ]

    def rel(p: Point) -> str:
        return f"({_fmt(p[0] - ox)}, {_fmt(p[1] - oy)})"

    errors = []
    for _, block in sexpr.top_level_blocks(board_text, "\t"):
        kw = sexpr.head(block)[0]
        hits: list[tuple[str, str]] = []  # (item, keepout name)
        if kw in ("segment", "arc"):
            half = _number(block, "width") / 2
            if kw == "segment":
                pts = [_xy(block, "start"), _xy(block, "end")]
            else:
                pts = _arc_points(*(_xy(block, k) for k in ("start", "mid", "end")))
            for name, box in boxes:
                if any(
                    _segment_rect(a, b, box) < half
                    for a, b in zip(pts, pts[1:], strict=False)
                ):
                    hits.append(
                        (f"{kw} on {_net(block)!r} {rel(pts[0])}-{rel(pts[-1])}", name)
                    )
        elif kw == "via":
            at = _xy(block, "at")
            radius = _number(block, "size") / 2
            for name, box in boxes:
                if _point_rect(at, box) < radius:
                    hits.append((f"via on {_net(block)!r} at {rel(at)}", name))
        elif kw == "zone":
            for fill in sexpr.children(block):
                if sexpr.head(fill)[0] != "filled_polygon":
                    continue
                layer_c = sexpr.child(fill, "layer")
                layer = (sexpr.head(layer_c)[1] or "") if layer_c is not None else ""
                if not layer.endswith(".Cu"):
                    continue
                pts_c = sexpr.child(fill, "pts")
                poly = [
                    (float(a), float(b))
                    for a, b in (sexpr.atoms(xy)[:2] for xy in sexpr.children(pts_c))
                ]
                for name, box in boxes:
                    centre = ((box[0] + box[2]) / 2, (box[1] + box[3]) / 2)
                    if _in_polygon(centre, poly) or any(
                        _clip(a, b, box)
                        for a, b in zip(poly, poly[1:] + poly[:1], strict=True)
                    ):
                        hits.append((f"pour on {_net(block)!r} ({layer})", name))
        for item, name in hits:
            errors.append(f"{item} enters keepout {name!r}")
    return errors


# How far a hole or slot corner on the board may sit from where board.toml puts it.
PANEL_TOLERANCE_MM = 0.002


def panel_board(board_text: str, data: dict[str, Any]) -> list[str]:
    """The board's mouse-bite tabs and Edge.Cuts slots against board.toml `[panel]`.

    Each tab is a board-only footprint (MB1, MB2, ...) of non-plated holes; each
    slot an Edge.Cuts rectangle inside the outline. In saw mode there must be
    neither. [] when they match; positions relative to the outline origin.
    """
    want = panel.build(data)
    ox, oy = data["outline"]["origin"]
    w, h = data["outline"]["size"]
    tol = PANEL_TOLERANCE_MM
    errors = []

    def rel(x: float, y: float) -> Point:
        return (round(x - ox, 4), round(y - oy, 4))

    got_tabs: dict[str, list[Point]] = {}
    got_drill: set[float] = set()
    for _, _, block in footprint_blocks(board_text):
        fp = parse_footprint(block)
        if not re.fullmatch(rf"{panel.FOOTPRINT_PREFIX}\d+", fp.ref):
            continue
        at = sexpr.atoms(sexpr.child(block, "at") or "(at 0 0)")
        fx, fy = float(at[0]), float(at[1])
        if len(at) > 2 and float(at[2]) % 360:
            errors.append(
                f"{fp.ref}: rotated {at[2]} degrees; tabs are placed unrotated"
            )
        attr = sexpr.child(block, "attr")
        if attr is None or "board_only" not in sexpr.atoms(attr):
            errors.append(f"{fp.ref}: not board-only, so not a mouse-bite tab")
        holes = got_tabs.setdefault(fp.ref, [])
        for c in sexpr.children(block):
            if sexpr.head(c)[0] != "pad":
                continue
            kinds = sexpr.atoms(c)
            pat = sexpr.atoms(sexpr.child(c, "at") or "(at 0 0)")
            drill = sexpr.child(c, "drill")
            if kinds[:1] != ["np_thru_hole"] or drill is None:
                errors.append(f"{fp.ref}: a pad that is not a non-plated hole")
                continue
            got_drill.add(round(float(sexpr.atoms(drill)[0]), 4))
            holes.append(rel(fx + float(pat[0]), fy + float(pat[1])))
    want_tabs = {t.name: t.holes for t in want.tabs} if want else {}
    for name in sorted(set(got_tabs) | set(want_tabs), key=netlist.ref_key):
        if name not in want_tabs:
            errors.append(f"{name}: a mouse-bite tab on the board, none in [panel]")
        elif name not in got_tabs:
            errors.append(f"{name}: a mouse-bite tab in [panel], not on the board")
        else:
            a, b = sorted(got_tabs[name]), sorted(want_tabs[name])
            if len(a) != len(b) or any(
                abs(p[0] - q[0]) > tol or abs(p[1] - q[1]) > tol for p, q in zip(a, b)
            ):
                errors.append(f"{name}: holes {a}, [panel] says {list(b)}")
    if want and got_drill - {round(want.hole_mm, 4)}:
        errors.append(
            f"mouse-bite drill {sorted(got_drill)}, [panel] says {want.hole_mm}"
        )

    got_slots = []
    for _, block in sexpr.top_level_blocks(board_text, "\t"):
        if sexpr.head(block)[0] != "gr_rect":
            continue
        layer = sexpr.child(block, "layer")
        if layer is None or sexpr.head(layer)[1] != "Edge.Cuts":
            continue
        (x0, y0), (x1, y1) = _xy(block, "start"), _xy(block, "end")
        a, b = rel(min(x0, x1), min(y0, y1)), rel(max(x0, x1), max(y0, y1))
        r = (*a, *b)
        if all(abs(v - u) <= tol for v, u in zip(r, (0.0, 0.0, w, h))):
            continue  # the outline
        got_slots.append(r)
    want_slots = sorted(want.slots) if want else []
    got_slots.sort()
    if len(got_slots) != len(want_slots) or any(
        any(abs(v - u) > tol for v, u in zip(g, s))
        for g, s in zip(got_slots, want_slots)
    ):
        errors.append(
            f"Edge.Cuts slots {[list(r) for r in got_slots]}, [panel] says "
            f"{[list(r) for r in want_slots]}"
        )
    return errors


def _fmt(v: float) -> str:
    s = f"{round(v, 4):.4f}".rstrip("0").rstrip(".")
    return "0" if s in ("-0", "") else s


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
            if not cfg.has_board:
                print(paths.skip_line(project, repo_root))
                continue
            module = functools.partial(paths.just_module, project, repo_root)
            nl = netlist.build(project, repo_root, vendored)
            ns = schematic.namespace(schematic.project_key(project, repo_root))
            pcb = board_path(project)
            if not pcb.is_file():
                raise KicadError(f"{pcb}: no board; {hint(module())}")
            text = pcb.read_text(encoding="utf-8")
            if args.relink:
                text, n = relink(text, ns)
                if n:
                    pcb.write_text(text, encoding="utf-8")
                print(f"relinked {n} footprint path(s) in {pcb.name}")
            errors = validate_config(cfg, nl)
            tabs = panel.build(cfg.data) if not errors else None
            skip = frozenset(t.name for t in tabs.tabs) if tabs else frozenset()
            errors += check(text, nl, ns, vendored, module=module, skip=skip)
            if not errors:
                errors += [
                    f"{e}; {hint(module())}" for e in panel_board(text, cfg.data)
                ]
            if not errors:
                copper = keepout_copper(
                    text, cfg.data.get("keepouts", []), cfg.data["outline"]["origin"]
                )
                errors += [f"{e}; {hint(module())}" for e in copper]
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
