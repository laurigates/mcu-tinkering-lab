"""Parts on the hole grid, and the placement check.

`build_parts` places every netlist component that has pins, in netlist order
(the drawing's draw order depends on it), on its perfboard footprint: pads
snapped to holes, body box in hole units. `check_placement` then fails a pad
off the board, two pads in one hole, two raised modules overlapping, a tall
part under a raised one, and two low parts overlapping.
"""

from __future__ import annotations

from collections.abc import Callable, Mapping
from dataclasses import dataclass
from pathlib import Path

from .. import footprints
from ..errors import KicadError
from ..netlist import Netlist
from ..registry import PartSpec
from .config import PerfboardConfig
from .grid import PITCH, Board, hole_name, overlaps, rotate, to_hole


@dataclass
class Part:
    ref: str
    value: str
    fpid: str
    pads: dict  # pin number -> (col, row) absolute hole
    body: tuple  # (c0, r0, c1, r1) in hole units, component-side view


def pinless(nl: Netlist) -> frozenset[str]:
    """Refs on no net and with no no-connect pin: parts with no pins at all.

    `netlist.build` puts every symbol pin on a net or in `no_connect`, so a
    component in neither has none (a mounting hole).
    """
    used = {ref for nodes in nl.nets.values() for ref, _ in nodes}
    used |= {ref for ref, _ in nl.no_connect}
    return frozenset(c.ref for c in nl.comps if c.ref not in used)


def build_parts(
    nl: Netlist,
    cfg: PerfboardConfig,
    specs: Mapping[str, PartSpec],
    footprint_file: Callable[[str], Path],
    skip: frozenset[str],
) -> tuple[dict[str, Part], frozenset[str], frozenset[str]]:
    """(parts by ref in netlist order, raised refs, tall refs).

    `specs` maps a registry slug to its entry; `footprint_file` resolves a
    footprint id to its `.kicad_mod`. Each part's perfboard footprint must have
    a through-hole pad for every netlist pin of its ref, on a net or not
    connected: the registry checks its own `[perfboard]` footprint so, and a
    `[swaps.REF]` footprint replaces it.
    """
    pins = netlist_pins(nl)
    parts = {}
    raised, tall = set(), set()
    for comp in nl.comps:
        ref = comp.ref
        if ref in skip:
            continue
        if ref not in cfg.placement:
            raise KicadError(f"{cfg.path} [placement]: {ref} has no entry")
        pb = specs[comp.registry].perfboard
        swap = cfg.swaps.get(ref)
        fpid = (swap and swap.footprint) or pb.footprint
        value = (swap and swap.value) or pb.value or comp.value
        pads_mm, crt = footprints.thru_hole_pads(footprint_file(fpid))
        missing = sorted(pins.get(ref, set()) - set(pads_mm), key=_pin_key)
        if missing:
            where = (
                f"{cfg.path} [swaps.{ref}]"
                if swap and swap.footprint
                else f"{ref}: registry {comp.registry} [perfboard]"
            )
            raise KicadError(
                f"{where}: footprint {fpid} has no through-hole pad for "
                f"{ref}'s netlist pin(s) {missing}"
            )
        if "1" not in pads_mm:
            # The wire list gives each part's pin-1 hole; the drawing squares it.
            raise KicadError(
                f"{ref}: perfboard footprint {fpid} has no through-hole pad 1"
            )
        c, r, rot = cfg.placement[ref]
        pads = {}
        for num, (x, y) in pads_mm.items():
            rx, ry = rotate(x, y, rot)
            pads[num] = (
                c + to_hole(rx, f"{ref}.{num}"),
                r + to_hole(ry, f"{ref}.{num}"),
            )
        if pb.body is not None:
            corners = [
                (pb.body[0] * PITCH, pb.body[1] * PITCH),
                (pb.body[2] * PITCH, pb.body[3] * PITCH),
            ]
        else:
            corners = [(crt[0], crt[1]), (crt[2], crt[3])]
        pts = [rotate(x, y, rot) for x, y in corners]
        xs = [c + p[0] / PITCH for p in pts]
        ys = [r + p[1] / PITCH for p in pts]
        parts[ref] = Part(ref, value, fpid, pads, (min(xs), min(ys), max(xs), max(ys)))
        if pb.height == "raised":
            raised.add(ref)
        elif pb.height == "tall":
            tall.add(ref)
    missing = set(cfg.placement) - set(parts)
    if missing:
        raise KicadError(
            f"{cfg.path} [placement]: entries with no netlist part to place: {sorted(missing)}"
        )
    unknown = (set(cfg.swaps) | cfg.skip) - {c.ref for c in nl.comps}
    if unknown:
        raise KicadError(
            f"{cfg.path}: [swaps] or skip name refs not in the netlist: {sorted(unknown)}"
        )
    return parts, frozenset(raised), frozenset(tall)


def netlist_pins(nl: Netlist) -> dict[str, set[str]]:
    """Every pin of each ref in the netlist: on a net or not connected."""
    pins: dict[str, set[str]] = {}
    for ref, pin in [*(n for nodes in nl.nets.values() for n in nodes), *nl.no_connect]:
        pins.setdefault(ref, set()).add(pin)
    return pins


def _pin_key(pin: str) -> tuple[int, str]:
    return (int(pin), pin) if pin.isdigit() else (1 << 30, pin)


def nets_of(nl: Netlist, skip: frozenset[str]) -> dict[str, list[tuple[str, str]]]:
    """Every net with two or more nodes on placed parts, by name."""
    nets = {}
    for name, members in nl.nets.items():
        nodes = [(ref, pin) for ref, pin in members if ref not in skip]
        if len(nodes) > 1:
            nets[name] = nodes
    return nets


def check_placement(parts, board: Board, raised, tall):
    errors = []
    holes = {}
    for p in parts.values():
        for num, (c, r) in p.pads.items():
            if not (0 <= c < board.cols and 0 <= r < board.rows):
                errors.append(f"{p.ref}.{num} at {hole_name(c, r)} is off the board")
            if (c, r) in holes:
                errors.append(
                    f"{p.ref}.{num} and {holes[(c, r)]} share hole {hole_name(c, r)}"
                )
            holes[(c, r)] = f"{p.ref}.{num}"
    refs = sorted(parts)
    for i, a in enumerate(refs):
        for b in refs[i + 1 :]:
            if not overlaps(parts[a].body, parts[b].body):
                continue
            ra, rb = a in raised, b in raised
            if ra and rb:
                errors.append(f"raised modules {a} and {b} overlap")
            elif ra != rb and (a in tall or b in tall):
                errors.append(
                    f"tall part {a if a in tall else b} sits under raised {a if ra else b}"
                )
            elif not ra and not rb:
                errors.append(f"{a} and {b} overlap on the board")
    return errors
