"""The carrier netlist: the hardware join's carrier view plus the parts registry.

    PYTHONPATH=tools python3 -m kicad.netlist [project ...] [--allow-none]

`build(project)` reads `hardware.join(project, view=CARRIER)` and maps every
endpoint (`imu.SDA`, `mcu.D4`) through the part's registry entry to a
reference designator and symbol pin number. The nets it emits:

- each signal role: the MCU pad its GPIO lands on, plus every `[[nets]]` load,
  named by `[net_names]` or else the role;
- each `[[channel_nets]]` role: the driver pad named by the channel number;
- each `[[bus_nets]]` role: two nets, the mux's `SD<n>` with every load's `SDA`
  and `SC<n>` with every `SCL`, named `<name>_SDA` / `<name>_SCL` where
  `<name>` is the role or its `[net_names]` entry;
- each rail, named by `[net_names]` or else the rail name;
- each output pin: `net.format(pin=p)` joining the driver's pin p to pin p of
  every load, or with `load_pins` to the load pin each load maps p to (#769).

With a `[[cables]]` entry (#750) the MCU sits on a head board and reaches the
main board through a cable: two connectors, `head_ref` and `main_ref`, on one
registry entry whose `[pins]` are keyed by MCU pad name. Every MCU end above,
role or rail, is split at the cable: `HEAD_<name>` joins the MCU pad to the
head connector's pin(s) for that pad, and the net `<name>` takes the main
connector's pin(s) in the pad's place. An MCU pad on a net that the cable does
not carry raises, as does a cable pin named for no pad of the MCU's board page.
The cable's `head_no_connect` pads may float, power pads included.

Every symbol pin must then be on a net or allowed to float: listed under the
entry's `no_connect`, or a spare GPIO pad of a `pin_numbers` part. Anything
else raises, so a dropped wire cannot become a quietly open pin. A power pad
(5V, GND, 3V3) is never implied floating: every socket pin is `passive`, so
ERC cannot see an unpowered XIAO.

The ordering is KiCad's for balancebot: components by natural
reference, nets by name, nodes by natural reference then pin as a plain string.
A no-connect pin's net in KiCad is `nc_net_name()`.
"""

from __future__ import annotations

import argparse
import re
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Any

from hardware import CARRIER, MCU, Board, HardwareError, join, parse_board_table

from . import library, paths, registry, symbols
from .errors import KicadError
from .paths import REPO_ROOT

_REF = re.compile(r"(#?[A-Z]+)(\d+)")
# The two ends of a [[cables]] entry, as keys of the placed parts.
HEAD, MAIN = "cable.head", "cable.main"


@dataclass(frozen=True)
class Component:
    ref: str
    value: str
    footprint: str
    lib_id: str
    fields: tuple[
        tuple[str, str], ...
    ]  # (("LCSC", "C17414"),): custom fields, in order
    in_bom: bool  # from the symbol's library flags
    on_board: bool
    part_key: str  # hardware.toml part id, or "mcu"
    registry: str  # parts-registry slug

    @property
    def lcsc(self) -> str:
        return dict(self.fields).get("LCSC", "")


@dataclass(frozen=True)
class Netlist:
    project: Path
    comps: tuple[Component, ...]  # sorted by natural ref
    nets: dict[str, tuple[tuple[str, str], ...]]  # name -> ((ref, pin), ...), sorted
    no_connect: tuple[tuple[str, str], ...]  # sorted like nodes


def ref_key(ref: str) -> tuple[str, int]:
    """Natural sort key: "R2" < "R10"; "#FLG01" -> ("#FLG", 1)."""
    m = _REF.fullmatch(ref)
    if not m:
        raise KicadError(f"{ref!r}: not a reference designator (letters, then digits)")
    return m.group(1), int(m.group(2))


def node_key(node: tuple[str, str]) -> tuple[tuple[str, int], str]:
    return ref_key(node[0]), node[1]


def nc_net_name(ref: str, pin_name: str, pin: str) -> str:
    """KiCad's net for a no-connect pin: `unconnected-(<ref>-<name>-Pad<n>)`.

    The pin name is the symbol's, markup included (`~{FLT}`); an unnamed pin
    gives `unconnected-(<ref>-Pad<n>)`.
    """
    if pin_name:
        return f"unconnected-({ref}-{pin_name}-Pad{pin})"
    return f"unconnected-({ref}-Pad{pin})"


@dataclass(frozen=True)
class _Placed:
    """One part on the carrier: its component, registry entry and pin map."""

    key: str
    comp: Component
    spec: registry.PartSpec
    board: Board | None
    pins: list[symbols.Pin]
    by_name: dict[str, tuple[str, ...]]  # hardware.toml pin name -> symbol pin numbers


def build(
    project_dir: Path,
    repo_root: Path = REPO_ROOT,
    vendored: Path = library.VENDORED,
) -> Netlist:
    """The carrier netlist of `project_dir`; raises KicadError naming the input."""
    try:
        model = join(project_dir, repo_root, view=CARRIER)
    except HardwareError as e:
        raise KicadError(str(e)) from None
    sidecar = model.project_dir / "hardware.toml"
    specs: dict[str, registry.PartSpec] = {}

    def spec_of(slug: str) -> registry.PartSpec:
        if slug not in specs:
            spec = registry.load(slug, repo_root)
            registry.validate(spec, vendored)
            specs[slug] = spec
        return specs[slug]

    if not model.mcu_part or not model.mcu_ref:
        raise KicadError(
            f"{sidecar} [source]: the carrier needs `part` and `ref` for the MCU"
        )
    placed: dict[str, _Placed] = {}
    entries = [(MCU, model.mcu_part, model.mcu_ref, "", "", model.mcu, model.board)]
    cable = model.cables[0] if model.cables else None
    if cable is not None:
        name = spec_of(cable.part).name
        entries.append((HEAD, cable.part, cable.head_ref, "", "", name, None))
        entries.append((MAIN, cable.part, cable.main_ref, "", "", name, None))
    for key, part in model.parts.items():
        if not part.part or not part.ref:
            raise KicadError(
                f"{sidecar} [parts.{key}]: the carrier needs `part` and `ref`"
            )
        # Only a pin_numbers entry maps through its page's GPIO pin table; a
        # breakout's page (TB6612FNG, PCA9685, ...) documents headers and has none.
        board = None
        if part.board and spec_of(part.part).pin_numbers:
            try:
                board = parse_board_table(repo_root / part.board)
            except HardwareError as e:
                raise KicadError(f"{sidecar} [parts.{key}]: {e}") from None
        entries.append(
            (key, part.part, part.ref, part.value, part.lcsc, part.name, board)
        )
    for key, slug, ref, value, lcsc, name, board in entries:
        spec = spec_of(slug)
        block = symbols.load(spec.symbol, vendored=vendored)
        sym_pins = symbols.pins(block)
        flags = symbols.flags(block)
        lcsc = lcsc or spec.lcsc
        comp = Component(
            ref=ref,
            value=value or spec.value or name,
            footprint=spec.footprint,
            lib_id=spec.symbol,
            fields=(("LCSC", lcsc),) if lcsc else (),
            in_bom=flags["in_bom"],
            on_board=flags["on_board"],
            part_key=key,
            registry=slug,
        )
        by_name = registry.pin_map(spec, board, sym_pins)
        placed[key] = _Placed(key, comp, spec, board, sym_pins, by_name)

    def nodes(where: str, part: str, pin: str) -> list[tuple[str, str]]:
        p = placed[part]
        if pin not in p.by_name:
            raise KicadError(
                f"{sidecar} {where}: {part}.{pin}: {p.spec.path.name} has no pin {pin!r} "
                f"(pins: {', '.join(p.by_name)})"
            )
        return [(p.comp.ref, n) for n in p.by_name[pin]]

    if cable is not None:
        carried = placed[HEAD].by_name
        cable_file = placed[HEAD].spec.path.name
        stray = [pad for pad in carried if pad not in model.board.by_name]
        if stray:
            raise KicadError(
                f"{sidecar} [[cables]]: {cable_file} [pins] {stray[0]!r} is not a pad "
                f"of {model.board.path.name}; a cable's [pins] are keyed by the MCU's "
                "pad names"
            )
        both = [pad for pad in cable.head_no_connect if pad in carried]
        if both:
            raise KicadError(
                f"{sidecar} [[cables]]: head_no_connect {both}: {cable_file} carries "
                "them to the main board"
            )

    groups: dict[str, tuple[str, list[tuple[str, str]]]] = {}  # name -> (what, nodes)

    def add(name: str, what: str, members: list[tuple[str, str]]) -> None:
        if name in groups and groups[name][0] != what:
            raise KicadError(
                f"{sidecar}: net name {name!r} is used by {groups[name][0]} and {what}"
            )
        groups.setdefault(name, (what, []))[1].extend(members)

    def mcu_nodes(what: str, name: str, pad: str) -> list[tuple[str, str]]:
        """The nodes net `name` gets for MCU pad `pad`; with a cable, the main end."""
        if cable is None:
            return nodes(what, MCU, pad)
        if pad not in placed[HEAD].by_name:
            raise KicadError(
                f"{sidecar}: {what} is on {model.mcu_ref} pad {pad}, which cable "
                f"{cable.head_ref}-{cable.main_ref} ({cable.part}) does not carry; "
                f"add {pad} to {placed[HEAD].spec.path.name} [pins]"
            )
        head = nodes(what, MCU, pad) + nodes(what, HEAD, pad)
        add(f"HEAD_{name}", f"head of {what}", head)
        return nodes(what, MAIN, pad)

    names = model.net_names
    for net in model.nets:
        pad = model.pin_for(net.role)
        if pad is None:
            raise KicadError(
                f"{sidecar}: role {net.role} (GPIO{model.roles[net.role]}) is not on a "
                f"header pad of {model.board.path.name}, so the carrier cannot wire it"
            )
        what = f"role {net.role}"
        name = names.get(net.role, net.role)
        add(name, what, mcu_nodes(what, name, pad.name))
        add(names.get(net.role, net.role), what, nodes(what, net.part, net.pin))
    for cn in model.channel_nets:
        what = f"channel role {cn.role}"
        add(names.get(cn.role, cn.role), what, nodes(what, cn.source, str(cn.channel)))
        add(names.get(cn.role, cn.role), what, nodes(what, cn.part, cn.pin))
    for bus in model.bus_nets:
        for pad, pin in bus.wires:
            what = f"bus role {bus.role} {pin}"
            name = f"{names.get(bus.role, bus.role)}_{pin}"
            add(name, what, nodes(what, bus.source, pad))
            add(name, what, nodes(what, bus.part, pin))
    for rail in model.rails:
        what = f"rail {rail.name}"
        name = names.get(rail.name, rail.name)
        for ep in (rail.source, *rail.loads):
            if ep.part == MCU:
                add(name, what, mcu_nodes(what, name, ep.pin))
            else:
                add(name, what, nodes(what, ep.part, ep.pin))
    for out in model.outputs:
        if not out.pins:
            raise KicadError(
                f"{sidecar}: [[outputs]] from {out.part!r} has no `pins`, so the carrier "
                "cannot wire it; add pins and a net template"
            )
        for pin in out.pins:
            what = f"output {out.part}.{pin}"
            members = nodes(what, out.part, pin)
            for driver_pin, load, load_pin in out.wires:
                if driver_pin == pin:
                    members += nodes(what, load, load_pin)
            add(out.net.format(pin=pin), what, members)

    owner: dict[tuple[str, str], str] = {}
    nets: dict[str, tuple[tuple[str, str], ...]] = {}
    for name in sorted(groups):
        members = sorted(set(groups[name][1]), key=node_key)
        for node in members:
            if node in owner:
                raise KicadError(
                    f"{sidecar}: {node[0]}.{node[1]} is on net {owner[node]!r} and net {name!r}"
                )
            owner[node] = name
        nets[name] = tuple(members)

    no_connect = []
    for p in placed.values():
        names_of = {n: name for name, nums in p.by_name.items() for n in nums}
        for pin in p.pins:
            node = (p.comp.ref, pin.number)
            name = names_of[pin.number]
            if node in owner:
                if pin.etype == "no_connect":
                    raise KicadError(
                        f"{sidecar}: {node[0]}.{node[1]} ({name}) is a no_connect pin of "
                        f"{p.spec.symbol}, but is on net {owner[node]!r}"
                    )
                continue
            if node in no_connect:
                continue  # a stacked pin number
            # A cable's head_no_connect pads stay open on the head, power pads too.
            head_open = (
                p.key == MCU and cable is not None and name in cable.head_no_connect
            )
            if head_open or registry.may_float(p.spec, p.board, name):
                no_connect.append(node)
            elif p.spec.pin_numbers:
                raise KicadError(
                    f"{sidecar}: {node[0]}.{node[1]} ({name}) is a power pad and is on no rail"
                )
            else:
                raise KicadError(
                    f"{sidecar}: {node[0]}.{node[1]} ({name}) unconnected, not declared: "
                    f"wire {p.key}.{name}, or list {name!r} under no_connect in "
                    f"{p.spec.path.relative_to(repo_root).as_posix()}"
                )

    comps = sorted((p.comp for p in placed.values()), key=lambda c: ref_key(c.ref))
    return Netlist(
        project=model.project_dir,
        comps=tuple(comps),
        nets=nets,
        no_connect=tuple(sorted(no_connect, key=node_key)),
    )


def unconnected(
    nl: Netlist, vendored: Path = library.VENDORED
) -> dict[str, tuple[str, str]]:
    """KiCad's net name for each no-connect pin -> (ref, pin)."""
    lib_ids = {c.ref: c.lib_id for c in nl.comps}
    pin_names: dict[str, dict[str, str]] = {}
    out = {}
    for ref, pin in nl.no_connect:
        lib_id = lib_ids[ref]
        if lib_id not in pin_names:
            block = symbols.load(lib_id, vendored=vendored)
            pin_names[lib_id] = {p.number: p.name for p in symbols.pins(block)}
        out[nc_net_name(ref, pin_names[lib_id][pin], pin)] = (ref, pin)
    return dict(sorted(out.items()))


def as_dict(nl: Netlist, vendored: Path = library.VENDORED) -> dict[str, Any]:
    """The netlist as plain JSON data, in the shape `parity.normalise` gives a KiCad export."""
    return {
        "components": {
            c.ref: {
                "footprint": c.footprint,
                "in_bom": c.in_bom,
                "lcsc": c.lcsc,
                "lib_id": c.lib_id,
                "value": c.value,
            }
            for c in nl.comps
        },
        "nets": {name: [list(n) for n in members] for name, members in nl.nets.items()},
        "unconnected": {
            name: list(node) for name, node in unconnected(nl, vendored).items()
        },
    }


def render(nl: Netlist, vendored: Path = library.VENDORED) -> str:
    """A plain-text listing: components, nets, then no-connects."""
    lines = [
        f"{len(nl.comps)} components, {len(nl.nets)} nets, {len(nl.no_connect)} no-connects"
    ]
    for c in nl.comps:
        extra = "".join(f" {k}={v}" for k, v in c.fields)
        bom = "" if c.in_bom else " (not in BOM)"
        lines.append(f"  {c.ref:4s} {c.value!r} {c.lib_id} {c.footprint}{extra}{bom}")
    for name, members in nl.nets.items():
        lines.append(f"  {name}: " + " ".join(f"{r}.{p}" for r, p in members))
    for name, (ref, pin) in unconnected(nl, vendored).items():
        lines.append(f"  {name}: {ref}.{pin}")
    return "\n".join(lines)


def main(argv: list[str] | None = None, repo_root: Path = REPO_ROOT) -> int:
    parser = argparse.ArgumentParser(
        prog="kicad.netlist", description=__doc__.split("\n")[0]
    )
    paths.add_project_args(parser)
    args = parser.parse_args(argv)
    try:
        for project in paths.select_projects(args.projects, args.allow_none, repo_root):
            print(f"{project}:")
            print(render(build(project, repo_root)))
    except KicadError as e:
        print(f"FAIL: {e}")
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
