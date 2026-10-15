"""The join: `<project>/hardware.toml` read against its header and its board.

hardware.toml is the parts-and-nets layer of ADR-021 and the project's
descriptor for the other two. It deliberately holds **no pin numbers** — every
`role` is a macro name that must resolve against the header, and an unknown key
is rejected so a `gpio = 5` cannot slip in beside one:

    [source]
    convention    = "robocar"                         # pin-macro convention
    header        = "main/pin_config.h"               # roles; project-relative
    extra_headers = ["main/planner_task.h"]           # more #defines, no roles
    board         = "docs/reference/boards/x.md"      # repo-relative

    [parts.amp]
    name  = "MAX98357A"
    kind  = "i2s-amp"
    board = "docs/reference/boards/adafruit-max98357a.md"  # optional; repo-relative

    [[nets]]
    role = "I2S_BCLK_PIN"
    to   = "amp.BCLK"
    note = "bit clock"          # optional

    [[channel_nets]]            # from a PWM driver's output, not the MCU (#666)
    role = "MOTOR_RIGHT_PWM_CHANNEL"  # a channel role in `header`
    from = "pwm"                # the driver part; its pad is the channel number
    to   = "motor_driver.PWMA"
    note = "speed"              # optional

    [[bus_nets]]                # an I2C mux channel to a load (#768)
    role = "I2C_BUS_CHANNEL_OLED"  # a bus channel role in `header`
    from = "mux"                # the mux part; channel n is its SDn/SCn pair
    to   = "oled"               # the load part; SDn -> SDA, SCn -> SCL
    note = "display"            # optional

    [[bus_nets]]                # a load on one line of a channel (#750)
    role = "I2C_BUS_CHANNEL_OLED"
    from = "mux"
    to   = "pull_sd1.B"         # 'part.PIN': the one pin on that line
    line = "SDA"                # SD1 -> pull_sd1.B; SC1 is untouched

    [[undrawn]]                 # a pin role deliberately on no drawn net
    role = "MIC_PDM_CLK_PIN"
    why  = "internal to the Sense module"

    [[rails]]                   # a supply rail: one source pin, its load pins
    name = "5V"
    from = "buck.OUT+"
    to   = ["mcu.5V", "amp.Vin"]

    [[outputs]]                 # loads a part drives from its own terminals
    from = "motor_driver"
    to   = ["motor_left", "motor_right"]

A rail endpoint is `part.PIN`. The reserved part id `mcu` is the project's MCU
board, and its pin must be a pad in the board reference (`[source] mcu` names
the board for display, default "MCU"). A part with a `board` page must have the
pin there too. One pin on two rails is a short, and a rail pin that is also a
signal net ties a GPIO to a supply; both are errors. The rail's `name` is its
only voltage fact.

A part's `board` names its own physical-layout page, where one exists (#629):
`hardware.pinout` draws that board and labels each pad a net lands on. A part
with no vendor-sourced layout leaves the key out and is not drawn.

A `[[channel_nets]]` role is a channel role (`header.CHANNEL_CONVENTIONS`),
resolved to its channel number from the header, so this file still holds no
number. Where a part has a `board` page, both ends must be pads there: the
driver's pad is named by the channel number, as the PCA9685's silkscreen is. Two
wired roles on one channel, and one pin driven by two outputs (two channels, or
a channel and an MCU net), are errors; one channel to several pins is a fan-out.
`[[nets]]` stays MCU-only, so every consumer that reads it as GPIO wiring still
can.

A `[[bus_nets]]` role is a bus channel role (`header.BUS_CONVENTIONS`), the
multiplexer channel the firmware selects for that load, resolved to its number
from the header. Each entry is two lines: the mux's `SD<n>` pad to the load's
`SDA`, and `SC<n>` to `SCL`; where a part has a `board` page those pads must be
on it. Two wired roles on one channel are an error, as is a pin on two nets of
any kind (a load on two channels, a bus pin that is also an MCU or channel
net); one channel to several loads is a fan-out. A load on one line, such
as a pull-up resistor, names its pin as `to = "part.PIN"` and the line it
joins as `line = "SDA"` or `"SCL"`. Like channel roles, bus
channel roles are not required to be wired. In the carrier view the lines are
named `<name>_SDA` and `<name>_SCL`, where `<name>` is the role or its
`[net_names]` entry.

`[[undrawn]]` is an array of tables rather than ADR-021's `undrawn = [...]`
key: written after a `[[nets]]` block, that key would belong to the last net.

Every pin role in `header` must be on a `[[nets]]` entry or listed under
`[[undrawn]]`; `join()` raises naming each one that is neither (#460). The check
covers pin roles only: channel roles (`*_CHANNEL`) are not required to be wired,
since some, such as `MOTOR_FIRST_CHANNEL`, are aliases the firmware reads.

Two pin roles in `header` on one GPIO, and two roles onto one part pin, are
errors (#722): each is two signals shorted together. A pin meant to answer to
two names is written as an alias, `#define B_PIN A_PIN`, which no convention
reads as a role.

Two builds read one file (#750): the hand-wired **harness** the docs describe,
and the **carrier** PCB `tools/kicad` generates. An entry in `[parts]`,
`[[nets]]`, `[[channel_nets]]`, `[[bus_nets]]`, `[[undrawn]]`, `[[rails]]` or
`[[outputs]]` may carry `build = "harness"` or `build = "carrier"`; with no `build` it is in
both. `join(..., view=)` keeps the entries of one view and runs every check
above on those alone, so completeness, shorts and the signal/rail clash are
per view. The default view is the harness: WIRING.md, the pinouts and
pin_defs.typ read it. Unknown keys and the types and formats below are checked
on every entry in every view, so a typo in a carrier-only entry still fails.

    [source]
    part = "seeed-xiao-rp2350-socket"  # docs/reference/parts/ entry of the MCU
    ref  = "U1"                         # its reference designator

    [parts.sense_top]
    name  = "100 kΩ pack divider, top"
    kind  = "resistor"
    build = "carrier"           # only on the carrier PCB
    part  = "r0805"             # docs/reference/parts/ entry
    ref   = "R2"                # reference designator: letters then digits
    value = "100k"              # KiCad Value; tools/kicad defaults it
    lcsc  = "C149504"           # JLCPCB part number: C then digits

    [[rails]]                   # a continuation: same name, no `from`
    name  = "VBAT"
    build = "carrier"
    to    = ["bulk_left.+"]     # loads joining the VBAT rail declared elsewhere

    [[outputs]]
    from = "drv_left"
    to   = ["stepper_left"]
    pins = ["A1", "A2"]         # driver pin p goes to pin p of every load
    net  = "MOTL_{pin}"         # one net per pin; `{pin}` is the only field

    [[outputs]]                 # one driver, two loads on separate pins (#769)
    from = "motor_driver"
    to   = ["motor_left", "motor_right"]
    pins = ["A01", "A02", "B01", "B02"]
    net  = "MOT_{pin}"
    load_pins.motor_right = { A01 = "1", A02 = "2" }  # driver pin = load pin
    load_pins.motor_left  = { B01 = "1", B02 = "2" }

    [net_names]                 # carrier net name, where it is not the role's
    PIN_STEP_LEFT = "STEP_L"    # key: a pin or channel role on a net, or a rail
    3V3 = "+3V3"

    [[cables]]                  # the MCU on a head board, cabled to the main board
    part     = "idc-2x07-head"  # docs/reference/parts/ entry, [pins] keyed by MCU pad
    head_ref = "J1"             # the connector beside the MCU
    main_ref = "J2"             # the connector on the main board; same entry
    head_no_connect = ["3V3"]   # optional: MCU pads left unconnected on the head
    build    = "carrier"        # required: a cable exists only on the carrier

A `[[cables]]` entry (#750) splits the carrier into a head board, holding the
MCU, and the main board. The join checks its keys and refs (two refs, unique
among all refs, in every view), allows one cable per carrier, and requires each
`head_no_connect` pad to be a pad of `[source] board` on no net and no rail.
The harness view never holds a cable. `tools/kicad/netlist.py` reads the
registry entry and does the rest: it splits every MCU net at the cable and
rejects an MCU pad on a net the cable does not carry. A role that stays on the
head goes under `[[undrawn]]`.

With `load_pins`, each load gets only the driver pins its table names, on the
load pin given; every load in `to` needs a table, every driver pin must reach a
load, and two driver pins on one load pin are a short. Without it, driver pin p
goes to pin p of every load.

A part outside the view is dropped, and an in-view entry that names it fails.
Within a view, the `[[rails]]` entries of one name form one rail: exactly one
carries `from`, the loads follow in file order (that entry's first), and the
rail sits where the first entry of that name does. An output's pins must be on
no rail and no signal net. In the carrier view every `[net_names]` key must be
on a net or a rail, and no two nets may end up with one name. Rail names,
`[net_names]` values and expanded output names are limited to letters, digits
and `_+-.`, because KiCad reads brackets, braces, `~`, `/` and spaces as bus or
markup syntax.
"""

from __future__ import annotations

import re
import string
import tomllib
from collections.abc import Callable
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

from .board import Board, BoardPin, parse_board_table
from .errors import HardwareError
from .header import (
    bus_channels_from_defines,
    channels_from_defines,
    parse_defines,
    roles_from_defines,
)
from .layout import parse_layout

SIDECAR = "hardware.toml"
REPO_ROOT = Path(__file__).resolve().parents[2]

_KEYS = {
    "": {
        "source",
        "parts",
        "nets",
        "channel_nets",
        "bus_nets",
        "undrawn",
        "rails",
        "outputs",
        "net_names",
        "cables",
    },
    "source": {"convention", "header", "extra_headers", "board", "mcu", "part", "ref"},
    "part": {"name", "kind", "note", "board", "build", "part", "ref", "value", "lcsc"},
    "net": {"role", "to", "note", "build"},
    "channel_net": {"role", "from", "to", "note", "build"},
    "bus_net": {"role", "from", "to", "line", "note", "build"},
    "undrawn": {"role", "why", "build"},
    "rail": {"name", "from", "to", "build"},
    "output": {"from", "to", "pins", "net", "load_pins", "build"},
    "cable": {"part", "head_ref", "main_ref", "head_no_connect", "build"},
}
HARNESS, CARRIER = "harness", "carrier"
VIEWS = (HARNESS, CARRIER)
MCU = "mcu"  # the reserved part id a rail endpoint uses for the MCU board

_REF = re.compile(r"[A-Z]+[0-9]+")
_LCSC = re.compile(r"C[0-9]+")
# An I2C mux channel n is the pad pair SDn/SCn; each joins the load pin named
# here, which is also the suffix of the line's carrier net name (#768).
I2C_LINES = (("SD", "SDA"), ("SC", "SCL"))
# KiCad reads `D[0..1]`, `{A B}`, `~{EN}`, `X/Y` and spaces as bus or markup.
_NET_NAME = re.compile(r"[A-Za-z0-9_+\-.]+")


@dataclass(frozen=True)
class Part:
    key: str  # the id nets refer to: "amp"
    name: str  # "MAX98357A"
    kind: str  # "i2s-amp"
    note: str = ""
    board: str = ""  # repo-relative physical-layout page; "" for none
    part: str = ""  # docs/reference/parts/ slug; "" for none
    ref: str = ""  # reference designator: "R2"
    value: str = ""  # KiCad Value field; "" leaves it to tools/kicad
    lcsc: str = ""  # JLCPCB part number: "C17414"
    build: str = ""  # HARNESS or CARRIER; "" for both views


@dataclass(frozen=True)
class Net:
    role: str
    part: str
    pin: str
    note: str


@dataclass(frozen=True)
class ChannelNet:
    role: str  # a channel role: "MOTOR_RIGHT_PWM_CHANNEL"
    channel: int  # its number, from the header
    source: str  # the driver part id: "pwm"
    part: str
    pin: str
    note: str


@dataclass(frozen=True)
class BusNet:
    role: str  # a bus channel role: "I2C_BUS_CHANNEL_OLED"
    channel: int  # its number, from the header
    source: str  # the mux part id: "mux"
    part: str  # the load part id: "oled"
    note: str
    # A one-line load (#750): its pin, on the line named; "" for both lines.
    line: str = ""  # "SDA" or "SCL"
    pin: str = ""  # the load's pin on that line: "B"

    @property
    def lines(self) -> tuple[tuple[str, str, str], ...]:
        """(line, mux pad, load pin) per line it joins: (("SDA", "SD1", "SDA"), ...).

        A whole-part load joins both lines on its own SDA and SCL; a one-line
        load joins only `line`, on `pin`.
        """
        return tuple(
            (line, f"{pad}{self.channel}", self.pin or line)
            for pad, line in I2C_LINES
            if not self.line or line == self.line
        )

    @property
    def wires(self) -> tuple[tuple[str, str], ...]:
        """(mux pad, load pin) per line: (("SD1", "SDA"), ("SC1", "SCL"))."""
        return tuple((pad, pin) for _, pad, pin in self.lines)


@dataclass(frozen=True)
class Undrawn:
    role: str
    why: str


@dataclass(frozen=True)
class Endpoint:
    part: str  # a [parts] id, or MCU for the board itself
    pin: str


@dataclass(frozen=True)
class Rail:
    name: str  # "5V"
    source: Endpoint
    loads: tuple[Endpoint, ...]


@dataclass(frozen=True)
class Output:
    part: str
    loads: tuple[str, ...]  # part ids
    pins: tuple[str, ...] = ()  # the driver pins, one net each
    net: str = ""  # net-name template: "MOTL_{pin}"; "" when `pins` is empty
    # load -> {driver pin: load pin}; empty: pin p goes to pin p of every load
    load_pins: dict[str, dict[str, str]] = field(default_factory=dict)

    @property
    def wires(self) -> tuple[tuple[str, str, str], ...]:
        """(driver pin, load, load pin) per wire, in `pins` order, then `loads`."""
        if not self.load_pins:
            return tuple((p, load, p) for p in self.pins for load in self.loads)
        return tuple(
            (p, load, self.load_pins[load][p])
            for p in self.pins
            for load in self.loads
            if p in self.load_pins[load]
        )


@dataclass(frozen=True)
class Cable:
    """A cable between the carrier's two boards: the MCU's head and the main board."""

    part: str  # parts-registry slug; both ends mount it, [pins] keyed by MCU pad
    head_ref: str  # the connector beside the MCU: "J1"
    main_ref: str  # the connector on the main board: "J2"
    head_no_connect: tuple[str, ...] = ()  # MCU pads left unconnected on the head


@dataclass(frozen=True)
class HardwareModel:
    project_dir: Path
    convention: str  # [source] convention: a key of header.CONVENTIONS
    headers: tuple[Path, ...]  # header first, then extra_headers, in order
    defines: dict[str, str]  # every #define across `headers`
    roles: dict[str, int]  # pin role -> GPIO, from `header` alone
    board: Board
    parts: dict[str, Part]
    nets: tuple[Net, ...]
    undrawn: tuple[Undrawn, ...]
    rails: tuple[Rail, ...] = ()
    outputs: tuple[Output, ...] = ()
    mcu: str = "MCU"  # display name of the MCU board
    channels: dict[str, int] = field(default_factory=dict)  # channel role -> number
    channel_nets: tuple[ChannelNet, ...] = ()
    view: str = HARNESS  # which build's entries this model holds
    net_names: dict[str, str] = field(default_factory=dict)  # [net_names] as read
    mcu_part: str = ""  # [source] part: the MCU's docs/reference/parts/ slug
    mcu_ref: str = ""  # [source] ref: the MCU's reference designator
    bus_channels: dict[str, int] = field(default_factory=dict)  # mux channel role -> n
    bus_nets: tuple[BusNet, ...] = ()
    cables: tuple[Cable, ...] = ()  # carrier view only; at most one

    def pin_for(self, role: str) -> BoardPin | None:
        """The header pad a role lands on, or None if its GPIO is not broken out."""
        return self.board.by_gpio.get(self.roles[role])


# (where, table) for each in-view entry of an array of tables.
Entries = list[tuple[str, dict[str, Any]]]
# Builds the error for a part id that is not in this view's [parts].
NoPart = Callable[[str, Any], HardwareError]


def _check_keys(where: str, kind: str, table: Any) -> None:
    if not isinstance(table, dict):
        raise HardwareError(f"{where}: expected a table, got {table!r}")
    unknown = sorted(set(table) - _KEYS[kind])
    if unknown:
        raise HardwareError(
            f"{where}: unknown key(s) {unknown}; allowed: {sorted(_KEYS[kind])}"
        )


def _require(where: str, table: dict[str, Any], key: str) -> str:
    value = table.get(key)
    if not isinstance(value, str) or not value:
        raise HardwareError(f"{where}: missing or non-string {key!r}")
    return value


def _optional(
    where: str, table: dict[str, Any], key: str, pattern: re.Pattern[str] | None = None
) -> str:
    """`key` as a non-empty string matching `pattern`, or "" when absent."""
    if key not in table:
        return ""
    value = _require(where, table, key)
    if pattern is not None and not pattern.fullmatch(value):
        raise HardwareError(
            f"{where}: {key} {value!r} does not match {pattern.pattern}"
        )
    return value


def _in_view(where: str, table: dict[str, Any], view: str) -> bool:
    """Whether an entry belongs to `view`, after checking its `build` key."""
    if "build" not in table:
        return True
    build = table["build"]
    if not isinstance(build, str) or build not in VIEWS:
        raise HardwareError(
            f"{where}: build must be one of {list(VIEWS)} (or absent, for both), "
            f"got {build!r}"
        )
    return build == view


def _net_name(where: str, name: str) -> str:
    if not _NET_NAME.fullmatch(name):
        raise HardwareError(
            f"{where}: {name!r} is not a usable net name: use letters, digits and "
            "_ + - . only (KiCad reads brackets, braces, '~', '/' and spaces as "
            "bus or markup syntax)"
        )
    return name


def _output_pins(
    where: str, table: dict[str, Any]
) -> tuple[tuple[str, ...], str, dict[str, dict[str, str]]]:
    """An output's `pins`, `net` template and `load_pins`, checked on their own."""
    if "pins" not in table:
        if "net" in table:
            raise HardwareError(
                f"{where}: 'net' names the nets of 'pins', and has none"
            )
        if "load_pins" in table:
            raise HardwareError(
                f"{where}: 'load_pins' maps the driver pins of 'pins', and has none"
            )
        return (), "", {}
    pins = table["pins"]
    if (
        not isinstance(pins, list)
        or not pins
        or not all(isinstance(p, str) and p for p in pins)
    ):
        raise HardwareError(
            f"{where}: 'pins' must be a non-empty list of pin names, got {pins!r}"
        )
    dupes = sorted({p for p in pins if pins.count(p) > 1})
    if dupes:
        raise HardwareError(f"{where}: pins {dupes} listed twice")
    if "net" not in table:
        raise HardwareError(
            f"{where}: 'pins' needs a 'net' name template, such as \"M_{{pin}}\""
        )
    net = _require(where, table, "net")
    try:
        fields = [
            (name, spec, conversion)
            for _, name, spec, conversion in string.Formatter().parse(net)
            if name is not None
        ]
    except ValueError as e:
        raise HardwareError(f"{where}: net {net!r}: {e}") from e
    if any(f != ("pin", "", None) for f in fields):
        raise HardwareError(f"{where}: net {net!r}: the only field allowed is {{pin}}")
    if len(pins) > 1 and not fields:
        raise HardwareError(
            f"{where}: net {net!r} names {len(pins)} pins and has no {{pin}} "
            "to tell their nets apart"
        )
    for p in pins:
        _net_name(f"{where} net", net.format(pin=p))
    return tuple(pins), net, _load_pins(where, table, pins)


def _load_pins(
    where: str, table: dict[str, Any], pins: list[str]
) -> dict[str, dict[str, str]]:
    """`load_pins`: per load, which driver pin lands on which of its pins (#769)."""
    if "load_pins" not in table:
        return {}
    value = table["load_pins"]
    if not isinstance(value, dict) or not all(
        isinstance(m, dict) and all(isinstance(v, str) and v for v in m.values())
        for m in value.values()
    ):
        raise HardwareError(
            f"{where}: 'load_pins' must map each load to a table of "
            f'driver pin = "load pin", got {value!r}'
        )
    loads = table.get("to")
    if isinstance(loads, list) and set(value) != set(loads):
        raise HardwareError(
            f"{where}: 'load_pins' must name each load in 'to' exactly once: "
            f"'to' has {sorted(map(str, loads))}, 'load_pins' {sorted(value)}"
        )
    for load, mapping in value.items():
        lands: dict[str, str] = {}
        for driver_pin, load_pin in mapping.items():
            if driver_pin not in pins:
                raise HardwareError(
                    f"{where}: load_pins.{load}: {driver_pin} is not in 'pins' {pins}"
                )
            if load_pin in lands:
                raise HardwareError(
                    f"{where}: load_pins.{load}: {lands[load_pin]} and {driver_pin} "
                    f"both go to {load}.{load_pin} — that is a short"
                )
            lands[load_pin] = driver_pin
    unused = [p for p in pins if all(p not in m for m in value.values())]
    if unused:
        raise HardwareError(f"{where}: load_pins: driver pin(s) {unused} reach no load")
    return {load: dict(mapping) for load, mapping in value.items()}


def _entries(
    sidecar: Path,
    data: dict[str, Any],
    key: str,
    kind: str,
    view: str,
    check: Callable[[str, dict[str, Any]], object] = lambda where, table: None,
) -> Entries:
    """The in-view entries of [[key]], each key-checked and `check`ed in every view."""
    value = data.get(key, [])
    if not isinstance(value, list):
        raise HardwareError(f"{sidecar}: {key} must be [[{key}]] tables")
    result: Entries = []
    for i, table in enumerate(value):
        where = f"{sidecar} [[{key}]] #{i + 1}"
        _check_keys(where, kind, table)
        in_view = _in_view(where, table, view)
        check(where, table)
        if in_view:
            result.append((where, table))
    return result


def _rail_name(where: str, table: dict[str, Any]) -> None:
    _net_name(where, _require(where, table, "name"))


def _power(
    view: str,
    rail_entries: Entries,
    output_entries: Entries,
    parts: dict[str, Part],
    no_part: NoPart,
    nets: list[Net],
    channel_nets: list[ChannelNet],
    bus_nets: list[BusNet],
    roles: dict[str, int],
    board: Board,
    repo_root: Path,
) -> tuple[tuple[Rail, ...], tuple[Output, ...]]:
    """[[rails]] and [[outputs]], checked against the parts, the nets and the boards."""
    pads: dict[str, set[str]] = {MCU: set(board.by_name)}
    for key, part in parts.items():
        if part.board:
            pads[key] = {p.name for p in parse_layout(repo_root / part.board).pads}
    signal = {(n.part, n.pin): n.role for n in nets}
    # A net's MCU end is a pad too: a rail on it ties the same GPIO to a supply.
    for n in nets:
        pad = board.by_gpio.get(roles[n.role])
        if pad is not None:
            signal[MCU, pad.name] = n.role
    # A channel net is a signal at both ends: the driver's output pad and the pin.
    for c in channel_nets:
        signal[c.part, c.pin] = c.role
        signal[c.source, str(c.channel)] = c.role
    # So is each line of a bus net: the mux's channel pad and the load's pin.
    for b in bus_nets:
        for pad, pin in b.wires:
            signal[b.source, pad] = b.role
            signal[b.part, pin] = b.role

    def endpoint(where: str, value: Any) -> Endpoint:
        part, dot, pin = (
            value.partition(".") if isinstance(value, str) else ("", "", "")
        )
        if not dot or not part or not pin:
            raise HardwareError(f"{where}: endpoint must be 'part.PIN', got {value!r}")
        if part != MCU and part not in parts:
            raise no_part(where, part)
        if part in pads and pin not in pads[part]:
            ref = board.path if part == MCU else repo_root / parts[part].board
            raise HardwareError(
                f"{where}: {value}: {ref.name} has no pad named {pin!r}"
            )
        if (part, pin) in signal:
            raise HardwareError(
                f"{where}: {value} is also the signal net {signal[part, pin]}"
            )
        return Endpoint(part, pin)

    def targets(where: str, table: dict[str, Any], what: str) -> list[Any]:
        value = table.get("to")
        if not isinstance(value, list) or not value:
            raise HardwareError(f"{where}: 'to' must be a non-empty list of {what}")
        return value

    # The entries of one name form one rail, placed where its first entry is.
    by_name: dict[str, Entries] = {}
    for where, table in rail_entries:
        by_name.setdefault(table["name"], []).append((where, table))

    rails: list[Rail] = []
    on_rail: dict[Endpoint, str] = {}
    for name, group in by_name.items():
        bases = [entry for entry in group if "from" in entry[1]]
        if not bases:
            raise HardwareError(
                f"{group[0][0]}: rail {name!r} has loads but no `from` in the "
                f"{view} view"
            )
        if len(bases) > 1:
            raise HardwareError(f"{bases[1][0]}: rail {name!r} declared twice")
        base_where, base = bases[0]
        source = endpoint(base_where, _require(base_where, base, "from"))
        loads: list[tuple[str, Endpoint]] = []
        for where, table in [bases[0], *(g for g in group if g[1] is not base)]:
            loads += [
                (where, endpoint(where, v)) for v in targets(where, table, "'part.PIN'")
            ]
        seen: set[Endpoint] = set()
        for where, e in loads:
            if e in seen:
                ends = [e for _, e in loads]
                dupes = sorted({f"{e.part}.{e.pin}" for e in ends if ends.count(e) > 1})
                raise HardwareError(f"{where}: {dupes} listed twice")
            seen.add(e)
        for where, e in [(base_where, source), *loads]:
            if e in on_rail:
                raise HardwareError(
                    f"{where}: {e.part}.{e.pin} is on rail {on_rail[e]!r} "
                    f"and rail {name!r} — that is a short"
                )
            on_rail[e] = name
        rails.append(Rail(name, source, tuple(e for _, e in loads)))

    outputs: list[Output] = []
    for where, table in output_entries:
        source = _require(where, table, "from")
        load_ids = targets(where, table, "part ids")
        for key in (source, *load_ids):
            if not isinstance(key, str) or key not in parts:
                raise no_part(where, key)
        dupes = sorted({k for k in load_ids if load_ids.count(k) > 1})
        if dupes:
            raise HardwareError(f"{where}: {dupes} listed twice")
        pins, net, load_pins = _output_pins(where, table)
        out = Output(
            part=source, loads=tuple(load_ids), pins=pins, net=net, load_pins=load_pins
        )
        ends = [(source, p) for p in pins] + [(load, p) for _, load, p in out.wires]
        for key, pin in ends:
            if (key, pin) in signal:
                raise HardwareError(
                    f"{where}: {key}.{pin} is also the signal net {signal[key, pin]}"
                )
            if Endpoint(key, pin) in on_rail:
                raise HardwareError(
                    f"{where}: {key}.{pin} is also rail {on_rail[Endpoint(key, pin)]}"
                )
        outputs.append(out)

    return tuple(rails), tuple(outputs)


def _channel_nets(
    entries: Entries,
    parts: dict[str, Part],
    no_part: NoPart,
    nets: list[Net],
    channels: dict[str, int],
    defines: dict[str, str],
    header: Path,
    repo_root: Path,
) -> list[ChannelNet]:
    """[[channel_nets]], checked against the header, the parts and their boards."""
    pads: dict[str, set[str]] = {}
    for key, part in parts.items():
        if part.board:
            pads[key] = {p.name for p in parse_layout(repo_root / part.board).pads}
    # Who drives each part pin: an MCU net, or a channel net added below.
    driver: dict[tuple[str, str], str] = {(n.part, n.pin): n.role for n in nets}
    on_channel: dict[tuple[str, int], str] = {}
    result: list[ChannelNet] = []
    for where, table in entries:
        role = _require(where, table, "role")
        if role not in channels:
            what = "is not a channel role" if role in defines else "is not defined"
            raise HardwareError(f"{where}: role {role!r} {what} in {header.name}")
        channel = channels[role]
        source = _require(where, table, "from")
        to = _require(where, table, "to")
        part, dot, pin = to.partition(".")
        if not dot or not part or not pin:
            raise HardwareError(f"{where}: 'to' must be 'part.PIN', got {to!r}")
        for key in (source, part):
            if key not in parts:
                raise no_part(where, key)
        if part == source:
            raise HardwareError(
                f"{where}: {role} runs from {source} to {to}, one of its own pins"
            )
        if source in pads and str(channel) not in pads[source]:
            raise HardwareError(
                f"{where}: {role} is channel {channel}, and "
                f"{Path(parts[source].board).name} has no pad named '{channel}'"
            )
        if part in pads and pin not in pads[part]:
            raise HardwareError(
                f"{where}: {to}: {Path(parts[part].board).name} has no pad named {pin!r}"
            )
        if any(c.role == role and c.part == part and c.pin == pin for c in result):
            raise HardwareError(f"{where}: {role} -> {to} is listed twice")
        other = on_channel.setdefault((source, channel), role)
        if other != role:
            raise HardwareError(
                f"{where}: channel {channel} of {source} is wired as both {other} and "
                f"{role}; {header.name} gives them the same number"
            )
        if (part, pin) in driver and driver[part, pin] != role:
            raise HardwareError(
                f"{where}: {to} is driven by both {driver[part, pin]} and {role}"
            )
        driver[part, pin] = role
        # The driver's own channel pad is an output too: nothing else may drive it.
        pad = (source, str(channel))
        if pad in driver and driver[pad] != role:
            raise HardwareError(
                f"{where}: {source}.{channel} is driven by both {driver[pad]} and {role}"
            )
        driver[pad] = role
        result.append(
            ChannelNet(
                role=role,
                channel=channel,
                source=source,
                part=part,
                pin=pin,
                note=table.get("note", ""),
            )
        )
    return result


def _bus_nets(
    entries: Entries,
    parts: dict[str, Part],
    no_part: NoPart,
    nets: list[Net],
    channel_nets: list[ChannelNet],
    bus_channels: dict[str, int],
    defines: dict[str, str],
    header: Path,
    repo_root: Path,
) -> list[BusNet]:
    """[[bus_nets]], checked against the header, the parts and their boards."""
    pads: dict[str, set[str]] = {}
    for key, part in parts.items():
        if part.board:
            pads[key] = {p.name for p in parse_layout(repo_root / part.board).pads}
    # Which net already holds each part pin: an MCU net, a channel net (both
    # ends), or a bus line added below.
    taken: dict[tuple[str, str], str] = {(n.part, n.pin): n.role for n in nets}
    for c in channel_nets:
        taken[c.part, c.pin] = c.role
        taken[c.source, str(c.channel)] = c.role
    on_channel: dict[tuple[str, int], str] = {}
    result: list[BusNet] = []
    for where, table in entries:
        role = _require(where, table, "role")
        if role not in bus_channels:
            what = "is not a bus channel role" if role in defines else "is not defined"
            raise HardwareError(f"{where}: role {role!r} {what} in {header.name}")
        channel = bus_channels[role]
        source = _require(where, table, "from")
        load = _require(where, table, "to")
        line = table.get("line", "")
        pin = ""
        if line:
            lines = [name for _, name in I2C_LINES]
            if line not in lines:
                raise HardwareError(
                    f"{where}: 'line' must be one of {lines}, got {line!r}"
                )
            load, dot, pin = load.partition(".")
            if not dot or not load or not pin:
                raise HardwareError(
                    f"{where}: with 'line', 'to' must be 'part.PIN' (the one pin "
                    f"on {line}), got {table['to']!r}"
                )
        elif "." in load:
            raise HardwareError(
                f"{where}: 'to' must be a part id (its SDA and SCL are wired), "
                f"got {load!r}; a load on one line takes 'part.PIN' and 'line'"
            )
        for key in (source, load):
            if key not in parts:
                raise no_part(where, key)
        if load == source:
            raise HardwareError(f"{where}: {role} runs from {source} to its own pins")
        if any(
            b.role == role and b.part == load and b.line == line and b.pin == pin
            for b in result
        ):
            raise HardwareError(f"{where}: {role} -> {table['to']} is listed twice")
        other = on_channel.setdefault((source, channel), role)
        if other != role:
            raise HardwareError(
                f"{where}: channel {channel} of {source} is wired as both {other} and "
                f"{role}; {header.name} gives them the same number"
            )
        bus = BusNet(role, channel, source, load, table.get("note", ""), line, pin)
        for pad, pin in bus.wires:
            for key, name in ((source, pad), (load, pin)):
                if key in pads and name not in pads[key]:
                    page = Path(parts[key].board).name
                    if key == source:
                        raise HardwareError(
                            f"{where}: {role} is channel {channel}, and {page} has "
                            f"no pad named {name!r}"
                        )
                    raise HardwareError(
                        f"{where}: {key}.{name}: {page} has no pad named {name!r}"
                    )
                held = taken.setdefault((key, name), role)
                if held != role:
                    raise HardwareError(
                        f"{where}: {key}.{name} is on both {held} and {role}"
                    )
        result.append(bus)
    return result


def _cable(where: str, table: dict[str, Any]) -> Cable:
    """One [[cables]] entry, checked on its own (in every view)."""
    if table.get("build") != CARRIER:
        raise HardwareError(
            f'{where}: a cable joins two carrier boards; set build = "carrier"'
        )
    head_ref = _optional(where, table, "head_ref", _REF)
    main_ref = _optional(where, table, "main_ref", _REF)
    for key, ref in (("head_ref", head_ref), ("main_ref", main_ref)):
        if not ref:
            raise HardwareError(f"{where}: missing or non-string {key!r}")
    if head_ref == main_ref:
        raise HardwareError(f"{where}: head_ref and main_ref are both {head_ref!r}")
    pads = table.get("head_no_connect", [])
    if not isinstance(pads, list) or not all(isinstance(p, str) and p for p in pads):
        raise HardwareError(
            f"{where}: head_no_connect must be a list of MCU pad names, got {pads!r}"
        )
    dupes = sorted({p for p in pads if pads.count(p) > 1})
    if dupes:
        raise HardwareError(f"{where}: head_no_connect lists {dupes} twice")
    return Cable(
        part=_require(where, table, "part"),
        head_ref=head_ref,
        main_ref=main_ref,
        head_no_connect=tuple(pads),
    )


def _head_no_connect(
    cable_entries: Entries,
    cables: list[Cable],
    nets: list[Net],
    rails: tuple[Rail, ...],
    roles: dict[str, int],
    board: Board,
) -> None:
    """A cable's head_no_connect pads are MCU pads, and on no net or rail."""
    used: dict[str, str] = {}
    for n in nets:
        pad = board.by_gpio.get(roles[n.role])
        if pad is not None:
            used.setdefault(pad.name, f"role {n.role}")
    for rail in rails:
        for e in (rail.source, *rail.loads):
            if e.part == MCU:
                used.setdefault(e.pin, f"rail {rail.name}")
    for (where, _), cable in zip(cable_entries, cables, strict=True):
        for pad in cable.head_no_connect:
            if pad not in board.by_name:
                raise HardwareError(
                    f"{where}: head_no_connect: {board.path.name} has no pad named "
                    f"{pad!r}"
                )
            if pad in used:
                raise HardwareError(
                    f"{where}: head_no_connect: {MCU}.{pad} is on {used[pad]}, so it "
                    "cannot stay unconnected on the head"
                )


def _read_net_names(sidecar: Path, data: dict[str, Any]) -> dict[str, str]:
    """[net_names], with each value checked as a net name (in every view)."""
    names = data.get("net_names", {})
    if not isinstance(names, dict):
        raise HardwareError(f"{sidecar}: net_names must be a [net_names] table")
    for key, value in names.items():
        where = f"{sidecar} [net_names] {key}"
        if not isinstance(value, str) or not value:
            raise HardwareError(
                f"{where}: expected a non-empty net name, got {value!r}"
            )
        _net_name(where, value)
    return dict(names)


def _net_names(
    sidecar: Path,
    names: dict[str, str],
    nets: list[Net],
    channel_nets: list[ChannelNet],
    bus_nets: list[BusNet],
    rails: tuple[Rail, ...],
    outputs: tuple[Output, ...],
) -> None:
    """Carrier view: every [net_names] key is used, and no two nets share a name."""
    roles = [*dict.fromkeys([n.role for n in nets] + [c.role for c in channel_nets])]
    buses = [*dict.fromkeys(b.role for b in bus_nets)]
    for key in names:
        if key not in roles and key not in buses and all(r.name != key for r in rails):
            raise HardwareError(
                f"{sidecar} [net_names] {key}: neither a role on a net nor a rail "
                f"in the {CARRIER} view"
            )
    claims = [(names.get(r, r), f"role {r}") for r in roles]
    claims += [
        (f"{names.get(b, b)}_{line}", f"bus {b} {line}")
        for b in buses
        for _, line in I2C_LINES
    ]
    claims += [(names.get(r.name, r.name), f"rail {r.name}") for r in rails]
    claims += [
        (o.net.format(pin=p), f"output {o.part}.{p}") for o in outputs for p in o.pins
    ]
    used: dict[str, str] = {}
    for name, who in claims:
        if name in used:
            raise HardwareError(
                f"{sidecar}: net name {name!r} is used by {used[name]} and {who}"
            )
        used[name] = who


def join(
    project_dir: Path, repo_root: Path = REPO_ROOT, view: str = HARNESS
) -> HardwareModel:
    """Parse `project_dir/hardware.toml` and the files it names into one model.

    `view` picks the build whose entries the model holds: HARNESS (the default,
    which every docs generator reads) or CARRIER (tools/kicad).
    """
    if view not in VIEWS:
        raise HardwareError(f"unknown view {view!r}; expected one of {list(VIEWS)}")
    project_dir = project_dir.resolve()
    sidecar = project_dir / SIDECAR
    if not sidecar.is_file():
        raise HardwareError(f"{sidecar}: not found — this project has no {SIDECAR}")
    try:
        data = tomllib.loads(sidecar.read_text(encoding="utf-8"))
    except tomllib.TOMLDecodeError as e:
        raise HardwareError(f"{sidecar}: {e}") from e
    _check_keys(str(sidecar), "", data)

    source = data.get("source", {})
    _check_keys(f"{sidecar} [source]", "source", source)
    convention = _require(f"{sidecar} [source]", source, "convention")
    header = project_dir / _require(f"{sidecar} [source]", source, "header")
    extra_names = source.get("extra_headers", [])
    if not isinstance(extra_names, list) or not all(
        isinstance(p, str) and p for p in extra_names
    ):
        raise HardwareError(
            f"{sidecar} [source]: extra_headers must be a list of paths, got {extra_names!r}"
        )
    extras = [project_dir / p for p in extra_names]
    headers = (header, *extras)
    for h in headers:
        if not h.is_file():
            raise HardwareError(f"{sidecar}: header {h} does not exist")
    board_path = repo_root / _require(f"{sidecar} [source]", source, "board")
    if not board_path.is_file():
        raise HardwareError(f"{sidecar}: board reference {board_path} does not exist")
    mcu_part = _optional(f"{sidecar} [source]", source, "part")
    mcu_ref = _optional(f"{sidecar} [source]", source, "ref", _REF)

    defines = parse_defines(headers)
    roles = roles_from_defines(parse_defines([header]), convention)
    # Two roles on one GPIO is a wiring conflict (#722). A pin meant to serve
    # two names is written as an alias (`#define B_PIN A_PIN`), which no
    # convention reads as a role, so it never reaches this check.
    by_gpio: dict[int, str] = {}
    for role, gpio in roles.items():
        if gpio in by_gpio:
            raise HardwareError(
                f"{header}: {by_gpio[gpio]} and {role} both resolve to GPIO{gpio}. "
                f"Give one of them another pin, or define it as the other's name "
                f"if they are meant to share it"
            )
        by_gpio[gpio] = role

    def resolve(where: str, role: str) -> str:
        if role not in roles:
            what = "is not a pin role" if role in defines else "is not defined"
            raise HardwareError(
                f"{where}: role {role!r} {what} in {header.name} ({convention} convention)"
            )
        return role

    def entries(key: str, kind: str, check=lambda where, table: None) -> Entries:
        return _entries(sidecar, data, key, kind, view, check)

    net_entries = entries("nets", "net")
    undrawn_entries = entries("undrawn", "undrawn")
    channel_entries = entries("channel_nets", "channel_net")
    bus_entries = entries("bus_nets", "bus_net")
    rail_entries = entries("rails", "rail", _rail_name)
    output_entries = entries("outputs", "output", _output_pins)
    cable_entries = entries("cables", "cable", _cable)
    net_names = _read_net_names(sidecar, data)

    # Only a sidecar that wires channels needs a channel convention, so a project
    # adopting hardware.toml extends CONVENTIONS alone until it has a PWM driver.
    channels = (
        channels_from_defines(parse_defines([header]), convention)
        if channel_entries
        else {}
    )
    bus_channels = (
        bus_channels_from_defines(parse_defines([header]), convention)
        if bus_entries
        else {}
    )

    parts_table = data.get("parts", {})
    if not isinstance(parts_table, dict):
        raise HardwareError(f"{sidecar}: parts must be [parts.<id>] tables")
    parts: dict[str, Part] = {}
    dropped: dict[str, str] = {}  # part id -> the build it belongs to instead
    for key, table in parts_table.items():
        where = f"{sidecar} [parts.{key}]"
        if key == MCU:
            raise HardwareError(
                f"{where}: the part id {MCU!r} is reserved for the MCU board"
            )
        _check_keys(where, "part", table)
        in_view = _in_view(where, table, view)
        board = ""
        if "board" in table:
            board = _require(where, table, "board")
            if not (repo_root / board).is_file():
                raise HardwareError(
                    f"{where}: board reference {repo_root / board} does not exist"
                )
        part = Part(
            key=key,
            name=_require(where, table, "name"),
            kind=_require(where, table, "kind"),
            note=table.get("note", ""),
            board=board,
            part=_optional(where, table, "part"),
            ref=_optional(where, table, "ref", _REF),
            value=_optional(where, table, "value"),
            lcsc=_optional(where, table, "lcsc", _LCSC),
            build=table.get("build", ""),
        )
        if in_view:
            parts[key] = part
        else:
            dropped[key] = part.build

    refs: dict[str, str] = {mcu_ref: "[source]"} if mcu_ref else {}
    for key, part in parts.items():
        if part.ref:
            if part.ref in refs:
                raise HardwareError(
                    f"{sidecar}: ref {part.ref!r} is used by {refs[part.ref]} "
                    f"and [parts.{key}]"
                )
            refs[part.ref] = f"[parts.{key}]"
    # The head board and the main board are one carrier with one cable (#750).
    if len(cable_entries) > 1:
        raise HardwareError(
            f"{cable_entries[1][0]}: one [[cables]] entry per carrier; "
            f"{cable_entries[0][0]} is the first"
        )
    cables = [_cable(where, table) for where, table in cable_entries]
    for (where, _), cable in zip(cable_entries, cables, strict=True):
        label = where.removeprefix(f"{sidecar} ")
        for ref in (cable.head_ref, cable.main_ref):
            if ref in refs:
                raise HardwareError(
                    f"{sidecar}: ref {ref!r} is used by {refs[ref]} and {label}"
                )
            refs[ref] = label

    def no_part(where: str, key: Any) -> HardwareError:
        if isinstance(key, str) and key in dropped:
            return HardwareError(
                f"{where}: part {key!r} is build = {dropped[key]!r} and not in "
                f"the {view} view"
            )
        return HardwareError(f"{where}: part {key!r} is not declared under [parts]")

    # One role on several nets is a fan-out (a bus pin to two devices) and is
    # allowed; the same role to the same endpoint twice is a copy-paste slip.
    nets: list[Net] = []
    for where, table in net_entries:
        role = resolve(where, _require(where, table, "role"))
        part, dot, pin = _require(where, table, "to").partition(".")
        if not dot or not part or not pin:
            raise HardwareError(
                f"{where}: 'to' must be 'part.PIN', got {table['to']!r}"
            )
        if part not in parts:
            raise no_part(where, part)
        if any(n.role == role and n.part == part and n.pin == pin for n in nets):
            raise HardwareError(f"{where}: {role} -> {part}.{pin} is listed twice")
        # Two roles onto one pin shorts their GPIOs together (#722).
        other = next((n.role for n in nets if (n.part, n.pin) == (part, pin)), None)
        if other is not None:
            raise HardwareError(
                f"{where}: {part}.{pin} is driven by both {other} and {role}"
            )
        nets.append(Net(role=role, part=part, pin=pin, note=table.get("note", "")))

    wired = {n.role for n in nets}
    undrawn: list[Undrawn] = []
    for where, table in undrawn_entries:
        role = resolve(where, _require(where, table, "role"))
        if role in wired:
            raise HardwareError(
                f"{where}: role {role!r} is on a net and excused as undrawn"
            )
        if any(u.role == role for u in undrawn):
            raise HardwareError(f"{where}: role {role!r} is excused twice")
        undrawn.append(Undrawn(role=role, why=_require(where, table, "why")))

    # Completeness (#460): every pin role in `header` is drawn or excused, so a
    # macro added to pin_config.h cannot pass every gate while on no net. Pin
    # roles only; channel roles (a PWM driver's outputs) are out of scope.
    missing = sorted(set(roles) - wired - {u.role for u in undrawn})
    if missing:
        listing = ", ".join(f"{r} (GPIO{roles[r]})" for r in missing)
        raise HardwareError(
            f"{sidecar}: pin role(s) on no net and not excused: {listing}. "
            f"Add a [[nets]] entry for each, or list it under [[undrawn]] with a `why`"
        )

    channel_nets = _channel_nets(
        channel_entries,
        parts,
        no_part,
        nets,
        channels,
        defines,
        header,
        repo_root,
    )

    bus_nets = _bus_nets(
        bus_entries,
        parts,
        no_part,
        nets,
        channel_nets,
        bus_channels,
        defines,
        header,
        repo_root,
    )

    board = parse_board_table(board_path)
    mcu = source.get("mcu", "MCU")
    if not isinstance(mcu, str) or not mcu:
        raise HardwareError(f"{sidecar} [source]: mcu must be a name, got {mcu!r}")
    rails, outputs = _power(
        view,
        rail_entries,
        output_entries,
        parts,
        no_part,
        nets,
        channel_nets,
        bus_nets,
        roles,
        board,
        repo_root,
    )
    _head_no_connect(cable_entries, cables, nets, rails, roles, board)
    if view == CARRIER:
        _net_names(sidecar, net_names, nets, channel_nets, bus_nets, rails, outputs)

    return HardwareModel(
        project_dir=project_dir,
        convention=convention,
        headers=headers,
        defines=defines,
        roles=roles,
        board=board,
        parts=parts,
        nets=tuple(nets),
        undrawn=tuple(undrawn),
        rails=rails,
        outputs=outputs,
        mcu=mcu,
        channels=channels,
        channel_nets=tuple(channel_nets),
        view=view,
        net_names=net_names,
        mcu_part=mcu_part,
        mcu_ref=mcu_ref,
        bus_channels=bus_channels,
        bus_nets=tuple(bus_nets),
        cables=tuple(cables),
    )
