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

`[[undrawn]]` is an array of tables rather than ADR-021's `undrawn = [...]`
key: written after a `[[nets]]` block, that key would belong to the last net.

Every pin role in `header` must be on a `[[nets]]` entry or listed under
`[[undrawn]]`; `join()` raises naming each one that is neither (#460). The check
covers pin roles only: channel roles (`*_CHANNEL`) are not required to be wired,
since some, such as `MOTOR_FIRST_CHANNEL`, are aliases the firmware reads.
"""

from __future__ import annotations

import tomllib
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

from .board import Board, BoardPin, parse_board_table
from .errors import HardwareError
from .header import channels_from_defines, parse_defines, roles_from_defines
from .layout import parse_layout

SIDECAR = "hardware.toml"
REPO_ROOT = Path(__file__).resolve().parents[2]

_KEYS = {
    "": {"source", "parts", "nets", "channel_nets", "undrawn", "rails", "outputs"},
    "source": {"convention", "header", "extra_headers", "board", "mcu"},
    "part": {"name", "kind", "note", "board"},
    "net": {"role", "to", "note"},
    "channel_net": {"role", "from", "to", "note"},
    "undrawn": {"role", "why"},
    "rail": {"name", "from", "to"},
    "output": {"from", "to"},
}
MCU = "mcu"  # the reserved part id a rail endpoint uses for the MCU board


@dataclass(frozen=True)
class Part:
    key: str  # the id nets refer to: "amp"
    name: str  # "MAX98357A"
    kind: str  # "i2s-amp"
    note: str = ""
    board: str = ""  # repo-relative physical-layout page; "" for none


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


@dataclass(frozen=True)
class HardwareModel:
    project_dir: Path
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

    def pin_for(self, role: str) -> BoardPin | None:
        """The header pad a role lands on, or None if its GPIO is not broken out."""
        return self.board.by_gpio.get(self.roles[role])


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


def _power(
    sidecar: Path,
    data: dict[str, Any],
    parts: dict[str, Part],
    nets: list[Net],
    channel_nets: list[ChannelNet],
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

    def endpoint(where: str, value: Any) -> Endpoint:
        part, dot, pin = (
            value.partition(".") if isinstance(value, str) else ("", "", "")
        )
        if not dot or not part or not pin:
            raise HardwareError(f"{where}: endpoint must be 'part.PIN', got {value!r}")
        if part != MCU and part not in parts:
            raise HardwareError(f"{where}: part {part!r} is not declared under [parts]")
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

    def array(key: str) -> list[Any]:
        value = data.get(key, [])
        if not isinstance(value, list):
            raise HardwareError(f"{sidecar}: {key} must be [[{key}]] tables")
        return value

    def targets(where: str, table: dict[str, Any], what: str) -> list[Any]:
        value = table.get("to")
        if not isinstance(value, list) or not value:
            raise HardwareError(f"{where}: 'to' must be a non-empty list of {what}")
        return value

    rails: list[Rail] = []
    on_rail: dict[Endpoint, str] = {}
    for i, table in enumerate(array("rails")):
        where = f"{sidecar} [[rails]] #{i + 1}"
        _check_keys(where, "rail", table)
        name = _require(where, table, "name")
        source = endpoint(where, _require(where, table, "from"))
        loads = tuple(endpoint(where, v) for v in targets(where, table, "'part.PIN'"))
        dupes = sorted({f"{e.part}.{e.pin}" for e in loads if loads.count(e) > 1})
        if dupes:
            raise HardwareError(f"{where}: {dupes} listed twice")
        for e in (source, *loads):
            if e in on_rail:
                raise HardwareError(
                    f"{where}: {e.part}.{e.pin} is on rail {on_rail[e]!r} "
                    f"and rail {name!r} — that is a short"
                )
            on_rail[e] = name
        rails.append(Rail(name, source, loads))

    outputs: list[Output] = []
    for i, table in enumerate(array("outputs")):
        where = f"{sidecar} [[outputs]] #{i + 1}"
        _check_keys(where, "output", table)
        source = _require(where, table, "from")
        loads = targets(where, table, "part ids")
        for key in (source, *loads):
            if not isinstance(key, str) or key not in parts:
                raise HardwareError(
                    f"{where}: part {key!r} is not declared under [parts]"
                )
        dupes = sorted({k for k in loads if loads.count(k) > 1})
        if dupes:
            raise HardwareError(f"{where}: {dupes} listed twice")
        outputs.append(Output(part=source, loads=tuple(loads)))

    return tuple(rails), tuple(outputs)


def _channel_nets(
    sidecar: Path,
    tables: list[Any],
    parts: dict[str, Part],
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
    for i, table in enumerate(tables):
        where = f"{sidecar} [[channel_nets]] #{i + 1}"
        _check_keys(where, "channel_net", table)
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
                raise HardwareError(
                    f"{where}: part {key!r} is not declared under [parts]"
                )
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


def join(project_dir: Path, repo_root: Path = REPO_ROOT) -> HardwareModel:
    """Parse `project_dir/hardware.toml` and the files it names into one model."""
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

    defines = parse_defines(headers)
    roles = roles_from_defines(parse_defines([header]), convention)

    def resolve(where: str, role: str) -> str:
        if role not in roles:
            what = "is not a pin role" if role in defines else "is not defined"
            raise HardwareError(
                f"{where}: role {role!r} {what} in {header.name} ({convention} convention)"
            )
        return role

    def array(key: str) -> list[Any]:
        value = data.get(key, [])
        if not isinstance(value, list):
            raise HardwareError(f"{sidecar}: {key} must be [[{key}]] tables")
        return value

    # Only a sidecar that wires channels needs a channel convention, so a project
    # adopting hardware.toml extends CONVENTIONS alone until it has a PWM driver.
    channels = (
        channels_from_defines(parse_defines([header]), convention)
        if array("channel_nets")
        else {}
    )

    parts_table = data.get("parts", {})
    if not isinstance(parts_table, dict):
        raise HardwareError(f"{sidecar}: parts must be [parts.<id>] tables")
    parts: dict[str, Part] = {}
    for key, table in parts_table.items():
        where = f"{sidecar} [parts.{key}]"
        if key == MCU:
            raise HardwareError(
                f"{where}: the part id {MCU!r} is reserved for the MCU board"
            )
        _check_keys(where, "part", table)
        board = ""
        if "board" in table:
            board = _require(where, table, "board")
            if not (repo_root / board).is_file():
                raise HardwareError(
                    f"{where}: board reference {repo_root / board} does not exist"
                )
        parts[key] = Part(
            key=key,
            name=_require(where, table, "name"),
            kind=_require(where, table, "kind"),
            note=table.get("note", ""),
            board=board,
        )

    # One role on several nets is a fan-out (a bus pin to two devices) and is
    # allowed; the same role to the same endpoint twice is a copy-paste slip.
    nets: list[Net] = []
    for i, table in enumerate(array("nets")):
        where = f"{sidecar} [[nets]] #{i + 1}"
        _check_keys(where, "net", table)
        role = resolve(where, _require(where, table, "role"))
        part, dot, pin = _require(where, table, "to").partition(".")
        if not dot or not part or not pin:
            raise HardwareError(
                f"{where}: 'to' must be 'part.PIN', got {table['to']!r}"
            )
        if part not in parts:
            raise HardwareError(f"{where}: part {part!r} is not declared under [parts]")
        if any(n.role == role and n.part == part and n.pin == pin for n in nets):
            raise HardwareError(f"{where}: {role} -> {part}.{pin} is listed twice")
        nets.append(Net(role=role, part=part, pin=pin, note=table.get("note", "")))

    wired = {n.role for n in nets}
    undrawn: list[Undrawn] = []
    for i, table in enumerate(array("undrawn")):
        where = f"{sidecar} [[undrawn]] #{i + 1}"
        _check_keys(where, "undrawn", table)
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
        sidecar,
        array("channel_nets"),
        parts,
        nets,
        channels,
        defines,
        header,
        repo_root,
    )

    board = parse_board_table(board_path)
    mcu = source.get("mcu", "MCU")
    if not isinstance(mcu, str) or not mcu:
        raise HardwareError(f"{sidecar} [source]: mcu must be a name, got {mcu!r}")
    rails, outputs = _power(
        sidecar, data, parts, nets, channel_nets, roles, board, repo_root
    )

    return HardwareModel(
        project_dir=project_dir,
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
    )
