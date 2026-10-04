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
    name = "MAX98357A"
    kind = "i2s-amp"

    [[nets]]
    role = "I2S_BCLK_PIN"
    to   = "amp.BCLK"
    note = "bit clock"          # optional

    [[undrawn]]                 # a pin role deliberately on no drawn net
    role = "MIC_PDM_CLK_PIN"
    why  = "internal to the Sense module"

`[[undrawn]]` is an array of tables rather than ADR-021's `undrawn = [...]`
key: written after a `[[nets]]` block, that key would belong to the last net.
"""

from __future__ import annotations

import tomllib
from dataclasses import dataclass
from pathlib import Path
from typing import Any

from .board import Board, BoardPin, parse_board_table
from .errors import HardwareError
from .header import parse_defines, roles_from_defines

SIDECAR = "hardware.toml"
REPO_ROOT = Path(__file__).resolve().parents[2]

_KEYS = {
    "": {"source", "parts", "nets", "undrawn"},
    "source": {"convention", "header", "extra_headers", "board"},
    "part": {"name", "kind", "note"},
    "net": {"role", "to", "note"},
    "undrawn": {"role", "why"},
}


@dataclass(frozen=True)
class Part:
    key: str  # the id nets refer to: "amp"
    name: str  # "MAX98357A"
    kind: str  # "i2s-amp"


@dataclass(frozen=True)
class Net:
    role: str
    part: str
    pin: str
    note: str


@dataclass(frozen=True)
class Undrawn:
    role: str
    why: str


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

    def pin_for(self, role: str) -> BoardPin | None:
        """The header pad a role lands on, or None if its GPIO is not broken out."""
        return self.board.by_gpio.get(self.roles[role])


def _check_keys(where: str, kind: str, table: dict[str, Any]) -> None:
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


def join(project_dir: Path, repo_root: Path = REPO_ROOT) -> HardwareModel:
    """Parse `project_dir/hardware.toml` and the files it names into one model."""
    project_dir = project_dir.resolve()
    sidecar = project_dir / SIDECAR
    if not sidecar.is_file():
        raise HardwareError(f"{sidecar}: not found — this project has no {SIDECAR}")
    try:
        data = tomllib.loads(sidecar.read_text())
    except tomllib.TOMLDecodeError as e:
        raise HardwareError(f"{sidecar}: {e}") from e
    _check_keys(str(sidecar), "", data)

    source = data.get("source", {})
    _check_keys(f"{sidecar} [source]", "source", source)
    convention = _require(f"{sidecar} [source]", source, "convention")
    header = project_dir / _require(f"{sidecar} [source]", source, "header")
    extras = [project_dir / p for p in source.get("extra_headers", [])]
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

    parts: dict[str, Part] = {}
    for key, table in data.get("parts", {}).items():
        where = f"{sidecar} [parts.{key}]"
        _check_keys(where, "part", table)
        parts[key] = Part(
            key=key,
            name=_require(where, table, "name"),
            kind=_require(where, table, "kind"),
        )

    nets: list[Net] = []
    for i, table in enumerate(data.get("nets", [])):
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
        nets.append(Net(role=role, part=part, pin=pin, note=table.get("note", "")))

    wired = {n.role for n in nets}
    undrawn: list[Undrawn] = []
    for i, table in enumerate(data.get("undrawn", [])):
        where = f"{sidecar} [[undrawn]] #{i + 1}"
        _check_keys(where, "undrawn", table)
        role = resolve(where, _require(where, table, "role"))
        if role in wired:
            raise HardwareError(
                f"{where}: role {role!r} is on a net and excused as undrawn"
            )
        undrawn.append(Undrawn(role=role, why=_require(where, table, "why")))

    return HardwareModel(
        project_dir=project_dir,
        headers=headers,
        defines=defines,
        roles=roles,
        board=parse_board_table(board_path),
        parts=parts,
        nets=tuple(nets),
        undrawn=tuple(undrawn),
    )
