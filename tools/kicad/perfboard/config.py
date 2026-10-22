"""perfboard.toml: a project's perfboard layout, loaded and validated.

    skip = ["H1"]                    # optional: refs to leave off the perfboard
                                     # (parts with no pins are left off anyway)
    [board]
    title = "Balancebot perfboard"   # the wire list's heading: "<title> wire list"
    cols = 18
    rows = 24

    [wires]
    heavy_class = "power"            # a [[net_classes]] name in ../carrier/board.toml
    heavy_label = "VBAT, GND, motor coils"
    heavy_gauge = "20 AWG"
    light_gauge = "26–30 AWG"

    [placement]                      # ref = [column, row, rotation_deg]
    U1 = [3, 3, 90]

    [swaps.J1]                       # optional; overrides the registry's [perfboard]
    footprint = "Lib:Name"           # vendored, with a through-hole pad for
    value = "XT30 pigtail"           # every netlist pin of J1

    [[steps]]                        # one solder-side panel each, in order
    title = "Power"
    subtitle = "VBAT, GND and the 5 V feed"
    nets = ["VBAT", "GND"]           # at most len(PALETTE) = 8: one colour each

    [continuity]                     # optional: rails continuity.md checks
    isolated = ["VBAT", "GND"]       # pairwise for no short before power-up

`load` checks keys and types and what needs no netlist: a step's net count
against the palette, a net in two steps, rotations. `check_steps`,
`check_isolated` and the placement checks in `place` compare it with the
netlist.
"""

from __future__ import annotations

import tomllib
from dataclasses import dataclass
from pathlib import Path
from typing import Any

from .. import library
from ..errors import KicadError
from .grid import Board
from .render_svg import PALETTE

FILENAME = "perfboard.toml"
# The git-tracked file that makes a directory a perfboard project, relative to it.
MARKER = f"hardware/perfboard/{FILENAME}"
TOP_KEYS = frozenset(
    {"skip", "board", "wires", "placement", "swaps", "steps", "continuity"}
)
BOARD_KEYS = frozenset({"title", "cols", "rows"})
WIRES_KEYS = frozenset({"heavy_class", "heavy_label", "heavy_gauge", "light_gauge"})
SWAP_KEYS = frozenset({"footprint", "value"})
STEP_KEYS = frozenset({"title", "subtitle", "nets"})
CONTINUITY_KEYS = frozenset({"isolated"})
ROTATIONS = (0, 90, 180, 270)


@dataclass(frozen=True)
class Step:
    title: str
    subtitle: str
    nets: tuple[str, ...]


@dataclass(frozen=True)
class Swap:
    footprint: str  # "" keeps the registry's
    value: str  # "" keeps the registry's


@dataclass(frozen=True)
class PerfboardConfig:
    path: Path
    title: str
    cols: int
    rows: int
    heavy_class: str
    heavy_label: str
    heavy_gauge: str
    light_gauge: str
    placement: dict[str, tuple[int, int, int]]  # ref -> (column, row, rotation)
    steps: tuple[Step, ...]
    swaps: dict[str, Swap]
    skip: frozenset[str]
    isolated: tuple[str, ...] = ()  # rails that must not beep to each other

    @property
    def board(self) -> Board:
        return Board(self.cols, self.rows)


def perfboard_dir(project: Path) -> Path:
    return project / "hardware" / "perfboard"


def load(path: Path, shown: Path | None = None) -> PerfboardConfig:
    """Read and validate `path`.

    `shown` is the path errors name and `PerfboardConfig.path` keeps (the CLI
    passes it repo-relative); default `path`.
    """
    shown = shown or path
    try:
        data = tomllib.loads(path.read_text(encoding="utf-8"))
    except FileNotFoundError:
        raise KicadError(f"{shown}: no {FILENAME}") from None
    except tomllib.TOMLDecodeError as e:
        raise KicadError(f"{shown}: {e}") from None
    return parse(data, shown)


def parse(data: dict[str, Any], path: Path) -> PerfboardConfig:
    _keys(path, "", data, TOP_KEYS)
    board = _table(path, data, "board", BOARD_KEYS)
    wires = _table(path, data, "wires", WIRES_KEYS)
    where = f"{path} [board]"
    cols = _count(where, board, "cols")
    rows = _count(where, board, "rows")
    return PerfboardConfig(
        path=path,
        title=_string(where, board, "title"),
        cols=cols,
        rows=rows,
        heavy_class=_string(f"{path} [wires]", wires, "heavy_class"),
        heavy_label=_string(f"{path} [wires]", wires, "heavy_label"),
        heavy_gauge=_string(f"{path} [wires]", wires, "heavy_gauge"),
        light_gauge=_string(f"{path} [wires]", wires, "light_gauge"),
        placement=_placement(path, data.get("placement")),
        steps=_steps(path, data.get("steps")),
        swaps=_swaps(path, data.get("swaps", {})),
        skip=_skip(path, data.get("skip", [])),
        isolated=_isolated(path, data.get("continuity", {})),
    )


def check_steps(cfg: PerfboardConfig, nets: dict) -> list[str]:
    """Every net of the netlist in exactly one step, and no step net unknown."""
    grouped = {n for s in cfg.steps for n in s.nets}
    missing, extra = set(nets) - grouped, grouped - set(nets)
    if missing or extra:
        return [
            f"{cfg.path} [[steps]] out of step with the netlist: "
            f"missing {sorted(missing)}, unknown {sorted(extra)}"
        ]
    return []


def check_isolated(cfg: PerfboardConfig, nets: dict) -> list[str]:
    """Every `[continuity] isolated` rail is a net of the netlist."""
    unknown = [n for n in cfg.isolated if n not in nets]
    if unknown:
        return [f"{cfg.path} [continuity] isolated: not a net: {unknown}"]
    return []


def _mapping(where: str, data: Any) -> None:
    if not isinstance(data, dict):
        raise KicadError(f"{where}: expected a table, got {data!r}")


def _keys(path: Path, table: str, data: Any, allowed: frozenset[str]) -> None:
    where = f"{path} {table}".rstrip()
    _mapping(where, data)
    unknown = sorted(set(data) - allowed)
    if unknown:
        raise KicadError(
            f"{where}: unknown key(s) {unknown}; allowed: {sorted(allowed)}"
        )


def _table(
    path: Path, data: dict[str, Any], name: str, allowed: frozenset[str]
) -> dict[str, Any]:
    if name not in data:
        raise KicadError(f"{path}: [{name}] is required")
    _keys(path, f"[{name}]", data[name], allowed)
    return data[name]


def _string(where: str, table: dict[str, Any], key: str) -> str:
    value = table.get(key)
    if not isinstance(value, str) or not value:
        raise KicadError(f"{where}: {key} must be a non-empty string, got {value!r}")
    return value


def _int(value: Any) -> bool:
    return isinstance(value, int) and not isinstance(value, bool)


def _count(where: str, table: dict[str, Any], key: str) -> int:
    value = table.get(key)
    if not _int(value) or value < 1:
        raise KicadError(f"{where}: {key} must be a positive integer, got {value!r}")
    return value


def _placement(path: Path, table: Any) -> dict[str, tuple[int, int, int]]:
    if table is None:
        raise KicadError(f"{path}: [placement] is required")
    _mapping(f"{path} [placement]", table)
    out = {}
    for ref, at in table.items():
        if (
            not isinstance(at, list)
            or len(at) != 3
            or not all(_int(v) for v in at)
            or at[2] not in ROTATIONS
        ):
            raise KicadError(
                f"{path} [placement] {ref}: expected [column, row, rotation] with "
                f"integer holes and a rotation in {list(ROTATIONS)}, got {at!r}"
            )
        out[ref] = (at[0], at[1], at[2])
    return out


def _steps(path: Path, steps: Any) -> tuple[Step, ...]:
    if not isinstance(steps, list) or not steps:
        raise KicadError(f"{path}: at least one [[steps]] is required")
    out = []
    owner: dict[str, str] = {}
    for i, step in enumerate(steps, 1):
        where = f"{path} [[steps]] #{i}"
        _keys(path, f"[[steps]] #{i}", step, STEP_KEYS)
        title = _string(where, step, "title")
        subtitle = _string(where, step, "subtitle")
        nets = step.get("nets")
        if (
            not isinstance(nets, list)
            or not nets
            or not all(isinstance(n, str) and n for n in nets)
        ):
            raise KicadError(f"{where}: nets must be a non-empty list of net names")
        if len(nets) > len(PALETTE):
            raise KicadError(
                f"{where} ({title!r}) has {len(nets)} nets; a step can colour at most "
                f"{len(PALETTE)} (the palette's slots), so split it into two steps"
            )
        for net in nets:
            if net in owner:
                raise KicadError(
                    f"{where}: net {net!r} is already in step {owner[net]!r}; "
                    "every net belongs to exactly one step"
                )
            owner[net] = title
        out.append(Step(title, subtitle, tuple(nets)))
    return tuple(out)


def _swaps(path: Path, table: Any) -> dict[str, Swap]:
    _mapping(f"{path} [swaps]", table)
    out = {}
    for ref, swap in table.items():
        where = f"{path} [swaps.{ref}]"
        _keys(path, f"[swaps.{ref}]", swap, SWAP_KEYS)
        if not swap:
            raise KicadError(f"{where}: give a footprint, a value, or both")
        footprint = _string(where, swap, "footprint") if "footprint" in swap else ""
        if footprint:
            try:
                library.split_id(footprint)
            except KicadError as e:
                raise KicadError(f"{where}: footprint: {e}") from None
        value = _string(where, swap, "value") if "value" in swap else ""
        out[ref] = Swap(footprint, value)
    return out


def _skip(path: Path, skip: Any) -> frozenset[str]:
    if not isinstance(skip, list) or not all(isinstance(r, str) and r for r in skip):
        raise KicadError(f"{path}: skip must be a list of references, got {skip!r}")
    return frozenset(skip)


def _isolated(path: Path, table: Any) -> tuple[str, ...]:
    _keys(path, "[continuity]", table, CONTINUITY_KEYS)
    nets = table.get("isolated", [])
    where = f"{path} [continuity] isolated"
    if not isinstance(nets, list) or not all(isinstance(n, str) and n for n in nets):
        raise KicadError(f"{where}: expected a list of net names, got {nets!r}")
    if len(nets) == 1 or len(set(nets)) != len(nets):
        raise KicadError(f"{where}: needs two or more distinct nets, got {nets!r}")
    return tuple(nets)
