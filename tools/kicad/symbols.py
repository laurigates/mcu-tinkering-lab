"""Library symbols: pins, flags, extent, and the `lib_symbols` embedding.

A `.kicad_sym` file is a header (version, generator) followed by top-level
`(symbol "Name" ...)` blocks at one tab of indent. Each holds its properties
and flags, then sub-unit blocks `(symbol "Name_<unit>_<style>" ...)` with the
graphics and pins: unit 0 is shared by every unit, style 1 is the normal body
and style 2 the De Morgan alternate.

The schematic writer places `(unit 1)(body_style 1)` only, so `pins()` and
`extent()` read units 0-1 and styles 0-1 and nothing else; the registry
rejects a symbol that has more.
"""

from __future__ import annotations

import math
import re
from collections.abc import Iterable
from dataclasses import dataclass
from pathlib import Path

from . import library, sexpr
from .errors import KicadError

# KiCad's value for a flag the symbol does not state.
FLAG_DEFAULTS = {
    "exclude_from_sim": False,
    "in_bom": True,
    "on_board": True,
    "in_pos_files": True,
}
_GRAPHICS = frozenset({"rectangle", "polyline", "circle", "arc", "bezier"})


@dataclass(frozen=True)
class Pin:
    """One symbol pin, in library coordinates (y points up).

    `x`, `y` is the connection point; `angle` points from it toward the body.
    """

    number: str
    name: str
    etype: str  # passive, power_in, input, output, no_connect, ...
    shape: str  # line, inverted, clock, ...
    x: float
    y: float
    angle: int
    length: float
    hidden: bool
    unit: int
    style: int


def header(lib_text: str) -> str:
    """The library text before its first symbol: `(kicad_symbol_lib` + version lines."""
    if not lib_text.startswith("(kicad_symbol_lib"):
        raise KicadError("not a KiCad symbol library: missing '(kicad_symbol_lib'")
    for start, block in sexpr.top_level_blocks(lib_text, "\t"):
        if sexpr.head(block)[0] == "symbol":
            return lib_text[:start]
    raise KicadError("symbol library holds no symbols")


def blocks(lib_text: str) -> dict[str, str]:
    """Symbol name -> its top-level block (indent included), in file order."""
    out = {}
    for _, block in sexpr.top_level_blocks(lib_text, "\t"):
        kw, name = sexpr.head(block)
        if kw == "symbol" and name is not None:
            out[name] = block
    return out


def find(lib_text: str, name: str, where: str = "library") -> str:
    block = blocks(lib_text).get(name)
    if block is None:
        raise KicadError(f"{where}: no symbol {name!r}")
    if sexpr.child(block, "extends") is not None:
        raise KicadError(
            f"{where}: symbol {name!r} uses (extends ...); derived symbols are not supported"
        )
    return block


def load(
    lib_id: str,
    vendored_only: bool = True,
    search_roots: Iterable[Path] | None = None,
    vendored: Path = library.VENDORED,
) -> str:
    """The top-level block of library symbol `"Lib:Name"`, indent included."""
    nick, name = library.split_id(lib_id)
    path = library.symbol_lib(nick, vendored_only, search_roots, vendored)
    return find(path.read_text(encoding="utf-8"), name, str(path))


def _units(block: str) -> list[tuple[int, int, str]]:
    """(unit, style, sub-block) for every sub-unit of a symbol block."""
    _, name = sexpr.head(block)
    pat = re.compile(rf"{re.escape(name or '')}_(\d+)_(\d+)")
    out = []
    for c in sexpr.children(block):
        kw, sub = sexpr.head(c)
        if kw != "symbol":
            continue
        m = pat.fullmatch(sub or "")
        if not m:
            raise KicadError(
                f"symbol {name!r}: sub-unit {sub!r} is not '{name}_<unit>_<style>'"
            )
        out.append((int(m.group(1)), int(m.group(2)), c))
    return out


def _drawn(block: str) -> list[tuple[int, int, str]]:
    return [(u, s, c) for u, s, c in _units(block) if u <= 1 and s <= 1]


def all_pins(block: str) -> list[Pin]:
    """Every pin of every unit and style, in file order."""
    return [p for u, s, c in _units(block) for p in _pins_in(c, u, s)]


def pins(block: str) -> list[Pin]:
    """The pins the placed (unit 1, style 1) symbol carries, in file order."""
    return [p for u, s, c in _drawn(block) for p in _pins_in(c, u, s)]


def _pins_in(unit_block: str, unit: int, style: int) -> list[Pin]:
    out = []
    for c in sexpr.children(unit_block):
        if sexpr.head(c)[0] != "pin":
            continue
        kinds = sexpr.atoms(c)
        at = sexpr.child(c, "at")
        length = sexpr.child(c, "length")
        name = sexpr.child(c, "name")
        number = sexpr.child(c, "number")
        if at is None or length is None or name is None or number is None:
            raise KicadError(f"pin without at/length/name/number: {c[:80]!r}")
        ax = sexpr.atoms(at)
        hide = sexpr.child(c, "hide")
        hidden = "hide" in kinds[2:] or (
            hide is not None and sexpr.atoms(hide) == ["yes"]
        )
        out.append(
            Pin(
                number=sexpr.head(number)[1] or "",
                name=sexpr.head(name)[1] or "",
                etype=kinds[0],
                shape=kinds[1] if len(kinds) > 1 else "line",
                x=float(ax[0]),
                y=float(ax[1]),
                angle=int(float(ax[2])) if len(ax) > 2 else 0,
                length=float(sexpr.atoms(length)[0]),
                hidden=hidden,
                unit=unit,
                style=style,
            )
        )
    return out


def extent(block: str) -> tuple[float, float, float, float]:
    """(xmin, ymin, xmax, ymax), library coordinates, of the placed body and pins.

    Covers the graphics (rectangle, polyline, circle, arc, bezier) and both ends
    of every pin of units 0-1, styles 0-1. Properties and text do not count.
    (0, 0, 0, 0) for a symbol that draws nothing.
    """
    xs: list[float] = []
    ys: list[float] = []
    for _, _, unit_block in _drawn(block):
        for c in sexpr.children(unit_block):
            kw = sexpr.head(c)[0]
            if kw in _GRAPHICS:
                _graphic_points(c, xs, ys)
        for p in _pins_in(unit_block, 0, 0):
            rad = math.radians(p.angle)
            xs += [p.x, p.x + p.length * math.cos(rad)]
            ys += [p.y, p.y + p.length * math.sin(rad)]
    if not xs:
        return (0.0, 0.0, 0.0, 0.0)
    return (round(min(xs), 4), round(min(ys), 4), round(max(xs), 4), round(max(ys), 4))


def _graphic_points(item: str, xs: list[float], ys: list[float]) -> None:
    kw = sexpr.head(item)[0]
    if kw == "circle":
        center = sexpr.child(item, "center")
        radius = sexpr.child(item, "radius")
        if center is not None and radius is not None:
            cx, cy = (float(v) for v in sexpr.atoms(center)[:2])
            r = float(sexpr.atoms(radius)[0])
            xs += [cx - r, cx + r]
            ys += [cy - r, cy + r]
        return
    for x, y in re.findall(r"\((?:xy|start|end|mid) (-?[\d.]+) (-?[\d.]+)\)", item):
        xs.append(float(x))
        ys.append(float(y))


def flags(block: str) -> dict[str, bool]:
    """exclude_from_sim / in_bom / on_board / in_pos_files, KiCad's default if absent."""
    out = dict(FLAG_DEFAULTS)
    for key in FLAG_DEFAULTS:
        c = sexpr.child(block, key)
        if c is None:
            continue
        val = sexpr.atoms(c)
        if val not in (["yes"], ["no"]):
            raise KicadError(f"symbol {sexpr.head(block)[1]!r}: ({key} ...) is {val}")
        out[key] = val == ["yes"]
    return out


def embed(block: str, lib_id: str) -> str:
    """A library block as a schematic `lib_symbols` entry.

    One more tab on every line, the top block renamed to `"Lib:Name"` (sub-unit
    names stay), blank lines dropped. Byte-identical to KiCad's own embedding.
    No trailing newline.
    """
    _, name = library.split_id(lib_id)
    lines = block.split("\n")
    first = f'\t(symbol "{name}"'
    if lines[0] != first:
        raise KicadError(f"{lib_id}: block starts {lines[0]!r}, expected {first!r}")
    quoted = lib_id.replace("\\", "\\\\").replace('"', '\\"')
    out = [f'\t\t(symbol "{quoted}"']
    out += ["\t" + ln for ln in lines[1:] if ln.strip()]
    return "\n".join(out)
