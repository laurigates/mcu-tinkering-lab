"""The parts registry: `docs/reference/parts/<slug>.toml` (format: its README.md).

A `hardware.toml` part names its registry entry with `part = "<slug>"`. The
entry maps the pin names hardware.toml uses (a breakout's silkscreen names) to
the KiCad symbol's pin numbers, and names the symbol and footprint:

    [part]
    name        = "..."            # display
    source      = "..."            # where the pin order came from
    symbol      = "Lib:Name"       # must be vendored (tools/kicad/lib/)
    footprint   = "Lib:Name"       # must be vendored
    value       = "..."            # optional default Value
    lcsc        = "C123"           # optional default LCSC
    buy         = "aliexpress:123" # optional default hand-assembly source (kicad.sourcing)
    no_connect  = ["FAULT"]        # pin names allowed to stay unconnected
    pin_numbers = "board-dip-ccw"  # instead of [pins]: numbers from a board page

    [pins]                         # pin name -> symbol pin number(s)
    GND = [1, 7]

    [perfboard]                    # optional; read by the perfboard generator
    footprint = "Lib:Name"         # through-hole stand-in; default [part] footprint
    value     = "..."              # default: the instance's value
    height    = "low"              # "low" | "raised" | "tall"
    body      = [x0, y0, x1, y1]   # outline in holes from pin 1; default F.CrtYd

`load` parses and type-checks an entry; `validate` checks it against the
vendored library: every symbol pin mapped exactly once and a pad of the
footprint, a supported (single-unit, no De Morgan, no hidden power) symbol, and
the polarity tripwire. A polarised pin ("+"/"-", "K"/"A" on a part with a
"K", or "G"/"D"/"S" on a part with all three) must be proved by the symbol's own pin name, by the footprint's silkscreen
text, or by a pinned convention for the symbol (CONVENTIONS); an unproved one
fails rather than being skipped, because a reversed electrolytic or diode
agrees with every other check (netlist, board, DRC) built from the same mistake.
A `[perfboard]` stand-in footprint's own marks must agree too.

`pin_numbers = "board-dip-ccw"` reads the numbers off the part's board page
(the MCU's is `[source] board`): Side L pos n is pin n, Side R pos n is pin
2N+1-n, N pads per side. `pin_map` derives and checks that map.
"""

from __future__ import annotations

import math
import re
import tomllib
from dataclasses import dataclass
from pathlib import Path
from typing import Any

from hardware import Board
from hardware.model import BUY

from . import footprints, library, symbols
from .errors import KicadError
from .paths import REPO_ROOT

REGISTRY_REL = Path("docs/reference/parts")
TABLES = frozenset({"part", "pins", "perfboard"})
PART_KEYS = frozenset(
    {
        "name",
        "source",
        "symbol",
        "footprint",
        "value",
        "lcsc",
        "buy",
        "no_connect",
        "pin_numbers",
    }
)
PERFBOARD_KEYS = frozenset({"footprint", "value", "height", "body"})
HEIGHTS = ("low", "raised", "tall")
PIN_NUMBERINGS = ("board-dip-ccw",)
# Symbols whose polarity is in the graphic, not in a pin name.
# C_Polarized's pins are unnamed; its "+" mark is drawn on pin 1's side, which
# test_registry checks against the vendored symbol.
CONVENTIONS: dict[str, dict[str, str]] = {"Device:C_Polarized": {"+": "1", "-": "2"}}

_SLUG = re.compile(r"[a-z0-9][a-z0-9.-]*")
_LCSC = re.compile(r"C[0-9]+")


@dataclass(frozen=True)
class PerfboardSpec:
    footprint: str  # resolved: the [part] footprint when the table omits it
    value: str  # "" means the instance's value
    height: str  # one of HEIGHTS
    body: tuple[float, float, float, float] | None  # None means the F.CrtYd box


@dataclass(frozen=True)
class PartSpec:
    slug: str
    path: Path
    name: str
    source: str
    symbol: str
    footprint: str
    value: str
    lcsc: str
    buy: str  # "" or a hardware.model.BUY source
    no_connect: tuple[str, ...]
    pin_numbers: str  # "" or one of PIN_NUMBERINGS
    pins: dict[
        str, tuple[str, ...]
    ]  # pin name -> symbol pin numbers; {} with pin_numbers
    perfboard: PerfboardSpec


def entry_path(slug: str, repo_root: Path = REPO_ROOT) -> Path:
    return repo_root / REGISTRY_REL / f"{slug}.toml"


def slugs(repo_root: Path = REPO_ROOT) -> list[str]:
    """Every entry's slug, sorted. Raises when the registry is empty or absent."""
    found = sorted(p.stem for p in (repo_root / REGISTRY_REL).glob("*.toml"))
    if not found:
        raise KicadError(f"{repo_root / REGISTRY_REL}: no parts registry entries")
    return found


def load(slug: str, repo_root: Path = REPO_ROOT) -> PartSpec:
    """Parse and type-check one entry; `validate` checks it against the library."""
    if not isinstance(slug, str) or not _SLUG.fullmatch(slug):
        raise KicadError(f"{slug!r}: not a parts-registry slug ({_SLUG.pattern})")
    path = entry_path(slug, repo_root)
    try:
        data = tomllib.loads(path.read_text(encoding="utf-8"))
    except FileNotFoundError:
        raise KicadError(
            f"{path}: no parts-registry entry {slug!r} (docs/reference/parts/README.md)"
        ) from None
    except tomllib.TOMLDecodeError as e:
        raise KicadError(f"{path}: {e}") from None
    return parse(data, slug, path)


def load_all(repo_root: Path = REPO_ROOT) -> list[PartSpec]:
    return [load(s, repo_root) for s in slugs(repo_root)]


def parse(data: dict[str, Any], slug: str, path: Path) -> PartSpec:
    """A PartSpec from an entry's TOML tables; key and type checks only."""
    _keys(path, "", data, TABLES)
    part = data.get("part")
    if not isinstance(part, dict):
        raise KicadError(f"{path}: [part] is required")
    _keys(path, "[part]", part, PART_KEYS)
    where = f"{path} [part]"
    name = _string(where, part, "name", required=True)
    source = _string(where, part, "source", required=True)
    symbol = _lib_id(where, part, "symbol")
    footprint = _lib_id(where, part, "footprint")
    value = _string(where, part, "value")
    lcsc = _string(where, part, "lcsc")
    if lcsc and not _LCSC.fullmatch(lcsc):
        raise KicadError(f"{where}: lcsc {lcsc!r} does not match {_LCSC.pattern}")
    buy = _string(where, part, "buy")
    if buy and not BUY.fullmatch(buy):
        raise KicadError(f"{where}: buy {buy!r} does not match {BUY.pattern}")
    no_connect = part.get("no_connect", [])
    if not isinstance(no_connect, list) or not all(
        isinstance(n, str) and n for n in no_connect
    ):
        raise KicadError(
            f"{where}: no_connect must be a list of pin names, got {no_connect!r}"
        )
    if len(set(no_connect)) != len(no_connect):
        raise KicadError(f"{where}: no_connect lists a pin twice: {no_connect}")
    pin_numbers = _string(where, part, "pin_numbers")
    if pin_numbers and pin_numbers not in PIN_NUMBERINGS:
        raise KicadError(
            f"{where}: pin_numbers {pin_numbers!r}; allowed: {list(PIN_NUMBERINGS)}"
        )
    pins = _pins(path, data, pin_numbers)
    if not pin_numbers:
        unknown = sorted(set(no_connect) - set(pins))
        if unknown:
            raise KicadError(f"{where}: no_connect {unknown} not in [pins]")
    return PartSpec(
        slug=slug,
        path=path,
        name=name,
        source=source,
        symbol=symbol,
        footprint=footprint,
        value=value,
        lcsc=lcsc,
        buy=buy,
        no_connect=tuple(no_connect),
        pin_numbers=pin_numbers,
        pins=pins,
        perfboard=_perfboard(path, data.get("perfboard", {}), footprint),
    )


def _keys(path: Path, table: str, data: Any, allowed: frozenset[str]) -> None:
    where = f"{path} {table}".rstrip()
    if not isinstance(data, dict):
        raise KicadError(f"{where}: expected a table, got {data!r}")
    unknown = sorted(set(data) - allowed)
    if unknown:
        raise KicadError(
            f"{where}: unknown key(s) {unknown}; allowed: {sorted(allowed)}"
        )


def _string(where: str, table: dict[str, Any], key: str, required: bool = False) -> str:
    if key not in table:
        if required:
            raise KicadError(f"{where}: {key!r} is required")
        return ""
    value = table[key]
    if not isinstance(value, str) or not value:
        raise KicadError(f"{where}: {key} must be a non-empty string, got {value!r}")
    return value


def _lib_id(where: str, table: dict[str, Any], key: str) -> str:
    value = _string(where, table, key, required=True)
    try:
        library.split_id(value)
    except KicadError as e:
        raise KicadError(f"{where}: {key}: {e}") from None
    return value


def _pins(
    path: Path, data: dict[str, Any], pin_numbers: str
) -> dict[str, tuple[str, ...]]:
    if pin_numbers:
        if "pins" in data:
            raise KicadError(
                f"{path}: pin_numbers = {pin_numbers!r} and [pins] are mutually exclusive"
            )
        return {}
    if "pins" not in data:
        raise KicadError(f"{path}: [pins] is required (empty for a part with no pins)")
    table = data["pins"]
    if not isinstance(table, dict):
        raise KicadError(f"{path}: pins must be a [pins] table, got {table!r}")
    out: dict[str, tuple[str, ...]] = {}
    for name, raw in table.items():
        items = raw if isinstance(raw, list) else [raw]
        nums = []
        for item in items:
            if isinstance(item, bool) or not isinstance(item, int | str) or item == "":
                raise KicadError(
                    f"{path} [pins] {name}: expected a pin number or a list of them, "
                    f"got {raw!r}"
                )
            nums.append(str(item))
        if not nums:
            raise KicadError(f"{path} [pins] {name}: an empty list maps no pin")
        out[name] = tuple(nums)
    return out


def _perfboard(path: Path, table: Any, footprint: str) -> PerfboardSpec:
    _keys(path, "[perfboard]", table, PERFBOARD_KEYS)
    where = f"{path} [perfboard]"
    fp = _lib_id(where, table, "footprint") if "footprint" in table else footprint
    height = _string(where, table, "height") or "low"
    if height not in HEIGHTS:
        raise KicadError(f"{where}: height {height!r}; allowed: {list(HEIGHTS)}")
    body = table.get("body")
    if body is not None:
        if (
            not isinstance(body, list)
            or len(body) != 4
            or not all(
                isinstance(v, int | float) and not isinstance(v, bool) for v in body
            )
        ):
            raise KicadError(
                f"{where}: body must be 4 numbers [x0, y0, x1, y1], got {body!r}"
            )
        body = tuple(float(v) for v in body)
    return PerfboardSpec(
        footprint=fp, value=_string(where, table, "value"), height=height, body=body
    )


def validate(spec: PartSpec, vendored: Path = library.VENDORED) -> None:
    """Check an entry against the vendored symbol and footprints; raises KicadError."""
    block = symbols.load(spec.symbol, vendored=vendored)
    sym_pins = _supported_pins(spec, block)
    fp_path = library.footprint_file(spec.footprint, vendored=vendored)
    fp = footprints.parse(fp_path)
    numbers = {p.number for p in sym_pins}
    _pads_cover(spec, fp, numbers, spec.footprint, "[part] footprint")
    if not spec.pin_numbers:
        check_map(spec, spec.pins, sym_pins, f"{spec.path} [pins]")
        _polarity(spec, sym_pins, fp_path)
    pb_path = library.footprint_file(spec.perfboard.footprint, vendored=vendored)
    if pb_path != fp_path:
        pb = footprints.parse(pb_path)
        _pads_cover(
            spec, pb, numbers, spec.perfboard.footprint, "[perfboard] footprint"
        )
        if not spec.pin_numbers:
            _text_proofs(spec, pb_path)


def _supported_pins(spec: PartSpec, block: str) -> list[symbols.Pin]:
    """The placed pins, after rejecting what the schematic writer cannot place."""
    where = f"{spec.path}: symbol {spec.symbol}"
    extra = [p for p in symbols.all_pins(block) if p.unit >= 2 or p.style >= 2]
    if extra:
        raise KicadError(
            f"{where} has pins in unit >= 2 or body style 2 ({sorted({p.number for p in extra})}); "
            "the writer places unit 1, body style 1 only"
        )
    pins = symbols.pins(block)
    hidden = sorted(p.number for p in pins if p.hidden and p.etype == "power_in")
    if hidden:
        raise KicadError(
            f"{where} has hidden power_in pins {hidden}; KiCad joins them to a "
            "global net by name, which the netlist cannot see"
        )
    return pins


def _pads_cover(
    spec: PartSpec, fp: footprints.Footprint, numbers: set[str], fpid: str, what: str
) -> None:
    missing = sorted(numbers - set(fp.pads), key=_num_key)
    if missing:
        raise KicadError(
            f"{spec.path}: {what} {fpid} has no pad for symbol pin(s) {missing}"
        )


def check_map(
    spec: PartSpec,
    mapping: dict[str, tuple[str, ...]],
    sym_pins: list[symbols.Pin],
    where: str,
) -> None:
    """A pin-name -> numbers map covers the symbol's pins exactly once each.

    Also the pin-name rule: a name that is a symbol pin's name must map to that
    pin, so a K/A swap on a D_Schottky (K = 1, A = 2) fails here.
    """
    numbers = {p.number for p in sym_pins}
    seen: dict[str, str] = {}
    for name, nums in mapping.items():
        for n in nums:
            if n in seen:
                raise KicadError(
                    f"{where}: pin {n} is mapped by both {seen[n]!r} and {name!r}"
                )
            seen[n] = name
    unknown = sorted(set(seen) - numbers, key=_num_key)
    if unknown:
        raise KicadError(f"{where}: {spec.symbol} has no pin(s) {unknown}")
    missing = sorted(numbers - set(seen), key=_num_key)
    if missing:
        raise KicadError(
            f"{where}: symbol pin(s) {missing} of {spec.symbol} are not mapped"
        )
    by_name: dict[str, set[str]] = {}
    for p in sym_pins:
        if p.name:
            by_name.setdefault(p.name, set()).add(p.number)
    for name, nums in mapping.items():
        if name in by_name and set(nums) != by_name[name]:
            raise KicadError(
                f"{where}: {name!r} maps to {list(nums)}, but {spec.symbol} names "
                f"pin(s) {sorted(by_name[name], key=_num_key)} {name!r}"
            )
    etype = {p.number: p.etype for p in sym_pins}
    for name, nums in mapping.items():
        if any(etype[n] == "no_connect" for n in nums) and name not in spec.no_connect:
            raise KicadError(
                f"{where}: {name!r} is a no_connect pin of {spec.symbol}; list it under no_connect"
            )


def _polarity(spec: PartSpec, sym_pins: list[symbols.Pin], fp_path: Path) -> None:
    """Each polarised pin name is proved by a pin name, a convention or the silkscreen."""
    named = {p.name for p in sym_pins if p.name}
    convention = CONVENTIONS.get(spec.symbol, {})
    marks = _text_proofs(spec, fp_path)
    for key in polarised(spec):
        if key in convention and spec.pins[key] != (convention[key],):
            raise KicadError(
                f"{spec.path} [pins] {key!r} = {list(spec.pins[key])}, but {spec.symbol}'s "
                f"{key!r} is pin {convention[key]} (registry.CONVENTIONS)"
            )
        if key not in named and key not in convention and key not in marks:
            raise KicadError(
                f"{spec.path} [pins] {key!r}: polarity unproven. {spec.symbol} has no pin "
                f"named {key!r}, registry.CONVENTIONS has no entry for it, and "
                f"{spec.footprint} prints no {key!r}; add the proof, do not skip it"
            )


def polarised(spec: PartSpec) -> list[str]:
    """The pin names the tripwire must prove: "+" and "-", "K" and "A" with a
    "K", and a transistor's "G", "D" and "S" when it has all three.

    An "A" without a "K" is a resistor's or capacitor's A/B, not an anode. A
    MOSFET fitted with drain and source swapped conducts through its body diode
    and still agrees with the netlist, the board and DRC. A "G" alone is an
    RGB LED's green.
    """
    keys = [k for k in ("+", "-") if k in spec.pins]
    if "K" in spec.pins:
        keys += [k for k in ("K", "A") if k in spec.pins]
    if {"G", "D", "S"} <= set(spec.pins):
        keys += ["G", "D", "S"]
    return keys


def _text_proofs(spec: PartSpec, fp_path: Path) -> set[str]:
    """The polarity marks a footprint prints that agree with [pins]; raises on one that does not.

    A "+"/"-"/"K"/"A" `fp_text user` belongs to the numbered pad nearest it.
    """
    fp = footprints.parse(fp_path)
    proved = set()
    for text, x, y in footprints.user_texts(fp_path):
        if text not in polarised(spec):
            continue
        pad = _nearest_pad(fp, x, y, fp_path, text)
        if pad not in spec.pins[text]:
            raise KicadError(
                f"{spec.path} [pins] {text!r} = {list(spec.pins[text])}, but "
                f"{fp_path.name} prints {text!r} beside pad {pad}"
            )
        proved.add(text)
    return proved


def _nearest_pad(
    fp: footprints.Footprint, x: float, y: float, path: Path, text: str
) -> str:
    ranked = sorted((math.hypot(p.x - x, p.y - y), n) for n, p in fp.pads.items())
    if not ranked:
        raise KicadError(f"{path}: {text!r} mark but no numbered pads")
    if len(ranked) > 1 and math.isclose(ranked[0][0], ranked[1][0]):
        raise KicadError(
            f"{path}: {text!r} mark is equidistant from pads {ranked[0][1]} and {ranked[1][1]}"
        )
    return ranked[0][1]


def pin_map(
    spec: PartSpec, board: Board | None, sym_pins: list[symbols.Pin]
) -> dict[str, tuple[str, ...]]:
    """The entry's pin name -> numbers map: [pins], or derived from the board page."""
    if not spec.pin_numbers:
        return dict(spec.pins)
    if board is None:
        raise KicadError(
            f"{spec.path}: pin_numbers = {spec.pin_numbers!r} needs a board page "
            "(the part's `board`, or [source] board for the MCU)"
        )
    sides: dict[str, list[int]] = {"L": [], "R": []}
    for p in board.pins:
        if p.side is None or p.pos is None:
            raise KicadError(
                f"{board.path}: pin {p.name} has no Side/Pos for {spec.pin_numbers}"
            )
        if p.side not in sides:
            raise KicadError(
                f"{spec.path}: pin {p.name} is on side {p.side!r} of {board.path.name}; "
                f"pin_numbers = {spec.pin_numbers!r} numbers only L/R rows; "
                "give the entry explicit [pins] instead"
            )
        sides[p.side].append(p.pos)
    n = len(sides["L"])
    for side, got in sides.items():
        if sorted(got) != list(range(1, n + 1)):
            raise KicadError(
                f"{board.path}: side {side} positions {sorted(got)}; {spec.pin_numbers} "
                f"needs 1..{n} on both sides"
            )
    mapping = {
        p.name: (str(p.pos if p.side == "L" else 2 * n + 1 - p.pos),)
        for p in board.pins
    }
    where = f"{spec.path} pin_numbers from {board.path.name}"
    unknown = sorted(set(spec.no_connect) - set(mapping))
    if unknown:
        raise KicadError(
            f"{where}: no_connect {unknown} are not pins of the board page"
        )
    check_map(spec, mapping, sym_pins, where)
    return mapping


def may_float(spec: PartSpec, board: Board | None, name: str) -> bool:
    """Whether pin `name` may stay off every net.

    Listed under no_connect; or, for a pin_numbers part, a board-page row with a
    GPIO (a spare I/O). A power pad (5V, GND, 3V3) never floats implicitly.
    """
    if name in spec.no_connect:
        return True
    if spec.pin_numbers and board is not None:
        pin = board.by_name.get(name)
        return pin is not None and pin.gpio is not None
    return False


def _num_key(n: str) -> tuple[int, int | str]:
    return (0, int(n)) if n.isdigit() else (1, n)
