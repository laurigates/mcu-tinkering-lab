"""Which carrier BOM lines are already on hand in the parts-stock inventory.

    PYTHONPATH=tools python3 -m kicad.stock [project ...] [--boards N] [--inventory PATH]

The inventory is a separate tree of TOML files (parts-stock, schema v1:
`items/<category>/<slug>.toml`). It is found at `--inventory`, else
`$INVENTORY_DIR`, else `../parts-stock` beside the main mcu-tinkering-lab
checkout (a worktree resolves to the main checkout's parent, not its own).
Item files are read with tomllib against schema v1; the parts-stock `inv`
package is not imported.

A registry entry's `[stock]` table (registry.StockSpec) says how to find a
part: by the item's own `mpn` (exact, case-insensitive), or by `category`,
`package` and the instance value. Each BOM line (identical registry entry and
value, refs grouped) gets one status:

- on hand: a matching item states every `needs` spec and, for count-mode
  stock, the matches hold at least count x boards; a level-mode item at
  `full` or `some` counts with its level shown;
- rating unknown: matched, but a needed spec (or a voltage the value names) is
  under `[specs] unknown` or not stated;
- short: matched and rated, but too few, or the level is `low` or `out`;
  pieces on `[[sources]]` whose order (orders/<shop>/<date>-<id>.toml) is
  `placed` or `shipped` are shown as incoming and never count as on hand;
- none: no match; the line's `buy`/`lcsc` is printed as the route;
- no stock key: the registry entry has no `[stock]` table.

Items that share the category and package (for an mpn: the first four
characters of the mpn, so ZXMP3A16GTA is near ZXMP4A16GTA) but do not match are listed as near matches, never
counted. A substitute is a datasheet decision (the part-substitution skill),
so this report never makes one. It is a report, not a gate: exit 0 unless the
inventory or a project cannot be read. Not run in CI.
"""

from __future__ import annotations

import argparse
import math
import os
import re
import subprocess
import sys
import tomllib
from dataclasses import dataclass
from pathlib import Path

from hardware import CARRIER, HardwareError, join

from . import netlist, paths, registry
from .errors import KicadError
from .paths import REPO_ROOT
from .registry import StockSpec

SCHEMA = 1  # parts-stock/inv/schema.py SCHEMA
ON_HAND = "on hand"
RATING_UNKNOWN = "rating unknown"
SHORT = "short"
NONE = "none"
NO_KEY = "no stock key"
STATUSES = (ON_HAND, RATING_UNKNOWN, SHORT, NONE, NO_KEY)
# parts-stock/inv/schema.py ORDER_STATUSES, and the two inv/store.py counts as
# incoming (UNDELIVERED).
ORDER_STATUSES = ("placed", "shipped", "delivered", "cancelled", "refunded", "unknown")
UNDELIVERED = ("placed", "shipped")
MPN_PREFIX = 4  # characters an mpn shares with a near match: ZXMP3.. ~ ZXMP4..

_MULT = {"p": 1e-12, "n": 1e-9, "u": 1e-6, "m": 1e-3, "r": 1.0, "k": 1e3, "M": 1e6}
_QUANTITY = re.compile(r"([0-9]+(?:\.[0-9]+)?)([pnumrkM]?)([0-9]*)")
_UNIT = re.compile(r"(ohms?|Ω|f)$", re.IGNORECASE)
_VOLTS = re.compile(r"([0-9]+(?:\.[0-9]+)?)\s*v", re.IGNORECASE)


@dataclass(frozen=True)
class Item:
    """The schema v1 fields of one inventory item that matching reads."""

    slug: str
    category: str
    package: str
    mpn: str
    mode: str  # "count" | "level"
    qty: int | None  # count mode
    level: str  # level mode: full | some | low | out
    specs: dict  # [specs] without `unknown`
    unknown: tuple[str, ...]  # [specs] unknown
    contents: tuple[str, ...]  # a kit's [[contents]] values
    # Pieces on [[sources]] whose order is placed or shipped: None when no
    # source is undelivered, 0 when one is but states no qty. Never on hand.
    incoming: int | None = None

    @property
    def stock_text(self) -> str:
        text = f"x{self.qty}" if self.mode == "count" else f"level {self.level}"
        if self.incoming is None:
            return text
        return f"{text} +{self.incoming or ''} incoming".replace("+ ", "+")


@dataclass(frozen=True)
class Need:
    """One BOM line: the parts with the same registry entry and value."""

    refs: tuple[str, ...]
    registry: str
    value: str
    stock: StockSpec | None
    lcsc: str
    buy: str

    @property
    def count(self) -> int:
        return len(self.refs)


@dataclass(frozen=True)
class Line:
    need: Need
    need_qty: int  # count x boards
    status: str
    counted: tuple[Item, ...]  # the items the status rests on
    unrated: tuple[tuple[Item, str], ...]  # matched, not counted: why
    near: tuple[tuple[Item, str], ...]  # near matches, never counted: why
    detail: str


# --- values ------------------------------------------------------------------


def norm_value(text: str) -> str:
    """A value, canonical: "100nF", "0.1 µF" and "100n" agree.

    Lowercase (but "M", mega), spaces gone, µ/μ to u, a trailing F/Ω/ohm dropped; a number
    with an SI or RKM letter ("4k7", "4R7", "100n") becomes its float in base
    units. Anything else is compared as the cleaned text.
    """
    token = "".join(text.split()).replace("µ", "u").replace("μ", "u")
    # Lowercase all but "M" (mega), so "100NF" and "10K" read as 100n and 10k.
    token = "".join(c if c == "M" else c.lower() for c in token)
    token = _UNIT.sub("", token)
    m = _QUANTITY.fullmatch(token)
    if m:
        whole, letter, frac = m.groups()
        if frac and "." in whole:
            return token.lower()
        number = float(f"{whole}.{frac}" if frac else whole)
        return repr(number * _MULT.get(letter, 1.0) if letter else number)
    return token.lower()


def same_value(instance: str, stocked: str) -> bool:
    """An instance value's first token ("100uF" of "100uF 35V") against an item's value."""
    first = instance.split()[0] if instance.strip() else ""
    na, nb = norm_value(first), norm_value(stocked)
    try:
        return math.isclose(float(na), float(nb), rel_tol=1e-9)
    except ValueError:
        return na == nb


def volts(value: object) -> float | None:
    """A voltage from "35V", "6.3 V", 50 or 50.0; None when there is none."""
    if isinstance(value, bool):
        return None
    if isinstance(value, int | float):
        return float(value)
    if isinstance(value, str):
        m = _VOLTS.fullmatch(value.strip()) or _VOLTS.fullmatch(value.strip() + "v")
        if m:
            return float(m.group(1))
    return None


def needed_volts(value: str) -> float | None:
    """The voltage token an instance value carries: "100uF 35V" -> 35.0."""
    for token in value.split():
        m = _VOLTS.fullmatch(token)
        if m:
            return float(m.group(1))
    return None


def _norm_package(text: str) -> str:
    return "".join(text.split()).casefold()


# --- matching (pure) -----------------------------------------------------------


def _identity(key: StockSpec, need: Need, item: Item) -> bool:
    """Whether the item is this part, ratings aside."""
    if key.mpn:
        return bool(item.mpn) and item.mpn.casefold() == key.mpn.casefold()
    if item.category.casefold() != key.category.casefold():
        return False
    if _norm_package(item.package) != _norm_package(key.package):
        return False
    values = list(item.contents)
    if "value" in item.specs:
        values.append(str(item.specs["value"]))
    return any(same_value(need.value, v) for v in values)


def _near(key: StockSpec, item: Item) -> bool:
    if key.mpn:
        return bool(item.mpn) and (
            item.mpn[:MPN_PREFIX].casefold() == key.mpn[:MPN_PREFIX].casefold()
        )
    return item.category.casefold() == key.category.casefold() and _norm_package(
        item.package
    ) == _norm_package(key.package)


def _rating(key: StockSpec, need: Need, item: Item) -> tuple[str, str]:
    """("ok" | "unknown" | "low", why) for a matched item."""
    missing = [n for n in key.needs if n not in item.specs]
    want = needed_volts(need.value)
    if want is not None:
        have = volts(item.specs["voltage"]) if "voltage" in item.specs else None
        if have is not None and have < want:
            return "low", f"rated {item.specs['voltage']}, needs {want:g}V"
        if have is None and "voltage" not in missing:
            missing.append("voltage")
    if missing:
        how = [
            f"{n} unknown" if n in item.unknown else f"{n} not stated" for n in missing
        ]
        return "unknown", ", ".join(how)
    return "ok", ""


def _near_why(key: StockSpec, need: Need, item: Item) -> str:
    if key.mpn:
        return f"mpn {item.mpn}, not {key.mpn}"
    values = list(item.contents) + (
        [str(item.specs["value"])] if "value" in item.specs else []
    )
    if any(same_value(need.value, v) for v in values):
        return _rating(key, need, item)[1]
    return f"value {', '.join(values) or 'not stated'}"


def judge(need: Need, items: list[Item], boards: int = 1) -> Line:
    """One BOM line's status against the loaded inventory items."""
    qty = need.count * boards
    key = need.stock
    if key is None:
        return Line(need, qty, NO_KEY, (), (), (), "")
    rated, unrated, near = [], [], []
    for item in items:
        if _identity(key, need, item):
            verdict, why = _rating(key, need, item)
            if verdict == "ok":
                rated.append(item)
            elif verdict == "unknown":
                unrated.append((item, why))
            else:
                near.append((item, why))
        elif _near(key, item):
            near.append((item, _near_why(key, need, item)))
    if not rated:
        status = RATING_UNKNOWN if unrated else NONE
        return Line(need, qty, status, (), tuple(unrated), tuple(near), "")
    counted = [i for i in rated if i.mode == "count"]
    total = sum(i.qty or 0 for i in counted)
    stocked = [i for i in rated if i.mode == "level" and i.level in ("full", "some")]
    if total >= qty:
        status = ON_HAND
        parts = [f"x{total}, enough for {total // need.count} board(s)"]
    elif stocked:
        status = ON_HAND
        parts = [", ".join(f"level {i.level}" for i in stocked) + " (not counted)"]
    else:
        status = SHORT
        parts = [f"x{total} on hand"] if counted else []
        parts += [f"level {i.level}" for i in rated if i.mode == "level"]
    parts.append(_incoming_text(rated, total, need.count))
    detail = ", ".join(p for p in parts if p)
    return Line(need, qty, status, tuple(rated), tuple(unrated), tuple(near), detail)


def _incoming_text(rated: list[Item], on_hand: int, per_board: int) -> str:
    """ "+100 incoming (enough for N board(s) once delivered)", or "" for none."""
    pending = [i.incoming for i in rated if i.incoming is not None]
    if not pending:
        return ""
    pieces = sum(pending)
    if not pieces:
        return "+incoming (quantity not stated)"
    boards = (on_hand + pieces) // per_board
    return f"+{pieces} incoming (enough for {boards} board(s) once delivered)"


def group(rows: list[tuple[str, str, str, StockSpec | None, str, str]]) -> list[Need]:
    """Needs from (ref, registry, value, stock, lcsc, buy) rows, in first-seen order."""
    order: list[tuple[str, str]] = []
    by_key: dict[tuple[str, str], list[tuple]] = {}
    for row in rows:
        k = (row[1], row[2])
        if k not in by_key:
            order.append(k)
            by_key[k] = []
        by_key[k].append(row)
    out = []
    for k in order:
        members = by_key[k]
        out.append(
            Need(
                refs=tuple(r[0] for r in members),
                registry=k[0],
                value=k[1],
                stock=members[0][3],
                lcsc=_joined(r[4] for r in members),
                buy=_joined(r[5] for r in members),
            )
        )
    return out


def _joined(values) -> str:
    return " | ".join(dict.fromkeys(v for v in values if v))


# --- IO ---------------------------------------------------------------------


def default_inventory(repo_root: Path = REPO_ROOT) -> Path:
    """$INVENTORY_DIR, else parts-stock beside the main checkout."""
    env = os.environ.get("INVENTORY_DIR")
    if env:
        return Path(env)
    common = subprocess.run(
        ["git", "rev-parse", "--path-format=absolute", "--git-common-dir"],
        cwd=repo_root,
        capture_output=True,
        text=True,
    )
    main = Path(common.stdout.strip()).parent if common.returncode == 0 else repo_root
    return main.parent / "parts-stock"


def load_items(root: Path) -> list[Item]:
    """Every items/<category>/<slug>.toml under the inventory root, sorted."""
    items_dir = root / "items"
    if not root.is_dir():
        raise KicadError(
            f"{root}: no inventory there; pass --inventory or set $INVENTORY_DIR"
        )
    orders = load_orders(root)
    return [parse_item(p, _toml(p), orders) for p in sorted(items_dir.glob("*/*.toml"))]


def load_orders(root: Path) -> dict[str, str]:
    """Order ref ("<shop>/<date>-<id>", the file's path under orders/) -> status."""
    out = {}
    for path in sorted((root / "orders").glob("*/*.toml")):
        order = _toml(path).get("order")
        status = order.get("status") if isinstance(order, dict) else None
        if status not in ORDER_STATUSES:
            raise KicadError(f"{path}: [order] status {status!r} is not schema v1")
        out[f"{path.parent.name}/{path.stem}"] = status
    return out


def _toml(path: Path) -> dict:
    try:
        return tomllib.loads(path.read_text(encoding="utf-8"))
    except (tomllib.TOMLDecodeError, UnicodeDecodeError) as e:
        raise KicadError(f"{path}: {e}") from None


def incoming(sources: list, orders: dict[str, str]) -> int | None:
    """inv/store.py Store.incoming: pieces on sources whose order is undelivered."""
    pending = [
        s
        for s in sources
        if isinstance(s, dict) and orders.get(s.get("order", "")) in UNDELIVERED
    ]
    if not pending:
        return None
    return sum(s.get("qty") or 0 for s in pending)


def parse_item(path: Path, data: dict, orders: dict[str, str] | None = None) -> Item:
    """The fields matching reads, with the schema v1 checks they depend on."""
    if data.get("schema") != SCHEMA:
        raise KicadError(
            f"{path}: schema {data.get('schema')!r}; kicad.stock reads {SCHEMA}"
        )
    item, stock = data.get("item"), data.get("stock")
    if not isinstance(item, dict) or not isinstance(stock, dict):
        raise KicadError(f"{path}: needs [item] and [stock]")
    category = item.get("category")
    if not isinstance(category, str) or category not in registry.STOCK_CATEGORIES:
        raise KicadError(f"{path}: [item] category {category!r} is not schema v1")
    mode = stock.get("mode")
    qty, level = stock.get("qty"), stock.get("level", "")
    if mode == "count":
        if isinstance(qty, bool) or not isinstance(qty, int) or qty < 0:
            raise KicadError(f"{path}: [stock] count mode needs qty >= 0, got {qty!r}")
    elif mode == "level":
        if level not in ("full", "some", "low", "out"):
            raise KicadError(f"{path}: [stock] level {level!r} is not schema v1")
        qty = None
    else:
        raise KicadError(f"{path}: [stock] mode {mode!r} is not count or level")
    specs = dict(data.get("specs", {}))
    unknown = specs.pop("unknown", [])
    contents = data.get("contents", [])
    return Item(
        slug=path.stem,
        category=category,
        package=str(item.get("package", "")),
        mpn=str(item.get("mpn", "")),
        mode=mode,
        qty=qty,
        level=level if mode == "level" else "",
        specs=specs,
        unknown=tuple(unknown),
        contents=tuple(str(c.get("value", "")) for c in contents),
        incoming=incoming(data.get("sources", []), orders or {}),
    )


def needs(project: Path, repo_root: Path = REPO_ROOT) -> list[Need]:
    """The carrier's BOM lines; instance value/lcsc/buy over the registry's."""
    try:
        model = join(project, repo_root, view=CARRIER)
    except HardwareError as e:
        raise KicadError(str(e)) from None
    rows = []
    for c in netlist.build(project, repo_root).comps:
        if not c.in_bom:
            continue
        spec = registry.load(c.registry, repo_root)
        instance = model.parts.get(c.part_key)
        buy = (instance.buy if instance else "") or spec.buy
        rows.append((c.ref, c.registry, c.value, spec.stock, c.lcsc, buy))
    return group(rows)


def _item_line(item: Item, why: str = "") -> str:
    specs = " ".join(f"{k}={v}" for k, v in item.specs.items() if k != "value")
    text = f"{item.slug} {item.mpn} {item.stock_text} {specs}"
    text = " ".join(text.split())
    return f"{text}: {why}" if why else text


def render(lines: list[Line]) -> str:
    out = []
    for status in STATUSES:
        group_ = [ln for ln in lines if ln.status == status]
        out.append(f"  {status}: {len(group_)}")
        for ln in group_:
            n = ln.need
            if status == NO_KEY:
                out.append(f"    {' '.join(n.refs)}  {n.value}  ({n.registry})")
                continue
            what = _key_text(n.stock)
            head = f"    {' '.join(n.refs)}  {n.value}  ({n.registry}, {what})  need {ln.need_qty}"
            if ln.detail:
                head += f"  {ln.detail}"
            out.append(head)
            out.extend(f"      matched: {_item_line(i)}" for i in ln.counted)
            out.extend(
                f"      matched, not counted: {_item_line(i, w)}" for i, w in ln.unrated
            )
            out.extend(
                f"      near match, not counted: {_item_line(i, w)}" for i, w in ln.near
            )
            if status in (NONE, SHORT):
                out.append(f"      route: {_route(n)}")
    return "\n".join(out)


def _key_text(key: StockSpec | None) -> str:
    if key is None:
        return ""
    return f"mpn {key.mpn}" if key.mpn else f"{key.category} {key.package}"


def _route(need: Need) -> str:
    routes = [f"buy {need.buy}"] if need.buy else []
    routes += [f"lcsc {need.lcsc}"] if need.lcsc else []
    return "; ".join(routes) or "no buy or lcsc recorded"


def main(argv: list[str] | None = None, repo_root: Path = REPO_ROOT) -> int:
    parser = argparse.ArgumentParser(
        prog="kicad.stock", description=__doc__.split("\n")[0]
    )
    paths.add_project_args(parser)
    parser.add_argument("--boards", type=int, default=1, help="boards to build")
    parser.add_argument(
        "--inventory",
        type=Path,
        help="parts-stock root (default: $INVENTORY_DIR, else ../parts-stock)",
    )
    args = parser.parse_args(argv)
    if args.boards < 1:
        parser.error("--boards must be >= 1")
    inventory = args.inventory or default_inventory(repo_root)
    try:
        projects = paths.select_projects(args.projects, args.allow_none, repo_root)
        items = load_items(inventory) if projects else []
        for project in projects:
            lines = [judge(n, items, args.boards) for n in needs(project, repo_root)]
            print(f"{project}: {args.boards} board(s), inventory {inventory}")
            print(render(lines))
    except KicadError as e:
        print(f"FAIL: {e}")
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
