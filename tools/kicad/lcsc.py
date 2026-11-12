"""Ask JLCPCB about each LCSC number the carriers use: class, stock, price, status.

    PYTHONPATH=tools python3 -m kicad.lcsc [C123 ...] [--project packages/<d>/<p>]
        [--refresh | --offline] [--max-age HOURS] [--json]

With no numbers and no `--project`, it checks every LCSC number in use
(`used`): each parts-registry entry's `lcsc`, and each part's in every
git-tracked `hardware.toml` under `packages/`, at any depth (a subproject such
as packages/robotics/robocar/integrated counts), resolved as kicad.netlist
resolves it (the part's own `lcsc`, else its registry entry's). `--project`
narrows that to one project's
hardware.toml and the registry entries its parts name. Numbers given on the
command line are checked as they are.

One row per number: MPN, package, class, assembly stock, the unit price at
quantity 1 (USD) with JLCPCB's minimum order, status and where it is used,
then a count of each class. The class is JLCPCB's:

- **basic**: `componentLibraryType` "base"; no loading fee.
- **preferred**: "expand" with the search's `preferredComponentFlag`; an
  extended part JLCPCB loads without the fee.
- **extended**: every other "expand" part, and any other library type (shown
  raw). Each extended BOM line adds JLCPCB's per-part loading fee (the
  `jlcpcb-order` skill has the last observed amount). A warning, never a
  failure.

It exits 1 when a number is unknown to JLCPCB, is not orderable, has no
assembly stock, or could not be fetched and has no cached copy; a cache that
cannot be written only warns. Not orderable is the detail's `isBuyComponent`
"0", with its `noBuyReason` in the report, or a `componentStatus` other than
"yes". `componentStatus` alone is not enough: on 2026-11-06 C9900140859
answered "yes" with `isBuyComponent` "0" and "This product is no longer
manufactured." (testdata/lcsc/detail-C9900140859.json); `_carrier-fab` runs it first, so a dead or
sold-out part stops the order before the upload.

Nothing is vendored: each number is asked for on its own, and the answer is
cached in `$XDG_CACHE_HOME/mcu-tinkering-lab/lcsc/<C>.json` (default
`~/.cache`), never committed. A copy younger than `--max-age` (24 h) is used
without a request; `--refresh` always asks; `--offline` never does and fails
a number with no copy. When a request fails and a copy exists, the copy is
used, marked stale in the report, with a warning on stderr. CI never runs
this, and the unit tests serve recorded answers (testdata/lcsc/).

Two endpoints, both answering curl's User-Agent (Python's default is refused,
as in kicad.lcsc_rotation), and no other header beyond the JSON content type:

- the detail, `getComponentDetail?componentCode=C...`: MPN, package,
  library type, status, prices, stock. An unknown number answers HTTP 200
  with `{"code": 400, "data": null}`.
- the search, `selectSmtComponentList`: the only one carrying
  `preferredComponentFlag`. It is a fuzzy search: "C17414" also lists
  C20622501, whose MPN is "C17414", and an unknown number lists dozens of
  unrelated parts, so only the entry whose `componentCode` matches counts.

Both carry a `stockCount`. Asked for all 31 numbers in use on 2026-11-06,
the two agreed for every part, and equalled the detail's
`overseasStockCount` (warehouse "sz", the Shenzhen assembly line); a figure
of 109466 seen once for C468250's detail, against the search's 17262, did
not recur. `canPresaleNumber` runs a little lower (stock less what is
already reserved) and can go negative (C2044594: -12 in the search, 0 in the
detail), so it is reported in `--json` but not gated on. The gate takes the
smaller of the two `stockCount`s, so a part either pool calls sold out
fails, and `--json` keeps both.
"""

from __future__ import annotations

import argparse
import http.client
import json
import os
import re
import sys
import tempfile
import urllib.request
from collections.abc import Callable
from dataclasses import dataclass, field
from datetime import UTC, datetime, timedelta
from pathlib import Path
from typing import Any

from hardware import HardwareError
from hardware.blocks import load_sidecar

from . import paths, registry
from .errors import KicadError
from .paths import REPO_ROOT

DETAIL_URL = (
    "https://cart.jlcpcb.com/shoppingCart/smtGood/getComponentDetail"
    "?componentCode={lcsc}"
)
SEARCH_URL = (
    "https://jlcpcb.com/api/overseas-pcb-order/v1/shoppingCart/smtGood/"
    "selectSmtComponentList"
)
USER_AGENT = "curl/8.5.0"
MAX_AGE_HOURS = 24.0
COMMAND_LINE = "(command line)"

# The fields kept from each answer: what the report reads, and a few beside it
# for a look by hand. The rest (images, attributes, signed URLs) is dropped.
DETAIL_KEYS = (
    "componentCode",
    "componentModelEn",
    "componentBrandEn",
    "componentSpecificationEn",
    "describe",
    "componentLibraryType",
    "stockCount",
    "overseasStockCount",
    "canPresaleNumber",
    "warehouseCode",
    "componentStatus",
    "assemblyProcess",
    "leastNumber",
    "leastNumberPrice",
    "leastPatchNumber",
    "minPurchaseNum",
    "initialPrice",
    "isBuyComponent",
    "noBuyReason",
    "specialComponentFee",
    "dataManualUrl",
)
PRICE_KEYS = ("startNumber", "endNumber", "productPrice")
SEARCH_KEYS = (
    "componentCode",
    "componentLibraryType",
    "preferredComponentFlag",
    "stockCount",
    "canPresaleNumber",
)
CLASSES = ("basic", "preferred", "extended")
_LCSC = re.compile(r"C[0-9]+")

Opener = Callable[[urllib.request.Request], bytes]
Clock = Callable[[], datetime]


def _open(req: urllib.request.Request) -> bytes:
    with urllib.request.urlopen(req, timeout=30) as resp:
        return resp.read()


def _utcnow() -> datetime:
    return datetime.now(UTC)


def cache_dir() -> Path:
    """$XDG_CACHE_HOME/mcu-tinkering-lab/lcsc, ~/.cache when unset."""
    base = os.environ.get("XDG_CACHE_HOME") or str(Path.home() / ".cache")
    return Path(base) / "mcu-tinkering-lab" / "lcsc"


# --- where each number is used ---------------------------------------------


@dataclass(frozen=True)
class Use:
    label: str  # "parts/<slug>" or "<domain>/<project> [parts.<key>]"
    footprint: str | None  # the registry entry's footprint; None: no entry


def used(
    projects: list[Path] | None = None, repo_root: Path = REPO_ROOT
) -> dict[str, list[Use]]:
    """LCSC number -> where it is used.

    With no `projects`: every registry entry with an `lcsc`, and every part of
    every git-tracked hardware.toml under packages/, at any depth. With
    `projects`: only their hardware.toml parts. A part's LCSC number is its
    own `lcsc`, else its registry entry's, as kicad.netlist fills the LCSC
    field.
    """
    found: dict[str, list[Use]] = {}
    specs: dict[str, registry.PartSpec] = {}

    def spec(slug: str) -> registry.PartSpec:
        if slug not in specs:
            specs[slug] = registry.load(slug, repo_root)
        return specs[slug]

    if projects is None:
        for slug in registry.slugs(repo_root):
            s = spec(slug)
            if s.lcsc:
                found.setdefault(s.lcsc, []).append(Use(f"parts/{slug}", s.footprint))
        sidecars = [
            p / "hardware.toml"
            for p in paths.tracked_projects(repo_root, "hardware.toml")
        ]
    else:
        sidecars = [p / "hardware.toml" for p in projects]
    for sidecar in sidecars:
        try:
            data = load_sidecar(sidecar, repo_root)
        except FileNotFoundError:
            raise KicadError(f"{sidecar}: no hardware.toml") from None
        except HardwareError as e:
            raise KicadError(str(e)) from None
        name = _project_label(sidecar.parent, repo_root)
        tables = [("source", data.get("source", {}))]
        tables += [(f"parts.{k}", v) for k, v in data.get("parts", {}).items()]
        for where, table in tables:
            slug, own = table.get("part"), table.get("lcsc", "")
            entry = spec(slug) if slug else None
            code = own or (entry.lcsc if entry else "")
            if not code:
                continue
            if not _LCSC.fullmatch(code):
                raise KicadError(f"{sidecar} [{where}]: lcsc {code!r} is not C<digits>")
            found.setdefault(code, []).append(
                Use(f"{name} [{where}]", entry.footprint if entry else None)
            )
    return found


def _project_label(project: Path, repo_root: Path) -> str:
    try:
        return (
            project.resolve().relative_to(repo_root.resolve() / "packages").as_posix()
        )
    except ValueError:
        return str(project)


# --- asking JLCPCB -----------------------------------------------------------


def _ask(lcsc: str, req: urllib.request.Request, opener: Opener, what: str) -> dict:
    try:
        payload = json.loads(opener(req))
    except (OSError, ValueError, http.client.HTTPException) as e:
        # HTTPException: a truncated body (IncompleteRead) or a garbled status
        # line, neither of them an OSError.
        reason = str(e) or type(e).__name__
        raise KicadError(f"{lcsc}: JLCPCB {what} lookup failed: {reason}") from None
    if not isinstance(payload, dict):
        raise KicadError(f"{lcsc}: JLCPCB {what} answered {payload!r}")
    return payload


def trim_detail(data: dict[str, Any]) -> dict[str, Any]:
    out = {k: data[k] for k in DETAIL_KEYS if k in data}
    out["prices"] = [
        {k: p.get(k) for k in PRICE_KEYS} for p in data.get("prices") or []
    ]
    return out


def trim_search(payload: dict[str, Any], lcsc: str) -> dict[str, Any] | None:
    """The search hit for exactly `lcsc`, trimmed; None when it lists none."""
    info = (payload.get("data") or {}).get("componentPageInfo") or {}
    for item in info.get("list") or []:
        if item.get("componentCode") == lcsc:
            return {k: item.get(k) for k in SEARCH_KEYS}
    return None


def fetch(lcsc: str, opener: Opener = _open) -> dict[str, Any]:
    """{"detail": ..., "search": ...}, trimmed; detail None: unknown to JLCPCB."""
    headers = {"User-Agent": USER_AGENT}
    req = urllib.request.Request(DETAIL_URL.format(lcsc=lcsc), headers=headers)
    detail = _ask(lcsc, req, opener, "detail")
    if detail.get("code") == 400 and detail.get("data") is None:
        return {"detail": None, "search": None}
    if detail.get("code") != 200 or not isinstance(detail.get("data"), dict):
        raise KicadError(
            f"{lcsc}: JLCPCB detail answered code {detail.get('code')}: "
            f"{detail.get('message')!r}"
        )
    body = json.dumps({"keyword": lcsc, "currentPage": 1, "pageSize": 5}).encode()
    req = urllib.request.Request(
        SEARCH_URL,
        data=body,
        headers={**headers, "Content-Type": "application/json"},
        method="POST",
    )
    search = _ask(lcsc, req, opener, "search")
    if search.get("code") != 200:
        raise KicadError(
            f"{lcsc}: JLCPCB search answered code {search.get('code')}: "
            f"{search.get('message')!r}"
        )
    return {"detail": trim_detail(detail["data"]), "search": trim_search(search, lcsc)}


# --- the answer, classified --------------------------------------------------


@dataclass(frozen=True)
class Part:
    lcsc: str
    mpn: str
    brand: str
    package: str
    library_type: str  # JLCPCB's raw componentLibraryType
    preferred: bool | None  # None: the search did not list the part
    klass: str  # one of CLASSES
    stock: int  # the gate's figure: the smaller of the two stockCounts
    detail_stock: int
    search_stock: int | None
    presale: int | None
    price_1: float | None  # USD per part at quantity 1
    min_qty: int | None  # JLCPCB's minimum order (leastNumber)
    status: str  # componentStatus; "yes" even for a discontinued part
    buyable: bool  # isBuyComponent other than "0"
    no_buy_reason: str  # noBuyReason, e.g. "This product is no longer manufactured."
    process: str  # assemblyProcess, e.g. "SMT"


def _int(value: Any) -> int:
    return value if isinstance(value, int) and not isinstance(value, bool) else 0


def _price_at_1(detail: dict[str, Any]) -> float | None:
    for tier in detail.get("prices") or []:
        start, end = tier.get("startNumber"), tier.get("endNumber")
        if isinstance(start, int) and start <= 1 and (end == -1 or (end or 0) >= 1):
            return tier.get("productPrice")
    return detail.get("initialPrice")


def part_from(lcsc: str, detail: dict[str, Any], search: dict[str, Any] | None) -> Part:
    library_type = str(detail.get("componentLibraryType") or "")
    preferred = None if search is None else bool(search.get("preferredComponentFlag"))
    if library_type == "base":
        klass = "basic"
    elif library_type == "expand" and preferred:
        klass = "preferred"
    else:
        klass = "extended"
    detail_stock = _int(detail.get("stockCount"))
    search_stock = None if search is None else _int(search.get("stockCount"))
    stock = detail_stock if search_stock is None else min(detail_stock, search_stock)
    min_qty = detail.get("leastNumber")
    return Part(
        lcsc=lcsc,
        mpn=str(detail.get("componentModelEn") or ""),
        brand=str(detail.get("componentBrandEn") or ""),
        package=str(detail.get("componentSpecificationEn") or ""),
        library_type=library_type,
        preferred=preferred,
        klass=klass,
        stock=stock,
        detail_stock=detail_stock,
        search_stock=search_stock,
        presale=detail.get("canPresaleNumber"),
        price_1=_price_at_1(detail),
        min_qty=min_qty if isinstance(min_qty, int) else None,
        status=str(detail.get("componentStatus") or ""),
        buyable=str(detail.get("isBuyComponent")) != "0",
        no_buy_reason=str(detail.get("noBuyReason") or ""),
        process=str(detail.get("assemblyProcess") or ""),
    )


# --- the cache ---------------------------------------------------------------


@dataclass(frozen=True)
class Answer:
    part: Part | None  # None: unknown to JLCPCB
    fetched_at: datetime
    stale: bool  # older than --max-age: a failed request, or --offline


def write_cache(path: Path, record: dict[str, Any]) -> None:
    """Write `record` as JSON to `path` through a temp file and a rename, so a
    reader never sees half a file and a failed write keeps the old one."""
    path.parent.mkdir(parents=True, exist_ok=True)
    fd, tmp = tempfile.mkstemp(dir=path.parent, prefix=f".{path.name}.", suffix=".tmp")
    try:
        with os.fdopen(fd, "w", encoding="utf-8") as f:
            json.dump(record, f, indent=1, sort_keys=True)
            f.write("\n")
        os.replace(tmp, path)
    except BaseException:
        Path(tmp).unlink(missing_ok=True)
        raise


def _read_cache(path: Path) -> tuple[dict[str, Any], datetime] | None:
    """The cached record and when it was fetched; None if absent or unreadable."""
    try:
        record = json.loads(path.read_text(encoding="utf-8"))
        fetched_at = datetime.fromisoformat(record["fetched_at"])
    except (OSError, ValueError, KeyError, TypeError):
        return None
    if "detail" not in record or "search" not in record:
        return None
    if fetched_at.tzinfo is None:
        fetched_at = fetched_at.replace(tzinfo=UTC)
    return record, fetched_at


def _answer(
    lcsc: str, record: dict[str, Any], fetched_at: datetime, stale: bool
) -> Answer:
    detail = record["detail"]
    part = None if detail is None else part_from(lcsc, detail, record["search"])
    return Answer(part, fetched_at, stale)


def lookup(
    lcsc: str,
    *,
    cache: Path,
    max_age: timedelta,
    refresh: bool = False,
    offline: bool = False,
    opener: Opener = _open,
    now: Clock = _utcnow,
) -> Answer:
    """JLCPCB's answer for `lcsc`, from the cache when fresh enough."""
    path = cache / f"{lcsc}.json"
    cached = _read_cache(path)
    if cached is not None and not refresh:
        record, fetched_at = cached
        expired = now() - fetched_at > max_age
        if offline or not expired:
            return _answer(lcsc, record, fetched_at, stale=expired)
    if offline:
        raise KicadError(f"{lcsc}: no cached answer in {cache} and --offline")
    try:
        record = fetch(lcsc, opener)
    except KicadError as e:
        if cached is None:
            raise KicadError(f"{e}; no cached copy") from None
        record, fetched_at = cached
        print(
            f"WARN: {e}; using the cached answer from {fetched_at.isoformat()}",
            file=sys.stderr,
        )
        return _answer(lcsc, record, fetched_at, stale=True)
    fetched_at = now()
    try:
        write_cache(
            path, {"lcsc": lcsc, "fetched_at": fetched_at.isoformat(), **record}
        )
    except OSError as e:
        # JLCPCB answered; an unwritable cache must not hide that answer.
        print(f"WARN: {lcsc}: could not cache the answer: {e}", file=sys.stderr)
    return _answer(lcsc, record, fetched_at, stale=False)


# --- the report --------------------------------------------------------------


@dataclass
class Row:
    lcsc: str
    used_in: list[str]
    answer: Answer | None = None
    failures: list[str] = field(default_factory=list)
    warnings: list[str] = field(default_factory=list)


def assess(
    lcsc: str, used_in: list[str], answer: Answer | None, error: str = ""
) -> Row:
    """A report row: what fails the gate and what only warns."""
    row = Row(lcsc, used_in, answer)
    if answer is None:
        row.failures.append(error)
        return row
    if answer.stale:
        row.warnings.append(
            f"stale: cached answer from {answer.fetched_at.isoformat()}"
        )
    part = answer.part
    if part is None:
        row.failures.append("unknown to JLCPCB")
        return row
    if not part.buyable:
        row.failures.append(
            f"not orderable: {part.no_buy_reason or 'isBuyComponent 0'}"
        )
    if part.status != "yes":
        row.failures.append(f"not orderable (componentStatus {part.status!r})")
    if part.stock <= 0:
        row.failures.append(
            f"no assembly stock (detail {part.detail_stock}, search {part.search_stock})"
        )
    if part.klass == "extended":
        raw = (
            ""
            if part.library_type == "expand"
            else f", library type {part.library_type!r}"
        )
        row.warnings.append(f"extended{raw}: JLCPCB's per-part loading fee applies")
    if part.library_type == "expand" and part.preferred is None:
        row.warnings.append("the search did not list it: preferred or not is unknown")
    return row


def check(
    numbers: dict[str, list[str]],
    *,
    cache: Path,
    max_age: timedelta,
    refresh: bool = False,
    offline: bool = False,
    opener: Opener = _open,
    now: Clock = _utcnow,
) -> list[Row]:
    """A row per number, sorted; `numbers` maps each to where it is used."""
    rows = []
    for lcsc in sorted(numbers, key=lambda c: int(c[1:])):
        try:
            answer = lookup(
                lcsc,
                cache=cache,
                max_age=max_age,
                refresh=refresh,
                offline=offline,
                opener=opener,
                now=now,
            )
        except KicadError as e:
            rows.append(assess(lcsc, numbers[lcsc], None, str(e)))
            continue
        rows.append(assess(lcsc, numbers[lcsc], answer))
    return rows


def summary(rows: list[Row]) -> dict[str, int]:
    counts = dict.fromkeys(CLASSES, 0)
    for r in rows:
        if r.answer is not None and r.answer.part is not None:
            counts[r.answer.part.klass] += 1
    return counts


def as_json(rows: list[Row]) -> dict[str, Any]:
    parts = []
    for r in rows:
        p = r.answer.part if r.answer else None
        parts.append(
            {
                "lcsc": r.lcsc,
                "mpn": p.mpn if p else None,
                "brand": p.brand if p else None,
                "package": p.package if p else None,
                "class": p.klass if p else None,
                "library_type": p.library_type if p else None,
                "preferred": p.preferred if p else None,
                "stock": p.stock if p else None,
                "detail_stock": p.detail_stock if p else None,
                "search_stock": p.search_stock if p else None,
                "presale": p.presale if p else None,
                "price_usd_1": p.price_1 if p else None,
                "min_qty": p.min_qty if p else None,
                "status": p.status if p else None,
                "buyable": p.buyable if p else None,
                "no_buy_reason": p.no_buy_reason if p else None,
                "process": p.process if p else None,
                "used_in": r.used_in,
                "fetched_at": r.answer.fetched_at.isoformat() if r.answer else None,
                "stale": r.answer.stale if r.answer else None,
                "failures": r.failures,
                "warnings": r.warnings,
            }
        )
    return {
        "ok": not any(r.failures for r in rows),
        "summary": summary(rows),
        "parts": parts,
    }


def _where(labels: list[str]) -> str:
    """The Used-in cell: registry entries as they are, a project's parts
    grouped under it, and counted past three."""
    groups: dict[str, list[str]] = {}
    for label in labels:
        name, sep, rest = label.partition(" [")
        groups.setdefault(name, [])
        if sep:
            groups[name].append(rest.rstrip("]").removeprefix("parts."))
    cells = []
    for name, keys in groups.items():
        if not keys:
            cells.append(name)
        elif len(keys) <= 3:
            cells.append(f"{name} [{', '.join(keys)}]")
        else:
            cells.append(f"{name} [{len(keys)} parts]")
    return ", ".join(cells)


def as_text(rows: list[Row]) -> str:
    head = (
        "LCSC",
        "MPN",
        "Package",
        "Class",
        "Stock",
        "USD@1",
        "Min",
        "Status",
        "Used in",
    )
    table = [head]
    for r in rows:
        p = r.answer.part if r.answer else None
        status = (
            (p.status if p else "?")
            + (" not buyable" if p and not p.buyable else "")
            + (" stale" if r.answer and r.answer.stale else "")
        )
        if p is None:
            table.append(
                (r.lcsc, "-", "-", "-", "-", "-", "-", status, _where(r.used_in))
            )
            continue
        table.append(
            (
                r.lcsc,
                p.mpn,
                p.package,
                p.klass,
                str(p.stock),
                "-" if p.price_1 is None else f"{p.price_1:.4f}",
                "-" if p.min_qty is None else str(p.min_qty),
                status,
                _where(r.used_in),
            )
        )
    widths = [max(len(row[i]) for row in table) for i in range(len(head) - 1)]
    lines = [
        "  ".join(cell.ljust(w) for cell, w in zip(row[:-1], widths, strict=True))
        + "  "
        + row[-1]
        for row in table
    ]
    for r in rows:
        lines += [f"WARN: {r.lcsc}: {w}" for w in r.warnings]
    for r in rows:
        lines += [f"FAIL: {r.lcsc}: {f}" for f in r.failures]
    n = summary(rows)
    line = (
        f"{len(rows)} part(s): {n['basic']} basic, {n['preferred']} preferred, "
        f"{n['extended']} extended"
    )
    if n["extended"]:
        line += "; each extended part adds JLCPCB's per-part loading fee"
    lines.append(line)
    return "\n".join(lines)


def main(
    argv: list[str] | None = None,
    opener: Opener = _open,
    now: Clock = _utcnow,
    repo_root: Path = REPO_ROOT,
) -> int:
    parser = argparse.ArgumentParser(
        prog="kicad.lcsc",
        description="Check the LCSC numbers in use against JLCPCB's parts library.",
    )
    parser.add_argument(
        "numbers", nargs="*", help="LCSC numbers (default: every one in use)"
    )
    parser.add_argument(
        "--project",
        action="append",
        type=Path,
        default=[],
        help="only this project's hardware.toml parts (repeatable)",
    )
    fresh = parser.add_mutually_exclusive_group()
    fresh.add_argument(
        "--refresh", action="store_true", help="ask JLCPCB, ignoring the cache"
    )
    fresh.add_argument(
        "--offline", action="store_true", help="the cache only; no request"
    )
    parser.add_argument(
        "--max-age",
        type=float,
        default=MAX_AGE_HOURS,
        metavar="HOURS",
        help=f"use a cached answer younger than this (default {MAX_AGE_HOURS:g})",
    )
    parser.add_argument("--json", action="store_true", help="machine-readable output")
    args = parser.parse_args(sys.argv[1:] if argv is None else argv)
    try:
        bad = [n for n in args.numbers if not _LCSC.fullmatch(n)]
        if bad:
            raise KicadError(f"not LCSC numbers (C<digits>): {bad}")
        if args.numbers and args.project:
            raise KicadError("give LCSC numbers or --project, not both")
        if args.numbers:
            numbers = {n: [COMMAND_LINE] for n in args.numbers}
        else:
            uses = used(args.project or None, repo_root)
            numbers = {c: sorted({u.label for u in us}) for c, us in uses.items()}
            if not numbers:
                raise KicadError("no LCSC number in use: nothing to check")
        rows = check(
            numbers,
            cache=cache_dir(),
            max_age=timedelta(hours=args.max_age),
            refresh=args.refresh,
            offline=args.offline,
            opener=opener,
            now=now,
        )
    except KicadError as e:
        print(f"FAIL: {e}")
        return 1
    if args.json:
        print(json.dumps(as_json(rows), indent=1, ensure_ascii=False))
    else:
        print(as_text(rows))
    return 1 if any(r.failures for r in rows) else 0


if __name__ == "__main__":
    sys.exit(main())
