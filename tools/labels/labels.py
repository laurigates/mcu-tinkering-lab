#!/usr/bin/env python3
"""Component sticker sheets: lay out, write the print plan and the kiss-cut files.

    python3 tools/labels/labels.py build <data.toml> <out dir> [--limit N]
    python3 tools/labels/labels.py check <data.toml>
    python3 tools/labels/labels.py candidates <data.toml> [--stock DIR]

A data file (see data/drawers-2026-10.toml) lists three kinds of sticker:

- ``[[label]]``  35 x 35 mm, for a divided container. The containers have a
                 raised rim, so the sticker sits 2.5 mm in from a 40 mm face.
- ``lid``        18 x 18 mm, an optional table on a label, for the 25 x 30 x 20 mm
                 flip-lid SMD boxes (1 mm in from a 20 mm face).
- ``[[strip]]``  80 x 20 mm, for bins of larger things; full size.

``build`` packs them onto A4 pages around Silhouette registration marks and
writes, into the output directory:

- ``plan.json``          positions plus each sticker's content, for labels.typ
- ``cut-p<N>.svg``       one kiss-cut file per page, for sendto_silhouette.py

Python owns the geometry so that the printed sticker and its cut line come from
one set of numbers; Typst only draws what the plan says. Units are millimetres
with the origin at the top-left corner of the page.

The registration marks follow inkscape-silhouette's render_silhouette_regmarks.py
(standard style): a 5 mm filled square at the origin and 20 mm L marks at the
top-right and bottom-left corners, 0.3 mm lines. The cutter's optical sensor
finds them and maps the cut onto the print.

``candidates`` reads a parts-stock tree (``items/**/*.toml``) read-only and prints
draft ``[[label]]`` entries for every item no label names with ``stock = "<slug>"``.
"""

from __future__ import annotations

import argparse
import json
import os
import sys
import tomllib
from dataclasses import dataclass
from pathlib import Path
from xml.sax.saxutils import escape

PAGE_W, PAGE_H = 210.0, 297.0

# Sticker sizes (w, h, corner radius), mm.
SIZES = {
    "label": (35.0, 35.0, 2.0),
    "lid": (18.0, 18.0, 1.5),
    "strip": (80.0, 20.0, 2.0),
}

# Registration marks, inkscape-silhouette's standard style, A4 defaults.
REG_SQUARE = 5.0
REG_ARM = 20.0
REG_STROKE = 0.3
DEFAULT_REG = {"x": 10.0, "y": 10.0, "w": 190.0, "l": 277.0}
# Stickers keep this far from any mark.
MARK_CLEARANCE = 2.0
# Space between neighbouring stickers: room to peel one without lifting the next,
# and for the printed colour bands' bleed.
DEFAULT_GAP = 3.0

# Render kinds labels.typ can draw. test_labels.py checks the template knows each.
RENDER_KINDS = {
    "chip",
    "tant",
    "polymer",
    "sop8",
    "usbc",
    "radial",
    "dip8",
    "led",
    "irrx",
    "tilt",
    "jack",
    "trim",
    "term",
    "module",
    "disc",
    "film",
    "tact",
    "card",
    "xt60",
    "antenna",
    "matrix",
    "axial",
    "to92",
    "to220",
    "sot223",
    "header",
    "housing",
    "barrel",
    "banana",
    "jumper",
    "spool",
    "ribbon",
    "coin",
    "cell",
    "pouch",
    "servo",
    "stepper",
    "dcmotor",
    "speaker",
    "buzzer",
    "button",
    "reed",
    "keypad",
    "ldr",
    "screw",
    "insert",
    "standoff",
    "heatsink",
    "dht",
    "laser",
    "holder",
    "magnet",
}
# Kinds whose drawing uses real dimensions, so the label also shows it at 1:1.
ACTUAL_SIZE_KINDS = {"chip", "tant", "polymer", "sop8", "usbc", "dip8", "led", "sot223"}
# Colour keys inside a render table that labels.py checks as #rrggbb.
RENDER_COLOUR_KEYS = ("fill", "pcb", "band")

CUT_COLOUR = "#e00000"
DEFAULT_STOCK = Path(
    os.environ.get("PARTS_STOCK", "~/repos/laurigates/parts-stock")
).expanduser()


class DataError(ValueError):
    """The data file breaks the schema below."""


@dataclass(frozen=True)
class Rect:
    x0: float
    y0: float
    x1: float
    y1: float

    def overlaps(self, other: Rect) -> bool:
        return (
            self.x0 < other.x1
            and other.x0 < self.x1
            and self.y0 < other.y1
            and other.y0 < self.y1
        )


@dataclass(frozen=True)
class Placed:
    kind: str  # label | lid | strip
    x: float
    y: float
    w: float
    h: float
    r: float
    content: dict

    @property
    def rect(self) -> Rect:
        return Rect(self.x, self.y, self.x + self.w, self.y + self.h)


# ---- data ----


def _hex(colour: str, where: str) -> str:
    c = colour.lstrip("#")
    if len(c) != 6 or any(ch not in "0123456789abcdefABCDEF" for ch in c):
        raise DataError(f"{where}: colour {colour!r} is not #rrggbb")
    return "#" + c.lower()


def _colour(name: str, colours: dict[str, str], where: str) -> str:
    if name.startswith("#"):
        return _hex(name, where)
    if name not in colours:
        raise DataError(f"{where}: colour {name!r} is not in [colors]")
    return colours[name]


def _lines(value, where: str) -> list[str]:
    if isinstance(value, str):
        return [value]
    if isinstance(value, list) and all(isinstance(v, str) for v in value):
        return list(value)
    raise DataError(f"{where}: expected a string or a list of strings")


def _render(raw, where: str) -> dict | None:
    if raw is None:
        return None
    if not isinstance(raw, dict) or "kind" not in raw:
        raise DataError(f"{where}: render needs a kind")
    if raw["kind"] not in RENDER_KINDS:
        raise DataError(
            f"{where}: render kind {raw['kind']!r} is not one of {sorted(RENDER_KINDS)}"
        )
    out = dict(raw)
    for key in RENDER_COLOUR_KEYS:
        if key in out:
            out[key] = _hex(out[key], f"{where}.render.{key}")
    if "bands" in out:
        out["bands"] = [
            _hex(b, f"{where}.render.bands[{i}]") for i, b in enumerate(out["bands"])
        ]
    out["actual"] = bool(out.get("actual", out["kind"] in ACTUAL_SIZE_KINDS))
    return out


def load(path: Path) -> dict:
    """Read and validate a data file; return it with colours resolved to hex."""
    with path.open("rb") as f:
        raw = tomllib.load(f)
    if raw.get("schema") != 1:
        raise DataError(f"{path}: schema must be 1")
    colours = {k: _hex(v, f"[colors].{k}") for k, v in raw.get("colors", {}).items()}
    sheet = raw.get("sheet", {})
    reg = {**DEFAULT_REG, **sheet.get("regmarks", {})}
    gap = float(sheet.get("gap", DEFAULT_GAP))

    labels = []
    for i, e in enumerate(raw.get("label", [])):
        where = f"label[{i}]"
        for key in ("cat", "color", "value", "spec", "foot"):
            if key not in e:
                raise DataError(f"{where} ({e.get('value', '?')}): missing {key}")
        label = {
            "cat": e["cat"],
            "color": _colour(e["color"], colours, where),
            "value": e["value"],
            "spec": _lines(e["spec"], f"{where}.spec"),
            "foot": e["foot"],
            "render": _render(e.get("render"), where),
            "stock": e.get("stock", ""),
        }
        if "lid" in e:
            lid = e["lid"]
            label["lid"] = {
                "value": lid.get("value", e["value"]),
                "spec": _lines(lid.get("spec", ""), f"{where}.lid.spec"),
            }
        labels.append(label)

    strips = []
    for i, s in enumerate(raw.get("strip", [])):
        where = f"strip[{i}]"
        for key in ("color", "title", "detail"):
            if key not in s:
                raise DataError(f"{where}: missing {key}")
        strips.append(
            {
                "color": _colour(s["color"], colours, where),
                "title": s["title"],
                "detail": s["detail"],
            }
        )

    return {
        "title": raw.get("title", path.stem),
        "regmarks": reg,
        "gap": gap,
        "labels": labels,
        "strips": strips,
    }


def stickers(data: dict) -> list[tuple[str, dict]]:
    """Every sticker in print order: labels, then lid labels, then strips."""
    out: list[tuple[str, dict]] = [("label", lab) for lab in data["labels"]]
    for lab in data["labels"]:
        if "lid" in lab:
            out.append(
                (
                    "lid",
                    {
                        "cat": lab["cat"],
                        "color": lab["color"],
                        "render": lab["render"],
                        **lab["lid"],
                    },
                )
            )
    out += [("strip", s) for s in data["strips"]]
    return out


# ---- layout ----


def keepouts(reg: dict) -> list[Rect]:
    """Areas around the three marks that no sticker may enter."""
    x, y, w, length = reg["x"], reg["y"], reg["w"], reg["l"]
    c, a = MARK_CLEARANCE, REG_ARM
    return [
        # the square, and the arm-length corner around it the sensor searches
        Rect(x - c, y - c, x + a + c, y + a + c),
        Rect(x + w - a - c, y - c, x + w + c, y + a + c),  # top-right L
        Rect(x - c, y + length - a - c, x + a + c, y + length + c),  # bottom-left L
    ]


def layout(data: dict) -> list[list[Placed]]:
    """Shelf-pack the stickers inside the marks; a new kind starts a new row.

    Returns one list of placed stickers per page.
    """
    reg, gap = data["regmarks"], data["gap"]
    area = Rect(reg["x"], reg["y"], reg["x"] + reg["w"], reg["y"] + reg["l"])
    blocked = keepouts(reg)

    pages: list[list[Placed]] = [[]]
    x, y, row_h, row_kind = area.x0, area.y0, 0.0, None

    def new_row():
        nonlocal x, y, row_h
        if row_h:
            y += row_h + gap
        x, row_h = area.x0, 0.0

    for kind, content in stickers(data):
        w, h, r = SIZES[kind]
        if w > area.x1 - area.x0 or h > area.y1 - area.y0:
            raise DataError(
                f"a {kind} sticker ({w} x {h} mm) does not fit inside the marks"
            )
        if row_kind is not None and kind != row_kind:
            new_row()
        row_kind = kind
        while True:
            if x + w > area.x1:
                new_row()
            if y + h > area.y1:
                pages.append([])
                x, y, row_h = area.x0, area.y0, 0.0
                continue
            cand = Rect(x, y, x + w, y + h)
            hit = next((b for b in blocked if b.overlaps(cand)), None)
            if hit is None:
                break
            x = hit.x1
        pages[-1].append(Placed(kind, round(x, 3), round(y, 3), w, h, r, content))
        x += w + gap
        row_h = max(row_h, h)
    return [p for p in pages if p]


# ---- outputs ----


def regmark_svg(reg: dict) -> str:
    x, y, w, ln = reg["x"], reg["y"], reg["w"], reg["l"]
    a, s = REG_ARM, REG_STROKE
    line = f"fill:none;stroke:#000000;stroke-width:{s}"
    # The notes line is the format render_silhouette_regmarks.py writes and
    # sendto_silhouette.py reads back to find the mark geometry.
    note = (
        f"mark distance from document: Left={x:g}mm, Top={y:g}mm; "
        f"mark to mark distance: X={w:g}mm, Y={ln:g}mm;"
    )
    return (
        '  <g inkscape:groupmode="layer" inkscape:label="Regmarks" id="regmark">\n'
        f'    <rect id="regmark-tl" x="{x:g}" y="{y:g}" '
        f'width="{REG_SQUARE:g}" height="{REG_SQUARE:g}" style="fill:#000000;stroke:none"/>\n'
        f'    <path id="regmark-tr" '
        f'd="M {x + w - a:g},{y:g} L {x + w:g},{y:g} L {x + w:g},{y + a:g}" style="{line}"/>\n'
        f'    <path id="regmark-bl" '
        f'd="M {x + a:g},{y + ln:g} L {x:g},{y + ln:g} L {x:g},{y + ln - a:g}" style="{line}"/>\n'
        f'    <text id="regmark-notes" x="{x + 30:g}" y="{y + ln + 6:g}" '
        f'style="font-size:2.5px">{escape(note)}</text>\n'
        "  </g>\n"
    )


def rounded_rect_path(p: Placed) -> str:
    """A closed path, clockwise from the top edge; arcs for the corners."""
    x0, y0, x1, y1, r = p.x, p.y, p.x + p.w, p.y + p.h, p.r
    return (
        f"M {x0 + r:g},{y0:g} H {x1 - r:g} A {r:g},{r:g} 0 0 1 {x1:g},{y0 + r:g} "
        f"V {y1 - r:g} A {r:g},{r:g} 0 0 1 {x1 - r:g},{y1:g} H {x0 + r:g} "
        f"A {r:g},{r:g} 0 0 1 {x0:g},{y1 - r:g} "
        f"V {y0 + r:g} A {r:g},{r:g} 0 0 1 {x0 + r:g},{y0:g} Z"
    )


def cut_svg(page: list[Placed], reg: dict) -> str:
    paths = "".join(
        f'    <path id="cut-{i + 1}" d="{rounded_rect_path(p)}" '
        f'style="fill:none;stroke:{CUT_COLOUR};stroke-width:0.1"/>\n'
        for i, p in enumerate(page)
    )
    return (
        '<?xml version="1.0" encoding="UTF-8"?>\n'
        '<svg xmlns="http://www.w3.org/2000/svg" '
        'xmlns:inkscape="http://www.inkscape.org/namespaces/inkscape" '
        f'width="{PAGE_W:g}mm" height="{PAGE_H:g}mm" viewBox="0 0 {PAGE_W:g} {PAGE_H:g}">\n'
        + regmark_svg(reg)
        + '  <g inkscape:groupmode="layer" inkscape:label="Cut" id="cut">\n'
        + paths
        + "  </g>\n</svg>\n"
    )


def plan(data: dict, pages: list[list[Placed]]) -> dict:
    return {
        "title": data["title"],
        "page": {"w": PAGE_W, "h": PAGE_H},
        "regmarks": {
            **data["regmarks"],
            "square": REG_SQUARE,
            "arm": REG_ARM,
            "stroke": REG_STROKE,
        },
        "pages": [
            [
                {
                    "kind": p.kind,
                    "x": p.x,
                    "y": p.y,
                    "w": p.w,
                    "h": p.h,
                    "r": p.r,
                    "content": p.content,
                }
                for p in page
            ]
            for page in pages
        ],
    }


def build(data_path: Path, out: Path, limit: int | None = None) -> list[list[Placed]]:
    data = load(data_path)
    pages = layout(data)
    out.mkdir(parents=True, exist_ok=True)
    for old in out.glob("cut-p*.svg"):
        old.unlink()
    (out / "plan.json").write_text(
        json.dumps(plan(data, pages), indent=1, ensure_ascii=False) + "\n"
    )
    for n, page in enumerate(pages, 1):
        cut = page[:limit] if limit else page
        (out / f"cut-p{n}.svg").write_text(cut_svg(cut, data["regmarks"]))
    return pages


# ---- parts-stock candidates ----

CATEGORY_NAMES = {
    "resistor": "Resistor",
    "capacitor": "Capacitor",
    "inductor": "Inductor",
    "diode": "Diode",
    "transistor": "Transistor",
    "ic": "IC",
    "module": "Module",
    "connector": "Connector",
    "wire": "Wire",
    "mechanical": "Mechanical",
    "consumable": "Consumable",
    "tool": "Tool",
    "board": "Dev board",
    "other": "Misc",
}
SPEC_ORDER = ("voltage", "current", "power", "tolerance", "dielectric", "pitch", "pins")


def _toml_str(s: str) -> str:
    return json.dumps(s, ensure_ascii=False)


def stock_items(stock: Path) -> list[tuple[str, dict]]:
    """(slug, item table) for every items/**/*.toml in a parts-stock tree."""
    items = []
    for path in sorted((stock / "items").glob("*/*.toml")):
        with path.open("rb") as f:
            items.append((path.stem, tomllib.load(f)))
    return items


def candidate(slug: str, doc: dict) -> str:
    """A draft [[label]] for one parts-stock item. Review before printing."""
    item, stock, specs = (
        doc.get("item", {}),
        doc.get("stock", {}),
        dict(doc.get("specs", {})),
    )
    unknown = specs.pop("unknown", [])
    value = str(specs.pop("value", "") or item.get("mpn") or item.get("name", slug))
    line1 = " · ".join(
        x
        for x in [item.get("package", "")]
        + [str(specs.pop(k)) for k in SPEC_ORDER if k in specs]
        if x
    )
    line2 = " · ".join(f"{k} {v}" for k, v in specs.items())
    spec = [s for s in (line1, line2) if s] or [item.get("name", slug)]
    if stock.get("mode") == "level":
        qty = f"level {stock.get('level', '?')}"
    else:
        qty = f"{stock.get('qty', '?')} pcs"
    dates = sorted(
        src.get("order", "").split("/")[-1][:7]
        for src in doc.get("sources", [])
        if src.get("order")
    )
    shops = sorted(
        {
            src.get("order", "").split("/")[0]
            for src in doc.get("sources", [])
            if src.get("order")
        }
    )
    when = (
        f"{shops[0].capitalize()} {dates[-1][5:7]}/{dates[-1][:4]}"
        if dates
        else "source in notes"
    )
    lines = [
        f"# parts-stock items/{item.get('category', '?')}/{slug}.toml: {item.get('name', '')}",
    ]
    if unknown:
        lines.append(f"# listing does not state: {', '.join(unknown)}")
    if not stock.get("verified"):
        lines.append("# quantity seeded from the order, not counted")
    lines += [
        "[[label]]",
        f"cat = {_toml_str(CATEGORY_NAMES.get(item.get('category', ''), 'Misc'))}",
        'color = "mech"  # TODO pick a [colors] key',
        f"value = {_toml_str(value)}",
        "spec = [" + ", ".join(_toml_str(s) for s in spec) + "]",
        f"foot = {_toml_str(f'{qty} · {when}')}",
        f"stock = {_toml_str(slug)}",
        "# render = { kind = ... }  # optional drawing; see labels.typ",
        "",
    ]
    return "\n".join(lines)


def candidates(data_path: Path, stock: Path) -> str:
    with data_path.open("rb") as f:
        raw = tomllib.load(f)
    labelled = {e["stock"] for e in raw.get("label", []) if e.get("stock")}
    items = stock_items(stock)
    if not items:
        raise DataError(f"{stock}/items holds no item files")
    out = [candidate(slug, doc) for slug, doc in items if slug not in labelled]
    header = f"# {len(out)} of {len(items)} parts-stock items have no label in {data_path.name}\n\n"
    return header + "\n".join(out)


# ---- CLI ----


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    sub = ap.add_subparsers(dest="cmd", required=True)
    b = sub.add_parser("build", help="write plan.json and cut-p<N>.svg")
    b.add_argument("data", type=Path)
    b.add_argument("out", type=Path)
    b.add_argument(
        "--limit",
        type=int,
        help="cut only the first N stickers of each page (a test cut)",
    )
    c = sub.add_parser("check", help="validate and lay out, write nothing")
    c.add_argument("data", type=Path)
    k = sub.add_parser(
        "candidates", help="draft labels for parts-stock items that have none"
    )
    k.add_argument("data", type=Path)
    k.add_argument("--stock", type=Path, default=DEFAULT_STOCK)
    args = ap.parse_args(argv)
    try:
        if args.cmd == "build":
            pages = build(args.data, args.out, args.limit)
            n = sum(len(p) for p in pages)
            print(f"{n} stickers on {len(pages)} page(s) -> {args.out}")
        elif args.cmd == "check":
            pages = layout(load(args.data))
            print(f"ok: {sum(len(p) for p in pages)} stickers on {len(pages)} page(s)")
        else:
            sys.stdout.write(candidates(args.data, args.stock))
    except (DataError, OSError, tomllib.TOMLDecodeError) as e:
        print(f"error: {e}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
