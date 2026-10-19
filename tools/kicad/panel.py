"""Mouse-bite panels: board.toml `[panel]` turned into slots and tab holes.

A carrier with several boards on one PCB (the robocar carriers' main board and
three head boards) is split along its `[silk] cut_lines`. By default
(`mode = "saw"`) those are only silkscreen saw lines over copper-free bands.
With `mode = "mousebites"` (#809) each cut line becomes routed slots on
Edge.Cuts, broken by tabs, and each tab carries two rows of non-plated holes,
one along each edge of the slot, to snap along:

    [panel]
    mode = "mousebites"
    slot_mm = 2.0        # slot width (JLCPCB: 1.6-2 mm gap, 1.2 mm minimum)
    hole_mm = 0.6        # mouse-bite drill (JLCPCB: 0.6 mm)
    pitch_mm = 0.95      # hole centre to centre: a 0.35 mm web (JLCPCB: 0.35-0.4, 0.3 minimum)
    holes = 5            # holes per row (JLCPCB: 5-8 per set)
    tab_mm = 5.0         # tab length along the line
    max_span_mm = 50.0   # longest slot between tabs (JLCPCB: a set every 50-60 mm)

    [[panel.lines]]      # optional, per cut line
    line = [x0, y0, x1, y1]   # one of [silk] cut_lines, exactly
    offset_mm = 0.0      # slot centre from the line, toward its left-hand side
    tabs = [49.5]        # interior tab centres, mm from the line's start

Geometry, all relative to the outline origin, cut lines horizontal or vertical:

- The slot lies on the line's left-hand side (where build_board.py draws the
  "CUT" label: the waste side), its near edge on the line, unless
  `offset_mm` moves its centre. A line between two boards takes
  `offset_mm = 0`, centred.
- A line whose end meets another cut line stops at that line's slot.
- Each line has a tab at both ends and, unless `tabs` lists them, as few
  evenly spaced interior tabs as keep every slot within `max_span_mm`.
- Each tab's holes sit in two rows on the slot's two edges, centred on the tab.

`validate` checks the table; `build` returns the `Panel` (None in saw mode),
which `kicad.boardinput` hands to build_board.py and `kicad.boardcheck` compares
with the committed board. Every slot and every hole must lie inside board.toml
`[[keepouts]]` (the copper-free bands), so routing, pours and DRC's
copper-to-edge clearance hold without new rules; build_board.py cuts each
tab's hole area (`Tab.clear`) out of those rule areas, since a rule area that
forbids pads also forbids a non-plated hole.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any

MODES = ("saw", "mousebites")
KEYS = frozenset(
    {
        "mode",
        "slot_mm",
        "hole_mm",
        "pitch_mm",
        "holes",
        "tab_mm",
        "max_span_mm",
        "lines",
    }
)
LINE_KEYS = frozenset({"line", "offset_mm", "tabs"})
DEFAULTS: dict[str, Any] = {
    "mode": "saw",
    "slot_mm": 2.0,
    "hole_mm": 0.6,
    "pitch_mm": 0.95,
    "holes": 5,
    "tab_mm": 5.0,
    "max_span_mm": 50.0,
}
# JLCPCB's mouse-bite guide (2026-10): gap 1.2 mm minimum, web between holes
# 0.3 mm minimum, at least 5 holes a set; its NPTH minimum is 0.5 mm.
MIN_SLOT_MM = 1.2
MIN_WEB_MM = 0.3
MIN_HOLES = 5
MIN_HOLE_MM = 0.5
# A tab's cut-out from the keepout rule areas reaches this far past its holes.
CLEAR_MARGIN_MM = 0.1
EPS = 1e-6
FOOTPRINT_PREFIX = "MB"  # tab footprints' references: MB1, MB2, ...

Rect = tuple[float, float, float, float]
Point = tuple[float, float]


@dataclass(frozen=True)
class Tab:
    name: str  # MB<n>, its footprint's reference
    at: Point  # centre, the footprint's anchor
    holes: tuple[Point, ...]
    clear: Rect  # area cut out of the keepout rule areas


@dataclass(frozen=True)
class Panel:
    hole_mm: float
    slots: tuple[Rect, ...]
    tabs: tuple[Tab, ...]


def settings(data: dict[str, Any]) -> dict[str, Any]:
    """`[panel]` with the defaults filled in."""
    return {**DEFAULTS, **data.get("panel", {})}


def _num(v: Any) -> bool:
    return isinstance(v, (int, float)) and not isinstance(v, bool)


def _line_ok(v: Any) -> bool:
    return isinstance(v, list) and len(v) == 4 and all(_num(x) for x in v)


@dataclass(frozen=True)
class _Line:
    p0: Point
    u: Point  # unit direction
    n: Point  # unit left-hand normal, build_board.py's label side
    length: float
    offset: float  # slot centre from the line along n


def _line(raw: list[float], offset: float) -> _Line:
    x0, y0, x1, y1 = raw
    length = math.hypot(x1 - x0, y1 - y0)
    u = ((x1 - x0) / length, (y1 - y0) / length)
    return _Line((x0, y0), u, (u[1], -u[0]), length, offset)


def _dot(a: Point, b: Point) -> float:
    return a[0] * b[0] + a[1] * b[1]


def _on(p: Point, raw: list[float]) -> bool:
    """Whether `p` lies on segment `raw`."""
    ax, ay, bx, by = raw
    dx, dy = bx - ax, by - ay
    t = max(0.0, min(1.0, ((p[0] - ax) * dx + (p[1] - ay) * dy) / (dx * dx + dy * dy)))
    return math.hypot(ax + t * dx - p[0], ay + t * dy - p[1]) < EPS


def _rect(line: _Line, t0: float, t1: float, a0: float, a1: float) -> Rect:
    """Axis-aligned rect spanning t0..t1 along `line` and a0..a1 along its normal."""
    pts = [
        (
            line.p0[0] + t * line.u[0] + a * line.n[0],
            line.p0[1] + t * line.u[1] + a * line.n[1],
        )
        for t in (t0, t1)
        for a in (a0, a1)
    ]
    xs, ys = [p[0] for p in pts], [p[1] for p in pts]
    return (round(min(xs), 6), round(min(ys), 6), round(max(xs), 6), round(max(ys), 6))


def _point(line: _Line, t: float, a: float) -> Point:
    return (
        round(line.p0[0] + t * line.u[0] + a * line.n[0], 6),
        round(line.p0[1] + t * line.u[1] + a * line.n[1], 6),
    )


def _covered(rect: Rect, rects: list[Rect]) -> bool:
    """Whether the union of `rects` covers `rect` (all axis-aligned)."""
    xs = sorted({rect[0], rect[2], *(r[i] for r in rects for i in (0, 2))})
    ys = sorted({rect[1], rect[3], *(r[i] for r in rects for i in (1, 3))})
    xs = [x for x in xs if rect[0] <= x <= rect[2]]
    ys = [y for y in ys if rect[1] <= y <= rect[3]]
    for xa, xb in zip(xs, xs[1:]):
        for ya, yb in zip(ys, ys[1:]):
            cx, cy = (xa + xb) / 2, (ya + yb) / 2
            if not any(r[0] <= cx <= r[2] and r[1] <= cy <= r[3] for r in rects):
                return False
    return True


def _overlap(a: Rect, b: Rect) -> bool:
    return (
        a[0] < b[2] - EPS
        and b[0] < a[2] - EPS
        and a[1] < b[3] - EPS
        and b[1] < a[3] - EPS
    )


def validate(where: str, data: dict[str, Any]) -> list[str]:
    """`[panel]`'s own values; [] when valid. Geometry errors come from `build`."""
    raw = data.get("panel")
    if raw is None:
        return []
    if not isinstance(raw, dict):
        return [f"{where}: [panel] must be a table"]
    errors = []
    unknown = sorted(set(raw) - KEYS)
    if unknown:
        errors.append(
            f"{where}: [panel]: unknown key(s) {unknown}; allowed: {sorted(KEYS)}"
        )
    s = settings(data)
    if s["mode"] not in MODES:
        errors.append(f"{where}: [panel] mode {s['mode']!r}, not one of {list(MODES)}")
    for k in ("slot_mm", "hole_mm", "pitch_mm", "tab_mm", "max_span_mm"):
        if not _num(s[k]) or s[k] <= 0:
            errors.append(f"{where}: [panel] {k} must be > 0")
    if (
        not isinstance(s["holes"], int)
        or isinstance(s["holes"], bool)
        or s["holes"] < 1
    ):
        errors.append(f"{where}: [panel] holes must be a positive integer")
    if errors:
        return errors
    if s["slot_mm"] < MIN_SLOT_MM:
        errors.append(
            f"{where}: [panel] slot_mm {s['slot_mm']} is under JLCPCB's {MIN_SLOT_MM} mm"
        )
    if s["hole_mm"] < MIN_HOLE_MM:
        errors.append(
            f"{where}: [panel] hole_mm {s['hole_mm']} is under JLCPCB's {MIN_HOLE_MM} mm"
        )
    if s["pitch_mm"] - s["hole_mm"] < MIN_WEB_MM - EPS:
        errors.append(
            f"{where}: [panel] pitch_mm - hole_mm leaves a {s['pitch_mm'] - s['hole_mm']:.2f} mm "
            f"web; JLCPCB's minimum is {MIN_WEB_MM} mm"
        )
    if s["holes"] < MIN_HOLES:
        errors.append(
            f"{where}: [panel] holes {s['holes']}: JLCPCB asks for at least {MIN_HOLES}"
        )
    row = (s["holes"] - 1) * s["pitch_mm"] + s["hole_mm"]
    if row + 2 * MIN_WEB_MM > s["tab_mm"] + EPS:
        errors.append(
            f"{where}: [panel] tab_mm {s['tab_mm']} is shorter than its {s['holes']} holes "
            f"({row:.2f} mm) plus a {MIN_WEB_MM} mm web each side"
        )
    cut_lines = data.get("silk", {}).get("cut_lines", [])
    lines = s.get("lines", [])
    if not isinstance(lines, list):
        return errors + [f"{where}: [[panel.lines]] must be an array of tables"]
    seen = []
    for entry in lines:
        if not isinstance(entry, dict):
            errors.append(f"{where}: [[panel.lines]] must be tables")
            continue
        extra = sorted(set(entry) - LINE_KEYS)
        if extra:
            errors.append(f"{where}: [[panel.lines]]: unknown key(s) {extra}")
        line = entry.get("line")
        if not _line_ok(line) or line not in cut_lines:
            errors.append(
                f"{where}: [[panel.lines]] line {line!r} is not one of [silk] cut_lines"
            )
        elif line in seen:
            errors.append(f"{where}: [[panel.lines]] line {line!r} is listed twice")
        else:
            seen.append(line)
        if "offset_mm" in entry and not _num(entry["offset_mm"]):
            errors.append(
                f"{where}: [[panel.lines]] {line!r}: offset_mm must be a number"
            )
        tabs = entry.get("tabs", [])
        if not isinstance(tabs, list) or not all(_num(t) for t in tabs):
            errors.append(
                f"{where}: [[panel.lines]] {line!r}: tabs must be a list of numbers"
            )
    return errors


def build(data: dict[str, Any]) -> Panel | None:
    """The panel board.toml describes; None in saw mode. Raises ValueError with
    every geometry problem (call `validate` first)."""
    s = settings(data)
    if s["mode"] != "mousebites":
        return None
    w, tab, n_holes = s["slot_mm"], s["tab_mm"], s["holes"]
    r = s["hole_mm"] / 2
    raws = data.get("silk", {}).get("cut_lines", [])
    overrides = {tuple(e["line"]): e for e in s.get("lines", [])}
    errors: list[str] = []
    if not raws:
        errors.append("[panel] mousebites needs [silk] cut_lines to cut along")
    for raw in raws:
        if abs(raw[0] - raw[2]) > EPS and abs(raw[1] - raw[3]) > EPS:
            errors.append(
                f"cut line {raw}: mousebites needs horizontal or vertical lines"
            )
    if errors:
        raise ValueError("\n".join(errors))
    lines = [
        _line(raw, overrides.get(tuple(raw), {}).get("offset_mm", w / 2))
        for raw in raws
    ]
    keepouts: list[Rect] = [tuple(k["rect"]) for k in data.get("keepouts", [])]
    slots: list[tuple[Rect, list[float]]] = []
    tabs: list[Tab] = []
    for raw, line in zip(raws, lines):
        where = f"cut line {raw}"
        # Where the line meets another one, stop at that line's slot.
        t_start, t_end = 0.0, line.length
        for other_raw, other in zip(raws, lines):
            if other_raw is raw:
                continue
            for end in (0, 1):
                p = (raw[0], raw[1]) if end == 0 else (raw[2], raw[3])
                if not _on(p, other_raw):
                    continue
                s_dir = _dot(line.u, other.n) * (1 if end == 0 else -1)
                if abs(s_dir) < EPS:
                    errors.append(f"{where}: runs along cut line {other_raw}")
                    continue
                trim = max(0.0, s_dir * other.offset + w / 2)
                if end == 0:
                    t_start = max(t_start, trim)
                else:
                    t_end = min(t_end, line.length - trim)
        if t_end - t_start < 2 * tab + w:
            errors.append(
                f"{where}: {t_end - t_start:.2f} mm of slot run is too short for two "
                f"{tab} mm end tabs and a {w} mm slot"
            )
            continue
        explicit = overrides.get(tuple(raw), {}).get("tabs")
        if explicit is None:
            free = t_end - t_start - 2 * tab
            k = 0
            while (free - k * tab) / (k + 1) > s["max_span_mm"] + EPS:
                k += 1
            gap = (free - k * tab) / (k + 1)
            interior = [
                t_start + tab + (i + 1) * gap + i * tab + tab / 2 for i in range(k)
            ]
        else:
            interior = sorted(explicit)
        centres = [t_start + tab / 2, *interior, t_end - tab / 2]
        for a, b in zip(centres, centres[1:]):
            s0, s1 = a + tab / 2, b - tab / 2
            if s1 - s0 < w - EPS:
                errors.append(
                    f"{where}: tabs at {a:.2f} and {b:.2f} mm leave {s1 - s0:.2f} mm of "
                    f"slot, under its {w} mm width"
                )
                continue
            if s1 - s0 > s["max_span_mm"] + EPS:
                errors.append(
                    f"{where}: {s1 - s0:.2f} mm of slot between tabs, over max_span_mm"
                )
            slots.append(
                (_rect(line, s0, s1, line.offset - w / 2, line.offset + w / 2), raw)
            )
        for c in centres:
            name = f"{FOOTPRINT_PREFIX}{len(tabs) + 1}"
            span = (n_holes - 1) * s["pitch_mm"]
            holes = tuple(
                _point(
                    line, c - span / 2 + i * s["pitch_mm"], line.offset + side * w / 2
                )
                for side in (-1, 1)
                for i in range(n_holes)
            )
            reach = w / 2 + r + CLEAR_MARGIN_MM
            tabs.append(
                Tab(
                    name=name,
                    at=_point(line, c, line.offset),
                    holes=holes,
                    clear=_rect(
                        line,
                        c - tab / 2,
                        c + tab / 2,
                        line.offset - reach,
                        line.offset + reach,
                    ),
                )
            )
            for hx, hy in holes:
                if not _covered((hx - r, hy - r, hx + r, hy + r), keepouts):
                    errors.append(
                        f"{where}: tab {name} hole at ({hx:g}, {hy:g}) is outside "
                        "[[keepouts]]; move the tab (tabs = [...]) away from copper"
                    )
                    break
    for i, (a, raw_a) in enumerate(slots):
        if not _covered(a, keepouts):
            errors.append(
                f"cut line {raw_a}: slot {list(a)} is not inside [[keepouts]]"
            )
        for b, raw_b in slots[i + 1 :]:
            if _overlap(a, b):
                errors.append(
                    f"slots {list(a)} ({raw_a}) and {list(b)} ({raw_b}) overlap"
                )
    if errors:
        raise ValueError("\n".join(errors))
    return Panel(
        hole_mm=s["hole_mm"],
        slots=tuple(s for s, _ in slots),
        tabs=tuple(tabs),
    )


def as_json(panel: Panel | None) -> dict[str, Any] | None:
    """The board input's `panel`: what build_board.py draws."""
    if panel is None:
        return None
    return {
        "hole_mm": panel.hole_mm,
        "slots": [list(s) for s in panel.slots],
        "tabs": [
            {
                "name": t.name,
                "at": list(t.at),
                "holes": [list(h) for h in t.holes],
                "clear": list(t.clear),
            }
            for t in panel.tabs
        ],
    }
