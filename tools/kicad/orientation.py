"""Wrong fits of a module socket, and the silkscreen marks that answer them.

A module socket is a footprint a whole module plugs into: a breakout in its
female headers. A rigid motion of the module that maps the socket's pad set
onto itself is a way the module physically fits wrong. If the motion moves a
pad onto a pad with another number, the pin-to-net map changes (J3's GY-521
turned 180 degrees puts +3V3 on its INT). If it keeps every pad number but
moves the module's outline (a 1xN row mirrored across its own axis), the module
sits face down: the wiring is right, the body lies on the other side of the row
and the sensor axes flip, which is what the 8d18cd8d balancebot board did. Each
such motion needs a silkscreen mark a builder can check before soldering, and
the carrier-pcb skill's trap 20 says so; this module makes it a gate.

Which footprints are module sockets (MODULE_SOCKETS):

- every `Breakout_Sockets:*` footprint (`kicad.sockets`): generated from a
  breakout's pad centres, with the module outline on F.Fab;
- `Module:Pololu_Breakout-*`, KiCad's Pololu-format carrier footprint, which
  the balancebot's DRV8825 drivers (A1, A2) sit in on two 1x8 strips; its F.Fab
  rectangle is the 15.2 x 20.3 mm carrier.

The XIAO sockets (`Seeed_XIAO:*-Socket`) are not on the list: Seeed's footprint
draws the module outline on F.SilkS with the USB-C connector at one end and a
pin-1 dot, so the footprint itself carries the orientation mark. A plain
PinSocket strip is not either: nothing says which module it takes or where
that module lies. A new module footprint joins the list here.

Finding the motions (`symmetries`): any rigid motion that maps a finite point
set onto itself fixes the set's centroid (it permutes the points, so it maps
their mean to their mean), so it is a rotation or a reflection about the
centroid. Such a map is fixed by where it sends two pads that are not
collinear with the centroid, and each must land on a pad at the same distance
from the centroid, so trying every such pair of target pads finds them all.
For the axis-aligned 2.54 mm grids every socket here uses, the result is a
subgroup of the square's eight symmetries (turns of 90, 180 and 270 degrees,
four mirrors): a lattice admits no other finite rotation than 60, 90 and 180
degrees, and a square grid has no 60. The search does not assume that,
so a socket with a tilted pad pattern is still covered. A set whose pads all
lie on one line through the centroid fixes only the line, so its candidates
are the four maps that keep the line: identity, the half turn, and the mirrors
across and along it.

`wrong_fits` classifies each motion other than the identity: a wrong fit when
it changes any pad's number, or, when the module outline is known (the F.Fab
box), when it keeps every pad but moves the outline. A motion that keeps both is
indistinguishable from the right fit and needs no mark.

`check` is the board gate `kicad.boardcheck` runs on each board: every module
socket with a wrong fit needs at least one board.toml `[silk] texts` entry with
`for = "<ref>"` (the orientation marks: pin names, a direction arrow, "CHIP
SIDE UP"); every text tagged `for` must lie within MARK_DISTANCE_MM of its
footprint's courtyard and be on the committed board; and every `[silk]
module_outlines` reference's F.Fab outline must be on the board's F.SilkS
(`outline_lines`, which `kicad.boardinput` hands to build_board.py).
"""

from __future__ import annotations

import fnmatch
import functools
import math
import re
from dataclasses import dataclass
from pathlib import Path
from typing import Any

from . import footprints, library, sexpr
from .errors import KicadError

MODULE_SOCKETS = ("Breakout_Sockets:*", "Module:Pololu_Breakout-*")
# How far a tagged text's anchor may sit from its footprint's courtyard box.
# The courtyard already covers the module's outline, so 4 mm is a 1.0 mm text
# label one text height or so outside the module, as the Pololu drivers' VMOT
# labels are (3.5 mm, beside the motor connectors); a text 20 mm off is over
# another part.
MARK_DISTANCE_MM = 4.0
# Coordinates in mm; pads sit on a 0.01 mm grid at worst.
TOL = 1e-3
# A text or line on the board must be where board.toml says, to KiCad's 1 nm
# rounding (build_board.py's resilk rounds positions back to board.toml's).
BOARD_TOL = 0.005

Point = tuple[float, float]
Box = tuple[float, float, float, float]
Segment = tuple[float, float, float, float]
Matrix = tuple[float, float, float, float]  # x' = a x + b y, y' = c x + d y


@dataclass(frozen=True)
class Pad:
    number: str
    x: float
    y: float


@dataclass(frozen=True)
class WrongFit:
    motion: str
    moves: tuple[tuple[str, str], ...]  # (number, the number it lands on), changed only
    face_down: bool  # a mirror: the module turned over

    def describe(self) -> str:
        side = " (module face down)" if self.face_down else ""
        what = _moves_text(self.moves) if self.moves else "same pads, the outline moves"
        return f"{self.motion}{side}: {what}"


def is_module_socket(fpid: str) -> bool:
    return any(fnmatch.fnmatchcase(fpid, pattern) for pattern in MODULE_SOCKETS)


def socket_pads(path: Path) -> list[Pad]:
    """Every numbered pad of a footprint, duplicates included (a twin's pads)."""
    text = path.read_text(encoding="utf-8")
    ((start, end),) = sexpr.spans(text)
    pads = []
    for c in sexpr.children(text[start:end]):
        kw, number = sexpr.head(c)
        if kw != "pad" or not number:
            continue
        at = sexpr.child(c, "at")
        if at is None:
            raise KicadError(f"{path}: pad {number!r} without (at ...)")
        x, y = (float(v) for v in sexpr.atoms(at)[:2])
        pads.append(Pad(number, x, y))
    return pads


def fab_lines(path: Path) -> list[Segment]:
    """The F.Fab outline as straight segments: each fp_line, fp_rect edge and
    fp_poly edge, in file order. Text, arcs and circles are left out."""
    return layer_lines(path, "F.Fab")


def layer_lines(path: Path, layer_name: str) -> list[Segment]:
    """A footprint's straight graphics on one layer, as segments in file order."""
    text = path.read_text(encoding="utf-8")
    ((start, end),) = sexpr.spans(text)
    out: list[Segment] = []
    for c in sexpr.children(text[start:end]):
        kw, _ = sexpr.head(c)
        layer = sexpr.child(c, "layer")
        if layer is None or sexpr.head(layer)[1] != layer_name:
            continue
        if kw == "fp_line":
            (ax, ay), (bx, by) = _xy(c, "start"), _xy(c, "end")
            out.append((ax, ay, bx, by))
        elif kw == "fp_rect":
            (ax, ay), (bx, by) = _xy(c, "start"), _xy(c, "end")
            corners = [(ax, ay), (bx, ay), (bx, by), (ax, by)]
            out += _closed(corners)
        elif kw == "fp_poly":
            pts = sexpr.child(c, "pts")
            if pts is None:
                raise KicadError(f"{path}: fp_poly without (pts ...)")
            corners = [
                (float(a), float(b))
                for a, b in (sexpr.atoms(xy)[:2] for xy in sexpr.children(pts))
            ]
            out += _closed(corners)
    return out


def fab_box(path: Path) -> Box | None:
    """The box over the F.Fab outline's segments: the module board, or None."""
    lines = fab_lines(path)
    if not lines:
        return None
    xs = [v for s in lines for v in (s[0], s[2])]
    ys = [v for s in lines for v in (s[1], s[3])]
    return min(xs), min(ys), max(xs), max(ys)


def _xy(block: str, keyword: str) -> Point:
    c = sexpr.child(block, keyword)
    if c is None:
        raise KicadError(f"{sexpr.head(block)[0]} without ({keyword} ...)")
    x, y = (float(v) for v in sexpr.atoms(c)[:2])
    return x, y


def _closed(corners: list[Point]) -> list[Segment]:
    return [
        (a[0], a[1], b[0], b[1])
        for a, b in zip(corners, corners[1:] + corners[:1], strict=True)
    ]


# --- symmetries -------------------------------------------------------------


def _apply(m: Matrix, p: Point) -> Point:
    return m[0] * p[0] + m[1] * p[1], m[2] * p[0] + m[3] * p[1]


def _key(p: Point) -> tuple[int, int]:
    return round(p[0] / TOL), round(p[1] / TOL)


def _maps_onto(m: Matrix, pts: list[Point]) -> bool:
    want = sorted(_key(p) for p in pts)
    got = sorted(_key(_apply(m, p)) for p in pts)
    return all(
        abs(a[0] - b[0]) <= 1 and abs(a[1] - b[1]) <= 1
        for a, b in zip(want, got, strict=True)
    )


def _centroid(pts: list[Point]) -> Point:
    return sum(p[0] for p in pts) / len(pts), sum(p[1] for p in pts) / len(pts)


def symmetries(points: list[Point]) -> list[Matrix]:
    """Every orthogonal map about the centroid that maps `points` onto itself,
    the identity first. Each matrix acts on coordinates relative to the centroid."""
    if not points:
        return []
    cx, cy = _centroid(points)
    rel = [(x - cx, y - cy) for x, y in points]
    norm = [math.hypot(*p) for p in rel]
    a = max(range(len(rel)), key=lambda i: norm[i])
    if norm[a] < TOL:  # every pad on the centroid: one pad, the identity only
        return [(1.0, 0.0, 0.0, 1.0)]
    ua = (rel[a][0] / norm[a], rel[a][1] / norm[a])
    cross = [ua[0] * p[1] - ua[1] * p[0] for p in rel]
    b = max(range(len(rel)), key=lambda i: abs(cross[i]))
    if abs(cross[b]) < TOL:
        # Collinear: the maps that keep the line through the centroid.
        ux, uy = ua
        reflect_line = (ux * ux - uy * uy, 2 * ux * uy, 2 * ux * uy, uy * uy - ux * ux)
        candidates = [
            (1.0, 0.0, 0.0, 1.0),
            (-1.0, 0.0, 0.0, -1.0),
            reflect_line,
            tuple(-v for v in reflect_line),
        ]
    else:
        pa, pb = rel[a], rel[b]
        det = pa[0] * pb[1] - pa[1] * pb[0]
        # inverse of the matrix whose columns are pa, pb
        inv = (pb[1] / det, -pb[0] / det, -pa[1] / det, pa[0] / det)
        candidates = []
        for qa in rel:
            if abs(math.hypot(*qa) - norm[a]) > TOL:
                continue
            for qb in rel:
                if abs(math.hypot(*qb) - math.hypot(*pb)) > TOL:
                    continue
                # M = [qa qb] @ inv
                m = (
                    qa[0] * inv[0] + qb[0] * inv[2],
                    qa[0] * inv[1] + qb[0] * inv[3],
                    qa[1] * inv[0] + qb[1] * inv[2],
                    qa[1] * inv[1] + qb[1] * inv[3],
                )
                if _orthogonal(m):
                    candidates.append(m)
    out: list[Matrix] = []
    for m in candidates:
        m = tuple(0.0 if abs(v) < 1e-9 else v for v in m)
        if _maps_onto(m, rel) and not any(_same(m, n) for n in out):
            out.append(m)
    out.sort(key=lambda m: (not _same(m, (1.0, 0.0, 0.0, 1.0)), _det(m) < 0, _angle(m)))
    return out


def _orthogonal(m: Matrix) -> bool:
    a, b, c, d = m
    return (
        abs(a * a + c * c - 1) < 1e-6
        and abs(b * b + d * d - 1) < 1e-6
        and abs(a * b + c * d) < 1e-6
    )


def _same(m: Matrix, n: Matrix) -> bool:
    return all(abs(x - y) < 1e-6 for x, y in zip(m, n, strict=True))


def _det(m: Matrix) -> float:
    return m[0] * m[3] - m[1] * m[2]


def _angle(m: Matrix) -> float:
    """A rotation's angle, or twice a mirror axis's angle, in [0, 360)."""
    return round(math.degrees(math.atan2(m[2], m[0])) % 360, 6)


def describe_motion(m: Matrix) -> str:
    """The motion in plain words, in the footprint's own frame (y down)."""
    if _det(m) > 0:
        return f"turned {_angle(m):g} degrees"
    axis = round(_angle(m) / 2, 6) % 180
    named = {
        0: "flipped top to bottom",
        90: "flipped left to right",
        45: "flipped along the diagonal",
        135: "flipped along the other diagonal",
    }
    return named.get(axis, f"flipped about the {axis:g}-degree axis")


def wrong_fits(pads: list[Pad], outline: Box | None) -> list[WrongFit]:
    """Each motion other than the identity that seats the module differently."""
    pts = [(p.x, p.y) for p in pads]
    if not pts:
        return []
    cx, cy = _centroid(pts)
    by_pos = {_key((p.x - cx, p.y - cy)): p.number for p in pads}
    out = []
    for m in symmetries(pts)[1:]:
        moves: dict[tuple[str, str], None] = {}
        for p in pads:
            q = _apply(m, (p.x - cx, p.y - cy))
            target = by_pos.get(_key(q))
            if target is None:  # off by a rounding step: find it within TOL
                target = next(
                    n
                    for k, n in by_pos.items()
                    if abs(k[0] - _key(q)[0]) <= 1 and abs(k[1] - _key(q)[1]) <= 1
                )
            if target != p.number:
                moves[(p.number, target)] = None
        face_down = _det(m) < 0
        if moves:
            out.append(WrongFit(describe_motion(m), tuple(moves), face_down))
        elif outline is not None and not _box_invariant(m, outline, (cx, cy)):
            out.append(WrongFit(describe_motion(m), (), face_down))
    return out


def _box_invariant(m: Matrix, box: Box, centre: Point) -> bool:
    cx, cy = centre
    corners = [
        _apply(m, (x - cx, y - cy)) for x in (box[0], box[2]) for y in (box[1], box[3])
    ]
    moved = (
        min(c[0] for c in corners) + cx,
        min(c[1] for c in corners) + cy,
        max(c[0] for c in corners) + cx,
        max(c[1] for c in corners) + cy,
    )
    return all(abs(a - b) < TOL for a, b in zip(moved, box, strict=True))


def _pad_key(n: str) -> tuple[int, int | str]:
    return (0, int(n)) if n.isdigit() else (1, n)


def _moves_text(moves: tuple[tuple[str, str], ...], limit: int = 4) -> str:
    """'pad 1 <-> pad 8, pad 2 <-> pad 7, ...' for swaps, 'pad 1 -> pad 5' otherwise."""
    pairs = set(moves)
    parts = []
    seen = set()
    for a, b in sorted(moves, key=lambda ab: _pad_key(ab[0])):
        if (a, b) in seen:
            continue
        if (b, a) in pairs:
            parts.append(f"pad {a} <-> pad {b}")
            seen |= {(a, b), (b, a)}
        else:
            parts.append(f"pad {a} -> pad {b}")
            seen.add((a, b))
    more = len(parts) - limit
    shown = ", ".join(parts[:limit])
    return f"{shown}, and {more} more" if more > 0 else shown


@functools.cache
def footprint_wrong_fits(path: Path) -> tuple[WrongFit, ...]:
    return tuple(wrong_fits(socket_pads(path), fab_box(path)))


# --- placement --------------------------------------------------------------


def place(local: Point, at: list[Any]) -> Point:
    """A footprint-local point on the board, origin-relative, for a board.toml
    [placement] entry [x, y, rot] or [x, y, rot, "back"]. KiCad's rotation
    turns counter-clockwise as drawn (y down); a back-side footprint mirrors
    its local y about the anchor first (carrier-pcb trap 21)."""
    lx, ly = local
    if at[3:] == ["back"]:
        ly = -ly
    t = math.radians(at[2])
    c, s = math.cos(t), math.sin(t)
    return at[0] + lx * c + ly * s, at[1] - lx * s + ly * c


def placed_box(box: Box, at: list[Any]) -> Box:
    corners = [place((x, y), at) for x in (box[0], box[2]) for y in (box[1], box[3])]
    xs = [c[0] for c in corners]
    ys = [c[1] for c in corners]
    return min(xs), min(ys), max(xs), max(ys)


# Where a module outline edge runs along the footprint's own silkscreen (the
# GY-521's rear edge is 0.06 mm from its socket's strip outline), the
# footprint's line already marks it, and two 0.15 mm strokes that close
# overlap: KiCad's DRC reports silk_overlap. A piece of the outline whose
# centre line comes within OUTLINE_GAP_MM of a footprint silkscreen line is left
# out: the two 0.15 mm strokes' half widths plus 0.05 mm. Pieces are
# OUTLINE_STEP_MM long, and a kept run shorter than OUTLINE_MIN_MM is a sliver
# and dropped.
OUTLINE_GAP_MM = 0.2
OUTLINE_STEP_MM = 0.05
OUTLINE_MIN_MM = 0.2


def outline_lines(fp_path: Path, at: list[Any]) -> list[list[float]]:
    """The footprint's F.Fab outline placed on the board, origin-relative,
    each segment [x0, y0, x1, y1] rounded to KiCad's 1 nm, less the pieces
    the footprint's own F.SilkS already draws (OUTLINE_GAP_MM)."""
    own = layer_lines(fp_path, "F.SilkS")
    out = []
    for seg in fab_lines(fp_path):
        for x0, y0, x1, y1 in clip_outline(seg, own):
            a, b = place((x0, y0), at), place((x1, y1), at)
            out.append([round(v, 6) for v in (*a, *b)])
    return out


def clip_outline(seg: Segment, obstacles: list[Segment]) -> list[Segment]:
    """The runs of `seg` farther than OUTLINE_GAP_MM from every obstacle line."""
    ax, ay, bx, by = seg
    length = math.hypot(bx - ax, by - ay)
    n = max(1, math.ceil(length / OUTLINE_STEP_MM))
    pts = [(ax + (bx - ax) * i / n, ay + (by - ay) * i / n) for i in range(n + 1)]

    def clear(p: Point, q: Point) -> bool:
        mid = ((p[0] + q[0]) / 2, (p[1] + q[1]) / 2)
        return all(
            _point_segment(r, o) >= OUTLINE_GAP_MM
            for o in obstacles
            for r in (p, mid, q)
        )

    runs: list[Segment] = []
    start = None
    for i in range(n):
        if clear(pts[i], pts[i + 1]):
            if start is None:
                start = i
            continue
        if start is not None:
            runs.append((*pts[start], *pts[i]))
            start = None
    if start is not None:
        runs.append((*pts[start], *pts[n]))
    return [r for r in runs if math.hypot(r[2] - r[0], r[3] - r[1]) >= OUTLINE_MIN_MM]


def _point_segment(p: Point, s: Segment) -> float:
    ax, ay, bx, by = s
    dx, dy = bx - ax, by - ay
    ll = dx * dx + dy * dy
    t = (
        0.0
        if ll == 0
        else max(0.0, min(1.0, ((p[0] - ax) * dx + (p[1] - ay) * dy) / ll))
    )
    return math.hypot(p[0] - (ax + t * dx), p[1] - (ay + t * dy))


def _box_distance(p: Point, box: Box) -> float:
    dx = max(box[0] - p[0], 0.0, p[0] - box[2])
    dy = max(box[1] - p[1], 0.0, p[1] - box[3])
    return math.hypot(dx, dy)


# --- the board gate ---------------------------------------------------------

_GR_TEXT = re.compile(
    r'\(gr_text "((?:[^"\\]|\\.)*)"\s*\(at (-?[\d.]+) (-?[\d.]+)(?: -?[\d.]+)?\)'
    r'\s*\(layer "F\.SilkS"\)'
)
_GR_LINE = re.compile(
    r"\(gr_line\s*\(start (-?[\d.]+) (-?[\d.]+)\)\s*\(end (-?[\d.]+) (-?[\d.]+)\)"
    r'\s*\(stroke\s*\(width [\d.]+\)\s*\(type \w+\)\s*\)\s*\(layer "F\.SilkS"\)'
)


def board_texts(board_text: str) -> list[tuple[str, float, float]]:
    """(text, x, y) of each board-level F.SilkS text, absolute mm."""
    return [
        (sexpr.unquote(m.group(1)), float(m.group(2)), float(m.group(3)))
        for m in _GR_TEXT.finditer(board_text)
    ]


def board_lines(board_text: str) -> list[Segment]:
    """Each board-level F.SilkS line segment, absolute mm."""
    return [tuple(float(v) for v in m.groups()) for m in _GR_LINE.finditer(board_text)]


def _segment_on(seg: Segment, lines: list[Segment]) -> bool:
    a, b = seg[:2], seg[2:]
    for line in lines:
        p, q = line[:2], line[2:]
        for u, v in ((p, q), (q, p)):
            if all(
                abs(s - t) <= BOARD_TOL for s, t in zip((*a, *b), (*u, *v), strict=True)
            ):
                return True
    return False


def check(
    data: dict[str, Any],
    comps: list[tuple[str, str]],
    vendored: Path = library.VENDORED,
    board_text: str | None = None,
) -> list[str]:
    """board.toml's orientation marks for (ref, footprint id) `comps`; [] when complete.

    `data` must already pass `boardcheck.validate_config`. With `board_text`
    the tagged texts and module outlines must also be on that board.
    """
    placement = data["placement"]
    silk = data.get("silk", {})
    texts = silk.get("texts", [])
    origin = data["outline"]["origin"]
    fpid = dict(comps)
    errors = []
    marks: dict[str, list[dict[str, Any]]] = {}
    for t in texts:
        if "for" in t:
            marks.setdefault(t["for"], []).append(t)
    for ref, fp in comps:
        if not is_module_socket(fp):
            continue
        fits = footprint_wrong_fits(library.footprint_file(fp, vendored=vendored))
        if fits and not marks.get(ref):
            ways = "; ".join(f.describe() for f in fits)
            errors.append(
                f"[silk] {ref} ({fp}) fits its socket {len(fits)} wrong "
                f'way(s) and no [silk] texts entry names it (for = "{ref}"): {ways}'
            )
    on_board = board_texts(board_text) if board_text is not None else []
    for ref, tagged in marks.items():
        path = library.footprint_file(fpid[ref], vendored=vendored)
        box = placed_box(footprints.parse(path).courtyard, placement[ref])
        for t in tagged:
            d = _box_distance(tuple(t["at"]), box)
            if d > MARK_DISTANCE_MM:
                errors.append(
                    f'[silk] texts: {t["text"]!r} (for = "{ref}") at {t["at"]} is '
                    f"{d:.1f} mm from {ref}'s courtyard; a mark must be within "
                    f"{MARK_DISTANCE_MM} mm"
                )
            if board_text is None:
                continue
            x, y = origin[0] + t["at"][0], origin[1] + t["at"][1]
            if not any(
                s == t["text"] and abs(bx - x) <= BOARD_TOL and abs(by - y) <= BOARD_TOL
                for s, bx, by in on_board
            ):
                errors.append(
                    f'[silk] texts: {t["text"]!r} (for = "{ref}") at {t["at"]} is '
                    "not on the board's F.SilkS"
                )
    if board_text is not None:
        lines = board_lines(board_text)
        for ref in silk.get("module_outlines", []):
            path = library.footprint_file(fpid[ref], vendored=vendored)
            want = outline_lines(path, placement[ref])
            missing = [
                s
                for s in want
                if not _segment_on(
                    (
                        s[0] + origin[0],
                        s[1] + origin[1],
                        s[2] + origin[0],
                        s[3] + origin[1],
                    ),
                    lines,
                )
            ]
            if missing:
                errors.append(
                    f"[silk] module_outlines: {len(missing)} of {ref}'s {len(want)} "
                    f"F.Fab outline segment(s) are not on the board's F.SilkS"
                )
    return errors


def validate(
    data: dict[str, Any],
    comps: list[tuple[str, str]],
    vendored: Path = library.VENDORED,
) -> list[str]:
    """`[silk] module_outlines` against the footprints: each named footprint
    has an F.Fab outline and sits on the front."""
    fpid = dict(comps)
    errors = []
    for ref in data.get("silk", {}).get("module_outlines", []):
        if ref not in fpid:
            continue  # boardcheck.validate_config reports it
        if data["placement"].get(ref, [])[3:] == ["back"]:
            errors.append(
                f"[silk] module_outlines: {ref} is on the back; only front-side "
                "outlines are drawn"
            )
        if not fab_lines(library.footprint_file(fpid[ref], vendored=vendored)):
            errors.append(
                f"[silk] module_outlines: {ref}'s footprint {fpid[ref]} has no "
                "F.Fab outline to draw"
            )
    return errors
