"""Footprints: numbered pads, the F.CrtYd box, and the `(attr ...)` line.

A `.kicad_mod` is copied into tools/kicad/lib/ unchanged, so this only reads
it. Unnumbered pads (`(pad "" ...)`, a connector's mechanical pegs) are not
connection points and are left out of `pads`.
"""

from __future__ import annotations

import math
import re
from dataclasses import dataclass
from pathlib import Path

from . import sexpr
from .errors import KicadError


@dataclass(frozen=True)
class PadInfo:
    number: str
    kind: str  # thru_hole, smd, np_thru_hole, connect
    shape: str  # rect, roundrect, circle, oval, ...
    x: float
    y: float


@dataclass(frozen=True)
class Footprint:
    name: str
    pads: dict[str, PadInfo]  # number -> its first pad, in file order
    courtyard: tuple[float, float, float, float]  # F.CrtYd (xmin, ymin, xmax, ymax)
    attr: str  # "smd", "through_hole", "exclude_from_pos_files exclude_from_bom", ...


def parse(path: Path) -> Footprint:
    text = path.read_text(encoding="utf-8")
    try:
        ((start, end),) = sexpr.spans(text)
    except ValueError:
        raise KicadError(f"{path}: expected one (footprint ...) expression") from None
    top = text[start:end]
    kw, name = sexpr.head(top)
    if kw != "footprint" or name is None:
        raise KicadError(f"{path}: not a footprint (starts with {kw!r})")
    pads: dict[str, PadInfo] = {}
    attr = ""
    for c in sexpr.children(top):
        ckw, number = sexpr.head(c)
        if ckw == "attr":
            attr = " ".join(sexpr.atoms(c))
        elif ckw == "pad" and number:
            kinds = sexpr.atoms(c)
            at = sexpr.child(c, "at")
            if at is None or len(kinds) < 2:
                raise KicadError(
                    f"{path}: pad {number!r} without a type, shape or (at ...)"
                )
            x, y = (float(v) for v in sexpr.atoms(at)[:2])
            pads.setdefault(number, PadInfo(number, kinds[0], kinds[1], x, y))
    return Footprint(name=name, pads=pads, courtyard=courtyard(top, path), attr=attr)


def user_texts(path: Path) -> list[tuple[str, float, float]]:
    """(text, x, y) of every `(fp_text user "...")`, on any layer, in file order.

    The parts registry reads the "+"/"-" and "K"/"A" marks among them to prove a
    polarised part's pin map against the footprint's own silkscreen.
    """
    text = path.read_text(encoding="utf-8")
    return [
        (sexpr.unquote(m.group(1)), float(m.group(2)), float(m.group(3)))
        for m in re.finditer(
            r'\(fp_text user "((?:[^"\\]|\\.)*)"\s*\(at (-?[\d.]+) (-?[\d.]+)', text
        )
    ]


def courtyard(
    text: str, where: object = "footprint"
) -> tuple[float, float, float, float]:
    """The (xmin, ymin, xmax, ymax) box over every top-level F.CrtYd graphic.

    Each `fp_line`/`fp_rect`/`fp_poly` contributes its points, an `fp_circle` its
    centre +/- radius, and an `fp_arc` its ends plus each axis extreme it sweeps
    through. Graphics on other layers contribute nothing.
    """
    try:
        ((start, end),) = sexpr.spans(text)
    except ValueError:
        raise KicadError(f"{where}: expected one (footprint ...) expression") from None
    xs: list[float] = []
    ys: list[float] = []
    for c in sexpr.children(text[start:end]):
        kw, _ = sexpr.head(c)
        if kw not in _GRAPHICS:
            continue
        layer = sexpr.child(c, "layer")
        if layer is None or sexpr.head(layer)[1] != "F.CrtYd":
            continue
        for x, y in _extent(c, kw, where):
            xs.append(x)
            ys.append(y)
    if not xs:
        raise KicadError(f"{where}: no F.CrtYd courtyard")
    return min(xs), min(ys), max(xs), max(ys)


_GRAPHICS = frozenset({"fp_line", "fp_rect", "fp_circle", "fp_arc", "fp_poly"})


def _point(block: str, keyword: str, where: object) -> tuple[float, float]:
    c = sexpr.child(block, keyword)
    if c is None:
        raise KicadError(f"{where}: {sexpr.head(block)[0]} without ({keyword} ...)")
    x, y = (float(v) for v in sexpr.atoms(c)[:2])
    return x, y


def _extent(block: str, kw: str, where: object) -> list[tuple[float, float]]:
    """Points whose box is the graphic's box."""
    if kw == "fp_poly":
        pts = sexpr.child(block, "pts")
        if pts is None:
            raise KicadError(f"{where}: fp_poly without (pts ...)")
        return [
            (float(a), float(b))
            for a, b in (sexpr.atoms(xy)[:2] for xy in sexpr.children(pts))
        ]
    if kw == "fp_circle":
        (cx, cy), (ex, ey) = _point(block, "center", where), _point(block, "end", where)
        r = math.hypot(ex - cx, ey - cy)
        return [(cx - r, cy - r), (cx + r, cy + r)]
    if kw == "fp_arc":
        return _arc_extent(*(_point(block, k, where) for k in ("start", "mid", "end")))
    return [_point(block, "start", where), _point(block, "end", where)]


def _arc_extent(s, m, e) -> list[tuple[float, float]]:
    """The ends of the arc s -> m -> e plus every axis extreme on it."""
    (ax, ay), (bx, by), (cx, cy) = s, m, e
    d = 2 * (ax * (by - cy) + bx * (cy - ay) + cx * (ay - by))
    if math.isclose(d, 0.0, abs_tol=1e-12):
        return [s, m, e]  # collinear: a straight segment
    ux = (
        (ax**2 + ay**2) * (by - cy)
        + (bx**2 + by**2) * (cy - ay)
        + (cx**2 + cy**2) * (ay - by)
    ) / d
    uy = (
        (ax**2 + ay**2) * (cx - bx)
        + (bx**2 + by**2) * (ax - cx)
        + (cx**2 + cy**2) * (bx - ax)
    ) / d
    r = math.hypot(ax - ux, ay - uy)

    def angle(p):
        return math.atan2(p[1] - uy, p[0] - ux) % math.tau

    a0, am, a1 = angle(s), angle(m), angle(e)
    ccw = (am - a0) % math.tau < (a1 - a0) % math.tau  # the sweep s -> m -> e
    lo, span = (a0, (a1 - a0) % math.tau) if ccw else (a1, (a0 - a1) % math.tau)
    out = [s, e]
    for k, (dx, dy) in enumerate(((1, 0), (0, 1), (-1, 0), (0, -1))):
        if (k * math.pi / 2 - lo) % math.tau <= span:
            out.append((ux + r * dx, uy + r * dy))
    return out


def perfboard_courtyard(text: str, where: object = "footprint") -> tuple:
    """perfboard.py's own reading of the F.CrtYd box (perfboard.py:96-107), verbatim.

    It differs from `courtyard` on some footprints, because it matches from any
    `(fp_...` up to the next F.CrtYd layer (so an earlier silkscreen or fab
    graphic's points are counted) and reads a circle as its centre and one rim
    point. It is kept, not fixed, so the perfboard drawing stays byte-identical
    when the generator moves onto this module (issue #750 PR C); the parts it
    places today are drawn with this box.
    """
    xs, ys = [], []
    for block in re.findall(
        r"\(fp_(?:line|rect|circle|arc|poly)(.*?)\(layer \"F\.CrtYd\"\)", text, re.S
    ):
        for x, y in re.findall(
            r"\((?:start|end|center|mid|xy) ([-\d.]+) ([-\d.]+)\)", block
        ):
            xs.append(float(x))
            ys.append(float(y))
    if not xs:
        raise KicadError(f"{where}: no F.CrtYd courtyard")
    return min(xs), min(ys), max(xs), max(ys)


def thru_hole_pads(path: Path) -> tuple[dict[str, tuple[float, float]], tuple]:
    """(pin number -> (x, y) of its first through-hole pad, perfboard's F.CrtYd box).

    The perfboard generator's reading, verbatim (perfboard.py:86-106), so its
    drawing stays byte-identical when it moves onto this module. The box is
    `perfboard_courtyard`, not `courtyard`; see there for where they differ.
    """
    text = path.read_text()
    pads = {}
    for m in re.finditer(
        r'\(pad "(\d+)" thru_hole \w+\s*\(at ([-\d.]+) ([-\d.]+)', text
    ):
        pads.setdefault(m.group(1), (float(m.group(2)), float(m.group(3))))
    if not pads:
        raise KicadError(f"{path}: no through-hole pads")
    return pads, perfboard_courtyard(text, path)
