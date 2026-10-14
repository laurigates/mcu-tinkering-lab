"""Footprints: numbered pads, the F.CrtYd box, and the `(attr ...)` line.

A `.kicad_mod` is copied into tools/kicad/lib/ unchanged, so this only reads
it. Unnumbered pads (`(pad "" ...)`, a connector's mechanical pegs) are not
connection points and are left out of `pads`.
"""

from __future__ import annotations

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
    return Footprint(name=name, pads=pads, courtyard=courtyard(text, path), attr=attr)


def courtyard(
    text: str, where: object = "footprint"
) -> tuple[float, float, float, float]:
    """The box over every F.CrtYd graphic's points, as perfboard has always read it."""
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
    """(pin number -> (x, y) of its first through-hole pad, F.CrtYd box).

    The perfboard generator's reading, verbatim (perfboard.py:86-106), so its
    drawing stays byte-identical when it moves onto this module.
    """
    text = path.read_text()
    pads = {}
    for m in re.finditer(
        r'\(pad "(\d+)" thru_hole \w+\s*\(at ([-\d.]+) ([-\d.]+)', text
    ):
        pads.setdefault(m.group(1), (float(m.group(2)), float(m.group(3))))
    if not pads:
        raise KicadError(f"{path}: no through-hole pads")
    return pads, courtyard(text, path)
