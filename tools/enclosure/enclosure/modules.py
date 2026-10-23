"""Where the look-alike breakout models go, for the product renders.

The parts enclosure.toml gives a height to have no KiCad 3D model, so the
board GLB shows bare sockets where the DRV8825s, XIAOs and Adafruit boards
sit. modules.toml says what each looks like; this module reads where each one
is from the committed .kicad_pcb (footprint position and rotation, pads, the
F.Fab outline) and hands blender/render_product.py one spec per part.

Spec coordinates are footprint-local millimetres, KiCad's y-down, as the
footprint file has them; `origin` and `angle` place that frame in the scene.
"""

from __future__ import annotations

import tomllib
from pathlib import Path

from kicad import paths, sexpr

from . import board

CATALOG = Path(__file__).resolve().parent.parent / "modules.toml"


def load_catalog(path: Path = CATALOG) -> dict:
    return tomllib.loads(path.read_text(encoding="utf-8"))


def _fab_bbox(fp: str) -> list[float] | None:
    pts = []
    for c in sexpr.children(fp):
        kind = sexpr.head(c)[0]
        if kind not in ("fp_line", "fp_rect", "fp_poly"):
            continue
        layer = sexpr.child(c, "layer")
        if layer is None or "F.Fab" not in layer:
            continue
        for kw in ("start", "end"):
            p = board._xy(c, kw)
            if p is not None:
                pts.append(p)
        poly = sexpr.child(c, "pts")
        if poly is not None:
            for xy in sexpr.children(poly):
                a = sexpr.atoms(xy)
                pts.append((float(a[0]), float(a[1])))
    if not pts:
        return None
    xs, ys = [p[0] for p in pts], [p[1] for p in pts]
    return [min(xs), min(ys), max(xs), max(ys)]


def _pads(fp: str) -> list[list[float]]:
    out = []
    for c in sexpr.children(fp):
        if sexpr.head(c)[0] != "pad":
            continue
        a = sexpr.atoms(sexpr.child(c, "at"))
        out.append([float(a[0]), float(a[1])])
    return out


def _to_local(rect: list[float], fx: float, fy: float, rot: float) -> list[float]:
    """A board-millimetre rectangle in the footprint's own frame."""
    corners = [
        board._rotate(x - fx, y - fy, -rot)
        for x in (rect[0], rect[2])
        for y in (rect[1], rect[3])
    ]
    xs, ys = [c[0] for c in corners], [c[1] for c in corners]
    return [min(xs), min(ys), max(xs), max(ys)]


def specs(project: Path, piece: board.Piece, frame, heights: dict) -> list[dict]:
    """One look-alike per part of the piece whose height enclosure.toml estimates.

    `frame` is the piece's case.Frame; `heights` the enclosure.toml [heights]
    table (for the `module` and `outline` overrides).
    """
    catalog = load_catalog()
    cfg = paths.load_board_config(project)
    ox, oy = cfg.data["outline"]["origin"]
    pcb = paths.kicad_project_dir(project) / f"{cfg.name}.kicad_pcb"
    fps = {board._reference(fp): fp for fp in board._footprints(pcb.read_text())}
    r = piece.region
    out = []
    for part in piece.parts:
        if not (
            part.side == "top"
            and part.height_source.startswith("enclosure.toml")
            and part.above
        ):
            continue
        fp = fps[part.ref]
        override = heights.get(part.ref, {})
        entry = catalog["footprints"].get(part.footprint, {})
        kind = override.get("module", entry.get("kind"))
        if kind is None:
            raise board.EnclosureError(
                f"{part.ref} ({part.footprint}): no module kind in modules.toml "
                "[footprints] and no `module` in enclosure.toml"
            )
        at = sexpr.atoms(sexpr.child(fp, "at"))
        fx, fy = float(at[0]) - ox, float(at[1]) - oy
        rot = float(at[2]) if len(at) > 2 else 0.0
        if "outline" in override:
            outline = _to_local(override["outline"], fx, fy, rot)
        else:
            outline = _fab_bbox(fp)
        if outline is None:
            raise board.EnclosureError(
                f"{part.ref}: footprint has no F.Fab outline; give one in enclosure.toml"
            )
        out.append(
            {
                "ref": part.ref,
                "kind": kind,
                # Board mm (y down) to the scene (y up), as assembly's board_offset.
                "origin": [fx + frame.off - r.x0, frame.off + r.y1 - fy],
                "angle": rot,
                "z": frame.z_top,
                "above": part.above,
                "outline": outline,
                "pads": _pads(fp),
                "sockets": entry.get("sockets", True),
                **catalog["kinds"][kind],
            }
        )
    return out
