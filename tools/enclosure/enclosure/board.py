"""One piece of a carrier board, as a case needs it.

Coordinates are board millimetres relative to board.toml's [outline] origin:
x right, y down, as in [placement]. case.py flips y when it builds solids.
"""

from __future__ import annotations

import math
import re
import subprocess
import tomllib
from dataclasses import dataclass, field
from pathlib import Path

from kicad import paths, sexpr

KICAD_CLI = Path("/Applications/KiCad/KiCad.app/Contents/MacOS/kicad-cli")
KICAD_3DMODELS = Path("/Applications/KiCad/KiCad.app/Contents/SharedSupport/3dmodels")
ENCLOSURE_TOML = "enclosure.toml"
MOUNTING_HOLE = re.compile(r"MountingHole_(\d+(?:\.\d+)?)mm")


class EnclosureError(Exception):
    pass


@dataclass(frozen=True)
class Rect:
    x0: float
    y0: float
    x1: float
    y1: float

    @property
    def width(self) -> float:
        return self.x1 - self.x0

    @property
    def height(self) -> float:
        return self.y1 - self.y0

    def contains(self, x: float, y: float, margin: float = 0.0) -> bool:
        return (
            self.x0 - margin <= x <= self.x1 + margin
            and self.y0 - margin <= y <= self.y1 + margin
        )

    def overlaps(self, o: Rect) -> bool:
        return self.x0 < o.x1 and o.x0 < self.x1 and self.y0 < o.y1 and o.y0 < self.y1


@dataclass
class Part:
    ref: str
    footprint: str
    side: str  # "top" or "bottom"
    courtyard: Rect  # board coordinates
    above: float | None = None  # mm above the board's top face
    below: float = 0.0  # mm below the board's bottom face (pins, back-side parts)
    height_source: str = ""


@dataclass(frozen=True)
class Hole:
    ref: str
    x: float
    y: float
    drill: float  # the PCB hole, mm


@dataclass
class Piece:
    """One sawn-off piece of a board and everything on it."""

    project: Path
    name: str
    region: Rect
    quantity: int
    thickness: float
    holes: list[Hole]
    parts: list[Part]
    openings: list[str] = field(default_factory=list)
    lid_windows: list[str] = field(default_factory=list)
    mount_tabs: bool = True  # False for a piece held some other way (the cradle)

    @property
    def tallest(self) -> Part:
        return max(self.parts, key=lambda p: p.above or 0.0)

    @property
    def deepest(self) -> float:
        return max((p.below for p in self.parts), default=0.0)


def _rotate(lx: float, ly: float, deg: float) -> tuple[float, float]:
    """A footprint-local offset turned by KiCad's (at x y deg), y pointing down."""
    t = math.radians(deg)
    c, s = math.cos(t), math.sin(t)
    return lx * c + ly * s, -lx * s + ly * c


def _xy(block: str, keyword: str) -> tuple[float, float] | None:
    c = sexpr.child(block, keyword)
    if c is None:
        return None
    a = sexpr.atoms(c)
    return float(a[0]), float(a[1])


def _footprints(pcb_text: str) -> list[str]:
    return [
        b
        for _, b in sexpr.top_level_blocks(pcb_text, "\t")
        if sexpr.head(b)[0] == "footprint"
    ]


def _reference(fp: str) -> str:
    for c in sexpr.children(fp):
        if sexpr.head(c) == ("property", "Reference"):
            return sexpr.unquote(
                re.search(r'"Reference"\s+"((?:[^"\\]|\\.)*)"', c).group(1)
            )
    raise EnclosureError(f"footprint without a Reference: {fp[:60]!r}")


def _courtyard_points(fp: str) -> list[tuple[float, float]]:
    """Footprint-local points of every courtyard line, rect, arc, circle or polygon."""
    pts: list[tuple[float, float]] = []
    for c in sexpr.children(fp):
        kind = sexpr.head(c)[0]
        if not kind.startswith("fp_"):
            continue
        layer = sexpr.child(c, "layer")
        if layer is None or "CrtYd" not in layer:
            continue
        if kind == "fp_circle":
            cx, cy = _xy(c, "center")
            ex, ey = _xy(c, "end")
            r = math.hypot(ex - cx, ey - cy)
            pts += [
                (cx - r, cy - r),
                (cx + r, cy + r),
                (cx - r, cy + r),
                (cx + r, cy - r),
            ]
            continue
        for kw in ("start", "mid", "end"):
            p = _xy(c, kw)
            if p is not None:
                pts.append(p)
        poly = sexpr.child(c, "pts")
        if poly is not None:
            for xy in sexpr.children(poly):
                a = sexpr.atoms(xy)
                pts.append((float(a[0]), float(a[1])))
    return pts


def read_board(project: Path) -> tuple[list[Part], list[Hole], float]:
    """Every footprint's placed courtyard, every mounting hole, and the board thickness."""
    cfg = paths.load_board_config(project)
    ox, oy = cfg.data["outline"]["origin"]
    pcb = paths.kicad_project_dir(project) / f"{cfg.name}.kicad_pcb"
    text = pcb.read_text(encoding="utf-8")
    m = re.search(r"\(general\s*\(thickness ([\d.]+)\)", text)
    thickness = float(m.group(1)) if m else 1.6
    parts, holes = [], []
    for fp in _footprints(text):
        name = sexpr.head(fp)[1] or ""
        ref = _reference(fp)
        at = sexpr.atoms(sexpr.child(fp, "at"))
        fx, fy = float(at[0]) - ox, float(at[1]) - oy
        rot = float(at[2]) if len(at) > 2 else 0.0
        side = "bottom" if '"B.Cu"' in (sexpr.child(fp, "layer") or "") else "top"
        hole = MOUNTING_HOLE.search(name)
        if hole:
            holes.append(Hole(ref, fx, fy, float(hole.group(1))))
            continue
        local = _courtyard_points(fp)
        if not local:
            continue  # fiducials, mouse bites: nothing a case has to clear
        placed = [_rotate(lx, ly, rot) for lx, ly in local]
        xs = [fx + dx for dx, _ in placed]
        ys = [fy + dy for _, dy in placed]
        parts.append(Part(ref, name, side, Rect(min(xs), min(ys), max(xs), max(ys))))
    return parts, holes, thickness


def export_step(project: Path, out: Path) -> Path:
    """The board with its parts' 3D models, as KiCad exports it (board top-left at 0,0)."""
    cfg = paths.load_board_config(project)
    ox, oy = cfg.data["outline"]["origin"]
    pcb = paths.kicad_project_dir(project) / f"{cfg.name}.kicad_pcb"
    out.parent.mkdir(parents=True, exist_ok=True)
    subprocess.run(
        [
            str(KICAD_CLI),
            "pcb",
            "export",
            "step",
            "--force",
            "--subst-models",
            "--user-origin",
            f"{ox}x{oy}mm",
            "--define-var",
            f"KICAD10_3DMODEL_DIR={KICAD_3DMODELS}",
            "-o",
            str(out),
            str(pcb),
        ],
        check=True,
        capture_output=True,
        encoding="utf-8",  # kicad-cli prints non-ASCII; the locale may be ASCII
        errors="replace",
    )
    return out


def step_heights(step: Path, parts: list[Part]) -> None:
    """Fill `above` and `below` from the STEP: each model goes to the part whose
    courtyard holds its centre (the STEP names models by footprint, not by ref)."""
    from build123d import import_step

    compound = import_step(step)
    board = next(c for c in compound.children if c.label.endswith("_PCB"))
    bb = board.bounding_box()
    top, bottom = bb.max.Z, bb.min.Z
    for child in compound.children:
        if child is board:
            continue
        b = child.bounding_box()
        cx, cy = (b.min.X + b.max.X) / 2, -(b.min.Y + b.max.Y) / 2
        hits = [p for p in parts if p.courtyard.contains(cx, cy, margin=0.5)]
        if not hits:
            raise EnclosureError(
                f"{step.name}: model {child.label!r} at ({cx:.1f}, {cy:.1f}) "
                "lands on no courtyard; is the placement transform wrong?"
            )
        part = min(hits, key=lambda p: p.courtyard.width * p.courtyard.height)
        part.above = max(part.above or 0.0, b.max.Z - top)
        part.below = max(part.below, bottom - b.min.Z)
        part.height_source = f"STEP model {child.label}"


def load_pieces(project: Path, step: Path) -> list[Piece]:
    """The pieces enclosure.toml names, with heights from the STEP and the toml."""
    cfg_path = paths.carrier_dir(project).parent / "enclosure" / ENCLOSURE_TOML
    try:
        cfg = tomllib.loads(cfg_path.read_text(encoding="utf-8"))
    except FileNotFoundError:
        raise EnclosureError(f"{cfg_path}: no enclosure.toml") from None
    parts, holes, thickness = read_board(project)
    step_heights(step, parts)
    by_ref = {p.ref: p for p in parts}
    for ref, spec in cfg.get("heights", {}).items():
        if ref not in by_ref:
            raise EnclosureError(f"{cfg_path}: [heights] {ref}: no such footprint")
        p = by_ref[ref]
        p.above = float(spec["above"])
        p.below = max(p.below, float(spec.get("below", 0.0)))
        p.height_source = f"enclosure.toml: {spec['source']}"
    pieces = []
    for spec in cfg["pieces"]:
        region = Rect(*spec["region"])
        inside = [p for p in parts if region.contains(*_centre(p.courtyard))]
        missing = sorted(p.ref for p in inside if p.above is None and p.side == "top")
        if missing:
            raise EnclosureError(
                f"{cfg_path}: piece {spec['name']}: no height for {missing} "
                "(no 3D model in the STEP); add them under [heights]"
            )
        for p in inside:
            if p.above is None:
                p.above = 0.0
        named = set(spec.get("openings", [])) | set(spec.get("lid_windows", []))
        unknown = sorted(named - {p.ref for p in inside})
        if unknown:
            raise EnclosureError(
                f"{cfg_path}: piece {spec['name']}: openings {unknown} not on it"
            )
        pieces.append(
            Piece(
                project=project,
                name=spec["name"],
                region=region,
                quantity=int(spec.get("quantity", 1)),
                thickness=thickness,
                holes=[h for h in holes if region.contains(h.x, h.y)],
                parts=inside,
                openings=list(spec.get("openings", [])),
                lid_windows=list(spec.get("lid_windows", [])),
                mount_tabs=spec.get("mount", "tabs") == "tabs",
            )
        )
    return pieces


def _centre(r: Rect) -> tuple[float, float]:
    return (r.x0 + r.x1) / 2, (r.y0 + r.y1) / 2
