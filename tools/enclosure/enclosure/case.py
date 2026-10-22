"""The open tray and the closed box around one board piece.

Both styles hold the board on standoffs at its mounting holes. One through
bolt per hole clamps the stack: (box lid pillar,) board, standoff, floor, and
on into the chassis or a nut under the floor. Case coordinates: x right, y up
(board y flipped), z up from the floor's underside; the board's bottom face
sits at `z_board`.
"""

from __future__ import annotations

from dataclasses import dataclass

from build123d import Align, Box, Cylinder, Part, Pos

from .board import EnclosureError, Piece, Rect
from .board import Part as BoardPart

A = (Align.MIN, Align.MIN, Align.MIN)
C = (Align.CENTER, Align.CENTER, Align.MIN)

# Bolt clearance and boss diameter by PCB hole: an M3 board (3.2 mm hole) and
# an M2 board (2.2 mm hole). ISO 273 medium clearance; boss = 2.2 x clearance.
BOLTS = {3.2: ("M3", 3.4, 7.0), 2.2: ("M2", 2.4, 5.0)}


@dataclass(frozen=True)
class Params:
    wall: float = 2.0
    floor: float = 2.0
    gap: float = 0.5  # board edge to wall
    under: float = 1.5  # clearance below the deepest pin or back-side part
    min_standoff: float = 3.0
    lip: float = 3.0  # tray wall above the board's top face
    lid: float = 2.0
    lid_clearance: float = 2.0  # above the tallest part
    plug: float = 6.0  # a mated plug's height over its header, opening refs only
    opening_margin: float = 1.0  # each side of an opening's courtyard

    @classmethod
    def from_toml(cls, data: dict) -> Params:
        unknown = set(data) - set(cls.__dataclass_fields__)
        if unknown:
            raise EnclosureError(f"[case]: unknown key(s) {sorted(unknown)}")
        return cls(**{k: float(v) for k, v in data.items()})


@dataclass
class Frame:
    """Where the board sits inside the case, from the piece and the params."""

    piece: Piece
    p: Params

    @property
    def off(self) -> float:
        return self.p.wall + self.p.gap

    @property
    def length(self) -> float:
        return self.piece.region.width + 2 * self.off

    @property
    def width(self) -> float:
        return self.piece.region.height + 2 * self.off

    def below(self, part: BoardPart) -> float:
        """Depth under the board's bottom face; a back-side opening part's plug adds."""
        extra = (
            self.p.plug
            if part.side == "bottom" and part.ref in self.piece.openings
            else 0.0
        )
        return part.below + extra

    @property
    def standoff(self) -> float:
        deepest = max((self.below(q) for q in self.piece.parts), default=0.0)
        return max(self.p.min_standoff, deepest + self.p.under)

    @property
    def z_board(self) -> float:
        return self.p.floor + self.standoff

    @property
    def z_top(self) -> float:
        return self.z_board + self.piece.thickness

    def above(self, part: BoardPart) -> float:
        extra = (
            self.p.plug
            if part.side == "top" and part.ref in self.piece.openings
            else 0.0
        )
        return (part.above or 0.0) + extra

    @property
    def headroom(self) -> float:
        return max(self.above(q) for q in self.piece.parts if q.side == "top")

    def xy(self, x: float, y: float) -> tuple[float, float]:
        r = self.piece.region
        return x - r.x0 + self.off, r.y1 - y + self.off

    def rect(self, r: Rect) -> tuple[float, float, float, float]:
        ax, ay = self.xy(r.x0, r.y1)
        return ax, ay, r.width, r.height


def _bolt(hole_drill: float) -> tuple[str, float, float]:
    try:
        return BOLTS[round(hole_drill, 1)]
    except KeyError:
        raise EnclosureError(
            f"no bolt for a {hole_drill} mm hole; add it to BOLTS"
        ) from None


def _shell(f: Frame, height: float) -> Part:
    p = f.p
    outer = Box(f.length, f.width, height, align=A)
    inner = Pos(p.wall, p.wall, p.floor) * Box(
        f.length - 2 * p.wall, f.width - 2 * p.wall, height, align=A
    )
    return outer - inner


def _standoffs(f: Frame, body: Part) -> Part:
    for h in f.piece.holes:
        _, _, boss = _bolt(h.drill)
        x, y = f.xy(h.x, h.y)
        body += Pos(x, y, 0) * Cylinder(boss / 2, f.z_board, align=C)
    return body


def _bores(f: Frame, body: Part, height: float) -> Part:
    for h in f.piece.holes:
        _, clear, _ = _bolt(h.drill)
        x, y = f.xy(h.x, h.y)
        body -= Pos(x, y, -1) * Cylinder(clear / 2, height + 2, align=C)
    return body


def _side(r: Rect, c: Rect) -> str:
    """The piece edge a part's courtyard is nearest to."""
    d = {
        "west": c.x0 - r.x0,
        "east": r.x1 - c.x1,
        "north": c.y0 - r.y0,
        "south": r.y1 - c.y1,
    }
    return min(d, key=lambda k: d[k])


def _openings(f: Frame, body: Part, z_to: float) -> Part:
    """A window through the wall beside each opening part: above the board for a
    top-side part, below it for a back-side one."""
    p, r = f.p, f.piece.region
    by_ref = {q.ref: q for q in f.piece.parts}
    for ref in f.piece.openings:
        part = by_ref[ref]
        c = part.courtyard
        side = _side(r, c)
        m = p.opening_margin
        if part.side == "top":
            z_from = f.z_top - 0.5
        else:
            z_from, z_to = p.floor, f.z_board + 0.5
        h = z_to - z_from
        depth = f.off + 2  # through the wall and the gap
        if side in ("west", "east"):
            x0, _ = f.xy(r.x0 if side == "west" else r.x1, 0)
            x = x0 - depth if side == "west" else x0 - 1
            _, y_lo = f.xy(0, c.y1)
            body -= Pos(x, y_lo - m, z_from) * Box(
                depth + 1, c.height + 2 * m, h, align=A
            )
        else:
            _, y0 = f.xy(0, r.y0 if side == "north" else r.y1)
            y = y0 - 1 if side == "north" else y0 - depth
            x_lo, _ = f.xy(c.x0, 0)
            body -= Pos(x_lo - m, y, z_from) * Box(
                c.width + 2 * m, depth + 1, h, align=A
            )
    return body


def tray(f: Frame) -> Part:
    height = f.z_top + f.p.lip
    body = _standoffs(f, _shell(f, height))
    body = _openings(f, body, height + 1)
    return _bores(f, body, height)


def box(f: Frame) -> tuple[Part, Part]:
    """(base, lid). The lid sits on the walls, a lip locates it, and a pillar
    over each hole bears on the board so the through bolt clamps it."""
    p = f.p
    wall_top = f.z_top + f.headroom + p.lid_clearance
    base = _standoffs(f, _shell(f, wall_top))
    base = _openings(f, base, wall_top + 1)
    base = _bores(f, base, wall_top)

    lid = Pos(0, 0, wall_top) * Box(f.length, f.width, p.lid, align=A)
    t = 0.3  # lip-to-wall slide fit
    lip_h = 3.0
    ring = Pos(p.wall + t, p.wall + t, wall_top - lip_h) * Box(
        f.length - 2 * (p.wall + t), f.width - 2 * (p.wall + t), lip_h, align=A
    )
    ring -= Pos(p.wall + t + 1.2, p.wall + t + 1.2, wall_top - lip_h - 1) * Box(
        f.length - 2 * (p.wall + t + 1.2),
        f.width - 2 * (p.wall + t + 1.2),
        lip_h + 2,
        align=A,
    )
    lid += ring
    for h in f.piece.holes:
        _, _, boss = _bolt(h.drill)
        x, y = f.xy(h.x, h.y)
        # A pillar bears on the board unless a part's courtyard reaches under
        # it; then it stops clear above that part and only guides the bolt.
        reach = Rect(h.x - boss / 2, h.y - boss / 2, h.x + boss / 2, h.y + boss / 2)
        under = [
            f.above(q)
            for q in f.piece.parts
            if q.side == "top" and q.courtyard.overlaps(reach)
        ]
        z0 = f.z_top + (max(under) + 0.5 if under else 0.0)
        if z0 < wall_top:
            lid += Pos(x, y, z0) * Cylinder(boss / 2, wall_top - z0, align=C)
    for ref in f.piece.lid_windows:
        part = next(q for q in f.piece.parts if q.ref == ref)
        x, y, w, h = f.rect(part.courtyard)
        m = p.opening_margin
        lid -= Pos(x - m, y - m, wall_top - lip_h - 1) * Box(
            w + 2 * m, h + 2 * m, lip_h + p.lid + 2, align=A
        )
    lid = _bores(f, lid, wall_top + p.lid)
    return base, lid


def keepout(f: Frame, board_step: Part | None) -> Part | None:
    """Everything the case must not touch: the board and its STEP models moved
    into case coordinates, plus a stand-in box for each part with no model."""
    r = f.piece.region
    solids: list[Part] = []
    if board_step is not None:
        # The STEP puts the board's top-left corner at (0, 0) with y up and
        # the board's bottom face at z 0.
        moved = Pos(f.off - r.x0, f.off + r.y1, f.z_board) * board_step
        solids.append(moved)
    for q in f.piece.parts:
        if q.side != "top" or not q.height_source.startswith("enclosure.toml"):
            continue
        x, y, w, h = f.rect(q.courtyard)
        solids.append(Pos(x, y, f.z_top) * Box(w, h, q.above or 0.01, align=A))
    if not solids:
        return None
    out = solids[0]
    for s in solids[1:]:
        out += s
    return out


def interference(case: Part, keep: Part | None, region_box: Part) -> float:
    """mm³ of case material inside the keep-out, within this piece's footprint."""
    if keep is None:
        return 0.0
    clash = case & (keep & region_box)
    return sum(s.volume for s in clash.solids()) if clash else 0.0
