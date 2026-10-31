"""The open tray and the closed box around one board piece.

The board screws down onto short standoffs at its mounting holes: a
self-tapping screw from above into a blind pilot hole (resin takes a thread
well; it does not take a heat-set insert). The case itself bolts to the
chassis through two floor tabs. The box lid hangs a tab inside each long
wall near each end, and a screw goes through the wall into each tab from
outside, so nothing in the base stands over the board: the board drops in
and every board screw can be driven before the lid goes on. Case
coordinates: x right, y up (board y flipped), z up from the floor's
underside; the board's bottom face sits at `z_board`.
"""

from __future__ import annotations

from dataclasses import dataclass

from build123d import Align, Box, Cylinder, Part, Pos, Rot

from .board import EnclosureError, Piece, Rect
from .board import Part as BoardPart

A = (Align.MIN, Align.MIN, Align.MIN)
C = (Align.CENTER, Align.CENTER, Align.MIN)

# Thinnest wall any feature may have. JLC3DP's upload page asks for walls over
# 1.2 mm, but its quote still flagged "Thin walls detected" on lids whose
# thinnest wall measured 1.6 mm, so the margin is wider than the page says.
MIN_WALL = 2.0

# Self-tapping screw by PCB hole: an M3 board (3.2 mm hole) and an M2 board
# (2.2 mm hole), as (name, pilot hole, boss diameter). A pilot near the
# screw's minor diameter is the usual practice for thread-forming screws in
# plastic; each boss leaves MIN_WALL round its pilot.
SCREWS = {
    3.2: ("M3", 2.5, 2.5 + 2 * MIN_WALL),
    2.2: ("M2", 1.6, 1.6 + 2 * MIN_WALL),
}
CHASSIS_CLEARANCE = 3.4  # M3, ISO 273 medium: the floor tabs' holes
LID_PILOT, LID_CLEARANCE = 2.5, 3.4  # M3 self-tapping lid screws, M3 x 6
SKIN = MIN_WALL  # material left under a blind pilot hole (the floor, a boss end)
MIN_THREAD = 4.0  # thread engagement a lid screw needs in its tab
SCREW_HEAD = (6.0, 2.6)  # M3 pan head: diameter, height, kept clear of the lid tabs
# A lid tab: TAB_T thick (the screw's thread runs through it, horizontally),
# TAB_W along the wall, hanging TAB_DROP under the lid. The wall keeps
# MIN_WALL over its clearance hole, the tab MIN_WALL round its pilot.
TAB_T = MIN_THREAD
TAB_W = 8.0
TAB_DROP = MIN_WALL + LID_CLEARANCE / 2 + LID_PILOT / 2 + MIN_WALL
LIP_FIT = 0.3  # lip and tab to wall: slide fit


@dataclass(frozen=True)
class Params:
    wall: float = 2.0
    floor: float = 2.0
    gap: float = 0.5  # board edge to wall
    under: float = 1.5  # clearance below the deepest pin or back-side part
    min_standoff: float = 4.0  # also the screw's thread engagement
    lip: float = 3.0  # tray wall above the board's top face
    lid: float = 2.0
    lid_clearance: float = 2.0  # above the tallest part
    plug: float = 6.0  # a mated plug's height over its header, opening refs only
    opening_margin: float = 1.0  # each side of an opening's courtyard
    tab: float = 10.0  # chassis tab length beyond the wall
    tab_width: float = 12.0

    @classmethod
    def from_toml(cls, data: dict) -> Params:
        unknown = set(data) - set(cls.__dataclass_fields__)
        if unknown:
            raise EnclosureError(f"[case]: unknown key(s) {sorted(unknown)}")
        params = cls(**{k: float(v) for k, v in data.items()})
        for name in ("wall", "floor", "lid"):
            if getattr(params, name) < MIN_WALL:
                raise EnclosureError(
                    f"[case] {name} = {getattr(params, name)} is under {MIN_WALL} mm"
                )
        return params


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


def _screw(hole_drill: float) -> tuple[str, float, float]:
    try:
        return SCREWS[round(hole_drill, 1)]
    except KeyError:
        raise EnclosureError(
            f"no screw for a {hole_drill} mm hole; add it to SCREWS"
        ) from None


def _shell(f: Frame, height: float) -> Part:
    p = f.p
    outer = Box(f.length, f.width, height, align=A)
    inner = Pos(p.wall, p.wall, p.floor) * Box(
        f.length - 2 * p.wall, f.width - 2 * p.wall, height, align=A
    )
    return outer - inner


def _untangent(f: Frame, x: float, y: float, r: float, inset: float) -> float:
    """A boss radius that is not tangent to the faces `inset` in from the outline.

    A cylinder touching a plane along a line meshes into an invalid 3MF
    (robocar main lid: H2's pillar met the lip's inner face exactly). Within
    0.05 mm of tangency, grow the boss 0.1 mm so the two solids overlap.
    """
    for d in (x - inset, f.length - inset - x, y - inset, f.width - inset - y):
        if abs(d - r) < 0.05:
            return r + 0.1
    return r


def _standoffs(f: Frame, body: Part) -> Part:
    """A solid boss under each mounting hole with a blind pilot for the screw."""
    for h in f.piece.holes:
        _, pilot, boss = _screw(h.drill)
        x, y = f.xy(h.x, h.y)
        r = _untangent(f, x, y, boss / 2, f.p.wall)
        body += Pos(x, y, 0) * Cylinder(r, f.z_board, align=C)
        body -= Pos(x, y, SKIN) * Cylinder(pilot / 2, f.z_board, align=C)
    return body


def _tabs(f: Frame, body: Part) -> Part:
    """Two floor tabs, one off each short end, each with a chassis bolt hole."""
    if not f.piece.mount_tabs:
        return body
    p = f.p
    y0 = (f.width - p.tab_width) / 2
    for x0 in (-p.tab, f.length):
        body += Pos(x0, y0, 0) * Box(p.tab, p.tab_width, p.floor, align=A)
        cx = x0 + p.tab / 2
        body -= Pos(cx, f.width / 2, -1) * Cylinder(
            CHASSIS_CLEARANCE / 2, p.floor + 2, align=C
        )
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


def _openings(f: Frame, body: Part, wall_top: float, closed: bool) -> Part:
    """A window through the wall beside each opening part: above the board for a
    top-side part, below it for a back-side one.

    `closed` windows end 1 mm above the connector and its mated plug, keeping
    at least MIN_WALL of wall above them (the box); open ones run out through
    the wall top (the tray's low walls).
    """
    p, r = f.p, f.piece.region
    by_ref = {q.ref: q for q in f.piece.parts}
    for ref in f.piece.openings:
        part = by_ref[ref]
        c = part.courtyard
        side = _side(r, c)
        m = p.opening_margin
        if part.side == "top":
            z_from = f.z_top - 0.5
            z_to = wall_top + 1
            if closed:
                z_to = min(f.z_top + f.above(part) + 1.0, wall_top - MIN_WALL)
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


@dataclass(frozen=True)
class Tab:
    """One lid tab: on a board side, at board coordinate `u` along it, with its
    pilot's centre (x, y, z) in case coordinates and the wall's outward normal."""

    side: str
    u: float
    x: float
    y: float
    z: float
    normal: tuple[float, float]


def _side_pairs(f: Frame) -> list[tuple[str, str]]:
    """Opposite wall pairs for the tabs: the long sides first."""
    r = f.piece.region
    ns, we = ("north", "south"), ("west", "east")
    return [ns, we] if r.width >= r.height else [we, ns]


def _window_spans(f: Frame, side: str) -> list[tuple[float, float]]:
    """Board-coordinate spans along `side` taken by its cable windows."""
    r, m = f.piece.region, f.p.opening_margin
    by_ref = {q.ref: q for q in f.piece.parts}
    out = []
    for ref in f.piece.openings:
        c = by_ref[ref].courtyard
        if _side(r, c) == side:
            lo, hi = (c.x0, c.x1) if side in ("north", "south") else (c.y0, c.y1)
            out.append((lo - m, hi + m))
    return out


def _tab_rect(f: Frame, side: str, u: float) -> Rect:
    """A tab's plan footprint in board coordinates."""
    r, g = f.piece.region, f.p.gap
    a, b = u - TAB_W / 2, u + TAB_W / 2
    n0, n1 = LIP_FIT - g, LIP_FIT - g + TAB_T  # inward from the board edge
    return {
        "north": Rect(a, r.y0 + n0, b, r.y0 + n1),
        "south": Rect(a, r.y1 - n1, b, r.y1 - n0),
        "west": Rect(r.x0 + n0, a, r.x0 + n1, b),
        "east": Rect(r.x1 - n1, a, r.x1 - n0, b),
    }[side]


def _tab_need(f: Frame, side: str, u: float) -> float:
    """The lowest lid (wall top) at which a tab at `u` clears what is under it:
    a part's model, or a board screw's head."""
    reach = _tab_rect(f, side, u)
    floor = f.z_top + 0.5
    for q in f.piece.parts:
        if q.side == "top" and q.courtyard.overlaps(reach):
            floor = max(floor, f.z_top + f.above(q) + 0.5)
    k = SCREW_HEAD[0] / 2
    for h in f.piece.holes:
        if reach.overlaps(Rect(h.x - k, h.y - k, h.x + k, h.y + k)):
            floor = max(floor, f.z_top + SCREW_HEAD[1] + 0.5)
    return floor + TAB_DROP


def _pick_pair(cands: list[tuple[float, float]], lo: float, hi: float):
    """Two candidates (u, need) at the lowest lid that allows a pair at least
    two tab widths apart: those nearest the quarter points, else the two ends."""
    q1, q3 = lo + (hi - lo) / 4, lo + 3 * (hi - lo) / 4
    for level in sorted({n for _, n in cands}):
        ok = [u for u, n in cands if n <= level]
        a = min(ok, key=lambda u: abs(u - q1))
        b = min(ok, key=lambda u: abs(u - q3))
        if b - a < 2 * TAB_W:
            a, b = min(ok), max(ok)
        if b - a >= 2 * TAB_W:
            return level, (a, b)
    return None


def lid_tabs(f: Frame) -> tuple[float, list[Tab]]:
    """(wall top, the four lid tabs): two on each long side, placed clear of the
    cable windows and as low a lid as the parts under them allow; the short
    sides when the cable windows leave the long ones no room. The box grows
    taller only when no pair of tab positions clears its parts."""
    errors = []
    for pair in _side_pairs(f):
        try:
            return _tabs_on(f, pair)
        except EnclosureError as e:
            errors.append(str(e))
    raise EnclosureError("; ".join(errors))


def _tabs_on(f: Frame, pair: tuple[str, str]) -> tuple[float, list[Tab]]:
    p, r = f.p, f.piece.region
    wall_top = f.z_top + f.headroom + p.lid_clearance
    picks = {}
    for side in pair:
        lo_b, hi_b = (r.x0, r.x1) if side in ("north", "south") else (r.y0, r.y1)
        lo, hi = lo_b - p.gap + TAB_W / 2, hi_b + p.gap - TAB_W / 2
        windows = _window_spans(f, side)
        cands = []
        u = lo
        while u <= hi + 1e-9:
            a, b = u - TAB_W / 2, u + TAB_W / 2
            if not any(a < w1 and w0 < b for w0, w1 in windows):
                cands.append((u, max(wall_top, _tab_need(f, side, u))))
            u += 0.5
        pick = _pick_pair(cands, lo, hi) if cands else None
        if pick is None:
            raise EnclosureError(
                f"{f.piece.name}: no room for two lid tabs on the {side} wall "
                "clear of its cable windows"
            )
        picks[side] = pick
    wall_top = max([wall_top] + [lv for lv, _ in picks.values()])
    z = wall_top - MIN_WALL - LID_CLEARANCE / 2
    inset = p.wall + LIP_FIT + TAB_T / 2  # outer edge to the pilot's middle
    tabs = []
    for side, (_, us) in picks.items():
        for u in us:
            if side in ("north", "south"):
                x, _ = f.xy(u, 0)
                y = f.width - inset if side == "north" else inset
                normal = (0.0, 1.0) if side == "north" else (0.0, -1.0)
            else:
                _, y = f.xy(0, u)
                x = inset if side == "west" else f.length - inset
                normal = (-1.0, 0.0) if side == "west" else (1.0, 0.0)
            tabs.append(Tab(side, u, x, y, z, normal))
    return wall_top, tabs


def _along_normal(radius: float, length: float, normal) -> Part:
    """A cylinder centred on the origin with its axis along a wall's normal."""
    rot = Rot(90, 0, 0) if normal[0] == 0 else Rot(0, 90, 0)
    return rot * Cylinder(radius, length)


def tray(f: Frame) -> Part:
    height = f.z_top + f.p.lip
    body = _standoffs(f, _shell(f, height))
    body = _openings(f, body, height, closed=False)
    return _tabs(f, body)


def box(f: Frame) -> tuple[Part, Part]:
    """(base, lid). The lid sits on the walls, a lip locates it, and four
    screws through the long walls hold its tabs (`lid_tabs`)."""
    p = f.p
    wall_top, tabs = lid_tabs(f)
    base = _standoffs(f, _shell(f, wall_top))
    for tab in tabs:
        nx, ny = tab.normal
        d = TAB_T / 2 + LIP_FIT + p.wall / 2  # pilot's middle to the wall's middle
        base -= Pos(tab.x + nx * d, tab.y + ny * d, tab.z) * _along_normal(
            LID_CLEARANCE / 2, p.wall + 2, tab.normal
        )
    base = _openings(f, base, wall_top, closed=True)
    base = _tabs(f, base)

    lid = Pos(0, 0, wall_top) * Box(f.length, f.width, p.lid, align=A)
    t = LIP_FIT
    lip_h = 3.0
    inner = p.wall + t + MIN_WALL
    ring = Pos(p.wall + t, p.wall + t, wall_top - lip_h) * Box(
        f.length - 2 * (p.wall + t), f.width - 2 * (p.wall + t), lip_h, align=A
    )
    ring -= Pos(inner, inner, wall_top - lip_h - 1) * Box(
        f.length - 2 * inner, f.width - 2 * inner, lip_h + 2, align=A
    )
    lid += ring
    for tab in tabs:
        nx, ny = tab.normal
        w = (TAB_W, TAB_T) if nx == 0 else (TAB_T, TAB_W)
        lid += Pos(tab.x, tab.y, wall_top - TAB_DROP) * Box(
            w[0], w[1], TAB_DROP + 0.01, align=C
        )
        lid -= Pos(tab.x, tab.y, tab.z) * _along_normal(
            LID_PILOT / 2, TAB_T + 2, tab.normal
        )
    for ref in f.piece.lid_windows:
        part = next(q for q in f.piece.parts if q.ref == ref)
        x, y, w, h = f.rect(part.courtyard)
        m = p.opening_margin
        # Clip the window inside the locating lip and notch it round the lid
        # screws: a window cut into either leaves a fin no printer holds
        # (0.15 mm on head 1, found by the thickness check, not by JLC3DP).
        x0, y0 = max(x - m, inner), max(y - m, inner)
        x1 = min(x + w + m, f.length - inner)
        y1 = min(y + h + m, f.width - inner)
        if x1 > x0 and y1 > y0:
            cut = Pos(x0, y0, wall_top - lip_h - 1) * Box(
                x1 - x0, y1 - y0, lip_h + p.lid + 2, align=A
            )
            for tab in tabs:
                w = (TAB_W, TAB_T) if tab.normal[0] == 0 else (TAB_T, TAB_W)
                grow = 2 * (t + MIN_WALL)
                cut -= Pos(tab.x, tab.y, wall_top - lip_h - 2) * Box(
                    w[0] + grow, w[1] + grow, lip_h + p.lid + 4, align=C
                )
            lid -= cut
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


def blocked_screws(f: Frame, body: Part) -> list[str]:
    """The board screws a screwdriver cannot reach once the board is in.

    A pan head (SCREW_HEAD) has to go straight down its hole's axis from the
    case's top to the board, so a cylinder of the head's diameter from just
    above the board to past the top must miss the case. A lid boss above a
    board screw leaves room for the head but not for the screw to get there.
    """
    top = body.bounding_box().max.Z
    out = []
    for h in f.piece.holes:
        x, y = f.xy(h.x, h.y)
        reach = Pos(x, y, f.z_top + 0.1) * Cylinder(
            SCREW_HEAD[0] / 2, top - f.z_top + 1, align=C
        )
        hit = body & reach
        if hit and sum(s.volume for s in hit.solids()) > 0.01:
            out.append(h.ref)
    return out
