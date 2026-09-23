"""Obstacle-aware Manhattan (orthogonal) auto-router for schemdraw circuits.

Schemdraw's ``elm.Wire`` only offers fixed shapes (straight, single-bend
elbow) chosen by hand per wire — it has no idea where other components or
wires already are, so keeping a circuit free of diagonal lines and
component/wire overlap means manually re-tuning every wire's shape and
offset whenever anything moves (see the git history of ``circuits/*.py``
before this module existed).

:class:`Router` fixes that: point it at a live ``Drawing``, ask it to
``.wire(a, b)`` between two pin anchors, and it grid-searches an
orthogonal (4-direction) path from ``a`` to ``b`` that avoids every placed
component's bounding box and discourages crossing wires already routed in
the same drawing — similar in spirit to the auto-routed connectors in
https://github.com/niknah/quick-connections, adapted to schemdraw's static
SVG output instead of a live canvas.

Usage::

    router = Router(d)
    router.wire(esp.GPIO5, amp.BCLK, net="i2s")
    router.finish()  # draws every recorded wire, in the order routed

Routing and drawing are separate steps (#492): ``wire()`` routes a net and
records its polyline, ``finish()`` adds the Paths to the drawing. Hops,
nudging and net ordering are properties of the *finished* set of wires — you
cannot know which wire jumps which until every wire exists — so drawing each
net the moment it is routed would leave no point at which to decide them.

``finish()`` is where the finished set is marked up (#493): a hop arc where
one wire crosses another, a junction dot where wires join, and a colour per
net class from :data:`NET_COLORS`. Without the first two a crossing and a
connection are the same mark.
"""

from __future__ import annotations

import heapq
import math
from dataclasses import dataclass, field

import schemdraw.elements as elm
from schemdraw.segments import SegmentCircle, SegmentPath

Coord = tuple[float, float]
# (x, y) directions, in routing preference order (helps produce consistent,
# tie-broken paths instead of arbitrary ones when costs are equal).
_DIRECTIONS: tuple[Coord, ...] = ((1, 0), (0, 1), (-1, 0), (0, -1))

# Share of ``overlap_penalty`` charged per same-axis wire in the lattice row
# (or column) directly beside a step. Must stay well under 0.5 so that even a
# step squeezed between two wires costs less than running on top of one.
_NEIGHBOUR_FRACTION = 0.25


@dataclass
class _BBox:
    xmin: float
    ymin: float
    xmax: float
    ymax: float

    def inflated(self, margin: float) -> "_BBox":
        return _BBox(
            self.xmin - margin,
            self.ymin - margin,
            self.xmax + margin,
            self.ymax + margin,
        )

    def contains(self, x: float, y: float, epsilon: float = 1e-6) -> bool:
        return (
            self.xmin - epsilon <= x <= self.xmax + epsilon
            and self.ymin - epsilon <= y <= self.ymax + epsilon
        )


def _snap(value: float, grid: float) -> float:
    return round(round(value / grid) * grid, 6)


# -- net classes ---------------------------------------------------------------
#
# Colour is assigned by *class*, never by graph colouring (ADR-023): a
# graph-colouring scheme hands out arbitrary colours again whenever an unrelated
# net moves, so every render becomes a large diff and the byte-comparison gate
# in schematics-check.yml stops being a usable review signal. A class colour
# only changes when somebody changes what the net *is*.
#
# The hues are the Okabe-Ito set, chosen to stay distinguishable under the
# common colour-vision deficiencies; its yellow is left out because it all but
# disappears on a white page. Ground keeps the black every wire used to be
# drawn in, and power takes vermillion, the nearest to the conventional red.
# The dict's order is the order the README documents them in.
NET_COLORS: dict[str, str] = {
    "power": "#D55E00",  # vermillion — supply rails
    "ground": "#000000",  # black
    "i2c": "#0072B2",  # blue — SDA/SCL, incl. behind the mux
    "i2s": "#009E73",  # bluish green — BCLK/LRC/DIN
    "pwm": "#E69F00",  # orange — PWM duty outputs (PCA9685, LEDC tones)
    "sensor": "#CC79A7",  # reddish purple — sensor trigger/echo/interrupt
    "signal": "#56B4E9",  # sky blue — any other digital GPIO (enable, step)
    "load": "#666666",  # grey — driver output into a motor coil or speaker
}

# A wire routed without a class. Drawn plainly rather than in a class colour
# so it cannot pass for a classified net; the real circuits are tested to
# have none.
UNCLASSIFIED_COLOR = "black"


def net_color(net: str) -> str:
    """The colour of net class ``net``, for hand-drawn leads that carry one.

    Routed wires take theirs from ``Router.wire(..., net=...)``; a bus stub
    drawn with ``elm.Wire``/``elm.Arrow`` uses this so it cannot drift from
    the palette by carrying a literal of its own.
    """
    if net not in NET_COLORS:
        raise ValueError(
            f"unknown net class {net!r}; expected one of: {', '.join(NET_COLORS)}"
        )
    return NET_COLORS[net]


# -- hop and junction geometry -------------------------------------------------

# Hop radius, in drawing units. Legible against the router's 0.25 grid while
# staying under the gap to the next lattice row, so an arc never touches a
# parallel wire one row away. Two crossings closer than a diameter — adjacent
# lattice columns — merge into one bridge (see _path_commands).
HOP_RADIUS = 0.15
# Junction dot radius. schemdraw's own Dot default (0.075) reads as a blob of
# line cap at this drawing's unit=2 scale.
JUNCTION_RADIUS = 0.11
# Control-point distance, as a fraction of the radius, that makes a cubic
# Bezier approximate a quarter circle: 4/3 * (sqrt(2) - 1).
_KAPPA = 0.5522847


def _canonical_direction(ux: float, uy: float) -> Coord:
    """``(ux, uy)`` or its reverse, whichever has its angle in ``[0, pi)``.

    The hop bulges to the left of this direction, so which side it lands on
    depends only on the segment's line, not on which way the router happened
    to traverse it — a net routed right-to-left must not hop downward while
    its neighbour hops up. Done by sign rather than ``atan2``: exact, so the
    normal carries no float noise into the SVG.
    """
    if uy > _EPS or (abs(uy) <= _EPS and ux > 0):
        return (ux, uy)
    return (-ux, -uy)


def _path_commands(
    points: list[Coord], hops: list[Coord], radius: float
) -> list[Coord | str]:
    """SVG-style ``M``/``L``/``C`` commands for ``points`` with hops at ``hops``.

    Each hop is two quarter-circle cubic Beziers — never the SVG ``A`` arc
    command, which schemdraw's matplotlib backend degrades to a MOVETO,
    silently breaking the path (ADR-023). Crossings on one segment closer
    than a diameter merge into a single bridge: quarter circle up, straight
    across, quarter circle down, since two overlapping arcs would double
    back on themselves. A bridge whose crossing sits nearer the segment's
    end than ``radius`` shrinks to fit rather than overshoot the bend.
    """
    cmds: list[Coord | str] = ["M", points[0]]
    for (ax, ay), (bx, by) in zip(points, points[1:]):
        length = abs(bx - ax) + abs(by - ay)  # axis-aligned: L1 == length
        if length <= _EPS:
            continue
        ux, uy = (bx - ax) / length, (by - ay) / length
        dx, dy = _canonical_direction(ux, uy)
        nx, ny = -dy, dx
        # Distance along the traversal of every hop strictly inside this
        # segment. The cross-product test keeps hops on other segments out.
        ts = sorted(
            t
            for hx, hy in hops
            if abs((hx - ax) * uy - (hy - ay) * ux) <= _EPS
            and _EPS < (t := (hx - ax) * ux + (hy - ay) * uy) < length - _EPS
        )
        clusters: list[list[float]] = []
        for t in ts:
            if clusters and t - clusters[-1][-1] < 2 * radius:
                clusters[-1].append(t)
            else:
                clusters.append([t])

        def at(t: float, lift: float = 0.0) -> Coord:
            return (
                round(ax + ux * t + nx * lift, 6) + 0.0,
                round(ay + uy * t + ny * lift, 6) + 0.0,
            )

        for cluster in clusters:
            t1, t2 = cluster[0], cluster[-1]
            r = min(radius, t1, length - t2)
            k = _KAPPA * r
            cmds += ["L", at(t1 - r)]
            cmds += ["C", at(t1 - r, k), at(t1 - k, r), at(t1, r)]
            if t2 > t1:
                cmds += ["L", at(t2, r)]
            cmds += ["C", at(t2 + k, r), at(t2 + r, k), at(t2 + r)]
        cmds += ["L", (bx, by)]
    return cmds


def _axis_segments(wire: list[Coord]):
    """``(index, axis, fixed, lo, hi)`` for each non-degenerate segment."""
    for i, ((ax, ay), (bx, by)) in enumerate(zip(wire, wire[1:])):
        if abs(ay - by) <= _EPS and abs(ax - bx) > _EPS:
            yield i, "H", ay, min(ax, bx), max(ax, bx)
        elif abs(ax - bx) <= _EPS and abs(ay - by) > _EPS:
            yield i, "V", ax, min(ay, by), max(ay, by)


def _on_wire(point: Coord, wire: list[Coord]) -> bool:
    """True if ``point`` lies anywhere on ``wire``, ends included."""
    x, y = point
    for _, axis, fixed, lo, hi in _axis_segments(wire):
        along, across = (x, y) if axis == "H" else (y, x)
        if abs(across - fixed) <= _EPS and lo - _EPS <= along <= hi + _EPS:
            return True
    return len(wire) == 1 and _same(point, wire[0])


def _same(a: Coord, b: Coord) -> bool:
    return abs(a[0] - b[0]) <= _EPS and abs(a[1] - b[1]) <= _EPS


def _hop_sites(
    wires: list[list[Coord]], leads: list[list[Coord]] = ()
) -> list[list[Coord]]:
    """Where each of ``wires`` hops, per KiCad's ``ShouldHopOver`` rules.

    A hop is drawn on the *horizontal* segment of a crossing, so of any two
    crossing wires exactly one hops — never both, never neither. A crossing
    counts only strictly inside both segments: a wire ending on another (a
    T) or two wires sharing an end is a connection, not a crossing. And a
    point that a third wire also passes through or ends on is a junction,
    where a bridge would draw one wire leaping a connection, so it is
    skipped. Between two of ``wires`` this is the same predicate as
    ``metrics.crossings`` plus that last rule, which is what lets the tests
    equate the two counts.

    ``leads`` are hand-drawn wires (a bus trunk, a power stub) already in the
    drawing. They are crossed but never redrawn, so a routed wire takes the
    hop across one whichever axis it runs on — the one exception to the
    horizontal rule, and the only way that crossing gets a hop at all.
    Crossings between two leads are not this router's to mark.

    Returns one list per wire, in input order, each ordered by segment and
    then along the wire's own traversal, so output never depends on hashing.
    """
    everything = [*wires, *leads]
    segs = [list(_axis_segments(w)) for w in everything]
    sites: list[list[Coord]] = [[] for _ in wires]
    for i in range(len(wires)):
        for idx, axis, fixed, lo, hi in segs[i]:
            found = []
            for j, other in enumerate(segs):
                if j == i:
                    continue
                # A routed vertical yields to a routed horizontal, which
                # hops it on its own turn; only a lead makes it hop instead.
                if axis == "V" and j < len(wires):
                    continue
                for _, o_axis, o_fixed, o_lo, o_hi in other:
                    if o_axis == axis:
                        continue
                    if not (
                        lo + _EPS < o_fixed < hi - _EPS
                        and o_lo + _EPS < fixed < o_hi - _EPS
                    ):
                        continue
                    point = (o_fixed, fixed) if axis == "H" else (fixed, o_fixed)
                    if sum(_on_wire(point, w) for w in everything) > 2:
                        continue
                    if not any(_same(point, p) for p in found):
                        found.append(point)
            # Order along this segment's traversal direction.
            k = 0 if axis == "H" else 1
            forward = wires[i][idx + 1][k] >= wires[i][idx][k]
            found.sort(key=lambda p: p[k] if forward else -p[k])
            sites[i] += found
    return sites


def _junction_points(
    wires: list[list[Coord]], closed: list[tuple[bool, bool]] = ()
) -> list[Coord]:
    """Every point where three or more wire branches meet, sorted by (x, y).

    A wire end contributes one branch; a wire passing through the point —
    along a segment or round one of its bends — contributes two. So three
    ends meeting is a junction, and so is one end landing on another wire's
    interior (a T); two ends meeting is only a bend, and a plain crossing
    has no end at the point at all, so it is never a candidate.

    ``closed``, parallel to ``wires``, flags a (first, last) end that is
    already terminated — on a power or ground tag — and so is no branch.
    """
    closed = list(closed) + [(False, False)] * (len(wires) - len(closed))
    ends: list[Coord] = []
    for w, shut in zip(wires, closed):
        if not w:
            continue
        for p, is_closed in zip((w[0], w[-1]), shut):
            if not is_closed and not any(_same(p, q) for q in ends):
                ends.append(p)
    found = []
    for p in ends:
        branches = 0
        for w, shut in zip(wires, closed):
            if not w:
                continue
            at_ends = sum(
                _same(p, q) and not is_closed
                for q, is_closed in zip((w[0], w[-1]), shut)
            )
            if not at_ends and any(_same(p, q) for q in (w[0], w[-1])):
                continue  # this wire only reaches p at a closed end
            if at_ends:
                branches += at_ends
            elif _on_wire(p, w):
                branches += 2
        if branches >= 3:
            found.append((round(p[0], 6) + 0.0, round(p[1], 6) + 0.0))
    return sorted(found)


class Path(elm.Element):
    """A single wire drawn through an arbitrary list of absolute points.

    ``polyline`` keeps the absolute points as routed; that, not the drawn
    segment, is the wire's geometry for anyone measuring it, because the
    drawn path also carries hop arcs and their Bezier control points.
    ``hops`` are absolute points on the polyline to bridge (see
    :func:`_path_commands`); :meth:`set_hops` redraws the wire with new
    ones in place, keeping the element — and its paint position — as is.
    """

    def __init__(
        self,
        points: list[Coord],
        *,
        hops: list[Coord] = (),
        radius: float = HOP_RADIUS,
        dot: bool = False,
        **kwargs,
    ):
        super().__init__(**kwargs)
        self.polyline = [(float(x), float(y)) for x, y in points]
        self.radius = radius
        self.hops: list[Coord] = []
        ox, oy = points[0]
        self.segments.append(SegmentPath([]))  # filled in by set_hops()
        if dot:
            rel_end = (points[-1][0] - ox, points[-1][1] - oy)
            self.segments.append(SegmentCircle(rel_end, JUNCTION_RADIUS, fill=True))
        self.set_hops(hops)
        self._userparams["at"] = (ox, oy)
        self.params["theta"] = 0

    def set_hops(self, hops: list[Coord]) -> None:
        """Redraw the wire's path with a hop at each of ``hops``."""
        self.hops = list(hops)
        ox, oy = self.polyline[0]
        relative = [(x - ox, y - oy) for x, y in self.polyline]
        rel_hops = [(x - ox, y - oy) for x, y in self.hops]
        self.segments[0] = SegmentPath(_path_commands(relative, rel_hops, self.radius))


@dataclass
class RoutedWire:
    """One net routed by :meth:`Router.wire`, drawn by :meth:`Router.finish`.

    ``points`` is the absolute polyline, fixed at routing time. ``element``
    is ``None`` until ``finish()`` adds the :class:`Path` for it, after
    which it is that Path — the handle ``wire()`` used to return directly.
    ``name`` and ``dot`` are applied to it then, since there is nothing to
    apply them to before. ``net`` is its class in :data:`NET_COLORS`, or
    ``None`` if it was routed without one. ``hops`` is filled in by
    ``finish()`` with the crossings this wire bridges.
    """

    points: list[Coord]
    net: str | None = None
    name: str | None = None
    dot: bool = False
    hops: list[Coord] = field(default_factory=list)
    element: Path | None = field(default=None, repr=False)

    @property
    def color(self) -> str:
        return NET_COLORS[self.net] if self.net else UNCLASSIFIED_COLOR


class Router:
    """Routes orthogonal, obstacle-avoiding wires between points in ``d``.

    Args:
        d: The schemdraw ``Drawing`` to read placed-component obstacles
            from and add routed wires to.
        grid: Search grid resolution, in drawing units. Finer grids hug
            obstacles more closely but search more cells.
        clearance: Extra margin added around each component's bounding box
            so wires don't run flush against chip outlines.
        stub: Length of the straight lead segment routed straight out from
            each pin before the orthogonal search takes over — keeps wires
            leaving a pin perpendicular to the chip edge, matching normal
            schematic convention.
        turn_penalty: Extra cost charged per direction change, in units of
            grid steps. Higher values produce straighter, fewer-bend paths.
        overlap_penalty: Extra cost charged per grid step that reuses a
            cell already occupied, along the same axis, by a previously
            routed wire in this router; a quarter of it is charged for each
            such wire in the lattice row/column directly beside the step.
            A hand-drawn lead already in the drawing charges the full
            penalty on its own cells too, but no neighbour share.
            Discourages (but doesn't forbid) wires overlapping or running
            one grid step apart.
    """

    def __init__(
        self,
        d,
        *,
        grid: float = 0.25,
        clearance: float = 0.3,
        stub: float = 0.75,
        turn_penalty: float = 4.0,
        overlap_penalty: float = 6.0,
    ) -> None:
        self.d = d
        self.grid = grid
        self.clearance = clearance
        self.stub = stub
        self.turn_penalty = turn_penalty
        self.overlap_penalty = overlap_penalty
        self._occupied: dict[tuple[int, int], set[str]] = {}
        # Every net wire() has routed, in routing order; finish() draws the
        # ones whose ``element`` is still None, in this same order.
        self._wires: list[RoutedWire] = []
        # Junction points finish() has dotted, in the order it dotted them.
        self.junctions: list[Coord] = []
        # Registered on the drawing so a harness holding only ``d`` (render.py
        # and metrics.py receive a finished Drawing, never the Router) can
        # tell a circuit that forgot finish() from one with no nets — the
        # former would otherwise render with every wire silently missing.
        # A list, not a set: order must never depend on hashing.
        routers = getattr(d, "_routers", None)
        if routers is None:
            routers = []
            d._routers = routers
        routers.append(self)

    # Elements that are never treated as routing obstacles: Wire and Line
    # (Arrow is a Line subclass) are simple leads, not component bodies — a
    # thin lead crossing another wire at a single point is normal schematic
    # notation (no junction dot = no connection), same as any two wires
    # crossing. Vdd/Ground are small single-terminal annotation symbols for
    # the same reason: like a real schematic tool, wires are free to pass
    # near a power/ground tag. Without these exclusions, e.g. a chip's own
    # GND tag — positioned right next to one of that chip's other pins — can
    # trap the router into thinking a neighboring pin is unreachable. A Dot
    # is a connection mark sitting on wires, not a body, for the same reason.
    _NON_OBSTACLES = (Path, elm.Wire, elm.Line, elm.Vdd, elm.Ground, elm.Dot)

    # -- obstacle model -----------------------------------------------
    def _component_boxes(self) -> list[_BBox]:
        boxes = []
        for el in self.d.elements:
            if isinstance(el, self._NON_OBSTACLES):
                continue
            xmin, ymin, xmax, ymax = el.get_bbox(transform=True, includetext=False)
            if math.isinf(xmin):
                continue
            boxes.append(_BBox(xmin, ymin, xmax, ymax))
        return boxes

    def _owning_box(self, point: Coord, boxes: list[_BBox]) -> _BBox | None:
        x, y = point
        best = None
        best_area = math.inf
        for box in boxes:
            if box.contains(x, y):
                area = (box.xmax - box.xmin) * (box.ymax - box.ymin)
                if area < best_area:
                    best, best_area = box, area
        return best

    def _exit_direction(self, point: Coord, box: _BBox) -> Coord:
        """Pick the outward direction from the nearest edge of ``box``."""
        x, y = point
        distances = {
            (-1.0, 0.0): abs(x - box.xmin),
            (1.0, 0.0): abs(x - box.xmax),
            (0.0, -1.0): abs(y - box.ymin),
            (0.0, 1.0): abs(y - box.ymax),
        }
        return min(distances, key=distances.get)

    def _stub_point(self, point: Coord, direction: Coord) -> Coord:
        """A short lead straight out from ``point``, clear of its own chip body.

        This is deliberately a single fixed-length hop, not a search: its
        only job is to clear the pin's own component footprint (already
        excluded from the caller's obstacle list) so the A* search below has
        room to maneuver. It must never grow to dodge a *different*
        obstacle — that would tunnel the "stub" straight through it instead
        of letting A* route around it.
        """
        dx, dy = direction
        if dx:
            return (_snap(point[0] + dx * self.stub, self.grid), point[1])
        return (point[0], _snap(point[1] + dy * self.stub, self.grid))

    # -- search ----------------------------------------------------------
    def _astar(
        self, start: Coord, goal: Coord, obstacles: list[_BBox]
    ) -> list[Coord] | None:
        # One lattice for every net: index (ix, iy) is the absolute point
        # (ix * grid, iy * grid), the same cells _mark_occupied keys by.
        # This used to be anchored at each net's own ``start`` (#491), so a
        # net leaving a pin at y = 6.125 searched rows 6.125 + k * grid while
        # occupancy was recorded on the absolute rows — the overlap penalty
        # then charged whichever absolute cell a row happened to *round* to,
        # which is not the cell another wire actually runs in. ``start`` and
        # ``goal`` are generally off-lattice along one axis (a stub point is
        # snapped only along its lead), so the search runs between their
        # nearest lattice points and the ends are pulled back onto the exact
        # points afterwards (_snap_initial_approach / _snap_final_approach).
        grid = self.grid
        sx, sy = round(start[0] / grid), round(start[1] / grid)
        gx, gy = round(goal[0] / grid), round(goal[1] / grid)

        # Bound the search to a margin around start/goal/obstacles. Without
        # this, a goal that's unreachable (e.g. a stub point that landed
        # inside another obstacle — a malformed layout) sends the search
        # sweeping outward across open free space forever, since nothing
        # else stops it from expanding: pathological input would hang
        # instead of failing fast with the RuntimeError below.
        margin = 4.0
        xs = (
            [start[0], goal[0]]
            + [b.xmin for b in obstacles]
            + [b.xmax for b in obstacles]
        )
        ys = (
            [start[1], goal[1]]
            + [b.ymin for b in obstacles]
            + [b.ymax for b in obstacles]
        )
        ix_lo = math.floor((min(xs) - margin) / grid)
        ix_hi = math.ceil((max(xs) + margin) / grid)
        iy_lo = math.floor((min(ys) - margin) / grid)
        iy_hi = math.ceil((max(ys) + margin) / grid)

        def blocked(ix: int, iy: int) -> bool:
            if not (ix_lo <= ix <= ix_hi and iy_lo <= iy <= iy_hi):
                return True
            x, y = ix * grid, iy * grid
            return any(b.contains(x, y) for b in obstacles)

        leads = self._lead_occupancy()

        def occ_penalty(ix: int, iy: int, axis: str) -> float:
            # Only penalize running along the *same* axis as a previously
            # routed wire or a hand-drawn lead. A perpendicular crossing is
            # normal, unambiguous schematic notation and costs nothing extra.
            key = (ix, iy)
            if axis in self._occupied.get(key, ()) or axis in leads.get(key, ()):
                return self.overlap_penalty
            # A wire one lattice row (or column) beside another is the
            # "near-parallel verticals cannot be told apart" defect of
            # ADR-023, and an exact-cell charge alone never sees it: with
            # the lattice shared (#491), 6, 60 and 1000 all routed the same
            # geometry, because the default already removed every on-top
            # run and adjacency was free. Charge each occupied neighbour a
            # fraction of the full penalty, so beside stays strictly cheaper
            # than on top — at an equal charge (fraction 1.0) the search
            # traded adjacency for collinear overlap, the one outcome worse
            # than a tight pair, and at 1/3 it still did once in balancebot.
            # Routed wires only: see _lead_occupancy for why a lead is not
            # charged as a neighbour.
            nx, ny = (0, 1) if axis == "H" else (1, 0)
            neighbours = sum(
                axis in self._occupied.get((ix + side * nx, iy + side * ny), ())
                for side in (-1, 1)
            )
            return self.overlap_penalty * _NEIGHBOUR_FRACTION * neighbours

        start_node = (sx, sy, None)  # (ix, iy, incoming direction)
        frontier: list[tuple[float, int, tuple]] = [(0.0, 0, start_node)]
        came_from: dict[tuple, tuple | None] = {start_node: None}
        cost_so_far: dict[tuple, float] = {start_node: 0.0}
        counter = 1

        while frontier:
            _, _, current = heapq.heappop(frontier)
            cix, ciy, cdir = current
            if (cix, ciy) == (gx, gy):
                path = self._reconstruct(came_from, current, grid)
                return _snap_initial_approach(path, start)

            for dx, dy in _DIRECTIONS:
                nix, niy = cix + dx, ciy + dy
                if blocked(nix, niy):
                    continue
                axis = "H" if dy == 0 else "V"
                step_cost = 1.0 + occ_penalty(nix, niy, axis)
                if cdir is not None and (dx, dy) != cdir:
                    step_cost += self.turn_penalty
                new_cost = cost_so_far[current] + step_cost
                nstate = (nix, niy, (dx, dy))
                if nstate not in cost_so_far or new_cost < cost_so_far[nstate]:
                    cost_so_far[nstate] = new_cost
                    priority = new_cost + (abs(gx - nix) + abs(gy - niy))
                    heapq.heappush(frontier, (priority, counter, nstate))
                    counter += 1
                    came_from[nstate] = current
        return None

    def _reconstruct(self, came_from, end_state, grid) -> list[Coord]:
        points: list[Coord] = []
        state = end_state
        while state is not None:
            ix, iy, _ = state
            points.append((round(ix * grid, 6), round(iy * grid, 6)))
            state = came_from[state]
        points.reverse()
        return _simplify(points)

    def _mark_occupied(self, points: list[Coord]) -> None:
        """Record every grid cell each segment of ``points`` passes through,
        tagged with that segment's axis, for future overlap-penalty checks."""
        _mark_cells(self._occupied, points, self.grid)

    def _lead_occupancy(self) -> dict[tuple[int, int], set[str]]:
        """Occupancy of the hand-drawn leads in ``d`` right now.

        A lead is a drawn wire like any routed one, so a net drawn on top of
        it is just as illegible. balancebot's GPIO4 -> DIR net did exactly
        that: its pin sits 0.07 below the nENABLE lead's row, the search ran
        on the lattice row nearest the pin — the lead's own — and the snap
        back onto the pin row left the net 0.07 under the lead for its whole
        length, with its hop over the trunk hidden under the trunk's junction
        dot. So a lead's cells charge the full overlap penalty, and the net
        now leaves that row. Leads are not *obstacles* — a crossing is fine
        (see _NON_OBSTACLES) — only occupied.

        Only the exact cell is charged, not the neighbour fraction routed
        wires pay. Power stubs sit one pin pitch from the signal pins on the
        same chip edge, so a neighbour charge taxes the unavoidable approach
        to the next pin rather than a real overlap. Measured: with it,
        balancebot's GPIO3 net was diverted by the 1.5-per-cell charge beside
        the left DRV8825's VDD stub, and GPIO0 then routed straight through
        that driver's GND tag.

        Recomputed per ``wire()`` rather than cached, because a circuit may
        add leads between nets; kept apart from ``_occupied`` so a lead never
        reads as a routed wire to anything else that consults it. Only leads
        already drawn are seen — circuits that draw their power stubs after
        routing are unaffected, as with components.
        """
        occupied: dict[tuple[int, int], set[str]] = {}
        for points, _ in self._leads():
            _mark_cells(occupied, points, self.grid)
        return occupied

    # -- public API --------------------------------------------------
    def wire(
        self,
        start: Coord,
        end: Coord,
        *,
        net: str | None = None,
        name: str | None = None,
        dot: bool = False,
    ):
        """Route an orthogonal, obstacle-avoiding wire from ``start`` to ``end``.

        ``net`` is the wire's class — a key of :data:`NET_COLORS` such as
        ``"i2c"`` or ``"pwm"`` — which alone decides its colour. There is no
        colour argument on purpose: a literal would let a net keep an old
        colour and silently opt out of the scheme (#493).

        The wire is recorded, not drawn: it appears in the drawing when
        :meth:`finish` is called. Returns its :class:`RoutedWire` handle,
        whose ``element`` is the drawn Path once ``finish()`` has run.

        ``start``/``end`` are normally pin anchors (e.g. ``esp.GPIO5``). A
        fixed stub lead carries the wire straight out of its own chip body;
        past that stub the chip is an obstacle like any other, so a net
        never cuts across the component it terminates on.
        """
        if net is not None:
            net_color(net)  # validates before any routing work
        start = (float(start[0]), float(start[1]))
        end = (float(end[0]), float(end[1]))
        boxes = self._component_boxes()
        inflated = [b.inflated(self.clearance) for b in boxes]

        start_owner = self._owning_box(start, boxes)
        end_owner = self._owning_box(end, boxes)
        start_dir = self._exit_direction(start, start_owner) if start_owner else None
        end_dir = self._exit_direction(end, end_owner) if end_owner else None
        entry = self._stub_point(start, start_dir) if start_dir else start
        exit_ = self._stub_point(end, end_dir) if end_dir else end

        # The A* search runs from ``entry`` to ``exit_``, so a box is only in
        # its way if its inflated footprint swallows one of those stub
        # points. Drop exactly those, and only when the same box also holds
        # the pin the stub hangs off: two components can legitimately
        # overlap a little (e.g. densely packed pin breakouts), and a second
        # box covering both a pin and its stub would otherwise make that end
        # unreachable outright.
        #
        # Everything else stays an obstacle for the whole search — above
        # all the pin's *own* chip, which the stub already clears. This
        # used to drop every box containing either raw endpoint instead,
        # which let a net enter its destination chip on one side and leave
        # on the other (#490). A foreign box that swallows a stub without
        # holding its pin is a malformed layout, and stays in so the search
        # fails fast rather than tunnelling through it.
        ends = ((start, entry), (end, exit_))
        obstacles = [
            b
            for raw, b in zip(boxes, inflated)
            if not any(raw.contains(*pin) and b.contains(*stub) for pin, stub in ends)
        ]

        path = self._astar(entry, exit_, obstacles)
        if path is None:
            raise RuntimeError(
                f"Router: no orthogonal path found from {start} to {end} "
                "(try a larger grid, more clearance, or check for a fully "
                "enclosed pin)"
            )
        path = _snap_final_approach(path, exit_)

        points = _simplify(
            [
                start,
                *([entry] if entry != start else []),
                *path,
                *([exit_] if exit_ != end else []),
                end,
            ]
        )
        # Occupancy is marked now, not at finish(): the next wire() must see
        # this one to be penalised for running on or beside it, exactly as
        # when routing and drawing were one step. Deferring the draw itself
        # cannot change routing, because a drawn Path is in _NON_OBSTACLES.
        self._mark_occupied(points)

        routed = RoutedWire(points, net=net, name=name, dot=dot)
        self._wires.append(routed)
        return routed

    @property
    def undrawn(self) -> list[RoutedWire]:
        """Recorded wires that ``finish()`` has not drawn yet, in routing order."""
        return [w for w in self._wires if w.element is None]

    def finish(self) -> list[Path]:
        """Draw every recorded wire not yet drawn, in the order it was routed.

        Call it once after a circuit's last ``wire()``. Where it is called
        decides where the Paths sit in the drawing's element list, and so in
        the SVG's paint order: circuits call it immediately after their
        wire block, where each Path used to be added as it was routed.
        Calling it again draws only wires recorded since, so it is safe to
        call more than once. Returns the Paths it added.

        This is where the finished set is marked up: each wire is drawn with
        its hops (:func:`_hop_sites`) and in its class colour, then a
        :class:`schemdraw.elements.Dot` goes on every junction
        (:func:`_junction_points`) not dotted before. Hops and junctions are
        computed over *every* recorded wire and lead, every call: a wire an
        earlier ``finish()`` drew that is now crossed by a later wire or lead
        has its Path redrawn in place (:meth:`Path.set_hops`), so where in
        the element list it sits — and so the SVG paint order — never moves.
        """
        polylines = [w.points for w in self._wires]
        leads = self._leads()
        sites = _hop_sites(polylines, [pts for pts, _ in leads])
        drawn = []
        for w, hops in zip(self._wires, sites):
            if w.element is not None:
                if hops != w.hops:
                    w.hops = hops
                    w.element.set_hops(hops)
                continue
            w.hops = hops
            el = self.d.add(Path(w.points, hops=hops, dot=w.dot, color=w.color))
            if w.name:
                el.name = w.name
            w.element = el
            drawn.append(el)

        # Every wire in the drawing, routed first, each with its colour.
        coloured = [(w.points, w.color) for w in self._wires] + leads
        # A lead that ends on a power or ground tag ends *on the tag*: the
        # tag is its connection, so that end is not free. Tags are not
        # routing obstacles, so a routed wire can pass through that point,
        # and counting the lead's end there would dot a connection to the
        # rail that does not exist. Only that end is discounted, not the
        # whole point: a genuine T of two other wires there keeps its dot.
        tags = [
            el.absanchors["start"]
            for el in self.d.elements
            if isinstance(el, (elm.Vdd, elm.Ground))
        ]
        closed = [(False, False)] * len(self._wires) + [
            tuple(any(_same(end, t) for t in tags) for end in (pts[0], pts[-1]))
            for pts, _ in leads
        ]
        for point in _junction_points([pts for pts, _ in coloured], closed):
            if any(_same(point, p) for p in self.junctions):
                continue
            # A dot takes the colour of the net it joins; where wires of
            # different classes meet, the first routed wins — deterministic.
            # A mixed-class junction is a drawing error (an uncoloured lead,
            # a mis-classed net), which test_routing.py fails on for every
            # real circuit rather than leaving to a reviewer's eye.
            color = next(c for pts, c in coloured if _on_wire(point, pts))
            self.d.add(elm.Dot(radius=JUNCTION_RADIUS).at(point).color(color))
            self.junctions.append(point)
        return drawn

    def _leads(self) -> list[tuple[list[Coord], str]]:
        """Absolute polyline and colour of every hand-drawn lead in ``d``.

        ``elm.Wire`` and ``elm.Line`` (so ``elm.Arrow``) elements a circuit
        drew itself — bus trunks, power stubs — in drawing order. They are
        rendered wires like any other, so finish() hops them and dots their
        junctions; only leads already added when it runs can be seen, which
        is why ``test_routing.py`` re-checks crossings over the final drawing.
        """
        leads = []
        for el in self.d.elements:
            if isinstance(el, Path) or not isinstance(el, (elm.Wire, elm.Line)):
                continue
            pts = [el.transform.transform(p) for p in el.segments[0].path]
            color = el._userparams.get("color") or UNCLASSIFIED_COLOR
            # Simplified because schemdraw puts a collinear midpoint in a
            # plain Line's path; a crossing landing exactly on that vertex
            # would otherwise sit at a segment end and never count.
            leads.append((_simplify([(float(x), float(y)) for x, y in pts]), color))
        return leads


def assert_finished(d) -> None:
    """Raise if any :class:`Router` on ``d`` still holds undrawn wires.

    A circuit module that routes its nets and forgets ``Router.finish()``
    produces a drawing that is valid, renders, and has no wires in it. The
    render and metrics harnesses call this on every drawing ``draw()``
    returns, so that mistake fails loudly instead. They check rather than
    call ``finish()`` themselves: drawing late would put the Paths after
    every element the circuit added past its wire block, reordering the SVG.
    """
    pending = [w for r in getattr(d, "_routers", ()) for w in r.undrawn]
    if pending:
        first = pending[0].points
        raise RuntimeError(
            f"{len(pending)} routed wire(s) were never drawn (first: "
            f"{first[0]} -> {first[-1]}); call router.finish() after the "
            "last router.wire()"
        )


def _mark_cells(
    occupied: dict[tuple[int, int], set[str]], points: list[Coord], grid: float
) -> None:
    """Add every lattice cell each segment of ``points`` passes through to
    ``occupied``, tagged with that segment's axis.

    The axis is decided with a tolerance and diagonal segments are skipped
    (:func:`_axis_segments`): a routed polyline is exact, but a lead's points
    come through a schemdraw transform, and ``1.0`` against ``1.0000000001``
    must not mark a horizontal lead as a vertical run. A diagonal (an
    annotation arrow) has no axis to overlap along.
    """
    for _, axis, fixed, lo, hi in _axis_segments(points):
        steps = round((hi - lo) / grid)
        for i in range(steps + 1):
            along = lo + (hi - lo) * i / steps if steps else lo
            x, y = (along, fixed) if axis == "H" else (fixed, along)
            occupied.setdefault((round(x / grid), round(y / grid)), set()).add(axis)


def _snap_initial_approach(path: list[Coord], start: Coord) -> list[Coord]:
    """Move the first segment so it leaves exactly from ``start``.

    The A* search runs on the global lattice, so its path begins at the
    lattice point nearest ``start`` — up to half a grid step off along
    whichever axis the stub was not snapped on. Rather than draw that sliver
    as a jog out of the pin's stub, shift the whole first run onto
    ``start``'s row (a horizontal first move) or column (a vertical one):
    the wire then leaves the stub in a straight line exactly as it did when
    the lattice was anchored at ``start``. The run keeps the occupancy cell
    the search charged for it, since a shift of under half a step rounds
    back to the same key in _mark_occupied.

    ``path[1]`` is only ever moved along the axis perpendicular to the first
    move, which is the axis the *next* segment runs along, so that segment
    stays axis-aligned. When the path is a single run, ``path[1]`` is the
    lattice goal and _snap_final_approach overwrites it anyway.
    """
    path = list(path)
    if len(path) < 2:
        return [start]
    ay, (bx, by) = path[0][1], path[1]
    if ay == by:  # horizontal first move -> carry it on start's row
        path[1] = (bx, start[1])
    else:  # vertical first move -> carry it on start's column
        path[1] = (start[0], by)
    path[0] = start
    return path


def _snap_final_approach(path: list[Coord], goal: Coord) -> list[Coord]:
    """Straighten the last segment so it lands exactly on ``goal``.

    The A* search runs on the global lattice, so an independently-computed
    ``goal`` is generally not on it — the search lands within half a grid
    cell of it. Left alone, appending the exact ``goal`` afterward draws a
    tiny, pointless extra jog. ``path[0]`` has already been moved onto the
    exact entry point by _snap_initial_approach, and must not move again.

    When the raw path has an interior bend (length >= 3), nudge the
    second-to-last point onto the same row/column as ``goal``. When it's a
    single straight run (length == 2), ``path[-2]`` *is* ``path[0]`` — it
    can't be nudged without moving the entry point, so an elbow is inserted
    instead, anchored on the entry's own axis.
    """
    path = list(path)
    if len(path) < 2:
        return [goal] if not path else [path[0], goal]

    lx, ly = path[-1]
    if len(path) >= 3:
        px, py = path[-2]
        if px == lx:  # final move was vertical -> align x to goal's x
            path[-2] = (goal[0], py)
        else:  # final move was horizontal -> align y to goal's y
            path[-2] = (px, goal[1])
    else:
        ex, ey = path[0]
        if ex == lx:  # single vertical run -> keep entry's x, then jog into goal
            path.insert(1, (ex, goal[1]))
        else:  # single horizontal run -> keep entry's y, then jog into goal
            path.insert(1, (goal[0], ey))
    path[-1] = goal
    return path


_EPS = 1e-6


def _simplify(points: list[Coord]) -> list[Coord]:
    """Drop consecutive duplicates and interior points collinear with their neighbors.

    Coordinates arriving here were computed along independent paths (grid
    search vs. exact pin/stub math), so they can differ by float noise far
    below drawing resolution. Comparisons use an epsilon rather than exact
    equality so that noise doesn't defeat collinearity merging and leave a
    sub-pixel visual kink.
    """
    if not points:
        return points
    out = [points[0]]
    for p in points[1:]:
        if abs(p[0] - out[-1][0]) > _EPS or abs(p[1] - out[-1][1]) > _EPS:
            out.append(p)
    if len(out) < 3:
        return out

    simplified = [out[0]]
    for i in range(1, len(out) - 1):
        ax, ay = simplified[-1]
        bx, by = out[i]
        cx, cy = out[i + 1]
        # Collinear (both segments axis-aligned in the same direction)?
        d1 = (bx - ax, by - ay)
        d2 = (cx - bx, cy - by)
        if abs(d1[0]) < _EPS and abs(d1[1]) < _EPS:
            continue
        same_dir = (
            abs(d1[0]) < _EPS and abs(d2[0]) < _EPS and (d1[1] > 0) == (d2[1] > 0)
        ) or (abs(d1[1]) < _EPS and abs(d2[1]) < _EPS and (d1[0] > 0) == (d2[0] > 0))
        if not same_dir:
            simplified.append(out[i])
    simplified.append(out[-1])
    # Drop any zero-length trailing segment.
    if len(simplified) >= 2 and (
        abs(simplified[-1][0] - simplified[-2][0]) < _EPS
        and abs(simplified[-1][1] - simplified[-2][1]) < _EPS
    ):
        simplified.pop()
    return simplified
