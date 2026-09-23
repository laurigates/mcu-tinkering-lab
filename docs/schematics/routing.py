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
    router.wire(esp.GPIO5, amp.BCLK, color='steelblue')
"""

from __future__ import annotations

import heapq
import math
from dataclasses import dataclass

import schemdraw.elements as elm
from schemdraw.segments import Segment

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


class Path(elm.Element):
    """A single wire drawn through an arbitrary list of absolute points."""

    def __init__(self, points: list[Coord], **kwargs):
        super().__init__(**kwargs)
        ox, oy = points[0]
        relative = [(x - ox, y - oy) for x, y in points]
        self.segments.append(Segment(relative))
        self._userparams["at"] = (ox, oy)
        self.params["theta"] = 0


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

    # Elements that are never treated as routing obstacles: Wire and Line
    # (Arrow is a Line subclass) are simple leads, not component bodies — a
    # thin lead crossing another wire at a single point is normal schematic
    # notation (no junction dot = no connection), same as any two wires
    # crossing. Vdd/Ground are small single-terminal annotation symbols for
    # the same reason: like a real schematic tool, wires are free to pass
    # near a power/ground tag. Without these exclusions, e.g. a chip's own
    # GND tag — positioned right next to one of that chip's other pins — can
    # trap the router into thinking a neighboring pin is unreachable.
    _NON_OBSTACLES = (Path, elm.Wire, elm.Line, elm.Vdd, elm.Ground)

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

        def occ_penalty(ix: int, iy: int, axis: str) -> float:
            # Only penalize running along the *same* axis as a previously
            # routed wire. A perpendicular crossing is normal, unambiguous
            # schematic notation and costs nothing extra.
            occupied = self._occupied
            if axis in occupied.get((ix, iy), ()):
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
            nx, ny = (0, 1) if axis == "H" else (1, 0)
            neighbours = sum(
                axis in occupied.get((ix + side * nx, iy + side * ny), ())
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
        grid = self.grid
        for (ax, ay), (bx, by) in zip(points, points[1:]):
            axis = "H" if ay == by else "V"
            steps = max(round(abs(bx - ax) / grid), round(abs(by - ay) / grid))
            for i in range(steps + 1):
                x = ax + (bx - ax) * i / steps if steps else ax
                y = ay + (by - ay) * i / steps if steps else ay
                key = (round(x / grid), round(y / grid))
                self._occupied.setdefault(key, set()).add(axis)

    # -- public API --------------------------------------------------
    def wire(
        self,
        start: Coord,
        end: Coord,
        *,
        color: str | None = None,
        name: str | None = None,
        dot: bool = False,
    ):
        """Route and draw an orthogonal, obstacle-avoiding wire from ``start`` to ``end``.

        ``start``/``end`` are normally pin anchors (e.g. ``esp.GPIO5``). A
        fixed stub lead carries the wire straight out of its own chip body;
        past that stub the chip is an obstacle like any other, so a net
        never cuts across the component it terminates on.
        """
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
        self._mark_occupied(points)

        el = self.d.add(Path(points, **({"color": color} if color else {})))
        if name:
            el.name = name
        if dot:
            el.dot()
        return el


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
