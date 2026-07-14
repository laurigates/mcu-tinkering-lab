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
            cell already occupied by a previously routed wire in this
            router. Discourages (but doesn't forbid) wires overlapping.
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
        grid = self.grid
        ox, oy = start
        gx, gy = round((goal[0] - ox) / grid), round((goal[1] - oy) / grid)

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
        ix_lo = math.floor((min(xs) - margin - ox) / grid)
        ix_hi = math.ceil((max(xs) + margin - ox) / grid)
        iy_lo = math.floor((min(ys) - margin - oy) / grid)
        iy_hi = math.ceil((max(ys) + margin - oy) / grid)

        def blocked(ix: int, iy: int) -> bool:
            if not (ix_lo <= ix <= ix_hi and iy_lo <= iy <= iy_hi):
                return True
            x, y = ox + ix * grid, oy + iy * grid
            return any(b.contains(x, y) for b in obstacles)

        def occ_penalty(ix: int, iy: int, axis: str) -> float:
            x, y = ox + ix * grid, oy + iy * grid
            key = (round(x / grid), round(y / grid))
            # Only penalize reusing a cell along the *same* axis (a wire
            # running parallel to / on top of another). A perpendicular
            # crossing is normal, unambiguous schematic notation and costs
            # nothing extra.
            return self.overlap_penalty if axis in self._occupied.get(key, ()) else 0.0

        start_node = (0, 0, None)  # (ix, iy, incoming direction)
        frontier: list[tuple[float, int, tuple]] = [(0.0, 0, start_node)]
        came_from: dict[tuple, tuple | None] = {start_node: None}
        cost_so_far: dict[tuple, float] = {start_node: 0.0}
        counter = 1

        while frontier:
            _, _, current = heapq.heappop(frontier)
            cix, ciy, cdir = current
            if (cix, ciy) == (gx, gy):
                return self._reconstruct(came_from, current, (ox, oy), grid)

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

    def _reconstruct(self, came_from, end_state, origin, grid) -> list[Coord]:
        ox, oy = origin
        points: list[Coord] = []
        state = end_state
        while state is not None:
            ix, iy, _ = state
            points.append((ox + ix * grid, oy + iy * grid))
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

        ``start``/``end`` are normally pin anchors (e.g. ``esp.GPIO5``);
        their owning component is automatically excluded from the obstacle
        set so the wire can leave the pin without being blocked by its own
        chip body.
        """
        start = (float(start[0]), float(start[1]))
        end = (float(end[0]), float(end[1]))
        boxes = self._component_boxes()
        inflated = [b.inflated(self.clearance) for b in boxes]

        start_owner = self._owning_box(start, boxes)
        end_owner = self._owning_box(end, boxes)
        # Exclude *every* box containing either endpoint, not just the
        # smallest ("owning") one: two components can legitimately overlap
        # a little (e.g. densely packed pin breakouts), and if a second
        # box also contains the goal point, leaving it in the obstacle list
        # makes the goal unreachable outright.
        obstacles = [
            b
            for raw, b in zip(boxes, inflated)
            if not raw.contains(*start) and not raw.contains(*end)
        ]

        start_dir = self._exit_direction(start, start_owner) if start_owner else None
        end_dir = self._exit_direction(end, end_owner) if end_owner else None
        entry = self._stub_point(start, start_dir) if start_dir else start
        exit_ = self._stub_point(end, end_dir) if end_dir else end

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


def _snap_final_approach(path: list[Coord], goal: Coord) -> list[Coord]:
    """Straighten the last segment so it lands exactly on ``goal``.

    The A* search runs on a grid anchored at the path's start (``path[0]``,
    a fixed entry point that must not move), so an independently-computed
    ``goal`` is generally not an exact multiple of the grid step away — the
    search lands within half a grid cell of it. Left alone, appending the
    exact ``goal`` afterward draws a tiny, pointless extra jog.

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
