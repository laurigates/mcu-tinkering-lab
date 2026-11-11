"""Can each part go in? A disassembly search over translations and tilts.

A part assembles if it can be taken out: from its seated pose, a chain of
small moves (lift, slide, tilt about either horizontal axis) must carry it
clear of the fixed bodies without touching them. Reversed, that chain is the
assembly motion, so a board that has to go in tilted, one edge tucked under an
overhang first, passes; a board that no motion gets past the lid bosses fails.

Collision is tested on voxel grids, which keeps a pose test to one numpy
lookup: the fixed bodies' grid against the moving part's surface points, and
the part's grid against the fixed bodies' surface points (a thin fixed feature
can poke between the part's samples). Every printed feature is at least
`case.MIN_WALL` (2 mm) thick and the grid pitch is 0.25 mm, so no feature
fits between grid cells. The search runs on a coarse lattice of moves with
coarse surface samples; the path it finds is then re-checked at fine
interpolation with every sample, and only a path that survives that counts.

The search is complete for its lattice: "no path" means no chain of those
moves within `max_tilt`, not a proof that no motion exists. A path is a proof.
"""

from __future__ import annotations

import heapq
import itertools
import math
from dataclasses import dataclass, field

import numpy as np
import trimesh

PITCH = 0.25  # voxel edge, mm
TOUCH = PITCH  # overlap read as contact: under a print's tolerance (about 0.2 mm)
SEAT_LIFT = 0.25  # start this far above the seated pose: the part rests on its
# standoffs, and a lid boss can start 0.5 mm over the board
STEP_MM = 0.5  # lattice translation step: the board-to-wall gap
STEP_DEG = 2.0  # lattice tilt step
MAX_TILT = 60.0  # degrees, either axis
MAX_NODES = 20000
MAX_REPAIRS = 8  # re-searches after the fine check refuses a hop


def mesh_of(part, tol: float = 0.05) -> trimesh.Trimesh:
    """A build123d shape as a trimesh, through its tessellation."""
    verts, tris = part.tessellate(tol)
    v = np.array([(p.X, p.Y, p.Z) for p in verts], dtype=float)
    return trimesh.Trimesh(v, np.array(tris, dtype=np.int64), process=True)


@dataclass(frozen=True)
class Grid:
    """Occupancy on a regular grid: cell (i, j, k) is solid if its centre is."""

    origin: np.ndarray  # world position of cell (0, 0, 0)'s lower corner
    pitch: float
    occ: np.ndarray  # bool (nx, ny, nz)

    def hits(self, pts, M: np.ndarray | None = None) -> np.ndarray:
        """Which points, moved by the 4x4 `M` if given, lie in a solid cell;
        points off the grid are free."""
        pts = np.asarray(pts, dtype=float)
        if M is None:
            M = np.eye(4)
        # Fold the grid's origin and pitch into the transform: one matmul.
        R = M[:3, :3] / self.pitch
        t = (M[:3, 3] - self.origin) / self.pitch
        idx = np.floor(pts @ R.T.astype(pts.dtype) + t).astype(np.int32)
        nx, ny, nz = self.occ.shape
        ok = (
            (idx[:, 0] >= 0)
            & (idx[:, 0] < nx)
            & (idx[:, 1] >= 0)
            & (idx[:, 1] < ny)
            & (idx[:, 2] >= 0)
            & (idx[:, 2] < nz)
        )
        out = np.zeros(len(pts), dtype=bool)
        i = idx[ok]
        out[ok] = self.flat[(i[:, 0] * ny + i[:, 1]) * nz + i[:, 2]]
        return out

    @property
    def flat(self) -> np.ndarray:
        return self.occ.reshape(-1)


def voxelize(mesh: trimesh.Trimesh, pitch: float = PITCH) -> Grid:
    """Solid cells of a closed mesh by ray parity, one ray up each cell column.

    Each triangle is rasterised onto the column centres its xy bounds cover;
    the z where it crosses a column is a parity crossing. Column centres sit
    a small irrational offset off the cell centres so a ray seldom runs exactly
    along an axis-aligned CAD edge, where a crossing would count twice.
    """
    lo, hi = mesh.bounds
    origin = lo - pitch
    n = np.ceil((hi - origin) / pitch).astype(int) + 2
    jitter = np.array([1e-4 * math.sqrt(2), 1e-4 * math.sqrt(3)])
    tri = mesh.triangles  # (t, 3, 3)
    a, b, c = tri[:, 0], tri[:, 1], tri[:, 2]
    # Column index ranges each triangle's xy box covers (column centre at i + 0.5).
    cmin = np.ceil((tri[:, :, :2].min(1) - origin[:2]) / pitch - 0.5 - jitter / pitch)
    cmax = np.floor((tri[:, :, :2].max(1) - origin[:2]) / pitch - 0.5 - jitter / pitch)
    cmin, cmax = cmin.astype(np.int64), cmax.astype(np.int64)
    span = np.maximum(cmax - cmin + 1, 0)
    count = span[:, 0] * span[:, 1]
    keep = count > 0
    t_idx = np.repeat(np.nonzero(keep)[0], count[keep])
    # Enumerate every (triangle, column) pair inside each triangle's box.
    starts = np.repeat(np.cumsum(count[keep]) - count[keep], count[keep])
    local = np.arange(len(t_idx)) - starts
    w = span[t_idx, 1]
    ci = cmin[t_idx, 0] + local // w
    cj = cmin[t_idx, 1] + local % w
    px = origin[0] + (ci + 0.5) * pitch + jitter[0]
    py = origin[1] + (cj + 0.5) * pitch + jitter[1]
    # Barycentric test in xy, then z on the triangle's plane.
    A, B, C = a[t_idx], b[t_idx], c[t_idx]
    v0x, v0y = B[:, 0] - A[:, 0], B[:, 1] - A[:, 1]
    v1x, v1y = C[:, 0] - A[:, 0], C[:, 1] - A[:, 1]
    v2x, v2y = px - A[:, 0], py - A[:, 1]
    den = v0x * v1y - v1x * v0y
    nz = np.abs(den) > 1e-12
    with np.errstate(divide="ignore", invalid="ignore"):
        u = (v2x * v1y - v1x * v2y) / den
        v = (v0x * v2y - v2x * v0y) / den
    inside = nz & (u >= 0) & (v >= 0) & (u + v <= 1)
    z = A[:, 2] + u * (B[:, 2] - A[:, 2]) + v * (C[:, 2] - A[:, 2])
    ci, cj, z = ci[inside], cj[inside], z[inside]
    order = np.lexsort((z, cj, ci))
    ci, cj, z = ci[order], cj[order], z[order]
    occ = np.zeros(n, dtype=bool)
    col = ci * n[1] + cj
    bounds = np.flatnonzero(np.diff(col)) + 1
    for s, e in zip(np.r_[0, bounds], np.r_[bounds, len(col)], strict=True):
        zs = z[s:e]
        if len(zs) % 2:  # a grazing ray: drop the column rather than guess
            continue
        i, j = ci[s], cj[s]
        for z0, z1 in zip(zs[0::2], zs[1::2], strict=True):
            k0 = math.ceil((z0 - origin[2]) / pitch - 0.5)
            k1 = math.floor((z1 - origin[2]) / pitch - 0.5)
            if k1 >= k0:
                occ[i, j, k0 : k1 + 1] = True
    return Grid(origin, pitch, occ)


def dilate(g: Grid, cells: int) -> Grid:
    """The grid grown by `cells` in every direction (a cube, axis by axis)."""
    occ = g.occ.copy()
    for axis in range(3):
        grown = occ.copy()
        for k in range(1, cells + 1):
            for sign in (1, -1):
                src = [slice(None)] * 3
                dst = [slice(None)] * 3
                if sign > 0:
                    src[axis], dst[axis] = slice(0, -k), slice(k, None)
                else:
                    src[axis], dst[axis] = slice(k, None), slice(0, -k)
                grown[tuple(dst)] |= occ[tuple(src)]
        occ = grown
    return Grid(g.origin, g.pitch, occ)


def erode(g: Grid, cells: int) -> Grid:
    """The grid shrunk by `cells` in every direction."""
    return Grid(g.origin, g.pitch, ~dilate(Grid(g.origin, g.pitch, ~g.occ), cells).occ)


def surface_points(mesh: trimesh.Trimesh, spacing: float, seed: int = 0) -> np.ndarray:
    """The mesh's vertices plus area-weighted samples about `spacing` apart."""
    count = int(mesh.area / (spacing * spacing)) + 1
    pts, _ = trimesh.sample.sample_surface(mesh, count, seed=seed)
    return np.vstack([mesh.vertices, pts]).astype(np.float32)


def transform(q: tuple[float, ...], pivot: np.ndarray) -> np.ndarray:
    """4x4 for pose (dx, dy, dz, rx, ry): tilt about x then y through `pivot`,
    then translate."""
    dx, dy, dz, rx, ry = q
    a, b = math.radians(rx), math.radians(ry)
    Rx = np.array(
        [[1, 0, 0], [0, math.cos(a), -math.sin(a)], [0, math.sin(a), math.cos(a)]]
    )
    Ry = np.array(
        [[math.cos(b), 0, math.sin(b)], [0, 1, 0], [-math.sin(b), 0, math.cos(b)]]
    )
    R = Ry @ Rx
    M = np.eye(4)
    M[:3, :3] = R
    M[:3, 3] = pivot - R @ pivot + np.array([dx, dy, dz])
    return M


def _apply(M: np.ndarray, pts: np.ndarray) -> np.ndarray:
    return pts @ M[:3, :3].T + M[:3, 3]


@dataclass
class Collider:
    """The fixed bodies and one moving part, with the part's pose relative to
    its seated pose."""

    fixed: trimesh.Trimesh
    moving: trimesh.Trimesh
    # Sample spacing for the search, and for the path check. Both are under
    # case.MIN_WALL (2 mm), the thinnest feature the wall gate lets through.
    coarse: float = 1.5
    fine: float = 0.5
    fixed_grid: Grid = field(init=False)
    moving_grid: Grid = field(init=False)

    def __post_init__(self) -> None:
        full = voxelize(self.fixed)
        # Within this margin of the fixed bodies, the search defers to the
        # fine check: coarse samples alone miss a shallow graze.
        self.fixed_near = dilate(full, math.ceil(self.coarse / 2 / PITCH))
        # Both grids lose their outer cell: a touch within TOUCH is contact,
        # not a clash. A cell centre decides a cell, so without this a 0.25 mm
        # gap (a lid boss 0.5 mm over the board, the board lifted 0.25 off its
        # standoffs) reads as either.
        self.fixed_grid = erode(full, 1)
        self.moving_grid = erode(voxelize(self.moving), 1)
        self.pivot = self.moving.bounds.mean(axis=0)
        self.pts = {
            "coarse": (
                surface_points(self.moving, self.coarse),
                surface_points(self.fixed, self.coarse),
            ),
            "fine": (
                surface_points(self.moving, self.fine),
                surface_points(self.fixed, self.fine),
            ),
        }
        self.top = self.fixed.bounds[1][2]

    def clashing(self, q, level: str = "fine") -> np.ndarray:
        """Moving-part points (in world coordinates) inside the fixed bodies,
        plus fixed points inside the part, at pose q."""
        M = transform(q, self.pivot)
        mov, fix = self.pts[level]
        a = _apply(M, mov[self.fixed_grid.hits(mov, M)])
        world_lo, world_hi = self._bounds(M)
        near = fix[np.all((fix >= world_lo) & (fix <= world_hi), axis=1)]
        b = near[self.moving_grid.hits(near, np.linalg.inv(M))]
        return np.vstack([a, b])

    def collides(self, q, level: str = "coarse") -> bool:
        """At "coarse", coarse samples against the fixed grid grown by half
        their spacing; only a pose that comes that near gets the fine test.
        At "fine", fine samples both ways: the part's in the fixed grid, and
        the fixed bodies' in the part's grid, so a thin fixed feature poking
        between the part's samples is caught."""
        M = transform(q, self.pivot)
        if level == "coarse":
            if not self.fixed_near.hits(self.pts["coarse"][0], M).any():
                return False
            level = "fine"
        mov, fix = self.pts[level]
        if self.fixed_grid.hits(mov, M).any():
            return True
        world_lo, world_hi = self._bounds(M)
        near = fix[np.all((fix >= world_lo) & (fix <= world_hi), axis=1)]
        return bool(self.moving_grid.hits(near, np.linalg.inv(M)).any())

    def _bounds(self, M):
        c = _apply(M, trimesh.bounds.corners(self.moving.bounds))
        return c.min(0) - 1, c.max(0) + 1

    def below_top(self, q) -> float:
        """How far the part's lowest point (of its box) is under the fixed
        bodies' top plus 0.5 mm; at or under 0 it is out."""
        M = transform(q, self.pivot)
        corners = trimesh.bounds.corners(self.moving.bounds)
        return self.top + 0.5 - _apply(M, corners)[:, 2].min()

    def clear(self, q) -> bool:
        return self.below_top(q) < 0


@dataclass
class Plan:
    found: bool
    path: list[tuple[float, ...]]  # seated pose first; reverse it to assemble
    nodes: int
    deepest: tuple[float, ...]  # the pose nearest to clear, when not found
    clash_points: np.ndarray | None = None  # at the first failing pose, if any
    note: str = ""


def _interpolate(a, b, step_mm: float, reach: float):
    """Poses from a to b with no point moving more than about one cell."""
    da = max(abs(b[i] - a[i]) for i in range(3)) / step_mm
    dr = max(abs(b[i] - a[i]) for i in (3, 4))
    dr = math.radians(dr) * reach / step_mm
    n = max(1, math.ceil(max(da, dr)))
    for s in range(1, n + 1):
        t = s / n
        yield tuple(a[i] + t * (b[i] - a[i]) for i in range(5))


def plan(
    col: Collider,
    max_nodes: int = MAX_NODES,
    max_tilt: float = MAX_TILT,
    step_mm: float = STEP_MM,
    step_deg: float = STEP_DEG,
    axes: tuple[int, ...] = (0, 2, 4),
) -> Plan:
    """Best-first search from the seated pose (lifted SEAT_LIFT) to clear.

    `axes` are the pose coordinates that may change, from (dx, dy, dz, rx,
    ry); every combination of one lattice step on each is a move, so a tilt
    and a lift can happen together through a passage neither fits alone.
    """
    start = (0, 0, 0, 0, 0)  # lattice units, offset by SEAT_LIFT in pose()

    def pose(n):
        return (
            n[0] * step_mm,
            n[1] * step_mm,
            SEAT_LIFT + n[2] * step_mm,
            n[3] * step_deg,
            n[4] * step_deg,
        )

    s = pose(start)
    if col.collides(s):
        pts = col.clashing(s)
        return Plan(False, [], 0, s, pts, "the seated pose already clashes")
    lim = int(max_tilt / step_deg)
    moves = [
        m
        for m in itertools.product((-1, 0, 1), repeat=5)
        if any(m) and all(m[i] == 0 for i in range(5) if i not in axes)
    ]

    def h(n):
        # Steps still to go: the lowest point's height under the top, so
        # flattening a tilt counts as progress as much as lifting does.
        return max(0.0, col.below_top(pose(n))) / step_mm

    reach = float(np.linalg.norm(col.moving.extents) / 2)

    def edge_free(n, k) -> bool:
        """The move's end pose and the poses between, no point moving over
        the coarse spacing between checks."""
        return not any(
            col.collides(q) for q in _interpolate(pose(n), pose(k), col.coarse, reach)
        )

    banned: set[tuple] = set()  # hops the fine check refused
    nodes = 0
    for _ in range(MAX_REPAIRS + 1):
        parent = {start: None}
        heap = [(h(start), 0, start)]
        best = start
        goal = None
        while heap and nodes < max_nodes:
            _, g, n = heapq.heappop(heap)
            nodes += 1
            if h(n) < h(best):
                best = n
            if col.clear(pose(n)):
                goal = n
                break
            for m in moves:
                k = tuple(a + b for a, b in zip(n, m, strict=True))
                if k in parent or abs(k[3]) > lim or abs(k[4]) > lim or k[2] < 0:
                    continue
                if (n, k) in banned or not edge_free(n, k):
                    continue  # another parent may still reach k
                parent[k] = n
                heapq.heappush(heap, (h(k) + 0.25 * (g + 1), g + 1, k))
        if goal is None:
            note = "search exhausted" if not heap else f"stopped at {max_nodes} poses"
            return Plan(False, [], nodes, pose(best), None, note)
        lattice = []
        n = goal
        while n is not None:
            lattice.append(n)
            n = parent[n]
        lattice.reverse()
        chain = [(0.0, 0.0, 0.0, 0.0, 0.0)] + [pose(n) for n in lattice]
        # Re-check every hop: no point moves more than the fine spacing between
        # checks, which is under any printed feature's thickness.
        bad = None
        for u, v in itertools.pairwise(lattice):
            for q in _interpolate(pose(u), pose(v), col.fine, reach):
                if col.collides(q, "fine"):
                    bad = (u, v, q, col.clashing(q, "fine"))
                    break
            if bad:
                break
        if bad is None:
            return Plan(True, chain, nodes, chain[-1])
        banned.add(bad[:2])
    return Plan(
        False, chain, nodes, bad[2], bad[3], "every path found clashes when refined"
    )


# Vertical lift, then the two planar tucks: tilt about y sliding in x, tilt
# about x sliding in y. A board tucks under an overhang in one plane.
STRATEGIES = {
    "lift": (2,),
    "tilt-about-y": (0, 2, 4),
    "tilt-about-x": (1, 2, 3),
}


def removal(col: Collider, max_nodes: int = MAX_NODES) -> tuple[str, Plan]:
    """The first strategy that takes the part out, else the one that got furthest."""
    tried = []
    for name, axes in STRATEGIES.items():
        p = plan(col, max_nodes=max_nodes, axes=axes)
        if p.found:
            return name, p
        tried.append((p.deepest[2], name, p))
    _, name, p = max(tried, key=lambda t: t[0])
    return name, p


def _hop_clear(col: Collider, a, b, level: str) -> bool:
    reach = float(np.linalg.norm(col.moving.extents) / 2)
    pitch = col.fine if level == "fine" else col.coarse
    return not any(col.collides(q, level) for q in _interpolate(a, b, pitch, reach))


def shortcut(col: Collider, path: list[tuple[float, ...]]) -> list[tuple[float, ...]]:
    """Straighten a found path: from each pose, jump to the furthest later pose
    the straight hop reaches without a clash (coarse, then fine)."""
    if len(path) < 3:
        return path
    out = [path[0], path[1]]  # the seat hop stays: the seated pose touches
    i = 1
    while i < len(path) - 1:
        j = len(path) - 1
        while j > i + 1 and not (
            _hop_clear(col, path[i], path[j], "coarse")
            and _hop_clear(col, path[i], path[j], "fine")
        ):
            j -= 1
        out.append(path[j])
        i = j
    return out


def simplify(path: list[tuple[float, ...]]) -> list[tuple[float, ...]]:
    """Drop the poses that lie on a straight run (same move repeated)."""
    if len(path) < 3:
        return path
    out = [path[0]]
    for prev, cur, nxt in zip(path, path[1:], path[2:], strict=False):
        d1 = tuple(round(c - p, 6) for p, c in zip(prev, cur, strict=True))
        d2 = tuple(round(n - c, 6) for c, n in zip(cur, nxt, strict=True))
        if d1 != d2:
            out.append(cur)
    out.append(path[-1])
    return out
