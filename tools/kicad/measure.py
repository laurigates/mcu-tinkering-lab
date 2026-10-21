"""Layer-aware copper measurements over a carrier board dump (pcb/dump.py).

    just <project>::carrier-measure <subcommand> [args] [--json] [--ref <git-ref>]
    python3 -m kicad.measure <dump.json> <subcommand> [args] [--json]

Copper joins only on one layer; a via or a plated through-hole pad is the only
join between F.Cu and B.Cu. A path script that joined a B.Cu track end to an
F.Cu track it merely crossed reported "no via" for a path through a via, and
that number reached a README; test_measure pins the difference.

Subcommands (one result per line, ``--json`` for a list of objects):

- ``path NET A:pad B:pad``: the routed path with the least track length
  (tracks, vias and pads; the pour is left out), its length (the summed length
  of the tracks it uses), via count and narrowest track.
- ``cuts NET A:pad``: each via whose removal disconnects pads from A, over
  tracks, vias, pads and the pour.
- ``vias-in-pads``: vias whose ring overlaps an SMD pad, ``hole-in-pad`` when
  the drill does, else ``ring-only``; board.toml ``[fanout] in_pad`` pads are
  left out (their via is meant to be there).
- ``clearance-to-holes``: per non-plated hole, the nearest copper (any net,
  either layer), edge to edge.
- ``plane``: per poured net, the components of pour, vias and through-hole pads
  alone (no tracks, no SMD pads), with fill area; a component the full copper
  joins to the largest one only through tracks is flagged ``tracks-only``, one
  nothing joins ``isolated``. ``--review`` prints carrier-review's section.
- ``resistance NET A:pad B:pad``: a DC resistance *estimate* over tracks, pour
  and vias on a raster (1 oz copper, 25 um via plating). Needs numpy and scipy;
  the recipe runs it through ``uv run --with``.

Everything but ``resistance`` is stdlib, like the rest of the package, so
``just kicad::test`` runs its tests without extra packages.
"""

from __future__ import annotations

import argparse
import heapq
import itertools
import json
import math
import sys
from collections import defaultdict
from dataclasses import dataclass, field

from .errors import KicadError

LAYERS = ("F.Cu", "B.Cu")
EPS = 1e-4  # mm: copper closer than this touches
RHO_CU = 1.72e-8  # ohm m, annealed copper at 20 C
CU_1OZ_M = 35e-6
PLATING_M = 25e-6
BOARD_M = 1.6e-3
PLANE_MIN_MM2 = 5.0  # plane components smaller than this are not reported
# resistance's cell size: a 0.25 mm track at 0.1 mm rasterises 2 or 3 cells
# wide and reads 26 % high; at 0.05 mm an axis-aligned one is within 1 %.
# Diagonal copper reads higher, as cells join only edge to edge: a 0.25 mm
# track at 45 degrees reads about 14 % high at 0.05 mm (test_measure).
RASTER_MM = 0.05

Pt = tuple[float, float]


# --------------------------------------------------------------------- geometry


@dataclass(frozen=True)
class Capsule:
    """A segment swept by a radius: a track, or a via (a == b)."""

    a: Pt
    b: Pt
    r: float

    @property
    def bbox(self) -> tuple[float, float, float, float]:
        return (
            min(self.a[0], self.b[0]) - self.r,
            min(self.a[1], self.b[1]) - self.r,
            max(self.a[0], self.b[0]) + self.r,
            max(self.a[1], self.b[1]) + self.r,
        )


@dataclass(frozen=True)
class Region:
    """A polygon with holes: a pad's shape or a pour island."""

    outline: tuple[Pt, ...]
    holes: tuple[tuple[Pt, ...], ...] = ()
    bbox: tuple[float, float, float, float] = field(init=False)

    def __post_init__(self):
        xs = [p[0] for p in self.outline]
        ys = [p[1] for p in self.outline]
        object.__setattr__(self, "bbox", (min(xs), min(ys), max(xs), max(ys)))

    @property
    def rings(self):
        return (self.outline, *self.holes)

    @property
    def area(self) -> float:
        return abs(ring_area(self.outline)) - sum(abs(ring_area(h)) for h in self.holes)


def region(d: dict) -> Region:
    return Region(
        tuple(tuple(p) for p in d["outline"]),
        tuple(tuple(tuple(p) for p in h) for h in d.get("holes", ())),
    )


def ring_area(ring) -> float:
    n = len(ring)
    return 0.5 * sum(
        ring[i][0] * ring[(i + 1) % n][1] - ring[(i + 1) % n][0] * ring[i][1]
        for i in range(n)
    )


def edges(ring):
    n = len(ring)
    for i in range(n):
        yield ring[i], ring[(i + 1) % n]


def bbox_gap(a, b) -> float:
    """Distance between two bounding boxes (0 when they overlap)."""
    dx = max(a[0] - b[2], b[0] - a[2], 0.0)
    dy = max(a[1] - b[3], b[1] - a[3], 0.0)
    return math.hypot(dx, dy)


def point_seg(p: Pt, a: Pt, b: Pt) -> float:
    dx, dy = b[0] - a[0], b[1] - a[1]
    n = dx * dx + dy * dy
    t = (
        0.0
        if n == 0
        else max(0.0, min(1.0, ((p[0] - a[0]) * dx + (p[1] - a[1]) * dy) / n))
    )
    return math.hypot(p[0] - (a[0] + t * dx), p[1] - (a[1] + t * dy))


def _orient(a: Pt, b: Pt, c: Pt) -> float:
    return (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])


def segs_cross(p1: Pt, p2: Pt, q1: Pt, q2: Pt) -> bool:
    d1, d2 = _orient(q1, q2, p1), _orient(q1, q2, p2)
    d3, d4 = _orient(p1, p2, q1), _orient(p1, p2, q2)
    return d1 * d2 < 0 and d3 * d4 < 0


def seg_seg(p1: Pt, p2: Pt, q1: Pt, q2: Pt) -> float:
    if segs_cross(p1, p2, q1, q2):
        return 0.0
    return min(
        point_seg(p1, q1, q2),
        point_seg(p2, q1, q2),
        point_seg(q1, p1, p2),
        point_seg(q2, p1, p2),
    )


def in_ring(p: Pt, ring) -> bool:
    x, y = p
    inside = False
    for (x0, y0), (x1, y1) in edges(ring):
        if (y0 > y) != (y1 > y) and x < x0 + (y - y0) * (x1 - x0) / (y1 - y0):
            inside = not inside
    return inside


def in_region(p: Pt, rg: Region) -> bool:
    b = rg.bbox
    if not (b[0] <= p[0] <= b[2] and b[1] <= p[1] <= b[3]):
        return False
    return in_ring(p, rg.outline) and not any(in_ring(p, h) for h in rg.holes)


def point_region(p: Pt, rg: Region) -> float:
    """Distance from a point to a region's copper (0 inside it)."""
    if in_region(p, rg):
        return 0.0
    return min(point_seg(p, a, b) for ring in rg.rings for a, b in edges(ring))


def point_shape(p: Pt, s) -> float:
    if isinstance(s, Capsule):
        return max(0.0, point_seg(p, s.a, s.b) - s.r)
    return point_region(p, s)


def _seg_region(a: Pt, b: Pt, rg: Region, reach: float) -> bool:
    """True when segment a-b comes within `reach` of the region's copper."""
    if in_region(a, rg) or in_region(b, rg):
        return True
    sb = (min(a[0], b[0]), min(a[1], b[1]), max(a[0], b[0]), max(a[1], b[1]))
    for ring in rg.rings:
        for e0, e1 in edges(ring):
            eb = (
                min(e0[0], e1[0]),
                min(e0[1], e1[1]),
                max(e0[0], e1[0]),
                max(e0[1], e1[1]),
            )
            if bbox_gap(sb, eb) <= reach and seg_seg(a, b, e0, e1) <= reach:
                return True
    return False


def _region_region(r1: Region, r2: Region) -> bool:
    for p in r1.outline:
        if in_region(p, r2):
            return True
    for p in r2.outline:
        if in_region(p, r1):
            return True
    near1 = [
        e
        for ring in r1.rings
        for e in edges(ring)
        if bbox_gap(_ebox(e), r2.bbox) <= EPS
    ]
    near2 = [
        e
        for ring in r2.rings
        for e in edges(ring)
        if bbox_gap(_ebox(e), r1.bbox) <= EPS
    ]
    for e in near1:
        b1 = _ebox(e)
        for f in near2:
            if bbox_gap(b1, _ebox(f)) <= EPS and seg_seg(e[0], e[1], f[0], f[1]) <= EPS:
                return True
    return False


def _ebox(e):
    (x0, y0), (x1, y1) = e
    return (min(x0, x1), min(y0, y1), max(x0, x1), max(y0, y1))


def touches(s1, s2) -> bool:
    """Whether two copper shapes on the same layer overlap or touch."""
    if bbox_gap(s1.bbox, s2.bbox) > EPS:
        return False
    if isinstance(s1, Capsule) and isinstance(s2, Capsule):
        return seg_seg(s1.a, s1.b, s2.a, s2.b) <= s1.r + s2.r + EPS
    if isinstance(s1, Capsule):
        return _seg_region(s1.a, s1.b, s2, s1.r + EPS)
    if isinstance(s2, Capsule):
        return _seg_region(s2.a, s2.b, s1, s2.r + EPS)
    return _region_region(s1, s2)


# ------------------------------------------------------------------ board items


@dataclass(frozen=True)
class Item:
    """One piece of copper on one layer. Items sharing a `key` are one
    conductor: a via or a through-hole pad on each of its layers."""

    key: tuple
    layer: str
    shape: object


def pad_name(p: dict) -> str:
    return f"{p['ref']}:{p['num']}"


def arc(a: Pt, mid: Pt, b: Pt):
    """The circle through an arc's three points: (centre, radius, start angle,
    signed sweep), or None when the points are collinear (a straight track)."""
    (ax, ay), (mx, my), (bx, by) = a, mid, b
    d = 2 * (ax * (my - by) + mx * (by - ay) + bx * (ay - my))
    if abs(d) < 1e-12:
        return None
    a2, m2, b2 = ax * ax + ay * ay, mx * mx + my * my, bx * bx + by * by
    cx = (a2 * (my - by) + m2 * (by - ay) + b2 * (ay - my)) / d
    cy = (a2 * (bx - mx) + m2 * (ax - bx) + b2 * (mx - ax)) / d
    r = math.hypot(ax - cx, ay - cy)
    t0 = math.atan2(ay - cy, ax - cx)
    tau = 2 * math.pi
    to_b = (math.atan2(by - cy, bx - cx) - t0) % tau
    to_mid = (math.atan2(my - cy, mx - cx) - t0) % tau
    sweep = to_b if to_mid <= to_b else to_b - tau
    return (cx, cy), r, t0, sweep


ARC_SAGITTA_MM = 0.002  # an arc's chords stay within this of the true arc


def track_points(t: dict) -> list[Pt]:
    """A track's centreline: its two ends, or an arc as chords whose sagitta
    is at most ARC_SAGITTA_MM."""
    a, b = tuple(t["a"]), tuple(t["b"])
    c = arc(a, tuple(t["mid"]), b) if "mid" in t else None
    if c is None:
        return [a, b]
    (cx, cy), r, t0, sweep = c
    step = 2 * math.acos(max(-1.0, 1 - ARC_SAGITTA_MM / r))
    n = max(2, math.ceil(abs(sweep) / step))
    inner = [
        (cx + r * math.cos(t0 + sweep * i / n), cy + r * math.sin(t0 + sweep * i / n))
        for i in range(1, n)
    ]
    return [a, *inner, b]


def track_shapes(t: dict):
    r = t["width"] / 2
    for a, b in itertools.pairwise(track_points(t)):
        yield Capsule(a, b, r)


def track_length(t: dict) -> float:
    """Centreline length; an arc's comes from its circle, not from chords."""
    a, b = tuple(t["a"]), tuple(t["b"])
    c = arc(a, tuple(t["mid"]), b) if "mid" in t else None
    if c is None:
        return math.dist(a, b)
    return c[1] * abs(c[3])


def via_layers(v: dict):
    # Two-layer boards: a via spans F.Cu to B.Cu.
    return [layer for layer in LAYERS if layer in v.get("layers", LAYERS)] or list(
        LAYERS
    )


def items(
    board: dict,
    net: str,
    *,
    tracks=True,
    vias=True,
    smd=True,
    tht=True,
    zones=True,
) -> list[Item]:
    out: list[Item] = []
    if tracks:
        for i, t in enumerate(board["tracks"]):
            if t["net"] == net:
                out += [Item(("T", i), t["layer"], s) for s in track_shapes(t)]
    if vias:
        for i, v in enumerate(board["vias"]):
            if v["net"] == net:
                c = Capsule(tuple(v["at"]), tuple(v["at"]), v["size"] / 2)
                out += [Item(("V", i), layer, c) for layer in via_layers(v)]
    for p in board["pads"]:
        if p["net"] != net or not ((p["tht"] and tht) or (not p["tht"] and smd)):
            continue
        for layer, polys in p["shapes"].items():
            out += [Item(("P", pad_name(p)), layer, region(poly)) for poly in polys]
    if zones:
        for zi, z in enumerate(board["zones"]):
            if z["net"] == net:
                for k, isl in enumerate(z["islands"]):
                    out.append(Item(("Z", zi, k), z["layer"], region(isl)))
    return out


def adjacency(its: list[Item]) -> dict[tuple, set[tuple]]:
    """Which items touch, per layer only; a key's layers are one node."""
    adj: dict[tuple, set[tuple]] = {it.key: set() for it in its}
    for layer in LAYERS:
        on = sorted(
            (it for it in its if it.layer == layer), key=lambda it: it.shape.bbox[0]
        )
        for i, a in enumerate(on):
            ax1 = a.shape.bbox[2]
            for b in on[i + 1 :]:
                if b.shape.bbox[0] > ax1 + EPS:
                    break
                if a.key != b.key and touches(a.shape, b.shape):
                    adj[a.key].add(b.key)
                    adj[b.key].add(a.key)
    return adj


def reach(adj, start, removed=frozenset()) -> set:
    seen = {start}
    stack = [start]
    while stack:
        u = stack.pop()
        for v in adj.get(u, ()):
            if v not in seen and v not in removed:
                seen.add(v)
                stack.append(v)
    return seen


def components(adj) -> list[set]:
    left = set(adj)
    out = []
    while left:
        c = reach(adj, next(iter(left)))
        out.append(c)
        left -= c
    return out


def need_pad(board: dict, net: str, name: str) -> dict:
    for p in board["pads"]:
        if pad_name(p) == name:
            if p["net"] != net:
                raise KicadError(f"{name} is on net {p['net']!r}, not {net!r}")
            return p
    raise KicadError(f"no pad {name} on the board")


# ------------------------------------------------------------------ subcommands


def hop(board: dict, k: tuple) -> str:
    if k[0] == "T":
        t = board["tracks"][k[1]]
        return f"track {t['layer']} w{t['width']} {fmt_xy(t['a'])}-{fmt_xy(t['b'])}"
    if k[0] == "V":
        v = board["vias"][k[1]]
        return f"via {fmt_xy(v['at'])} {v['size']}/{v['drill']}"
    return f"pad {k[1]}"


def path(board: dict, net: str, a: str, b: str) -> dict:
    """The routed path from pad a to pad b with the least track length."""
    need_pad(board, net, a)
    need_pad(board, net, b)
    adj = adjacency(items(board, net, zones=False))
    src, dst = ("P", a), ("P", b)

    def cost(k):
        return (
            track_length(board["tracks"][k[1]]) if k[0] == "T" else 0.0,
            k[0] == "V",
        )

    best = {src: (0.0, 0)}
    prev = {}
    pq = [(0.0, 0, src)]
    while pq:
        length, nv, u = heapq.heappop(pq)
        if u == dst:
            break
        if (length, nv) > best[u]:
            continue
        for v in adj[u]:
            dl, dv = cost(v)
            nc = (length + dl, nv + dv)
            if nc < best.get(v, (math.inf, 0)):
                best[v] = nc
                prev[v] = u
                heapq.heappush(pq, (*nc, v))
    res = {"kind": "path", "net": net, "from": a, "to": b}
    if dst not in best:
        return {**res, "connected": False}
    keys = [dst]
    while keys[-1] != src:
        keys.append(prev[keys[-1]])
    keys.reverse()
    tracks = [board["tracks"][k[1]] for k in keys if k[0] == "T"]
    vias = [board["vias"][k[1]] for k in keys if k[0] == "V"]
    return {
        **res,
        "connected": True,
        "length_mm": round(sum(track_length(t) for t in tracks), 2),
        "vias": len(vias),
        "tracks": len(tracks),
        "narrowest_mm": min((t["width"] for t in tracks), default=None),
        "layers": sorted({t["layer"] for t in tracks}),
        "via_at": [v["at"] for v in vias],
        "hops": [hop(board, k) for k in keys],
    }


def cuts(board: dict, net: str, a: str) -> list[dict]:
    """Each via of `net` whose removal disconnects pads from pad a."""
    need_pad(board, net, a)
    adj = adjacency(items(board, net))
    src = ("P", a)
    base = reach(adj, src)
    pads = sorted(k for k in base if k[0] == "P")
    out = []
    for i, v in enumerate(board["vias"]):
        if v["net"] != net or ("V", i) not in base:
            continue
        got = reach(adj, src, frozenset({("V", i)}))
        lost = [k[1] for k in pads if k not in got]
        if lost:
            out.append(
                {
                    "kind": "cut",
                    "net": net,
                    "from": a,
                    "via_at": v["at"],
                    "via": [v["size"], v["drill"]],
                    "disconnects": lost,
                }
            )
    return out


def vias_in_pads(board: dict) -> list[dict]:
    exempt = set(((board.get("spec") or {}).get("fanout") or {}).get("in_pad", []))
    out = []
    for v in board["vias"]:
        c = tuple(v["at"])
        for p in board["pads"]:
            if p["tht"] or pad_name(p) in exempt:
                continue
            for layer, polys in p["shapes"].items():
                if layer not in via_layers(v):
                    continue
                d = min(point_region(c, region(poly)) for poly in polys)
                if d < v["size"] / 2 - EPS:
                    out.append(
                        {
                            "kind": "via-in-pad",
                            "net": v["net"],
                            "via_at": v["at"],
                            "via": [v["size"], v["drill"]],
                            "pad": pad_name(p),
                            "pad_net": p["net"],
                            "layer": layer,
                            "overlap": "hole-in-pad"
                            if d < v["drill"] / 2 - EPS
                            else "ring-only",
                            "centre_to_pad_mm": round(d, 3),
                        }
                    )
                    break
    return out


def copper(board: dict):
    """(description, layer, shape) for every piece of copper on the board."""
    for t in board["tracks"]:
        for s in track_shapes(t):
            yield f"track {t['net']} {t['layer']} w{t['width']}", t["layer"], s
    for v in board["vias"]:
        c = Capsule(tuple(v["at"]), tuple(v["at"]), v["size"] / 2)
        for layer in via_layers(v):
            yield f"via {v['net']} {v['at']}", layer, c
    for p in board["pads"]:
        for layer, polys in p["shapes"].items():
            for poly in polys:
                yield f"pad {pad_name(p)} {p['net']}", layer, region(poly)
    for z in board["zones"]:
        for isl in z["islands"]:
            yield f"{z['net']} fill", z["layer"], region(isl)


def clearance_to_holes(board: dict) -> list[dict]:
    cu = list(copper(board))
    out = []
    for h in sorted(board["holes"], key=lambda h: (h["ref"], h["num"], h["at"])):
        c, r = tuple(h["at"]), h["drill"] / 2
        best, what, where = math.inf, "-", None
        hb = (c[0], c[1], c[0], c[1])
        for desc, layer, s in cu:
            if bbox_gap(hb, s.bbox) - r >= best:
                continue
            d = point_shape(c, s) - r
            if d < best:
                best, what, where = d, desc, layer
        out.append(
            {
                "kind": "hole",
                "hole": f"{h['ref']}:{h['num']}" if h["num"] else h["ref"],
                "at": h["at"],
                "drill": h["drill"],
                "clearance_mm": round(best, 3) if best < math.inf else None,
                "nearest": what,
                "layer": where,
            }
        )
    return out


def poured_nets(board: dict) -> list[str]:
    regions = (((board.get("spec") or {}).get("pour")) or {}).get("regions") or []
    nets = {r["net"] for r in regions} or {z["net"] for z in board["zones"]}
    return sorted(nets)


def plane(board: dict, min_area: float = PLANE_MIN_MM2) -> list[dict]:
    """Per poured net, the pour + via + through-hole-pad components."""
    out = []
    for net in poured_nets(board):
        pl = items(board, net, tracks=False, smd=False)
        comps = components(adjacency(pl))
        full = adjacency(items(board, net))
        area = defaultdict(float)
        boxes: dict[tuple, list] = defaultdict(list)
        for it in pl:
            if it.key[0] == "Z":
                area[it.key] = it.shape.area
            boxes[it.key].append(it.shape.bbox)
        rows = []
        for c in comps:
            a = sum(area[k] for k in c)
            bb = [b for k in c for b in boxes[k]]
            rows.append(
                {
                    "keys": c,
                    "area_mm2": a,
                    "bbox": [
                        round(min(b[0] for b in bb), 1),
                        round(min(b[1] for b in bb), 1),
                        round(max(b[2] for b in bb), 1),
                        round(max(b[3] for b in bb), 1),
                    ],
                    "vias": sum(1 for k in c if k[0] == "V"),
                    "tht_pads": sorted(k[1] for k in c if k[0] == "P"),
                }
            )
        rows.sort(key=lambda r: -r["area_mm2"])
        main_reach = reach(full, next(iter(rows[0]["keys"]))) if rows else set()
        small = 0
        for i, r in enumerate(rows):
            if i and r["area_mm2"] < min_area:
                small += 1
                continue
            if i == 0:
                status = "main"
            elif next(iter(r["keys"])) in main_reach:
                status = "tracks-only"
            else:
                status = "isolated"
            keys = r.pop("keys")
            out.append(
                {
                    "kind": "plane",
                    "net": net,
                    "status": status,
                    **{
                        k: (round(v, 1) if isinstance(v, float) else v)
                        for k, v in r.items()
                    },
                    "zone_islands": sum(1 for k in keys if k[0] == "Z"),
                }
            )
        out.append(
            {
                "kind": "plane-summary",
                "net": net,
                "components": len(rows),
                "listed": len(rows) - small,
                "below_min_area": small,
                "min_area_mm2": min_area,
                "flag": len(rows) - small > 1,
            }
        )
    return out


def resistance(
    board: dict, net: str, a: str, b: str, res_mm: float = RASTER_MM
) -> dict:
    """DC resistance estimate between two pads of `net`, on a raster.

    Each layer's copper (tracks, pour, pads) becomes square cells of side
    `res_mm` with sheet resistance rho / 35 um between neighbours; each via
    and plated hole joins the layers through its barrel (rho * 1.6 mm over a
    25 um wall); each end pad's cells are shorted together. The raster
    under-resolves tracks narrower than about five cells and reads diagonal
    tracks high (RASTER_MM).
    """
    try:
        import numpy as np
        import scipy.sparse as sp
        import scipy.sparse.csgraph as csg
        import scipy.sparse.linalg as sla
    except ImportError as e:  # pragma: no cover - depends on the environment
        raise KicadError(
            f"resistance needs numpy and scipy ({e}); run it through "
            "`just <project>::carrier-measure`, which uses `uv run --with`"
        ) from e

    pa, pb = need_pad(board, net, a), need_pad(board, net, b)
    its = items(board, net)
    bb = [it.shape.bbox for it in its]
    x0, y0 = min(q[0] for q in bb) - res_mm, min(q[1] for q in bb) - res_mm
    x1, y1 = max(q[2] for q in bb) + res_mm, max(q[3] for q in bb) + res_mm
    xs = np.arange(x0, x1, res_mm) + res_mm / 2
    ys = np.arange(y0, y1, res_mm) + res_mm / 2
    X, Y = np.meshgrid(xs, ys)

    def window(box):
        j0 = max(0, int((box[0] - x0) / res_mm) - 1)
        j1 = min(len(xs), int((box[2] - x0) / res_mm) + 2)
        i0 = max(0, int((box[1] - y0) / res_mm) - 1)
        i1 = min(len(ys), int((box[3] - y0) / res_mm) + 2)
        return slice(i0, i1), slice(j0, j1)

    def ring_mask(ring, px, py):
        inside = np.zeros(px.shape, bool)
        for (ex0, ey0), (ex1, ey1) in edges(ring):
            if ey0 == ey1:
                continue
            crosses = (ey0 > py) != (ey1 > py)
            xc = ex0 + (py - ey0) * (ex1 - ex0) / (ey1 - ey0)
            inside ^= crosses & (px < xc)
        return inside

    def mask(shape):
        m = np.zeros(X.shape, bool)
        win = window(shape.bbox)
        px, py = X[win], Y[win]
        if isinstance(shape, Capsule):
            (ax, ay), (bx, by) = shape.a, shape.b
            dx, dy = bx - ax, by - ay
            n = dx * dx + dy * dy
            t = (
                np.zeros(px.shape)
                if n == 0
                else np.clip(((px - ax) * dx + (py - ay) * dy) / n, 0, 1)
            )
            m[win] = np.hypot(px - (ax + t * dx), py - (ay + t * dy)) <= shape.r
        else:
            inside = ring_mask(shape.outline, px, py)
            for h in shape.holes:
                inside &= ~ring_mask(h, px, py)
            m[win] = inside
        return m

    cu = {layer: np.zeros(X.shape, bool) for layer in LAYERS}
    for it in its:
        cu[it.layer] |= mask(it.shape)
    ids = {}
    n = 0
    for layer in LAYERS:
        ids[layer] = np.full(X.shape, -1)
        k = int(cu[layer].sum())
        ids[layer][cu[layer]] = np.arange(n, n + k)
        n += k
    g_sheet = CU_1OZ_M / RHO_CU
    rows, cols, vals = [], [], []
    for layer in LAYERS:
        idx = ids[layer]
        for di, dj in ((0, 1), (1, 0)):
            p = idx[: idx.shape[0] - di, : idx.shape[1] - dj]
            q = idx[di:, dj:]
            ok = (p >= 0) & (q >= 0)
            rows.append(p[ok])
            cols.append(q[ok])
            vals.append(np.full(int(ok.sum()), g_sheet))

    def barrel(drill_mm):
        return math.pi * drill_mm * 1e-3 * PLATING_M / (RHO_CU * BOARD_M)

    def join(shape, g):
        m = mask(shape)
        p, q = ids["F.Cu"][m], ids["B.Cu"][m]
        ok = (p >= 0) & (q >= 0)
        k = int(ok.sum())
        if k:
            rows.append(p[ok])
            cols.append(q[ok])
            vals.append(np.full(k, g / k))

    for v in board["vias"]:
        if v["net"] == net:
            join(
                Capsule(tuple(v["at"]), tuple(v["at"]), v["size"] / 2),
                barrel(v["drill"]),
            )
    for p in board["pads"]:
        if p["net"] == net and p["tht"] and len(p["shapes"]) == 2:
            c = Capsule(tuple(p["at"]), tuple(p["at"]), p["drill"] / 2 + 0.1)
            join(c, barrel(p["drill"]))

    def pad_cells(p):
        out = []
        for layer, polys in p["shapes"].items():
            for poly in polys:
                m = mask(region(poly)) & (ids[layer] >= 0)
                out.append(ids[layer][m])
        return np.concatenate(out) if out else np.array([], int)

    ca, cb = pad_cells(pa), pad_cells(pb)
    res = {"kind": "resistance", "net": net, "from": a, "to": b, "estimate": True}
    if not len(ca) or not len(cb):
        return {**res, "connected": False}
    sa, sb = n, n + 1
    big = 1e6
    rows += [ca, np.full(len(ca), sa), cb, np.full(len(cb), sb)]
    cols += [np.full(len(ca), sa), ca, np.full(len(cb), sb), cb]
    vals += [np.full(len(ca), big)] * 2 + [np.full(len(cb), big)] * 2
    r = np.concatenate(rows)
    c = np.concatenate(cols)
    w = np.concatenate(vals)
    G = sp.coo_matrix((w, (r, c)), shape=(n + 2, n + 2)).tocsr()
    G = G + G.T
    _, label = csg.connected_components(G, directed=False)
    if label[sa] != label[sb]:
        return {**res, "connected": False}
    keep = np.flatnonzero(label == label[sa])
    G = G[keep][:, keep]
    lap = (sp.diags(np.asarray(G.sum(axis=1)).ravel()) - G).tocsr()
    pos = {int(k): i for i, k in enumerate(keep)}
    free = np.array([i for i in range(len(keep)) if i != pos[sb]])
    rhs = np.zeros(len(keep))
    rhs[pos[sa]] = 1.0  # 1 A in at a, b held at 0 V
    volts = sla.spsolve(lap[free][:, free].tocsc(), rhs[free])
    ia = int(np.flatnonzero(free == pos[sa])[0])
    return {
        **res,
        "connected": True,
        "milliohm": round(float(volts[ia]) * 1e3, 2),
        "raster_mm": res_mm,
    }


# ----------------------------------------------------------------------- output


def fmt_xy(p) -> str:
    return f"({p[0]:.2f}, {p[1]:.2f})"


def text(r: dict) -> str:
    k = r["kind"]
    if k == "path":
        head = f"{r['net']} {r['from']} -> {r['to']}"
        if not r["connected"]:
            return f"{head}: no routed path (tracks, vias, pads; the pour is left out)"
        v = r["vias"]
        w = r["narrowest_mm"]
        narrow = "narrowest -, no tracks" if w is None else f"narrowest {w:.3f} mm"
        return (
            f"{head}: {r['length_mm']:.2f} mm of track over {r['tracks']} tracks, "
            f"{v} via{'s' * (v != 1)}, {narrow}, "
            f"layers {'+'.join(r['layers']) or '-'}"
        )
    if k == "cut":
        return (
            f"{r['net']} via {fmt_xy(r['via_at'])} {r['via'][0]}/{r['via'][1]}: "
            f"removing it cuts {', '.join(r['disconnects'])} from {r['from']}"
        )
    if k == "via-in-pad":
        return (
            f"{r['net']} via {fmt_xy(r['via_at'])} {r['via'][0]}/{r['via'][1]} in "
            f"{r['pad']} ({r['pad_net']}, {r['layer']}): {r['overlap']}, centre "
            f"{r['centre_to_pad_mm']:.3f} mm from the pad"
        )
    if k == "hole":
        c = r["clearance_mm"]
        cs = (
            "no copper" if c is None else f"{c:.3f} mm to {r['nearest']} ({r['layer']})"
        )
        return f"{r['hole']} {fmt_xy(r['at'])} d{r['drill']}: {cs}"
    if k == "plane":
        pads = f", THT {', '.join(r['tht_pads'][:6])}" if r["tht_pads"] else ""
        return (
            f"{r['net']} plane {r['status']}: {r['area_mm2']:.1f} mm2 fill in "
            f"{r['zone_islands']} island(s), {r['vias']} via(s){pads}, bbox {r['bbox']}"
        )
    if k == "plane-summary":
        return (
            f"{r['net']}: {r['components']} plane component(s), {r['listed']} at or above "
            f"{r['min_area_mm2']} mm2{'  <-- split' if r['flag'] else ''}"
        )
    if k == "resistance":
        head = f"{r['net']} {r['from']} -> {r['to']}"
        if not r["connected"]:
            return f"{head}: not connected"
        return (
            f"{head}: ~{r['milliohm']:.2f} mOhm DC (estimate: 1 oz Cu, "
            f"{r['raster_mm']} mm raster, tracks + pour + vias)"
        )
    return json.dumps(r)


def review_text(rows: list[dict]) -> list[str]:
    """carrier-review's plane section; never fails."""
    flags = sum(1 for r in rows if r["kind"] == "plane-summary" and r["flag"])
    out = [
        "== plane connectivity: pour + vias + THT pads, no tracks (flag: a second "
        + f"component of {PLANE_MIN_MM2} mm2 or more)"
    ]
    out += [f"   {text(r)}" for r in rows]
    out.append(
        f"== {flags} plane flag(s); this report does not gate (carrier-check does)"
    )
    return out


def main(argv: list[str] | None = None) -> int:
    common = argparse.ArgumentParser(add_help=False)
    common.add_argument("--json", action="store_true", help="print a JSON list")
    ap = argparse.ArgumentParser(
        prog="python3 -m kicad.measure", description=__doc__.split("\n")[0]
    )
    ap.add_argument("dump", help="JSON from tools/kicad/pcb/dump.py")
    sub = ap.add_subparsers(dest="cmd", required=True)
    for name in ("path", "resistance"):
        s = sub.add_parser(name, parents=[common])
        s.add_argument("net")
        s.add_argument("a")
        s.add_argument("b")
        if name == "path":
            s.add_argument("--hops", action="store_true", help="list each hop")
        else:
            s.add_argument(
                "--raster", type=positive_mm, default=RASTER_MM, help="cell size, mm"
            )
    s = sub.add_parser("cuts", parents=[common])
    s.add_argument("net")
    s.add_argument("a")
    sub.add_parser("vias-in-pads", parents=[common])
    sub.add_parser("clearance-to-holes", parents=[common])
    s = sub.add_parser("plane", parents=[common])
    s.add_argument("--min-area", type=float, default=PLANE_MIN_MM2)
    s.add_argument("--review", action="store_true", help="carrier-review's section")
    argv = sys.argv[1:] if argv is None else argv
    if not argv:
        print(f"FAIL: expected <dump.json> <subcommand> [args]\n{__doc__}")
        return 1
    args = ap.parse_args(argv)
    if args.cmd == "plane" and args.review and args.json:
        ap.error("plane: --review prints carrier-review's text; drop it for --json")
    try:
        return run(args)
    except (KicadError, OSError, ValueError) as e:
        print(f"FAIL: {e}")
        return 1
    except KeyError as e:
        print(f"FAIL: {args.dump} is not a pcb/dump.py dump: no key {e}")
        return 1


def positive_mm(s: str) -> float:
    v = float(s)
    if not v > 0:
        raise argparse.ArgumentTypeError(f"must be greater than 0 mm, got {s}")
    return v


def run(args) -> int:
    with open(args.dump, encoding="utf-8") as f:
        board = json.load(f)

    if args.cmd == "path":
        rows = [path(board, args.net, args.a, args.b)]
    elif args.cmd == "resistance":
        rows = [resistance(board, args.net, args.a, args.b, args.raster)]
    elif args.cmd == "cuts":
        rows = cuts(board, args.net, args.a)
    elif args.cmd == "vias-in-pads":
        rows = vias_in_pads(board)
    elif args.cmd == "clearance-to-holes":
        rows = clearance_to_holes(board)
    else:
        rows = plane(board, args.min_area)
        if args.review:
            print("\n".join(review_text(rows)))
            return 0

    if args.json:
        print(json.dumps(rows, indent=1))
    elif rows:
        print("\n".join(text(r) for r in rows))
        if args.cmd == "path" and args.hops and rows[0]["connected"]:
            print("\n".join(f"   {h}" for h in rows[0]["hops"]))
    else:
        print(f"{args.cmd}: none")
    return 0


if __name__ == "__main__":
    sys.exit(main())
