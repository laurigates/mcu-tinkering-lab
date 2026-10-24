"""The hole router: each net grown as a tree of wires over the holes, and a
check of the routes that shares none of its code.

Routes are half-pitch paths: even coordinates are holes, odd ones the gaps
between them, so a hole-to-hole segment is named by its midpoint. The rules
are the ones a hand-wired pad-per-hole board follows:

- A wire runs along a row or column of holes, lying over free holes. It never
  lies over a pad, a joint or along a bus: those are soldered.
- A joint is a hole with conductors in it: a pin, a bus, wire ends (pushed
  through beside the pin or each other, or lapped on the pad). It holds at
  most `conductors` of them, the pin counting as one. So with the default 3 a
  pad takes two wire ends, and a free hole three: a splice.
- A bus is a bare tinned wire laid straight along holes and soldered at
  every one; pads of its own net under it join it there, and other pads may
  not be under it.
- A net grows from its buses and pins: the cheapest next pin joins any joint
  of the tree that has room, or splices into one of the tree's own wires at a
  free hole that wire lies over.
- Wires are insulated: they may cross, and lie over wires of earlier steps.
  No wire ends in a hole another wire lies over. Within one step, for a
  readable drawing, no two wires share a segment and none bends on another.
"""

from __future__ import annotations

import heapq
import math
import random
from dataclasses import dataclass, field

from ..errors import KicadError
from .grid import Board, hole_name

# Costs in half-pitch steps: one hole is 2. A bend costs a hole; lying over
# another wire (a crossing, or along an earlier step's wire) or splicing two;
# crossing a bus, over its solder joint, four.
STEP_COST, BEND_COST, CROSS_COST, BUS_CROSS_COST, SPLICE_COST = 1, 2, 4, 8, 4
ORDER_TRIES = 8  # net orders tried per step; the cheapest result is kept
CONDUCTORS = 3  # per hole: a pin, the bus and wire ends each count one
DIRS = ((1, 0), (-1, 0), (0, 1), (0, -1))
SPLICE = "splice"


def bus_label(net):
    return f"{net} bus"


def is_bus(wire):
    """A planned or numbered wire whose ends are labelled as its net's bus."""
    net, a, b = wire[:3] if len(wire) == 4 else (wire[2], wire[4], wire[5])
    return a[1] == b[1] == bus_label(net)


def path_of(holes):
    """Holes, each a neighbour of the last, as a half-pitch path."""
    path = [(2 * holes[0][0], 2 * holes[0][1])]
    for (c, r), (d, e) in zip(holes, holes[1:]):
        path += [(c + d, r + e), (2 * d, 2 * e)]
    return path


def is_hole(node):
    return node[0] % 2 == 0 and node[1] % 2 == 0


def mid(a, b):
    """A half-pitch edge's name: the sum of its two nodes."""
    return (a[0] + b[0], a[1] + b[1])


def axis(a, b):
    return "H" if a[1] == b[1] else "V"


@dataclass
class State:
    """What is laid so far, on half-pitch nodes; copied per try."""

    fixed: set = field(default_factory=set)  # bus edges, never shared
    old: set = field(default_factory=set)  # edges of earlier steps' wires
    segs: set = field(default_factory=set)  # edges of this step's wires
    over: dict = field(
        default_factory=dict
    )  # node -> axes of this step's wires over it
    under: dict = field(default_factory=dict)  # node -> wires of any step over it
    bus_over: dict = field(default_factory=dict)  # node -> net of the bus along it
    load: dict = field(default_factory=dict)  # hole node -> conductors in it

    def copy(self):
        return State(
            set(self.fixed),
            set(self.old),
            set(self.segs),
            {k: set(v) for k, v in self.over.items()},
            dict(self.under),
            dict(self.bus_over),
            dict(self.load),
        )

    def next_step(self):
        self.old |= self.segs
        self.segs, self.over = set(), {}


def lay(state, path):
    """Occupy `path`'s edges, the nodes it lies over and its two end holes."""
    for a, b in zip(path, path[1:]):
        state.segs.add(mid(a, b))
    for a, p in zip(path, path[1:-1]):
        state.over.setdefault(p, set()).add(axis(a, p))
        state.under[p] = state.under.get(p, 0) + 1
    for h in (path[0], path[-1]):
        state.load[h] = state.load.get(h, 0) + 1


class Router:
    def __init__(self, parts, nets, board: Board, conductors=CONDUCTORS):
        self.board, self.conductors = board, conductors
        self.owner = {
            h: f"{p.ref}.{n}" for p in parts.values() for n, h in p.pads.items()
        }
        self.pad_net = {
            parts[ref].pads[pin]: net
            for net, nodes in nets.items()
            for ref, pin in nodes
        }
        self.pins = {
            net: [parts[ref].pads[pin] for ref, pin in nodes]
            for net, nodes in nets.items()
        }

    def initial(self):
        return State(load={(2 * c, 2 * r): 1 for c, r in self.owner})

    def label(self, node, net, state):
        hole = (node[0] // 2, node[1] // 2)
        if hole in self.owner:
            return self.owner[hole]
        return bus_label(net) if state.bus_over.get(node) == net else SPLICE

    def free(self, state, hole):
        """Room in `hole` (a node) for one more wire end."""
        return state.load.get(hole, 0) < self.conductors and not state.under.get(hole)

    def lay_bus(self, state, net, a, b):
        """The bus wire from hole `a` to `b`, laid; raises if it may not be."""
        (c0, r0), (c1, r1) = a, b
        n = max(abs(c1 - c0), abs(r1 - r0))
        dc, dr = (c1 > c0) - (c1 < c0), (r1 > r0) - (r1 < r0)
        holes = [(c0 + i * dc, r0 + i * dr) for i in range(n + 1)]
        for h in holes:
            if not (0 <= h[0] < self.board.cols and 0 <= h[1] < self.board.rows):
                raise KicadError(f"bus {net} runs off the board at {hole_name(*h)}")
            if h in self.owner and self.pad_net.get(h) != net:
                raise KicadError(
                    f"bus {net} runs over {self.owner[h]} ({hole_name(*h)}), "
                    f"on {self.pad_net.get(h) or 'no net'}"
                )
            if (2 * h[0], 2 * h[1]) in state.bus_over:
                raise KicadError(f"bus {net} meets another bus at {hole_name(*h)}")
        path = path_of(holes)
        for p, q in zip(path, path[1:]):
            state.fixed.add(mid(p, q))
        for p in path:
            state.bus_over[p] = net
        for p in path[::2]:
            state.load[p] = state.load.get(p, 0) + 1
        return (net, (a, bus_label(net)), (b, bus_label(net)), path), set(path[::2])

    def search(self, state, sources, goals):
        """Cheapest (cost, path) from any source hole to any goal hole, on
        half-pitch nodes.

        `sources` maps a hole node to its starting cost. Every node in between
        lies under the wire: a gap between holes, or a free hole; a wire there
        already is crossed square-on.
        """
        gx, gy = 2 * self.board.cols - 2, 2 * self.board.rows - 2
        heap, best, prev, tie = [], {}, {}, 0
        for h, c in sources.items():
            heap.append((c, tie, h, None))
            best[(h, None)] = c
            tie += 1
        heapq.heapify(heap)
        while heap:
            cost, _, p, came = heapq.heappop(heap)
            if cost > best[(p, came)]:
                continue
            if p in goals and came is not None:
                path, key = [p], (p, came)
                while key in prev:
                    key = prev[key]
                    path.append(key[0])
                return cost, path[::-1]
            straight = came is not None and (p in state.over or p in state.bus_over)
            for d in DIRS:
                if came is not None and d == (-came[0], -came[1]):
                    continue
                if straight and d != came:
                    continue
                n = (p[0] + d[0], p[1] + d[1])
                if not (0 <= n[0] <= gx and 0 <= n[1] <= gy):
                    continue
                m = (p[0] + n[0], p[1] + n[1])  # mid(p, n), inline: the hot loop
                if m in state.segs or m in state.fixed:
                    continue
                step = STEP_COST + (BEND_COST if came and d != came else 0)
                if m in state.old:
                    step += CROSS_COST
                if n not in goals:
                    if not (n[0] | n[1]) & 1 and (  # is_hole(n), inline
                        (n[0] // 2, n[1] // 2) in self.owner
                        or state.load.get(n, 0) > (n in state.bus_over)
                    ):
                        continue  # a pad or a joint
                    if n in state.bus_over:
                        step += BUS_CROSS_COST
                    elif state.under.get(n):
                        step += CROSS_COST
                key = (n, d)
                if cost + step < best.get(key, math.inf):
                    best[key] = cost + step
                    prev[key] = (p, came)
                    tie += 1
                    heapq.heappush(heap, (cost + step, tie, n, d))
        return None

    def splice(self, state, planned, i, node):
        """Cut wire `i` at `node`, a free hole it lies over, into two."""
        net, a, b, path = planned[i]
        k = path.index(node)
        state.over[node].discard(axis(path[k - 1], node))
        if not state.over[node]:
            del state.over[node]
        state.under[node] -= 1
        state.load[node] = state.load.get(node, 0) + 2
        end = ((node[0] // 2, node[1] // 2), SPLICE)
        planned[i : i + 1] = [
            (net, a, end, path[: k + 1]),
            (net, end, b, path[k:]),
        ]

    def grow(self, state, net, planned, groups):
        """Join `groups` (sets of holes already joined among themselves) into
        one tree, appending wires to `planned`; returns the cost or None."""
        tree, rest, total = set(groups[0]), [set(g) for g in groups[1:]], 0
        while rest:
            sources = {h: 0 for h in tree if self.free(state, h)}
            spliceable = {}
            # A splice holds the two cut ends and the new one: conductors >= 3.
            for i, w in enumerate(planned):
                if w[0] != net or is_bus(w):
                    continue
                for h in w[3][1:-1]:
                    if is_hole(h) and state.under.get(h) == 1 and h in state.over:
                        spliceable[h] = i
                        sources.setdefault(h, SPLICE_COST)
            goals = {h for g in rest for h in g if self.free(state, h)}
            found = self.search(state, sources, goals)
            if found is None:
                return None
            cost, path = found
            total += cost
            start, end = path[0], path[-1]
            if start not in tree:
                self.splice(state, planned, spliceable[start], start)
                tree.add(start)
            lay(state, path)
            planned.append(
                (
                    net,
                    ((start[0] // 2, start[1] // 2), self.label(start, net, state)),
                    ((end[0] // 2, end[1] // 2), self.label(end, net, state)),
                    path,
                )
            )
            joined = next(g for g in rest if end in g)
            rest.remove(joined)
            tree |= joined
        return total

    def route_net(self, state, net, planned, buses):
        """Grow `net`'s tree from its laid `buses` (sets of holes) and its
        pins; returns the cost or None."""
        merged = []
        pins = [{(2 * c, 2 * r)} for c, r in self.pins[net]]
        for g in [set(b) for b in buses] + pins:
            # Buses that meet, or pins under a bus, are already one group.
            for m in [m for m in merged if m & g]:
                merged.remove(m)
                g |= m
            merged.append(g)
        return self.grow(state, net, planned, merged)


def plan(
    parts,
    nets,
    steps,
    board: Board,
    buses=None,
    conductors=CONDUCTORS,
    tries=ORDER_TRIES,
):
    """Every net's wires, step by step: [(net, (hole, label), (hole, label), path)].

    `buses` maps a net to its ((hole, hole), ...) bus wires, laid before any
    other wire. Each step is routed in its own net order first, then in seeded
    shuffles of it, and the cheapest is kept, so a run is repeatable.
    """
    router = Router(parts, nets, board, conductors)
    state, bus_wires, bus_holes = router.initial(), {}, {}
    for net, ends in sorted((buses or {}).items()):
        for a, b in ends:
            wire, holes = router.lay_bus(state, net, a, b)
            bus_wires.setdefault(net, []).append(wire)
            bus_holes.setdefault(net, []).append(holes)
    planned = []
    for step, s in enumerate(steps):
        rng = random.Random(step)
        orders = [list(s.nets)] + [
            rng.sample(s.nets, len(s.nets)) for _ in range(tries - 1)
        ]
        best = None
        for order in orders:
            st, wires, total = state.copy(), [], 0
            for net in order:
                wires += bus_wires.get(net, [])
                cost = router.route_net(st, net, wires, bus_holes.get(net, []))
                if cost is None:
                    break
                total += cost
            else:
                if best is None or total < best[0]:
                    best = (total, st, wires)
        if best is None:
            raise KicadError(f"step {s.title!r}: cannot route nets {list(s.nets)}")
        _, state, wires = best
        state.next_step()
        planned += wires
    return planned


def check_routes(numbered, routes, parts, nets, conductors=CONDUCTORS):
    """Independent of the router: re-walk each path against the routing rules."""
    pad_net = {parts[r].pads[p]: net for net, nodes in nets.items() for r, p in nodes}
    pads = {h for p in parts.values() for h in p.pads.values()}
    errors = []
    load = {h: 1 for h in pads}
    edges = {}  # (step, edge) -> wire; a bus's step is None: no wire shares it
    laid = {}  # (step, node) -> [(wire, bends there)]
    under, ends, bus_at = {}, {}, {}
    for n, step, net, _, (ha, la), (hb, lb) in numbered:
        path = routes.get(n)
        if not path:
            errors.append(f"wire {n} has no route")
            continue
        bus = la == lb == bus_label(net)
        if path[0] != (2 * ha[0], 2 * ha[1]) or path[-1] != (2 * hb[0], 2 * hb[1]):
            errors.append(f"wire {n} does not run between its own holes")
        for a, b in zip(path, path[1:]):
            if abs(a[0] - b[0]) + abs(a[1] - b[1]) != 1:
                errors.append(f"wire {n} jumps from {a} to {b}")
            e = (a[0] + b[0], a[1] + b[1])
            other = edges.get((None if bus else step, e)) or edges.get((None, e))
            if bus and other is None:
                other = next((w for (s, f), w in edges.items() if f == e), None)
            if other:
                errors.append(f"wires {other} and {n} share a segment at {a}")
            edges[(None if bus else step, e)] = n
        for i, p in enumerate(path):
            h = (p[0] // 2, p[1] // 2) if p[0] % 2 == 0 and p[1] % 2 == 0 else None
            if bus:
                bus_at[p] = n
                if h is None:
                    continue
                if h in pads and pad_net.get(h) != net:
                    errors.append(
                        f"bus {n} runs over pad {hole_name(*h)} of another net"
                    )
                load[h] = load.get(h, 0) + 1
            elif 0 < i < len(path) - 1:
                if h in pads:
                    errors.append(f"wire {n} lies over pad {hole_name(*h)}")
                a, b = path[i - 1], path[i + 1]
                bend = (a[0] == p[0]) != (p[0] == b[0])
                laid.setdefault((step, p), []).append((n, bend))
                if h is not None:
                    under.setdefault(h, []).append(n)
            else:
                h = (p[0] // 2, p[1] // 2)
                ends.setdefault(h, []).append(n)
                load[h] = load.get(h, 0) + 1
    for (_, p), v in laid.items():
        if (len(v) > 1 or p in bus_at) and any(bend for _, bend in v):
            names = [n for n, _ in v] + ([bus_at[p]] if p in bus_at else [])
            errors.append(f"wires {names} meet at a bend at {p}")
    for h, over in under.items():
        if h in ends:
            errors.append(
                f"wires {ends[h]} end under wire {over[0]} at {hole_name(*h)}"
            )
    for h, c in sorted(load.items()):
        if c > conductors:
            errors.append(
                f"{hole_name(*h)} holds {c} conductors, more than {conductors}"
            )
    return errors
