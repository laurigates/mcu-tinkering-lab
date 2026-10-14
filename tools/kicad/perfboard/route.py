"""The channel router, and a check of its routes that shares none of its code.

Maze routing runs on a half-pitch grid: even coordinates are holes, odd ones
the channels between them. A wire leaves its hole by a half-pitch stub, runs
in channels, and never passes over any other hole. Within one build step two
wires may cross but never share a channel; a bend cannot sit on a crossing.
Wires in different steps are not checked against each other: each panel is
drawn on its own, and insulated wires may cross on the board.
"""

from __future__ import annotations

import heapq
import math
import random

from ..errors import KicadError
from .grid import Board, hole_name

# Costs in half-pitch steps. Swept on the balancebot board, a crossing cost of
# 60 gives 18 crossings against 25 at 12, for 3 % more wire.
STEP_COST, BEND_COST, CROSS_COST = 1, 4, 60
ORDER_TRIES = 24  # wire orders tried per step; the cheapest result is kept
DIRS = {"E": (1, 0), "W": (-1, 0), "S": (0, 1), "N": (0, -1)}
AXIS = {"E": "H", "W": "H", "S": "V", "N": "V"}
BACK = {"E": "W", "W": "E", "S": "N", "N": "S"}


def route_wire(board: Board, start, goal, used, stubs):
    """(cost, path) of the cheapest channel route from hole `start` to hole
    `goal`, or None if there is none.

    `used` maps a channel node to the axes ("H", "V") already run through it,
    `stubs` maps a hole to the directions its earlier wires leave by.
    """
    gx, gy = 2 * board.cols - 2, 2 * board.rows - 2
    s, g = (2 * start[0], 2 * start[1]), (2 * goal[0], 2 * goal[1])
    heap = [(0, 0, s, "")]  # cost, tie-break, node, direction arrived in
    best, prev, tie = {(s, ""): 0}, {}, 0
    while heap:
        cost, _, node, came = heapq.heappop(heap)
        if node == g:
            path, key = [node], (node, came)
            while key in prev:
                key = prev[key]
                path.append(key[0])
            return cost, path[::-1]
        if cost > best[(node, came)]:
            continue
        for d, (dx, dy) in DIRS.items():
            if came and d == BACK[came]:
                continue
            nxt = (node[0] + dx, node[1] + dy)
            if not (0 <= nxt[0] <= gx and 0 <= nxt[1] <= gy):
                continue
            if node == s:
                if d in stubs.get(s, ()):
                    continue
            else:
                axes = {AXIS[came], AXIS[d]}
                if used.get(node, set()) & axes or (len(axes) == 2 and used.get(node)):
                    continue
            step = STEP_COST + (BEND_COST if came and d != came else 0)
            if nxt[0] % 2 == 0 and nxt[1] % 2 == 0:  # a hole
                if nxt != g or BACK[d] in stubs.get(g, ()):
                    continue
            elif used.get(nxt):
                step += CROSS_COST
            key = (nxt, d)
            if cost + step < best.get(key, math.inf):
                best[key] = cost + step
                prev[key] = (node, came)
                tie += 1
                heapq.heappush(heap, (cost + step, tie, nxt, d))
    return None


def route_step(board: Board, wires):
    """Route one step's wires in the given order; a wire that finds no path
    moves to the front and the step starts over. Returns (total cost, routes)."""
    order = list(wires)
    for _ in range(len(order) + 1):
        used, stubs, routes, total = {}, {}, {}, 0
        for w in order:
            found = route_wire(board, w[4][0], w[5][0], used, stubs)
            if found is None:
                order.remove(w)
                order.insert(0, w)
                break
            cost, path = found
            total += cost
            routes[w[0]] = path
            for a, b in ((path[0], path[1]), (path[-1], path[-2])):
                d = next(k for k, v in DIRS.items() if (a[0] + v[0], a[1] + v[1]) == b)
                stubs.setdefault(a, set()).add(d)
            for i in range(1, len(path) - 1):
                axes = {
                    "H" if path[i - 1][1] == path[i][1] else "V",
                    "H" if path[i][1] == path[i + 1][1] else "V",
                }
                used.setdefault(path[i], set()).update(axes)
        else:
            return total, routes
    raise KicadError(
        f"cannot route wires {[w[0] for w in wires]} without sharing a channel"
    )


def route_wires(numbered, nsteps, board: Board, tries=ORDER_TRIES):
    """Route each step in several wire orders and keep the cheapest. Shortest
    first is tried first; the rest are seeded shuffles, so a run is repeatable."""
    routes = {}
    for step in range(nsteps):
        wires = sorted(
            (w for w in numbered if w[1] == step),
            key=lambda w: (
                abs(w[4][0][0] - w[5][0][0]) + abs(w[4][0][1] - w[5][0][1]),
                w[0],
            ),
        )
        rng = random.Random(step)
        orders = [wires] + [rng.sample(wires, len(wires)) for _ in range(tries - 1)]
        routes.update(
            min((route_step(board, o) for o in orders), key=lambda r: r[0])[1]
        )
    return routes


def check_routes(numbered, routes, nsteps):
    """Independent of the router: re-walk each path against the routing rules."""
    errors = []
    for step in range(nsteps):
        seen = {}  # half-pitch edge -> wire number
        visits = {}  # channel node -> [(wire number, bends there)]
        for n, st, _, _, (ha, _), (hb, _) in numbered:
            if st != step:
                continue
            path = routes.get(n)
            if not path:
                errors.append(f"wire {n} has no route")
                continue
            if path[0] != (2 * ha[0], 2 * ha[1]) or path[-1] != (2 * hb[0], 2 * hb[1]):
                errors.append(f"wire {n} does not run between its own holes")
            for a, b in zip(path, path[1:]):
                if abs(a[0] - b[0]) + abs(a[1] - b[1]) != 1:
                    errors.append(f"wire {n} jumps from {a} to {b}")
                edge = frozenset((a, b))
                if edge in seen:
                    errors.append(f"wires {seen[edge]} and {n} share a channel at {a}")
                seen[edge] = n
            for a, p, b in zip(path, path[1:], path[2:]):
                if p[0] % 2 == 0 and p[1] % 2 == 0:
                    errors.append(
                        f"wire {n} passes over hole {hole_name(p[0] // 2, p[1] // 2)}"
                    )
                bend = (a[0] == p[0]) != (p[0] == b[0])
                visits.setdefault(p, []).append((n, bend))
        for p, v in visits.items():
            if len(v) > 1 and any(bend for _, bend in v):
                errors.append(f"wires {[n for n, _ in v]} meet at a bend at {p}")
    return errors
