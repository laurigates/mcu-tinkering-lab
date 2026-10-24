"""Wires: the router's plan checked independently, then numbered.

A planned wire is `(net, (hole_a, label_a), (hole_b, label_b), path)`, a label
being "REF.pin", "splice" (a free hole where wire ends meet) or "<net> bus". A numbered wire
is `(n, step, net, colour, (hole_a, label_a), (hole_b, label_b))`, numbered
1..N in build-step order.
"""

from __future__ import annotations

from .config import Step
from .grid import hole_name
from .render_svg import PALETTE
from .route import SPLICE, bus_label


def check_wires(parts, nets, planned):
    """Independent of the router: union-find over the holes the wires join.

    A wire joins its two end holes; a bus every hole it is laid along.
    """
    owner = {hole: f"{p.ref}.{n}" for p in parts.values() for n, hole in p.pads.items()}
    parent = {}

    def find(x):
        parent.setdefault(x, x)
        while parent[x] != x:
            parent[x] = parent[parent[x]]
            x = parent[x]
        return x

    for name, (ha, la), (hb, lb), path in planned:
        joined = (
            [(x // 2, y // 2) for x, y in path[::2]]
            if la == lb == bus_label(name)
            else [hb]
        )
        for h in joined:
            parent[find(ha)] = find(h)
    errors = []
    group_net = {}
    for name, nodes in nets.items():
        roots = {find(parts[ref].pads[pin]) for ref, pin in nodes}
        if len(roots) != 1:
            errors.append(f"net {name} is split into {len(roots)} groups")
        for root in roots:
            if root in group_net and group_net[root] != name:
                errors.append(f"nets {name} and {group_net[root]} are joined")
            group_net[root] = name
    for name, (ha, la), (hb, lb), _ in planned:
        for hole, label in ((ha, la), (hb, lb)):
            if la == lb == bus_label(name):
                ok = True  # a bus: what it lies over is the union-find's
            elif hole in owner:
                ok = owner[hole] == label
            else:
                ok = label in (SPLICE, bus_label(name))
            if not ok:
                errors.append(
                    f"wire end {label} lands on {hole_name(*hole)} owned by {owner.get(hole)}"
                )
    return errors


def number_wires(nets, wires, steps: tuple[Step, ...]):
    """Group the planned wires by build step and number them 1..N in that order;
    returns (numbered wires, route by wire number).

    Within a step, nets take the palette's slots in the step's order. The
    steps must already cover `nets` exactly (`config.check_steps`).
    """
    by_net = {}
    for w in wires:
        by_net.setdefault(w[0], []).append(w)
    numbered, routes, n = [], {}, 0
    for step, s in enumerate(steps):
        for slot, net in enumerate(s.nets):
            for _, a, b, path in by_net.get(net, []):
                n += 1
                numbered.append((n, step, net, PALETTE[slot], a, b))
                routes[n] = path
    return numbered, routes
