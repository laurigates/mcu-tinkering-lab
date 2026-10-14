"""Wires: a minimum spanning tree per net, its independent check, and numbering.

A wire is `(net, (hole_a, "REF.pin"), (hole_b, "REF.pin"))`. A numbered wire
is `(n, step, net, colour, (hole_a, label_a), (hole_b, label_b))`, numbered
1..N in build-step order.
"""

from __future__ import annotations

from .config import Step
from .grid import hole_name
from .render_svg import PALETTE


def plan_wires(parts, nets):
    """Minimum spanning tree per net, Manhattan distance, on pad holes only."""
    wires = []
    for name, nodes in sorted(nets.items()):
        pts = [(parts[ref].pads[pin], f"{ref}.{pin}") for ref, pin in nodes]
        tree, rest = [pts[0]], pts[1:]
        while rest:
            best = min(
                (
                    (abs(a[0][0] - b[0][0]) + abs(a[0][1] - b[0][1]), a, b)
                    for a in tree
                    for b in rest
                ),
                key=lambda t: (t[0], t[1][1], t[2][1]),
            )
            _, a, b = best
            wires.append((name, a, b))
            tree.append(b)
            rest.remove(b)
    return wires


def check_wires(parts, nets, wires):
    """Independent of plan_wires: union-find over the holes the wires join."""
    owner = {hole: f"{p.ref}.{n}" for p in parts.values() for n, hole in p.pads.items()}
    parent = {}

    def find(x):
        parent.setdefault(x, x)
        while parent[x] != x:
            parent[x] = parent[parent[x]]
            x = parent[x]
        return x

    for _, (ha, _), (hb, _) in wires:
        parent[find(ha)] = find(hb)
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
    for _, (ha, la), (hb, lb) in wires:
        for hole, label in ((ha, la), (hb, lb)):
            if owner.get(hole) != label:
                errors.append(
                    f"wire end {label} lands on {hole_name(*hole)} owned by {owner.get(hole)}"
                )
    return errors


def number_wires(nets, wires, steps: tuple[Step, ...]):
    """Group the planned wires by build step and number them 1..N in that order.

    Within a step, nets take the palette's slots in the step's order. The
    steps must already cover `nets` exactly (`config.check_steps`).
    """
    by_net = {}
    for w in wires:
        by_net.setdefault(w[0], []).append(w)
    numbered, n = [], 0
    for step, s in enumerate(steps):
        for slot, net in enumerate(s.nets):
            for _, a, b in by_net[net]:
                n += 1
                numbered.append((n, step, net, PALETTE[slot], a, b))
    return numbered
