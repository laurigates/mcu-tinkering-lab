"""Tests for the perfboard hole router and its independent route check.

Small synthetic boards: holes are (column, row); routes are half-pitch paths
(even coordinates are holes, odd ones the gaps between them). Each pin is a
one-pad part named after its hole. A numbered wire is
`(n, step, net, colour, (hole_a, label_a), (hole_b, label_b))`.

    python3 -m unittest discover -s tools/kicad -t tools
"""

from __future__ import annotations

import unittest

from kicad.errors import KicadError
from kicad.perfboard.config import Step
from kicad.perfboard.grid import Board
from kicad.perfboard.place import Part
from kicad.perfboard.route import (
    SPLICE,
    bus_label,
    check_routes,
    is_bus,
    path_of,
    plan,
)
from kicad.perfboard.wires import check_wires, number_wires

B5 = Board(5, 5)


def board(**nets):
    """(parts, nets) from net name -> [hole, ...]; a net named "_" is unconnected pads."""
    parts, out = {}, {}
    for net, holes in nets.items():
        for c, r in holes:
            ref = f"P{c}_{r}"
            parts[ref] = Part(ref, "", "Lib:Fp", {"1": (c, r)}, (c, r, c, r))
            if net != "_":
                out.setdefault(net, []).append((ref, "1"))
    return parts, out


def run(parts, nets, b=B5, buses=None, conductors=3, steps=None):
    steps = steps or (Step("all", "", tuple(nets)),)
    planned = plan(parts, nets, steps, b, buses, conductors, tries=4)
    numbered, routes = number_wires(nets, planned, steps)
    return planned, numbered, routes


def bends(path):
    return sum(
        (a[0] == p[0]) != (p[0] == b[0]) for a, p, b in zip(path, path[1:], path[2:])
    )


def wire(n, a, b, step=0, net="N", la=None, lb=None):
    return (n, step, net, "#000", (a, la or f"X{n}.1"), (b, lb or f"X{n}.2"))


class PlanTests(unittest.TestCase):
    def test_straight_over_free_holes(self):
        # A1 to E1: B1..D1 are free, so the wire lies straight over them.
        parts, nets = board(N=[(0, 0), (4, 0)])
        planned, numbered, routes = run(parts, nets)
        self.assertEqual(len(planned), 1)
        self.assertEqual(routes[1], path_of([(c, 0) for c in range(5)]))
        self.assertEqual(check_routes(numbered, routes, parts, nets), [])

    def test_never_over_another_pad(self):
        # C1 is another net's pad: the wire goes round it, between the holes.
        parts, nets = board(N=[(0, 0), (4, 0)], _=[(2, 0)])
        planned, numbered, routes = run(parts, nets)
        self.assertNotIn((4, 0), routes[1])
        self.assertEqual(check_routes(numbered, routes, parts, nets), [])
        self.assertEqual(check_wires(parts, nets, planned), [])

    def test_splice_into_the_tree(self):
        # C1 joins the A3-E3 wire at C3, a free hole it lies over: a T.
        parts, nets = board(N=[(0, 2), (4, 2), (2, 0)])
        planned, numbered, routes = run(parts, nets)
        labels = [lab for _, a, b, _ in planned for _, lab in (a, b)]
        self.assertEqual(labels.count(SPLICE), 3)
        self.assertEqual(len(planned), 3)
        self.assertEqual(sum(bends(p) for p in routes.values()), 0)
        self.assertEqual(check_wires(parts, nets, planned), [])
        self.assertEqual(check_routes(numbered, routes, parts, nets), [])

    def test_a_pad_takes_two_wire_ends(self):
        # C3 is the nearest joint for all four outer pins, but holds the pin
        # and two ends at most.
        parts, nets = board(N=[(2, 2), (0, 2), (4, 2), (2, 0), (2, 4)])
        planned, numbered, routes = run(parts, nets)
        ends = [h for _, a, b, _ in planned for h, _ in (a, b)]
        self.assertLessEqual(ends.count((2, 2)), 2)
        self.assertEqual(check_routes(numbered, routes, parts, nets), [])
        self.assertEqual(check_wires(parts, nets, planned), [])

    def test_bus_joins_pads_under_it_and_others_wire_to_it(self):
        parts, nets = board(G=[(0, 2), (4, 2), (1, 0), (3, 4)])
        buses = {"G": (((0, 2), (4, 2)),)}
        planned, numbered, routes = run(parts, nets, buses=buses)
        bus = [w for w in planned if is_bus(w)]
        self.assertEqual(len(bus), 1)
        self.assertEqual(len(planned), 3)  # the bus, then one wire per pin off it
        for _, a, b, path in planned:
            if not is_bus((_, a, b, path)):
                self.assertEqual(len(path), 5)  # two holes straight down to the bus
        self.assertEqual(check_wires(parts, nets, planned), [])
        self.assertEqual(check_routes(numbered, routes, parts, nets), [])

    def test_bus_over_another_net_raises(self):
        parts, nets = board(G=[(0, 2), (4, 2)], V=[(2, 2), (2, 4)])
        with self.assertRaisesRegex(
            KicadError, r"bus G runs over P2_2\.1 \(C3\), on V"
        ):
            run(parts, nets, buses={"G": (((0, 2), (4, 2)),)})

    def test_later_steps_may_cross_and_lie_over_earlier_wires(self):
        parts, nets = board(A=[(0, 2), (4, 2)], B=[(2, 0), (2, 4)])
        steps = (Step("a", "", ("A",)), Step("b", "", ("B",)))
        _, numbered, routes = run(parts, nets, steps=steps)
        self.assertEqual(sum(bends(p) for p in routes.values()), 0)
        self.assertEqual(check_routes(numbered, routes, parts, nets), [])

    def test_repeatable(self):
        parts, nets = board(A=[(0, 0), (4, 4), (0, 4)], B=[(4, 0), (1, 2)])
        self.assertEqual(run(parts, nets), run(parts, nets))

    def test_boxed_in_pin_raises(self):
        # A one-column board: A2, another net's pad, walls A1 off from A3.
        parts, nets = board(N=[(0, 0), (0, 2)], _=[(0, 1)])
        with self.assertRaisesRegex(
            KicadError, r"step 'all': cannot route nets \['N'\]"
        ):
            run(parts, nets, b=Board(1, 3))


class CheckRoutesTests(unittest.TestCase):
    """Each rule of check_routes, broken on purpose in a hand-made route."""

    def setUp(self):
        self.parts, self.nets = board(N=[(0, 0), (2, 0)], M=[(0, 2), (2, 2)])
        self.w1 = wire(1, (0, 0), (2, 0), la="P0_0.1", lb="P2_0.1")
        self.straight = path_of([(0, 0), (1, 0), (2, 0)])

    def check(self, wires, routes):
        return check_routes(wires, routes, self.parts, self.nets)

    def test_good_route(self):
        self.assertEqual(self.check([self.w1], {1: self.straight}), [])

    def test_no_route(self):
        self.assertEqual(self.check([self.w1], {}), ["wire 1 has no route"])

    def test_wrong_ends(self):
        errors = self.check([self.w1], {1: self.straight[:-1]})
        self.assertIn("wire 1 does not run between its own holes", errors)

    def test_jump(self):
        errors = self.check([self.w1], {1: [(0, 0), (2, 0), (3, 0), (4, 0)]})
        self.assertIn("wire 1 jumps from (0, 0) to (2, 0)", errors)

    def test_lies_over_a_pad(self):
        parts, nets = board(N=[(0, 0), (2, 0)], _=[(1, 0)])
        errors = check_routes([self.w1], {1: self.straight}, parts, nets)
        self.assertEqual(errors, ["wire 1 lies over pad B1"])

    def test_shared_segment_in_one_step(self):
        w2 = wire(2, (0, 0), (2, 0))
        errors = self.check([self.w1, w2], {1: self.straight, 2: self.straight})
        self.assertIn("wires 1 and 2 share a segment at (0, 0)", errors)

    def test_other_steps_may_share(self):
        w2 = wire(2, (0, 0), (2, 0), step=1, la="P0_0.1", lb="P2_0.1")
        self.assertEqual(
            self.check([self.w1, w2], {1: self.straight, 2: self.straight}), []
        )

    def test_end_under_a_wire(self):
        # Wire 2 ends at B1, which wire 1 lies over.
        w2 = wire(2, (1, 2), (1, 0), step=1, net="N", la="x", lb=SPLICE)
        errors = self.check(
            [self.w1, w2], {1: self.straight, 2: path_of([(1, 2), (1, 1), (1, 0)])}
        )
        self.assertIn("wires [2] end under wire 1 at B1", errors)

    def test_meet_at_a_bend(self):
        # Wire 2 turns at the gap between A1 and B1, which wire 1 runs along.
        w2 = wire(2, (0, 1), (1, 0))
        bent = [(0, 2), (1, 2), (1, 1), (1, 0), (2, 0)]
        errors = self.check([self.w1, w2], {1: self.straight, 2: bent})
        self.assertIn("wires [1, 2] meet at a bend at (1, 0)", errors)

    def test_overfilled_hole(self):
        wires = [wire(n, (0, 0), (1, 0) if n % 2 else (0, 1)) for n in (1, 2, 3)]
        routes = {
            1: [(0, 0), (1, 0), (2, 0)],
            2: [(0, 0), (0, 1), (0, 2)],
            3: [(0, 0), (-1, 0), (-1, 1), (0, 1), (1, 1), (2, 1), (2, 0)],
        }
        wires[2] = wire(3, (0, 0), (1, 0), step=1)
        errors = self.check(wires, routes)
        self.assertIn("A1 holds 4 conductors, more than 3", errors)

    def test_bus_over_another_net(self):
        bus = wire(1, (0, 2), (2, 2), net="N", la=bus_label("N"), lb=bus_label("N"))
        errors = self.check([bus], {1: path_of([(0, 2), (1, 2), (2, 2)])})
        self.assertIn("bus 1 runs over pad A3 of another net", errors)


if __name__ == "__main__":
    unittest.main()
