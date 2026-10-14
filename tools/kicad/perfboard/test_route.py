"""Tests for the perfboard channel router and its independent route check.

Small synthetic boards: holes are (column, row); routes are half-pitch paths
(even coordinates are holes, odd ones channels). A numbered wire is
`(n, step, net, colour, (hole_a, label_a), (hole_b, label_b))`.

    python3 -m unittest discover -s tools/kicad -t tools
"""

from __future__ import annotations

import unittest

from kicad.errors import KicadError
from kicad.perfboard.grid import Board
from kicad.perfboard.route import (
    BEND_COST,
    STEP_COST,
    check_routes,
    route_step,
    route_wire,
    route_wires,
)

B4 = Board(4, 4)


def wire(n, a, b, step=0, net="N"):
    return (n, step, net, "#000", (a, f"X{n}.1"), (b, f"X{n}.2"))


def is_hole(p):
    return p[0] % 2 == 0 and p[1] % 2 == 0


class RouteWireTests(unittest.TestCase):
    def test_neighbouring_holes(self):
        cost, path = route_wire(B4, (0, 0), (1, 0), {}, {})
        self.assertEqual(path, [(0, 0), (1, 0), (2, 0)])
        self.assertEqual(cost, 2 * STEP_COST)

    def test_goes_around_a_hole_never_over_it(self):
        # A1 to C1 on the top row: hole B1 is in the way, so the wire drops to
        # the channel below, runs along it and comes back up.
        cost, path = route_wire(B4, (0, 0), (2, 0), {}, {})
        self.assertEqual(path[0], (0, 0))
        self.assertEqual(path[-1], (4, 0))
        self.assertFalse([p for p in path[1:-1] if is_hole(p)])
        self.assertEqual(cost, 6 * STEP_COST + 2 * BEND_COST)
        for a, b in zip(path, path[1:], strict=False):
            self.assertEqual(abs(a[0] - b[0]) + abs(a[1] - b[1]), 1)

    def test_stays_on_the_board(self):
        _, path = route_wire(B4, (0, 0), (3, 3), {}, {})
        for x, y in path:
            self.assertTrue(0 <= x <= 6 and 0 <= y <= 6)

    def test_no_route_when_the_start_hole_is_boxed_in(self):
        # A corner hole has two ways out; earlier wires already leave by both.
        self.assertIsNone(route_wire(B4, (0, 0), (1, 0), {}, {(0, 0): {"E", "S"}}))

    def test_a_used_channel_is_not_shared(self):
        # The only channel out of A1 eastward is taken along the same axis.
        used = {(1, 0): {"H"}}
        _, path = route_wire(B4, (0, 0), (1, 0), used, {})
        self.assertNotIn((1, 0), path)


class RouteStepTests(unittest.TestCase):
    def test_routes_every_wire_and_passes_the_check(self):
        wires = [
            wire(1, (0, 0), (3, 0)),
            wire(2, (0, 3), (3, 3)),
            wire(3, (1, 1), (2, 2)),
        ]
        routes = route_wires(wires, 1, B4)
        self.assertEqual(sorted(routes), [1, 2, 3])
        self.assertEqual(check_routes(wires, routes, 1), [])

    def test_repeatable(self):
        wires = [
            wire(1, (0, 0), (3, 0)),
            wire(2, (0, 3), (3, 3)),
            wire(3, (1, 1), (2, 2)),
            wire(4, (0, 0), (3, 3), step=1),
        ]
        self.assertEqual(route_wires(wires, 2, B4), route_wires(wires, 2, B4))
        self.assertEqual(check_routes(wires, route_wires(wires, 2, B4), 2), [])

    def test_unroutable_step_raises(self):
        # A 2 x 1 board: the second wire between the same two holes has no
        # channel left that the first does not use.
        board = Board(2, 1)
        wires = [wire(1, (0, 0), (1, 0)), wire(2, (0, 0), (1, 0))]
        with self.assertRaisesRegex(KicadError, r"cannot route wires \[1, 2\]"):
            route_step(board, wires)


class CheckRoutesTests(unittest.TestCase):
    """Each rule of check_routes, broken on purpose in a hand-made route."""

    def setUp(self):
        self.w1 = wire(1, (0, 0), (2, 0))
        self.good = [(0, 0), (0, 1), (1, 1), (2, 1), (3, 1), (4, 1), (4, 0)]

    def test_good_route(self):
        self.assertEqual(check_routes([self.w1], {1: self.good}, 1), [])

    def test_no_route(self):
        self.assertEqual(check_routes([self.w1], {}, 1), ["wire 1 has no route"])

    def test_wrong_ends(self):
        errors = check_routes([self.w1], {1: self.good[:-1]}, 1)
        self.assertIn("wire 1 does not run between its own holes", errors)

    def test_jump(self):
        jumped = [(0, 0), (0, 1), (2, 1), (3, 1), (4, 1), (4, 0)]
        errors = check_routes([self.w1], {1: jumped}, 1)
        self.assertIn("wire 1 jumps from (0, 1) to (2, 1)", errors)

    def test_passes_over_a_hole(self):
        over = [(0, 0), (1, 0), (2, 0), (3, 0), (4, 0)]
        errors = check_routes([self.w1], {1: over}, 1)
        self.assertEqual(errors, ["wire 1 passes over hole B1"])

    def test_shared_channel(self):
        w2 = wire(2, (0, 1), (2, 1))
        other = [(0, 2), (1, 2), (1, 1), (2, 1), (3, 1), (3, 2), (4, 2)]
        errors = check_routes([self.w1, w2], {1: self.good, 2: other}, 1)
        self.assertIn("wires 1 and 2 share a channel at (2, 1)", errors)

    def test_meet_at_a_bend(self):
        # Both wires turn at channel crossing (1, 1), each on its own two
        # sides of it: no channel shared, but the turns touch.
        w1 = [(0, 0), (1, 0), (1, 1), (2, 1), (3, 1), (3, 0), (4, 0)]
        w2 = wire(2, (0, 1), (1, 1))
        other = [(0, 2), (0, 1), (1, 1), (1, 2), (2, 2)]
        errors = check_routes([self.w1, w2], {1: w1, 2: other}, 1)
        self.assertEqual(errors, ["wires [1, 2] meet at a bend at (1, 1)"])

    def test_other_steps_are_not_compared(self):
        w2 = wire(2, (0, 0), (2, 0), step=1)
        self.assertEqual(
            check_routes([self.w1, w2], {1: self.good, 2: self.good}, 2), []
        )


if __name__ == "__main__":
    unittest.main()
