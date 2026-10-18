"""tools/kicad/pcb/fanout.py: candidate via centres beside a pad.

`unittest discover` does not reach pcb/, so this test loads `pcb/fanout.py` by
path, as test_dsn.py does for dsn.py.
"""

from __future__ import annotations

import importlib.util
import math
import unittest
from pathlib import Path

HERE = Path(__file__).resolve().parent


def load_fanout():
    spec = importlib.util.spec_from_file_location(
        "kicad_pcb_fanout", HERE / "pcb/fanout.py"
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


fanout = load_fanout()


class EdgeDistanceTests(unittest.TestCase):
    def test_axes_and_diagonal(self):
        self.assertEqual(fanout.edge_distance(1.0, 0.5, 1.0, 0.0), 1.0)
        self.assertEqual(fanout.edge_distance(1.0, 0.5, 0.0, -1.0), 0.5)
        # a 45 degree ray leaves a 2 x 1 box through its long side
        s = math.sqrt(0.5)
        self.assertAlmostEqual(fanout.edge_distance(1.0, 0.5, s, s), 0.5 / s)


class CandidateTests(unittest.TestCase):
    def test_nearest_ring_first_in_ray_order(self):
        c = fanout.candidates(10.0, 20.0, 1.0, 0.5, 0.3, 0.2, 2.0, 0.25)
        # the first ring: every ray at gap + radius (0.5) beyond the edge
        self.assertEqual(len(c[: fanout.DIRECTIONS]), fanout.DIRECTIONS)
        x, y = c[0]
        self.assertAlmostEqual(x, 11.5)
        self.assertAlmostEqual(y, 20.0)
        x, y = c[4]  # ray 4 is +y (y down on a board)
        self.assertAlmostEqual(x, 10.0)
        self.assertAlmostEqual(y, 21.0)
        x, y = c[fanout.DIRECTIONS]  # the second ring starts at ray 0, one step out
        self.assertAlmostEqual(x, 11.75)

    def test_reach_bounds_the_rings(self):
        # rings at 0.5, 0.75, ... 2.0 beyond the edge: seven of them
        c = fanout.candidates(0.0, 0.0, 1.0, 1.0, 0.3, 0.2, 2.0, 0.25)
        self.assertEqual(len(c), 7 * fanout.DIRECTIONS)
        self.assertEqual(fanout.candidates(0.0, 0.0, 1.0, 1.0, 0.3, 0.2, 0.4, 0.25), [])

    def test_prefer_orders_each_ring(self):
        # away from a part to the left (-x): the +x ray first, then its neighbours
        c = fanout.candidates(
            0.0, 0.0, 1.0, 1.0, 0.3, 0.2, 2.0, 0.25, prefer=(1.0, 0.0)
        )
        self.assertAlmostEqual(c[0][1], 0.0)
        self.assertGreater(c[0][0], 0)
        # toward -y (up on a board): ray 12 first
        c = fanout.candidates(
            0.0, 0.0, 1.0, 1.0, 0.3, 0.2, 2.0, 0.25, prefer=(0.0, -1.0)
        )
        self.assertAlmostEqual(c[0][0], 0.0)
        self.assertLess(c[0][1], 0)
        # the preference only reorders a ring: it never tries one step out first
        plain = fanout.candidates(0.0, 0.0, 1.0, 1.0, 0.3, 0.2, 2.0, 0.25)
        ring = fanout.DIRECTIONS
        self.assertEqual(sorted(c[:ring]), sorted(plain[:ring]))

    def test_deterministic(self):
        args = (3.0, 4.0, 0.6, 0.7, 0.3, 0.2, 1.5, 0.25)
        self.assertEqual(fanout.candidates(*args), fanout.candidates(*args))


if __name__ == "__main__":
    unittest.main()
