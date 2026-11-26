"""tools/kicad/pcb/fanout.py: candidate via centres beside a pad.

`unittest discover` does not reach pcb/, so this test loads `pcb/fanout.py` by
path, as test_dsn.py does for dsn.py.
"""

from __future__ import annotations

import importlib.util
import math
import unittest
from pathlib import Path
from typing import ClassVar

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


class StubWidthTests(unittest.TestCase):
    def test_capped_at_the_pads_narrow_side(self):
        # a 0.4 mm stub from a 0.25 x 0.82 mm TQFN pad at 0.5 mm pitch: its
        # end cap alone reaches 0.175 mm from the next pin, under 0.2 mm
        self.assertAlmostEqual(fanout.stub_width(0.4, 0.41, 0.125), 0.25)
        self.assertAlmostEqual(fanout.stub_width(0.4, 0.125, 0.41), 0.25)

    def test_track_width_when_the_pad_is_wider(self):
        self.assertEqual(fanout.stub_width(0.4, 0.6, 0.725), 0.4)
        self.assertEqual(fanout.stub_width(0.4, 0.74, 0.2), 0.4)


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

    def test_every_candidate_clears_the_pad_by_gap(self):
        """#825: on a diagonal ray, gap + radius past the edge along the ray
        left less than `gap` between the via ring and the pad's corner (C3:2
        on #823 came 0.01 mm over)."""
        for half_w, half_h in ((1.0, 0.5), (0.6, 0.725), (0.125, 0.41), (1.0, 1.0)):
            via_r, gap = 0.5, 0.2
            c = fanout.candidates(0.0, 0.0, half_w, half_h, via_r, gap, 2.0, 0.25)
            ring = c[: fanout.DIRECTIONS]
            with self.subTest(pad=(half_w, half_h)):
                for x, y in c:
                    self.assertGreaterEqual(
                        fanout.box_distance(half_w, half_h, x, y), gap + via_r - 1e-9
                    )
                for x, y in ring:  # and the first ring sits at exactly that
                    self.assertAlmostEqual(
                        fanout.box_distance(half_w, half_h, x, y), gap + via_r
                    )

    def test_diagonal_ray_beside_a_corner(self):
        # ray 1 (22.5 degrees) leaves a 1.0 x 0.5 box through its right edge
        # near the corner; along the ray, 0.5 past it is 0.46 mm from the edge
        half_w, half_h = 1.0, 0.5
        a = 2 * math.pi / fanout.DIRECTIONS
        dx, dy = math.cos(a), math.sin(a)
        edge = fanout.edge_distance(half_w, half_h, dx, dy)
        along = fanout.box_distance(
            half_w, half_h, dx * (edge + 0.5), dy * (edge + 0.5)
        )
        self.assertLess(along, 0.5 - 0.01)
        x, y = fanout.candidates(0.0, 0.0, half_w, half_h, 0.3, 0.2, 2.0, 0.25)[1]
        self.assertAlmostEqual(fanout.box_distance(half_w, half_h, x, y), 0.5)

    def test_deterministic(self):
        args = (3.0, 4.0, 0.6, 0.7, 0.3, 0.2, 1.5, 0.25)
        self.assertEqual(fanout.candidates(*args), fanout.candidates(*args))


if __name__ == "__main__":
    unittest.main()


class InPadPointTests(unittest.TestCase):
    """fanout.in_pad_points: [fanout] in_pad_vias offsets turned with the part."""

    OFFSETS: ClassVar[list] = [[0.0, 0.0], [1.0, 0.0], [0.0, -1.0]]

    def test_unturned_offsets_add_to_the_centre(self):
        self.assertEqual(
            fanout.in_pad_points(10.0, 20.0, 0.0, self.OFFSETS),
            [(10.0, 20.0), (11.0, 20.0), (10.0, 19.0)],
        )

    def test_offsets_turn_with_the_footprint_like_its_pads(self):
        # KiCad turns a footprint counter-clockwise as seen with y down: at
        # 90 degrees a pad at local +x lands at -y (verified against
        # pcbnew pad positions of a part at 90 and 270 degrees, #1280).
        pts = fanout.in_pad_points(0.0, 0.0, 90.0, self.OFFSETS)
        self.assertEqual(
            [(round(x, 9), round(y, 9)) for x, y in pts],
            [(0.0, 0.0), (0.0, -1.0), (-1.0, 0.0)],
        )
        pts = fanout.in_pad_points(0.0, 0.0, 270.0, [[1.0, 0.0]])
        self.assertEqual([(round(x, 9), round(y, 9)) for x, y in pts], [(0.0, 1.0)])
