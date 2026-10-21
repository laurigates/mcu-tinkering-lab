"""tools/kicad/pcb/stitch.py: the stitching-via grid route_board.py tests against the fill.

`unittest discover` does not reach pcb/, so this test loads `pcb/stitch.py` by
path, as test_fanout.py does for fanout.py.
"""

from __future__ import annotations

import importlib.util
import itertools
import math
import unittest
from pathlib import Path

HERE = Path(__file__).resolve().parent


def load_stitch():
    spec = importlib.util.spec_from_file_location(
        "kicad_pcb_stitch", HERE / "pcb/stitch.py"
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


stitch = load_stitch()


class GridTests(unittest.TestCase):
    def test_centred_with_equal_margins(self):
        pts = stitch.grid(90.0, 62.0, 6.0)
        xs = sorted({x for x, _ in pts})
        ys = sorted({y for _, y in pts})
        self.assertEqual(len(xs), 15)
        self.assertEqual(len(ys), 10)
        self.assertAlmostEqual(xs[0], 90.0 - xs[-1])
        self.assertAlmostEqual(ys[0], 62.0 - ys[-1])
        self.assertTrue(
            all(math.isclose(b - a, 6.0) for a, b in itertools.pairwise(xs))
        )

    def test_exclude_drops_points_inside_and_on_the_edge(self):
        every = stitch.grid(30.0, 30.0, 10.0)
        self.assertEqual(
            every, [(x, y) for y in (5.0, 15.0, 25.0) for x in (5.0, 15.0, 25.0)]
        )
        kept = stitch.grid(30.0, 30.0, 10.0, [[10.0, 10.0, 15.0, 30.0]])
        self.assertNotIn((15.0, 15.0), kept)
        self.assertNotIn((15.0, 25.0), kept)
        self.assertIn((5.0, 15.0), kept)
        self.assertEqual(len(kept), 7)

    def test_pitch_must_be_positive(self):
        with self.assertRaises(ValueError):
            stitch.grid(10.0, 10.0, 0.0)


class RingTests(unittest.TestCase):
    def test_centre_then_points_on_the_circle(self):
        pts = stitch.ring(1.0, 2.0, 0.8, n=8)
        self.assertEqual(pts[0], (1.0, 2.0))
        self.assertEqual(len(pts), 9)
        for x, y in pts[1:]:
            self.assertAlmostEqual(math.hypot(x - 1.0, y - 2.0), 0.8)


if __name__ == "__main__":
    unittest.main()
