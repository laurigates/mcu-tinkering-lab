"""Tests for foil-test-board.py: the committed files are current, and the geometry
is what the board claims to test.

Stdlib unittest only, so the pre-commit hook runs it with nothing but python3:

    python3 -m unittest tools/test_foil_test_board.py

The script is run as a subprocess for --check, and loaded (not re-implemented)
for its shapes: a ladder that claims a 0.4 mm gap must have one, and no two
islands may sit closer than the smallest gap the board means to test.
"""

from __future__ import annotations

import importlib.util
import itertools
import subprocess
import sys
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path

SCRIPT = Path(__file__).with_name("foil-test-board.py")
SPEC = importlib.util.spec_from_file_location("foil_test_board", SCRIPT)
assert SPEC and SPEC.loader
FTB = importlib.util.module_from_spec(SPEC)
# Registered before exec: @dataclass resolves its module through sys.modules.
sys.modules[SPEC.name] = FTB
SPEC.loader.exec_module(FTB)
NS = "{http://www.w3.org/2000/svg}"
EPS = 1e-6
CROSSOVER_PAIRS = (
    {"crossover through", "crossover upper"},
    {"crossover through", "crossover lower"},
)


def gap(a, b):
    """The clear distance between two bounding boxes (0 if they touch or overlap)."""
    dx = max(b[0] - a[2], a[0] - b[2], 0)
    dy = max(b[1] - a[3], a[1] - b[3], 0)
    return (dx**2 + dy**2) ** 0.5


def centre(box):
    return (box[0] + box[2]) / 2, (box[1] + box[3]) / 2


class FoilTestBoardTests(unittest.TestCase):
    shapes = {s.name: s for s in FTB.shapes()}

    def test_committed_files_are_current(self):
        r = subprocess.run(
            [sys.executable, str(SCRIPT), "--check"], capture_output=True, text=True
        )
        self.assertEqual(r.returncode, 0, r.stdout + r.stderr)

    def test_files_are_mm_and_every_cut_path_closed(self):
        for path, text in FTB.outputs().items():
            with self.subTest(file=path.name):
                root = ET.fromstring(text)
                w, h = root.get("width") or "", root.get("height") or ""
                self.assertTrue(w.endswith("mm") and h.endswith("mm"))
                box = [float(v) for v in (root.get("viewBox") or "").split()]
                self.assertEqual(box[2:], [float(w[:-2]), float(h[:-2])])
                for p in root.iter(f"{NS}path"):
                    self.assertTrue(
                        (p.get("d") or "").endswith("Z"), p.get("data-name")
                    )

    def test_cut_files_split_by_material(self):
        files = {p.name: ET.fromstring(t) for p, t in FTB.outputs().items()}
        names = {
            n: {p.get("data-name") for p in r.iter(f"{NS}path")}
            for n, r in files.items()
        }
        copper = {
            s.name for s in self.shapes.values() if s.layer in ("copper", "outline")
        }
        kapton = {s.name for s in self.shapes.values() if s.layer == "kapton"}
        self.assertEqual(names["copper.svg"], copper)
        self.assertEqual(names["kapton.svg"], kapton)

    def test_islands_on_the_board_keep_the_smallest_tested_gap(self):
        board = self.shapes["board outline"].bbox
        on_board = [
            s for s in self.shapes.values()
            if s.layer == "copper" and not s.name.startswith("spare")
        ]  # fmt: skip
        for s in on_board:
            with self.subTest(island=s.name):
                x0, y0, x1, y1 = s.bbox
                self.assertGreaterEqual(min(x0, y0, board[2] - x1, board[3] - y1), 0.5)
        # Bounding boxes are conservative for L shapes and dogbones: a pass here
        # holds for the real outlines too.
        for a, b in itertools.combinations(on_board, 2):
            if a.name.startswith("gap ") and b.name.startswith("gap "):
                continue  # the ladder's own pairs are checked exactly below
            if {a.name, b.name} in CROSSOVER_PAIRS:
                continue  # the through trace's box spans its end pads; exact below
            with self.subTest(pair=(a.name, b.name)):
                self.assertGreaterEqual(gap(a.bbox, b.bbox), min(FTB.GAPS) - EPS)

    def test_crossover_stubs_clear_the_through_trace(self):
        d = self.shapes["crossover through"].d
        ys = sorted({float(pt.split()[1]) for pt in d[1:-1].split("L")})
        neck_top, neck_bottom = ys[1], ys[2]
        self.assertAlmostEqual(neck_bottom - neck_top, FTB.CROSS_W, delta=EPS)
        upper = self.shapes["crossover upper"].bbox
        lower = self.shapes["crossover lower"].bbox
        self.assertAlmostEqual(neck_top - upper[3], FTB.CROSS_CLEAR, delta=EPS)
        self.assertAlmostEqual(lower[1] - neck_bottom, FTB.CROSS_CLEAR, delta=EPS)

    def test_gap_ladder_has_exactly_its_gaps(self):
        for g in FTB.GAPS:
            up = self.shapes[f"gap {FTB.fmt(g)} upper"].bbox
            lo = self.shapes[f"gap {FTB.fmt(g)} lower"].bbox
            self.assertAlmostEqual(lo[1] - up[3], g, delta=EPS)
        pairs = [
            (
                self.shapes[f"gap {FTB.fmt(g)} upper"],
                self.shapes[f"gap {FTB.fmt(g)} lower"],
            )
            for g in FTB.GAPS
        ]
        for (_, a), (b, _) in zip(pairs, pairs[1:]):
            self.assertGreater(gap(a.bbox, b.bbox), max(FTB.GAPS))

    def test_width_ladder_necks(self):
        for w in FTB.WIDTHS:
            d = self.shapes[f"width {FTB.fmt(w)}"].d
            ys = sorted({float(pt.split()[1]) for pt in d[1:-1].split("L")})
            # four distinct y values: pad top, neck top, neck bottom, pad bottom
            self.assertEqual(len(ys), 4)
            self.assertAlmostEqual(ys[2] - ys[1], w, delta=1e-3)

    def test_socket_pads_follow_the_footprint(self):
        centres = [centre(self.shapes[f"socket pin {n}"].bbox) for n in range(1, 9)]
        axis = sum(y for _, y in centres) / len(centres)
        for (xa, _), (xb, _) in zip(centres, centres[1:]):
            self.assertAlmostEqual(xb - xa, FTB.SOCKET_PITCH, delta=EPS)
        for n, (_, y) in enumerate(centres, 1):
            # odd pins (pin 1 first) on the top row, as the guide labels them
            want = -FTB.SOCKET_OFFSET if n % 2 else FTB.SOCKET_OFFSET
            self.assertAlmostEqual(y - axis, want, delta=EPS)

    def test_kapton_patch_covers_the_trace_and_leaves_the_pads(self):
        patch = self.shapes["spare patch 1"].bbox
        height = patch[3] - patch[1]
        through = self.shapes["crossover through"]
        upper = self.shapes["crossover upper"].bbox
        lower = self.shapes["crossover lower"].bbox
        between = lower[1] - upper[3]  # clear span between the two stub pads
        self.assertGreater(height, FTB.CROSS_W)
        self.assertLess(height, between)
        self.assertGreater(patch[2] - patch[0], FTB.CROSS_PAD)
        bridge = self.shapes["spare bridge 1"].bbox
        # the bridge reaches from the far edge of one stub pad to the other's
        self.assertAlmostEqual(
            bridge[3] - bridge[1], between + 2 * FTB.CROSS_PAD, delta=EPS
        )
        self.assertEqual(through.layer, "copper")

    def test_stub_shorter_than_its_pads_is_refused(self):
        with self.assertRaises(AssertionError):
            FTB.stub_with_pads(0, 0, 1.5, 1.0, 2.0)


if __name__ == "__main__":
    unittest.main()
