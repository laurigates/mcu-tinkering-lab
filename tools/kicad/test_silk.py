"""tools/kicad/pcb/silk.py: the silkscreen stroke floor build_board.py applies.

`unittest discover` does not reach pcb/, so this test loads `pcb/silk.py` by
path, as test_fanout.py does for fanout.py. The vendored footprints are the
evidence: each width the floor must act on is read from one of them.
"""

from __future__ import annotations

import importlib.util
import re
import unittest
from pathlib import Path

HERE = Path(__file__).resolve().parent
STOCK = HERE / "lib/kicad-stock/footprints"


def load_silk():
    spec = importlib.util.spec_from_file_location(
        "kicad_pcb_silk", HERE / "pcb/silk.py"
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


silk = load_silk()
NM = 1_000_000  # nanometres per millimetre


def silk_widths(path: Path) -> set[float]:
    """Stroke widths (mm) of a footprint's F.SilkS graphics."""
    text = path.read_text(encoding="utf-8")
    widths = set()
    for block in re.findall(
        r"\(fp_(?:line|arc|circle|rect|poly)\b.*?\n\t\)", text, re.DOTALL
    ):
        if '(layer "F.SilkS")' in block:
            m = re.search(r"\(width ([0-9.]+)\)", block)
            if m:
                widths.add(float(m.group(1)))
    return widths


class WidenedTests(unittest.TestCase):
    def test_floor_is_jlcpcbs_minimum(self):
        self.assertEqual(silk.MIN_STROKE_NM, 150_000)

    def test_thin_strokes_are_raised_to_the_floor(self):
        for mm_ in (0.1, 0.12, 0.127, 0.149):
            with self.subTest(width=mm_):
                self.assertEqual(silk.widened(round(mm_ * NM)), silk.MIN_STROKE_NM)

    def test_strokes_at_or_above_the_floor_are_kept(self):
        for nm in (150_000, 160_000, 250_000):
            self.assertEqual(silk.widened(nm), nm)

    def test_a_filled_shapes_zero_outline_stays_zero(self):
        self.assertEqual(silk.widened(0), 0)
        self.assertEqual(silk.widened(-1), -1)


class StockFootprintTests(unittest.TestCase):
    """The marks the floor is for are drawn under it in the vendored library."""

    def test_polarity_marks_are_drawn_under_the_floor(self):
        for name in (
            "Capacitor_SMD.pretty/CP_Elec_6.3x7.7.kicad_mod",  # "+" mark
            "Diode_SMD.pretty/D_SOD-123.kicad_mod",  # cathode bar
        ):
            with self.subTest(footprint=name):
                widths = silk_widths(STOCK / name)
                self.assertTrue(widths, f"{name}: no F.SilkS graphics read")
                thin = [w for w in widths if round(w * NM) < silk.MIN_STROKE_NM]
                self.assertTrue(thin, f"{name}: {widths}; the floor would not act")
                for w in thin:
                    self.assertEqual(silk.widened(round(w * NM)), silk.MIN_STROKE_NM)


class ClipTests(unittest.TestCase):
    """clip_segment: what remains of a silkscreen line once the pieces a pad
    would make JLCPCB's CAM delete are cut out."""

    STEP = 50_000  # 0.05 mm
    MIN = 200_000  # 0.2 mm

    def clip(self, a, b, blocked):
        return silk.clip_segment(a, b, blocked, self.STEP, self.MIN)

    def test_a_line_clear_of_every_pad_is_kept_whole(self):
        a, b = (0, 0), (2_000_000, 0)
        self.assertEqual(self.clip(a, b, lambda p, q: False), [(a, b)])

    def test_a_blocked_middle_splits_the_line_in_two(self):
        # blocked wherever the piece reaches 0.9-1.1 mm
        def blocked(p, q):
            lo, hi = sorted((p[0], q[0]))
            return lo < 1_100_000 and hi > 900_000

        pieces = self.clip((0, 0), (2_000_000, 0), blocked)
        self.assertEqual(
            pieces, [((0, 0), (900_000, 0)), ((1_100_000, 0), (2_000_000, 0))]
        )

    def test_slivers_shorter_than_the_minimum_are_dropped(self):
        def blocked(p, q):
            return max(p[0], q[0]) > 150_000

        self.assertEqual(self.clip((0, 0), (2_000_000, 0), blocked), [])

    def test_a_fully_blocked_line_is_removed(self):
        self.assertEqual(self.clip((0, 0), (1_000_000, 0), lambda p, q: True), [])

    def test_diagonal_and_reversed_lines_keep_their_direction(self):
        pieces = self.clip((1_000_000, 1_000_000), (0, 0), lambda p, q: False)
        self.assertEqual(pieces, [((1_000_000, 1_000_000), (0, 0))])

    def test_the_clearance_is_jlcpcbs_and_includes_half_the_stroke(self):
        self.assertEqual(silk.PAD_CLEARANCE_NM, 150_000)
        self.assertEqual(
            silk.clearance(150_000, mask_nm=50_000),
            150_000 + 75_000 + 50_000 + silk.MARGIN_NM,
        )


if __name__ == "__main__":
    unittest.main()
