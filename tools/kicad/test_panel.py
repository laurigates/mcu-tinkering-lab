"""Tests for mouse-bite panels: board.toml `[panel]` to slots and tab holes, and
boardcheck's comparison with a board (#809).

The board is synthetic: a 60 x 40 mm outline, a main board left of x 30 and a
head right of it, split by a vertical cut line at x 30 inside a 3 mm keepout
band, and a waste strip below the main board (y 30-40) cut along y 30.

    python3 -m unittest discover -s tools/kicad -t tools
"""

from __future__ import annotations

import copy
import unittest

from kicad import boardcheck, panel

VERTICAL = [30.0, 39.5, 30.0, 0.5]  # between two boards: centred slot
BOTTOM = [30.0, 30.0, 0.5, 30.0]  # main board / waste: slot on the left (+y)
DATA = {
    "outline": {"origin": [100.0, 100.0], "size": [60.0, 40.0]},
    "keepouts": [
        {"name": "vertical band", "rect": [28.5, 0.0, 31.5, 40.0]},
        {"name": "waste", "rect": [0.0, 30.0, 28.5, 40.0]},
        {"name": "edge above the waste", "rect": [0.0, 29.5, 28.5, 30.0]},
    ],
    "silk": {"cut_lines": [VERTICAL, BOTTOM]},
    "panel": {
        "mode": "mousebites",
        "lines": [{"line": VERTICAL, "offset_mm": 0.0}],
    },
}


def data(**panel_edits) -> dict:
    d = copy.deepcopy(DATA)
    d["panel"].update(panel_edits)
    return d


class ValidateTests(unittest.TestCase):
    def errors(self, **edits) -> list[str]:
        return panel.validate("board.toml", data(**edits))

    def test_defaults_valid(self):
        self.assertEqual(self.errors(), [])
        self.assertEqual(panel.validate("board.toml", {}), [])

    def test_saw_mode_builds_nothing(self):
        self.assertIsNone(panel.build({}))
        self.assertIsNone(panel.build(data(mode="saw")))

    def test_bad_values(self):
        for edits, msg in (
            ({"mode": "vcut"}, "mode 'vcut'"),
            ({"slot_mm": 0}, "slot_mm must be > 0"),
            ({"holes": 2.5}, "holes must be a positive integer"),
            ({"colour": "red"}, "unknown key(s) ['colour']"),
        ):
            with self.subTest(msg=msg):
                self.assertTrue(
                    any(msg in e for e in self.errors(**edits)), self.errors(**edits)
                )

    def test_jlcpcb_minimums(self):
        for edits, msg in (
            ({"slot_mm": 1.0}, "under JLCPCB's 1.2 mm"),
            ({"hole_mm": 0.4, "pitch_mm": 0.75}, "under JLCPCB's 0.5 mm"),
            ({"pitch_mm": 0.75}, "0.15 mm web"),  # #809's 0.5 mm at 0.75 is 0.25
            ({"holes": 4}, "at least 5"),
            ({"tab_mm": 4.0}, "tab_mm 4.0 is shorter"),
        ):
            with self.subTest(msg=msg):
                self.assertTrue(
                    any(msg in e for e in self.errors(**edits)), self.errors(**edits)
                )

    def test_line_must_be_a_cut_line(self):
        errs = self.errors(lines=[{"line": [1.0, 2.0, 3.0, 2.0]}])
        self.assertTrue(any("is not one of [silk] cut_lines" in e for e in errs), errs)
        errs = self.errors(lines=[{"line": VERTICAL}, {"line": VERTICAL}])
        self.assertTrue(any("listed twice" in e for e in errs), errs)


class BuildTests(unittest.TestCase):
    def test_geometry(self):
        p = panel.build(data())
        # The vertical line: centred, 39 mm, end tabs only (39 - 10 < 50).
        # The bottom line: starts at the vertical slot's edge (x 29), slot on
        # its left-hand side, toward +y (y 30-32), runs to x 0.5.
        self.assertEqual(
            sorted(p.slots),
            [(5.5, 30.0, 24.0, 32.0), (29.0, 5.5, 31.0, 34.5)],
        )
        names = [t.name for t in p.tabs]
        self.assertEqual(names, ["MB1", "MB2", "MB3", "MB4"])
        vtop = next(t for t in p.tabs if t.at == (30.0, 3.0))
        # Two rows of five on the slot's edges (x 29, x 31), 0.95 mm apart.
        self.assertEqual(len(vtop.holes), 10)
        self.assertEqual({h[0] for h in vtop.holes}, {29.0, 31.0})
        ys = sorted({h[1] for h in vtop.holes})
        self.assertAlmostEqual(ys[1] - ys[0], 0.95)
        self.assertAlmostEqual((ys[0] + ys[-1]) / 2, 3.0)
        bottom_end = next(t for t in p.tabs if t.at == (26.5, 31.0))
        self.assertEqual({h[1] for h in bottom_end.holes}, {30.0, 32.0})
        self.assertEqual(bottom_end.clear, (24.0, 29.6, 29.0, 32.4))

    def test_interior_tabs_keep_slots_under_max_span(self):
        p = panel.build(data(max_span_mm=15.0))
        vertical = sorted(s for s in p.slots if s[0] == 29.0)
        self.assertEqual(len(vertical), 2)  # 39 - 10 = 29 mm: one interior tab
        self.assertTrue(all(s[3] - s[1] <= 15.0 for s in vertical))

    def test_explicit_tabs(self):
        d = data(lines=[{"line": VERTICAL, "offset_mm": 0.0, "tabs": [19.5]}])
        p = panel.build(d)
        self.assertIn((30.0, 20.0), [t.at for t in p.tabs])  # 19.5 from y 39.5

    def test_hole_outside_keepouts_fails(self):
        d = data(
            lines=[
                {"line": VERTICAL, "offset_mm": 0.0},
                {"line": BOTTOM, "tabs": [10.0]},
            ]
        )
        d["keepouts"][2]["rect"] = [0.0, 29.5, 15.0, 30.0]
        with self.assertRaisesRegex(
            ValueError, r"hole at .* is outside \[\[keepouts\]\]"
        ):
            panel.build(d)

    def test_slot_outside_keepouts_fails(self):
        d = data()
        d["keepouts"][1]["rect"] = [10.0, 30.0, 28.5, 40.0]
        with self.assertRaisesRegex(ValueError, "not inside"):
            panel.build(d)

    def test_short_run_fails(self):
        d = data(tab_mm=20.0)
        with self.assertRaisesRegex(ValueError, "too short"):
            panel.build(d)

    def test_diagonal_line_fails(self):
        d = data()
        d["silk"]["cut_lines"].append([0.0, 0.0, 10.0, 10.0])
        with self.assertRaisesRegex(ValueError, "horizontal or vertical"):
            panel.build(d)

    def test_validate_config_reports_geometry(self):
        errs = boardcheck._validate_panel("board.toml", data(tab_mm=20.0))
        self.assertTrue(any(e.startswith("board.toml: [panel] ") for e in errs), errs)


def board_text(
    p: panel.Panel, *, drop_slot=False, move_hole=False, extra_slot=None
) -> str:
    """A board as KiCad 10 writes the parts panel_board reads."""
    ox, oy = 100.0, 100.0
    out = ["(kicad_pcb", "\t(version 20260206)"]
    rects = [(0.0, 0.0, 60.0, 40.0), *p.slots[: len(p.slots) - drop_slot]]
    if extra_slot:
        rects.append(extra_slot)
    for x0, y0, x1, y1 in rects:
        out += [
            "\t(gr_rect",
            f"\t\t(start {ox + x0:g} {oy + y0:g})",
            f"\t\t(end {ox + x1:g} {oy + y1:g})",
            '\t\t(layer "Edge.Cuts")',
            "\t)",
        ]
    for i, t in enumerate(p.tabs):
        out += [
            '\t(footprint ""',
            f"\t\t(at {ox + t.at[0]:g} {oy + t.at[1]:g})",
            f'\t\t(property "Reference" "{t.name}"',
            "\t\t)",
            "\t\t(attr board_only exclude_from_pos_files exclude_from_bom)",
        ]
        for j, (hx, hy) in enumerate(t.holes):
            dx = hx - t.at[0] + (0.5 if move_hole and i == j == 0 else 0)
            out += [
                '\t\t(pad "" np_thru_hole circle',
                f"\t\t\t(at {dx:g} {hy - t.at[1]:g})",
                "\t\t\t(size 0.6 0.6)",
                "\t\t\t(drill 0.6)",
                "\t\t)",
            ]
        out.append("\t)")
    out.append(")")
    return "\n".join(out) + "\n"


class PanelBoardTests(unittest.TestCase):
    def setUp(self):
        self.data = data()
        self.p = panel.build(self.data)

    def test_matching_board(self):
        self.assertEqual(boardcheck.panel_board(board_text(self.p), self.data), [])

    def test_missing_slot(self):
        errs = boardcheck.panel_board(board_text(self.p, drop_slot=True), self.data)
        self.assertTrue(any("Edge.Cuts slots" in e for e in errs), errs)

    def test_moved_hole(self):
        errs = boardcheck.panel_board(board_text(self.p, move_hole=True), self.data)
        self.assertTrue(any(e.startswith("MB1: holes") for e in errs), errs)

    def test_saw_mode_rejects_panel_features(self):
        errs = boardcheck.panel_board(board_text(self.p), data(mode="saw"))
        self.assertTrue(
            any("a mouse-bite tab on the board, none in [panel]" in e for e in errs)
        )
        self.assertTrue(any("Edge.Cuts slots" in e for e in errs))

    def test_saw_board_passes_in_saw_mode(self):
        empty = panel.Panel(hole_mm=0.6, slots=(), tabs=())
        self.assertEqual(
            boardcheck.panel_board(board_text(empty), data(mode="saw")), []
        )


if __name__ == "__main__":
    unittest.main()
