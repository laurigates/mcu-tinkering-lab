"""Tests for a module socket's wrong fits and the orientation-mark gate.

The symmetry tests use pad patterns written out here (and, as a cross-check,
the vendored footprints they come from); the gate tests use a synthetic
board.toml dict and the vendored GY521_Socket and Pololu footprints, never the
live project or the committed board.

    python3 -m unittest discover -s tools/kicad -t tools
"""

from __future__ import annotations

import unittest

from kicad import library, orientation
from kicad.orientation import Pad

GY521 = [Pad(str(n + 1), n * 2.54, 0.0) for n in range(8)]
GY521_OUTLINE = (-1.61, -14.61, 19.39, 1.27)
# KiCad's Pololu_Breakout-16: pads 1-8 down x = 0, 9-16 up x = 12.7.
POLOLU = [Pad(str(n + 1), 0.0, n * 2.54) for n in range(8)] + [
    Pad(str(16 - n), 12.7, n * 2.54) for n in range(8)
]
POLOLU_OUTLINE = (-1.27, -1.27, 13.97, 19.05)
# CJMCU-2317: outer and inner columns 2.54 mm apart, the single column 15.24 mm
# further, ten rows (sockets.toml's pad_xy).
CJMCU = [
    Pad(str(col * 10 + row + 1), x, row * 2.54)
    for col, x in enumerate((0.0, 2.54, 17.78))
    for row in range(10)
]
CJMCU_OUTLINE = (-1.61, -1.57, 19.39, 24.43)


def motions(fits):
    return [f.motion for f in fits]


class SymmetryTests(unittest.TestCase):
    def test_gy521_row_with_an_off_centre_body_fits_three_wrong_ways(self):
        fits = orientation.wrong_fits(GY521, GY521_OUTLINE)
        self.assertEqual(
            motions(fits),
            ["turned 180 degrees", "flipped top to bottom", "flipped left to right"],
        )
        turned, across, end_to_end = fits
        self.assertIn(("1", "8"), turned.moves)  # VCC onto INT
        self.assertEqual(across.moves, ())  # every pad on its own number
        self.assertTrue(across.face_down)
        self.assertEqual(
            across.describe(),
            "flipped top to bottom (module face down): same pads, the outline moves",
        )
        self.assertTrue(
            end_to_end.describe().startswith(
                "flipped left to right (module face down): pad 1 <-> pad 8, pad 2 <-> pad 7"
            )
        )

    def test_a_row_centred_in_its_body_fits_two_wrong_ways(self):
        """The mirror across the row keeps both pads and outline: no mark needed."""
        centred = (-1.61, -5.0, 19.39, 5.0)
        self.assertEqual(
            motions(orientation.wrong_fits(GY521, centred)),
            ["turned 180 degrees", "flipped left to right"],
        )

    def test_pololu_rows_turned_180_swap_pad_1_and_9(self):
        fits = orientation.wrong_fits(POLOLU, POLOLU_OUTLINE)
        self.assertEqual(fits[0].motion, "turned 180 degrees")
        self.assertIn(("1", "9"), fits[0].moves)
        self.assertIn(("8", "16"), fits[0].moves)
        self.assertEqual(len(fits), 3)  # and both mirrors: rows 12.7 x 17.78, no 90

    def test_cjmcu_three_columns_fit_wrong_only_flipped_top_to_bottom(self):
        fits = orientation.wrong_fits(CJMCU, CJMCU_OUTLINE)
        self.assertEqual(motions(fits), ["flipped top to bottom"])
        self.assertIn(("1", "10"), fits[0].moves)  # GND onto GPA7
        self.assertIn(("21", "30"), fits[0].moves)  # A2 onto VCC

    def test_asymmetric_pattern_has_no_wrong_fit(self):
        pads = [Pad("1", 0, 0), Pad("2", 2.54, 0), Pad("3", 0, 5.08)]
        self.assertEqual(
            orientation.symmetries([(p.x, p.y) for p in pads]), [(1.0, 0.0, 0.0, 1.0)]
        )
        self.assertEqual(orientation.wrong_fits(pads, (-1, -1, 4, 6)), [])

    def test_square_grid_finds_the_quarter_turns(self):
        pads = [
            Pad(str(i + 1), x, y)
            for i, (x, y) in enumerate([(0, 0), (2.54, 0), (2.54, 2.54), (0, 2.54)])
        ]
        fits = orientation.wrong_fits(pads, None)
        self.assertEqual(len(fits), 7)
        self.assertIn("turned 90 degrees", motions(fits))
        self.assertIn("flipped along the diagonal", motions(fits))

    def test_tilted_pattern_beyond_the_square_group(self):
        """The search does not assume axis-aligned symmetry axes."""
        square = [(0, 0), (2, 1), (1, 3), (-1, 2)]  # a square turned atan(1/2)
        pads = [Pad(str(i + 1), x, y) for i, (x, y) in enumerate(square)]
        self.assertEqual(len(orientation.symmetries(square)), 8)
        names = motions(orientation.wrong_fits(pads, None))
        self.assertTrue(any("degree axis" in n for n in names), names)

    def test_same_pads_without_an_outline_is_not_a_wrong_fit(self):
        self.assertEqual(
            motions(orientation.wrong_fits(GY521, None)),
            ["turned 180 degrees", "flipped left to right"],
        )


class FootprintTests(unittest.TestCase):
    """The vendored footprints read the same as the patterns above."""

    def fits(self, fpid):
        return orientation.footprint_wrong_fits(library.footprint_file(fpid))

    def test_gy521_socket(self):
        path = library.footprint_file("Breakout_Sockets:GY521_Socket")
        self.assertEqual(orientation.socket_pads(path), GY521)
        self.assertEqual(orientation.fab_box(path), GY521_OUTLINE)
        self.assertEqual(len(self.fits("Breakout_Sockets:GY521_Socket")), 3)

    def test_pololu_breakout(self):
        fpid = "Module:Pololu_Breakout-16_15.2x20.3mm"
        path = library.footprint_file(fpid)
        self.assertEqual(
            sorted(orientation.socket_pads(path), key=lambda p: int(p.number)),
            sorted(POLOLU, key=lambda p: int(p.number)),
        )
        self.assertEqual(orientation.fab_box(path), POLOLU_OUTLINE)
        self.assertEqual(len(self.fits(fpid)), 3)

    def test_cjmcu_socket(self):
        fits = self.fits("Breakout_Sockets:CJMCU_2317_Socket")
        self.assertEqual(motions(fits), ["flipped top to bottom"])

    def test_which_footprints_are_module_sockets(self):
        self.assertTrue(orientation.is_module_socket("Breakout_Sockets:GY521_Socket"))
        self.assertTrue(
            orientation.is_module_socket("Module:Pololu_Breakout-16_15.2x20.3mm")
        )
        self.assertFalse(orientation.is_module_socket("Seeed_XIAO:XIAO-RP2350-Socket"))
        self.assertFalse(
            orientation.is_module_socket(
                "Connector_PinSocket_2.54mm:PinSocket_1x08_P2.54mm_Vertical"
            )
        )


class OutlineTests(unittest.TestCase):
    def test_gy521_outline_on_the_board_leaves_out_the_socket_strips_edge(self):
        """The module's rear edge runs 0.06 mm from the strip's own silkscreen,
        which already marks it; the other three edges are drawn."""
        path = library.footprint_file("Breakout_Sockets:GY521_Socket")
        lines = orientation.outline_lines(path, [36.11, 47.0, 0])
        self.assertEqual(
            sorted(lines),
            sorted(
                [
                    [34.5, 32.39, 55.5, 32.39],
                    [55.5, 32.39, 55.5, 48.27],
                    [34.5, 48.27, 34.5, 32.39],
                ]
            ),
        )

    def test_place_turns_counter_clockwise_as_drawn(self):
        # KiCad's (at x y 90): local +x points up the board (-y).
        x, y = orientation.place((10.0, 0.0), [50.0, 50.0, 90])
        self.assertAlmostEqual(x, 50.0)
        self.assertAlmostEqual(y, 40.0)
        x, y = orientation.place((10.0, 2.0), [50.0, 50.0, 180])
        self.assertAlmostEqual(x, 40.0)
        self.assertAlmostEqual(y, 48.0)

    def test_clip_drops_slivers(self):
        seg = (0.0, 0.0, 10.0, 0.0)
        wall = (0.15, 0.1, 9.9, 0.1)  # 0.1 mm off: blocks all but the ends
        self.assertEqual(orientation.clip_outline(seg, [wall]), [])
        self.assertEqual(orientation.clip_outline(seg, []), [seg])


def board_data(texts, placement=None, outlines=()):
    return {
        "outline": {"origin": [100.0, 100.0], "size": [90.0, 62.0]},
        "placement": placement or {"J3": [36.11, 47.0, 0], "R1": [5.0, 5.0, 0]},
        "silk": {"texts": texts, "module_outlines": list(outlines)},
    }


COMPS = [
    ("J3", "Breakout_Sockets:GY521_Socket"),
    ("R1", "Resistor_SMD:R_0805_2012Metric_Pad1.20x1.40mm_HandSolder"),
]
VCC = {"text": "VCC", "at": [32.6, 47.0], "size": 1.0, "for": "J3"}


def gr_text(text, x, y):
    return f'\t(gr_text "{text}"\n\t\t(at {x:g} {y:g} 0)\n\t\t(layer "F.SilkS")\n\t)\n'


def gr_line(x0, y0, x1, y1):
    return (
        f"\t(gr_line\n\t\t(start {x0:g} {y0:g})\n\t\t(end {x1:g} {y1:g})\n"
        "\t\t(stroke\n\t\t\t(width 0.15)\n\t\t\t(type default)\n\t\t)\n"
        '\t\t(layer "F.SilkS")\n\t)\n'
    )


class GateTests(unittest.TestCase):
    def test_socket_without_a_mark_fails_naming_its_wrong_fits(self):
        errors = orientation.check(board_data([]), COMPS)
        self.assertEqual(len(errors), 1, errors)
        self.assertIn(
            "[silk] J3 (Breakout_Sockets:GY521_Socket) fits its socket 3", errors[0]
        )
        self.assertIn("turned 180 degrees: pad 1 <-> pad 8", errors[0])
        self.assertIn("flipped top to bottom (module face down): same pads", errors[0])

    def test_untagged_text_does_not_count(self):
        untagged = {k: v for k, v in VCC.items() if k != "for"}
        self.assertEqual(len(orientation.check(board_data([untagged]), COMPS)), 1)

    def test_tagged_mark_beside_the_socket_passes(self):
        self.assertEqual(orientation.check(board_data([VCC]), COMPS), [])

    def test_mark_20_mm_away_fails(self):
        far = dict(VCC, at=[12.6, 47.0])  # 20 mm left of [32.6, 47.0]
        errors = orientation.check(board_data([far]), COMPS)
        self.assertEqual(len(errors), 1, errors)
        # The courtyard's left edge is 36.11 - 1.86 = 34.25.
        self.assertIn("is 21.6 mm from J3's courtyard", errors[0])

    def test_mark_on_a_rotated_socket_uses_the_placed_courtyard(self):
        # J3 at 90 degrees: the module lies toward -x of the column at x 36.11.
        turned = {"J3": [36.11, 47.0, 90], "R1": [5.0, 5.0, 0]}
        left = dict(VCC, at=[24.0, 40.0])
        right = dict(VCC, at=[44.0, 40.0])
        self.assertEqual(orientation.check(board_data([left], turned), COMPS), [])
        self.assertEqual(len(orientation.check(board_data([right], turned), COMPS)), 1)

    def test_board_must_carry_the_tagged_text(self):
        data = board_data([VCC])
        self.assertEqual(
            orientation.check(data, COMPS, board_text=gr_text("VCC", 132.6, 147)), []
        )
        errors = orientation.check(data, COMPS, board_text=gr_text("VCC", 132.6, 148))
        self.assertEqual(len(errors), 1, errors)
        self.assertIn("is not on the board's F.SilkS", errors[0])

    def test_board_must_carry_the_module_outline(self):
        data = board_data([VCC], outlines=["J3"])
        lines = [
            (134.5, 132.39, 155.5, 132.39),
            (155.5, 148.27, 155.5, 132.39),  # drawn the other way round
            (134.5, 148.27, 134.5, 132.39),
        ]
        board = gr_text("VCC", 132.6, 147) + "".join(gr_line(*s) for s in lines)
        self.assertEqual(orientation.check(data, COMPS, board_text=board), [])
        short = gr_text("VCC", 132.6, 147) + "".join(gr_line(*s) for s in lines[:2])
        errors = orientation.check(data, COMPS, board_text=short)
        self.assertEqual(len(errors), 1, errors)
        self.assertIn("1 of J3's 3 F.Fab outline segment(s)", errors[0])

    def test_validate_refuses_a_back_side_or_fab_less_outline(self):
        back = board_data([], {"J3": [36.11, 47.0, 0, "back"], "R1": [5, 5, 0]}, ["J3"])
        self.assertIn("on the back", orientation.validate(back, COMPS)[0])
        hole = [("H1", "MountingHole:MountingHole_3.2mm_M3")]
        data = board_data([], {"H1": [4, 4, 0]}, ["H1"])
        self.assertIn("no F.Fab outline", orientation.validate(data, hole)[0])


if __name__ == "__main__":
    unittest.main()
