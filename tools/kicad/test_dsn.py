"""Tests for the Specctra DSN net-class rewrite (tools/kicad/pcb/dsn.py).

`pcb/` is a directory of scripts for KiCad's bundled Python, not a package, so
`unittest discover` does not reach it; this test loads `pcb/dsn.py` by path.

The fixture `testdata/balancebot-carrier.dsn` is `pcbnew.ExportSpecctraDSN` of
the unrouted balancebot board that `kicad.boardinput` + `pcb/build_board.py`
place, which is what `pcb/route_board.py` exports before it rewrites the
classes; line 1, which carries the output path, is normalised to
`(pcb carrier.dsn`. The golden `balancebot-carrier.rewritten.dsn` is what
`rewrite_dsn` in the balancebot `route_board.py` (as of fb24b28, before the
script moved to tools/kicad, with its hard-coded classes) writes from that
fixture. Here the classes come from the frozen board.toml
(`kicad.testfixture`) through `kicad.boardinput`, so the test also pins that
board.toml says what the script's constants said.

The component and pad order of the export follows the board's footprint
order, which KiCad sets from each footprint's UUID; build_board.py takes those
from the board input (uuid5 of the carrier directory and `fp/<ref>`), so every
placement run exports the same file. `test_build_board` (local, KiCad's Python)
builds the frozen balancebot twice and holds both exports equal to the fixture.

To refresh both files, in a scratch clone whose balancebot inputs equal
`testdata/balancebot-749/` (`.frozen` suffixes dropped), with $KICAD_PYTHON
being KiCad's bundled python3 and `pcb` the scratch board path:

    PYTHONPATH=tools python3 -m kicad.boardinput packages/robotics/balancebot
    $KICAD_PYTHON tools/kicad/pcb/build_board.py packages/robotics/balancebot/hardware/carrier/output/board-input.json
    $KICAD_PYTHON -c 'import pcbnew, sys; sys.exit(not pcbnew.ExportSpecctraDSN(pcbnew.LoadBoard(sys.argv[1]), sys.argv[2]))' "$pcb" raw.dsn

then replace line 1 of raw.dsn with `(pcb carrier.dsn` to get the fixture,
and run `rewrite_dsn` from
`git show fb24b28:packages/robotics/balancebot/hardware/carrier/scripts/route_board.py`
(the constants and the two functions above `gnd_zone`; the module imports
pcbnew, so lift them out) on the fixture to get the golden. Check that the
lifted function still reproduces the previous golden from the previous
fixture before replacing either.

    python3 -m unittest discover -s tools/kicad -t tools
"""

from __future__ import annotations

import copy
import importlib.util
import unittest
from pathlib import Path

from kicad import boardinput, testfixture

HERE = Path(__file__).resolve().parent
TESTDATA = HERE / "testdata"
ROOT, PROJECT = testfixture.shared()


def load_dsn():
    spec = importlib.util.spec_from_file_location("kicad_pcb_dsn", HERE / "pcb/dsn.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


dsn = load_dsn()


class RewriteTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        spec = boardinput.build(PROJECT, ROOT)
        cls.classes = spec["classes"]
        cls.signal = spec["signal"]
        cls.fixture = (TESTDATA / "balancebot-carrier.dsn").read_text()
        cls.golden = (TESTDATA / "balancebot-carrier.rewritten.dsn").read_text()

    def rewrite(self, classes=None, signal=None, text=None):
        return dsn.rewrite(
            self.fixture if text is None else text,
            self.classes if classes is None else classes,
            self.signal if signal is None else signal,
        )

    def test_equals_the_golden(self):
        self.assertEqual(self.rewrite(), self.golden)

    def test_patterns_expand_against_the_dsn(self):
        classes = copy.deepcopy(self.classes)
        classes[0]["nets"] = ["VBAT", "GND", "MOT?_*"]
        self.assertEqual(self.rewrite(classes=classes), self.golden)

    def test_absent_net(self):
        classes = copy.deepcopy(self.classes)
        classes[1]["nets"].append("VMOT")
        with self.assertRaisesRegex(dsn.DsnError, "'VMOT' is not in the board"):
            self.rewrite(classes=classes)

    def test_pattern_matching_nothing(self):
        classes = copy.deepcopy(self.classes)
        classes[0]["nets"].append("MOTX_*")
        with self.assertRaisesRegex(dsn.DsnError, "'MOTX_\\*' matches no net"):
            self.rewrite(classes=classes)

    def test_net_in_two_classes(self):
        classes = copy.deepcopy(self.classes)
        classes[1]["nets"].append("GND")
        with self.assertRaisesRegex(
            dsn.DsnError, "'GND' is in classes 'power' and 'logic_power'"
        ):
            self.rewrite(classes=classes)

    def test_default_via_mismatch(self):
        signal = dict(self.signal, via_um=[700, 350])
        with self.assertRaisesRegex(
            dsn.DsnError, "default via 'Via\\[0-1\\]_600:300_um'"
        ):
            self.rewrite(signal=signal)

    def test_no_default_class(self):
        text = self.fixture.replace("(class kicad_default", "(class other", 1)
        with self.assertRaisesRegex(dsn.DsnError, "no kicad_default class"):
            self.rewrite(text=text)

    def test_shared_via_is_listed_once(self):
        classes = copy.deepcopy(self.classes)
        classes[1]["via_um"] = list(classes[0]["via_um"])
        out = self.rewrite(classes=classes)
        self.assertEqual(out.count('(padstack "Via[0-1]_1000:500_um"'), 1)
        self.assertNotIn('(padstack "Via[0-1]_800:400_um"', out)
        self.assertIn('(via "Via[0-1]_600:300_um" "Via[0-1]_1000:500_um")', out)

    def test_via_name(self):
        self.assertEqual(dsn.via_name([1000, 500]), "Via[0-1]_1000:500_um")


class TrackKeepoutTests(unittest.TestCase):
    """Freerouting 2.5.0 drops a DSN's `wiring` section, fixed wires included,
    and routes through them; a [[tracks]] trunk reaches it as keepouts."""

    TEXT = '  (structure\n    (via "Via[0-1]_600:300_um")\n  )\n'
    TRUNK = {
        "name": "trunk",
        "net": "+5V",
        "layer": "F.Cu",
        "width_mm": 1.0,
        "points": [[10.0, 5.0], [20.0, 5.0]],
    }

    def test_a_segment_is_a_keepout_on_its_layer(self):
        out = dsn.track_keepouts(self.TEXT, [self.TRUNK], [100.0, 100.0], 200)
        # half-width 500 um + 200 um margin to each side, flush with both ends:
        # a track ending inside a pad leaves the pad's centre to the router
        self.assertIn(
            '    (keepout "" (polygon F.Cu 0  110000 -104300  120000 -104300'
            "  120000 -105700  110000 -105700  110000 -104300))\n"
            '    (via "Via[0-1]_600:300_um")',
            out,
        )

    def test_every_segment_of_a_polyline(self):
        trunk = dict(self.TRUNK, points=[[0, 0], [10, 0], [10, 10]], layer="B.Cu")
        out = dsn.track_keepouts(self.TEXT, [trunk], [0, 0], 0)
        self.assertEqual(out.count('(keepout "" (polygon B.Cu 0'), 2)

    def test_interior_corners_are_covered(self):
        trunk = dict(self.TRUNK, points=[[0, 0], [10, 0], [10, 10]], width_mm=2.0)
        out = dsn.track_keepouts(self.TEXT, [trunk], [0, 0], 0)
        # the first segment runs on past the corner by the half-width, the
        # second starts that far before it; the polyline's own ends are flush
        self.assertIn("(polygon F.Cu 0  0 1000  11000 1000  11000 -1000  0 -1000", out)
        self.assertIn(
            "(polygon F.Cu 0  11000 1000  11000 -10000  9000 -10000  9000 1000", out
        )

    def test_no_tracks_leaves_the_text(self):
        self.assertEqual(dsn.track_keepouts(self.TEXT, [], [0, 0], 200), self.TEXT)

    def test_needs_the_structure_via_line(self):
        with self.assertRaisesRegex(dsn.DsnError, "no structure"):
            dsn.track_keepouts("(pcb x)", [self.TRUNK], [0, 0], 0)


class RectKeepoutTests(unittest.TestCase):
    """[[route_keepouts]] bind Freerouting on one layer and nothing else."""

    TEXT = '  (structure\n    (via "Via[0-1]_600:300_um")\n  )\n'

    def test_a_rect_is_a_keepout_on_its_layer(self):
        area = {"name": "under FB", "layer": "B.Cu", "rect": [1.0, 2.0, 3.0, 5.0]}
        out = dsn.rect_keepouts(self.TEXT, [area], [100.0, 100.0])
        self.assertIn(
            '    (keepout "" (polygon B.Cu 0  101000 -102000  103000 -102000'
            "  103000 -105000  101000 -105000  101000 -102000))\n"
            '    (via "Via[0-1]_600:300_um")',
            out,
        )

    def test_none_leaves_the_text(self):
        self.assertEqual(dsn.rect_keepouts(self.TEXT, [], [0, 0]), self.TEXT)

    def test_no_via_at_smd_goes_in_the_structure_once(self):
        out = dsn.no_via_at_smd(self.TEXT)
        self.assertIn('    (control (via_at_smd off))\n    (via "', out)
        self.assertEqual(dsn.no_via_at_smd(out), out)

    def test_no_via_at_smd_needs_the_structure_via_line(self):
        with self.assertRaisesRegex(dsn.DsnError, "no structure"):
            dsn.no_via_at_smd("(pcb x)")


class DropPinsTests(unittest.TestCase):
    """A pad a [[tracks]] entry already joins to its net leaves the netlist
    Freerouting routes, or Freerouting routes it again around the keepouts."""

    TEXT = (
        "  (network\n"
        "    (net +5V\n"
        "      (pins J1-1 L1-2 U7-13\n"
        "        C2-1 C13-1)\n"
        "    )\n"
        '    (net "SPK_VO-"\n'
        "      (pins J19-2 U8-10)\n"
        "    )\n"
        "    (net GND\n"
        "      (pins U10-1 U10-11 C11-2)\n"
        "    )\n"
    )

    def test_drops_the_pin_from_its_net(self):
        out = dsn.drop_pins(self.TEXT, [("+5V", "L1-2"), ("GND", "U10-1")])
        self.assertIn("(pins J1-1 U7-13\n        C2-1 C13-1)", out)
        # a prefix of another pin (U10-1 of U10-11) is not that pin
        self.assertIn("(pins U10-11 C11-2)", out)

    def test_the_last_pin_of_a_line(self):
        out = dsn.drop_pins(self.TEXT, [("+5V", "U7-13")])
        self.assertIn("(pins J1-1 L1-2\n        C2-1 C13-1)", out)

    def test_a_quoted_net(self):
        out = dsn.drop_pins(self.TEXT, [("SPK_VO-", "U8-10")])
        self.assertIn("(pins J19-2)", out)

    def test_only_that_net(self):
        with self.assertRaisesRegex(dsn.DsnError, "C11-2 is not a pin of net '\\+5V'"):
            dsn.drop_pins(self.TEXT, [("+5V", "C11-2")])

    def test_an_unknown_net(self):
        with self.assertRaisesRegex(dsn.DsnError, "no net 'VBAT'"):
            dsn.drop_pins(self.TEXT, [("VBAT", "U10-3")])


if __name__ == "__main__":
    unittest.main()
