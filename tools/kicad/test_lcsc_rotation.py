"""Tests for deriving a JLCPCB rotation offset from LCSC's footprint.

Offline: the EasyEDA records are synthetic, shaped like the API's (pads as
`PAD~shape~x~y~w~h~layer~net~number~...`, 10 mil units, y down). The KiCad
side is the vendored footprint.

    python3 -m unittest discover -s tools/kicad -t tools
"""

from __future__ import annotations

import contextlib
import io
import unittest
from unittest import mock

from kicad import lcsc_rotation
from kicad.errors import KicadError

TSSOP24 = "Package_SO:TSSOP-24_4.4x7.8mm_P0.65mm"
R0805 = "Resistor_SMD:R_0805_2012Metric_Pad1.20x1.40mm_HandSolder"


def record(pads: dict[str, tuple[float, float]], package: str = "PKG") -> dict:
    """An EasyEDA record with the given pads (mm, y down) about (4000, 3000)."""
    shape = [
        f"PAD~RECT~{4000 + x / 0.254}~{3000 + y / 0.254}~1~1~1~~{n}~0~~0~g{n}"
        for n, (x, y) in pads.items()
    ]
    shape.append("TRACK~0.6~3~~3990 3000 4010 3000~gge1~0")
    return {
        "success": True,
        "result": {
            "title": "PART",
            "packageDetail": {
                "dataStr": {"head": {"c_para": {"package": package}}, "shape": shape}
            },
        },
    }


def turned(pads, turn: int) -> dict[str, tuple[float, float]]:
    """`pads` (y down) turned clockwise on screen by `turn` degrees, i.e. the
    footprint LCSC would draw if JLCPCB needed `turn` added."""
    out = {}
    for n, (x, y) in pads.items():
        for _ in range(turn // 90):
            x, y = -y, x  # 90 degrees clockwise on screen (y down)
        out[n] = (x, y)
    return out


class DeriveTests(unittest.TestCase):
    def test_vendored_tssop_against_itself(self):
        kicad = lcsc_rotation.kicad_pads(TSSOP24)
        self.assertEqual(len(kicad), 24)
        for turn in (0, 90, 180, 270):
            with self.subTest(turn=turn):
                lcsc = lcsc_rotation.easyeda_pads(record(turned(kicad, turn)))
                got, err = lcsc_rotation.derive(lcsc, kicad)
                self.assertEqual(got, turn)
                self.assertLess(err, 0.001)

    def test_pin1_bottom_left_needs_270(self):
        """LCSC's "-BL" footprints: pin 1 bottom-left, rows along x. KiCad's
        TSSOP has pin 1 top-left, rows along y."""
        kicad = lcsc_rotation.kicad_pads(TSSOP24)
        bl = {n: (y, -x) for n, (x, y) in kicad.items()}  # pin 1 to bottom-left
        self.assertLess(bl["1"][0], 0)
        self.assertGreater(bl["1"][1], 0)
        got, _ = lcsc_rotation.derive(lcsc_rotation.easyeda_pads(record(bl)), kicad)
        self.assertEqual(got, 270)

    def test_two_pad_part_keeps_pad_order(self):
        kicad = lcsc_rotation.kicad_pads(R0805)
        same = {"1": (-0.95, 0.0), "2": (0.95, 0.0)}
        swapped = {"1": (0.95, 0.0), "2": (-0.95, 0.0)}
        self.assertEqual(lcsc_rotation.derive(same, kicad)[0], 0)
        self.assertEqual(lcsc_rotation.derive(swapped, kicad)[0], 180)

    def test_extra_lcsc_pad_ignored(self):
        """SOT-223: LCSC numbers the tab 4, KiCad's TabPin2 numbers it 2."""
        kicad = {"1": (-2.3, 3.15), "2": (0.0, 3.15), "3": (2.3, 3.15)}
        lcsc = {"1": (2.3, -3.15), "2": (0.0, -3.15), "3": (-2.3, -3.15), "4": (0, 3)}
        self.assertEqual(lcsc_rotation.derive(lcsc, kicad)[0], 180)

    def test_missing_pad_fails(self):
        with self.assertRaisesRegex(KicadError, r"KiCad pads \['3'\]"):
            lcsc_rotation.derive(
                {"1": (0, 0), "2": (1, 0)}, {"1": (0, 0), "2": (1, 0), "3": (2, 0)}
            )

    def test_different_package_fails(self):
        kicad = lcsc_rotation.kicad_pads(TSSOP24)
        wider = {n: (x * 1.5, y) for n, (x, y) in kicad.items()}
        with self.assertRaisesRegex(KicadError, "not the same package"):
            lcsc_rotation.derive(wider, kicad)

    def test_pads_subset_ignores_a_pad_drawn_differently(self):
        """PowerDI3333-8: KiCad numbers the drain pad and its fingers 5, one
        pad centred on the drain; LCSC numbers the fingers 5-8, so its pad 5 is
        a corner finger. Matching the pins alone still finds the turn."""
        kicad = {"1": (-1.5, -0.975), "2": (-1.5, -0.325), "3": (-1.5, 0.325)}
        kicad |= {"4": (-1.5, 0.975), "5": (0.455, 0.0)}
        lcsc = turned(kicad, 270)
        lcsc["5"] = (lcsc["5"][0] + 1.0, lcsc["5"][1] - 0.6)  # a corner finger
        with self.assertRaisesRegex(KicadError, "not the same package"):
            lcsc_rotation.derive(lcsc, kicad)
        got, err = lcsc_rotation.derive(lcsc, kicad, pads=("1", "2", "3", "4"))
        self.assertEqual(got, 270)
        self.assertLess(err, 0.001)

    def test_pads_subset_must_be_kicad_pads(self):
        with self.assertRaisesRegex(KicadError, r"pads \['9'\]"):
            lcsc_rotation.derive({"1": (0, 0)}, {"1": (0, 0)}, pads=("1", "9"))

    def test_unnumbered_pads_skipped(self):
        rec = record({"1": (-1, 0), "2": (1, 0)})
        rec["result"]["packageDetail"]["dataStr"]["shape"].append(
            "PAD~RECT~4000~3000~1~1~1~~~0~~0~gx"
        )
        self.assertEqual(set(lcsc_rotation.easyeda_pads(rec)), {"1", "2"})


class CliTests(unittest.TestCase):
    def run_main(self, argv):
        with contextlib.redirect_stdout(io.StringIO()) as out:
            code = lcsc_rotation.main(argv)
        return code, out.getvalue()

    def test_entry(self):
        kicad = lcsc_rotation.kicad_pads(TSSOP24)
        rec = record(turned(kicad, 270), package="TSSOP-24-BL")
        with mock.patch.object(lcsc_rotation, "fetch", return_value=rec):
            code, out = self.run_main(["C130026", TSSOP24])
        self.assertEqual(code, 0, out)
        self.assertIn("[C130026] # PART\n", out)
        self.assertIn(f'footprint = "{TSSOP24}"\n', out)
        self.assertIn('lcsc_footprint = "TSSOP-24-BL"\n', out)
        self.assertIn("rotation = 270\n", out)

    def test_entry_with_pads(self):
        kicad = lcsc_rotation.kicad_pads(TSSOP24)
        rec = record(turned(kicad, 270))
        with mock.patch.object(lcsc_rotation, "fetch", return_value=rec):
            code, out = self.run_main(["C130026", TSSOP24, "--pads", "1,24"])
        self.assertEqual(code, 0, out)
        self.assertIn("rotation = 270\n", out)
        self.assertIn('pads = ["1", "24"]\n', out)

    def test_check_matches_only_an_entrys_pads(self):
        from kicad import jlc

        table = {"C9": jlc.Rotation("A:B", 270, ("1", "2", "3", "4"))}
        kicad = {"1": (-1.5, -0.975), "2": (-1.5, -0.325), "3": (-1.5, 0.325)}
        kicad |= {"4": (-1.5, 0.975), "5": (0.455, 0.0)}
        lcsc = turned(kicad, 270)
        lcsc["5"] = (lcsc["5"][0] + 1.0, lcsc["5"][1] - 0.6)
        with (
            mock.patch.object(jlc, "load_rotations", return_value=table),
            mock.patch.object(lcsc_rotation, "fetch", return_value=record(lcsc)),
            mock.patch.object(lcsc_rotation, "kicad_pads", return_value=kicad),
        ):
            self.assertEqual(lcsc_rotation.check(), [])

    def test_check_reports_disagreement(self):
        def fake(lcsc):
            # every part drawn as KiCad draws it: offset 0 everywhere
            from kicad import jlc

            fp = jlc.load_rotations()[lcsc].footprint
            return record(lcsc_rotation.kicad_pads(fp))

        with mock.patch.object(lcsc_rotation, "fetch", side_effect=fake):
            code, out = self.run_main(["--check"])
        self.assertEqual(code, 1)
        self.assertIn("FAIL: C130026: table says 270, LCSC's footprint 0", out)
        self.assertNotIn("C17414", out)

    def test_bad_args(self):
        code, out = self.run_main(["C1"])
        self.assertEqual((code, out.startswith("FAIL:")), (1, True))


if __name__ == "__main__":
    unittest.main()
