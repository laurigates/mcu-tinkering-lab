"""Tests for the JLCPCB assembly files cut from kicad-cli's BOM and CPL.

The inputs are balancebot's own `kicad-cli sch export bom` / `pcb export pos`
rows (the connector and module rows trimmed), and the expected outputs are
what jlc_assembly.py wrote from them before it moved to tools/kicad: 6 BOM
lines and 7 placements, U1 dropped because it has no LCSC number. `FabTests`
runs `kicad.fab` with kicad-cli's exports replaced by the same rows.

    python3 -m unittest discover -s tools/kicad -t tools
"""

from __future__ import annotations

import contextlib
import io
import tempfile
import unittest
from pathlib import Path
from unittest import mock

from kicad import fab, jlc
from kicad.errors import KicadError

R0805 = "Resistor_SMD:R_0805_2012Metric_Pad1.20x1.40mm_HandSolder"
C0805 = "Capacitor_SMD:C_0805_2012Metric_Pad1.18x1.45mm_HandSolder"
BOM = f"""\
"Comment","Designator","Footprint","LCSC"
"DRV8825 left","A1","Module:Pololu_Breakout-16_15.2x20.3mm",""
"100uF 35V","C1,C2","Capacitor_SMD:CP_Elec_6.3x7.7","C2836437"
"100nF","C3","{C0805}","C49678"
"B5819W","D1","Diode_SMD:D_SOD-123","C8598"
"XT30 pack in","J1","Connector_AMASS:AMASS_XT30PW-M_1x02_P2.50mm_Horizontal",""
"10k","R1","{R0805}","C17414"
"100k","R2","{R0805}","C149504"
"27k","R3","{R0805}","C17593"
"XIAO RP2350","U1","Seeed_XIAO:XIAO-RP2350-Socket",""
"""
POS = """\
Ref,Val,Package,PosX,PosY,Rot,Side
"C1","100uF 35V","CP_Elec_6.3x7.7",109.750000,-144.000000,0.000000,top
"C2","100uF 35V","CP_Elec_6.3x7.7",177.750000,-114.000000,0.000000,top
"C3","100nF","C_0805_2012Metric_Pad1.18x1.45mm_HandSolder",161.500000,-114.500000,0.000000,top
"D1","B5819W","D_SOD-123",171.000000,-150.000000,0.000000,top
"R1","10k","R_0805_2012Metric_Pad1.20x1.40mm_HandSolder",131.500000,-131.000000,90.000000,top
"R2","100k","R_0805_2012Metric_Pad1.20x1.40mm_HandSolder",162.000000,-104.000000,0.000000,top
"R3","27k","R_0805_2012Metric_Pad1.20x1.40mm_HandSolder",162.000000,-109.000000,0.000000,top
"U1","XIAO RP2350","XIAO-RP2350-Socket",145.000000,-114.000000,90.000000,top
"""
BOM_JLC = "\r\n".join(
    [
        "Comment,Designator,Footprint,LCSC Part #",
        '100uF 35V,"C1,C2",Capacitor_SMD:CP_Elec_6.3x7.7,C2836437',
        f"100nF,C3,{C0805},C49678",
        "B5819W,D1,Diode_SMD:D_SOD-123,C8598",
        f"10k,R1,{R0805},C17414",
        f"100k,R2,{R0805},C149504",
        f"27k,R3,{R0805},C17593",
        "",
    ]
)
CPL_JLC = "\r\n".join(
    [
        "Designator,Mid X,Mid Y,Layer,Rotation",
        "C1,109.7500mm,-144.0000mm,Top,0",
        "C2,177.7500mm,-114.0000mm,Top,0",
        "C3,161.5000mm,-114.5000mm,Top,0",
        "D1,171.0000mm,-150.0000mm,Top,0",
        "R1,131.5000mm,-131.0000mm,Top,90",
        "R2,162.0000mm,-104.0000mm,Top,0",
        "R3,162.0000mm,-109.0000mm,Top,0",
        "",
    ]
)


class JlcTests(unittest.TestCase):
    def setUp(self):
        tmp = tempfile.TemporaryDirectory()
        self.addCleanup(tmp.cleanup)
        self.dir = Path(tmp.name)

    def write(self, bom: str = BOM, pos: str = POS) -> tuple[Path, Path]:
        b, p = self.dir / "bom.csv", self.dir / "pos.csv"
        b.write_text(bom)
        p.write_text(pos)
        return b, p

    def quiet(self, fn, *args):
        with contextlib.redirect_stdout(io.StringIO()) as out:
            result = fn(*args)
        return result, out.getvalue()

    def test_current_behaviour(self):
        b, p = self.write()
        counts, out = self.quiet(jlc.write, b, p, self.dir)
        self.assertEqual(counts, (6, 7))
        self.assertIn("wrote 6 BOM lines, 7 placements", out)
        self.assertEqual((self.dir / "bom-jlc.csv").read_bytes(), BOM_JLC.encode())
        self.assertEqual((self.dir / "cpl-jlc.csv").read_bytes(), CPL_JLC.encode())

    def test_cli(self):
        b, p = self.write()
        code, _ = self.quiet(jlc.main, [str(b), str(p), str(self.dir)])
        self.assertEqual(code, 0)
        self.assertEqual((self.dir / "cpl-jlc.csv").read_bytes(), CPL_JLC.encode())
        code, out = self.quiet(jlc.main, [str(b)])
        self.assertEqual((code, out.startswith("FAIL:")), (1, True))

    def test_no_lcsc_part(self):
        bom = "\n".join(
            line
            for line in BOM.splitlines()
            if line.endswith('""') or "Comment" in line
        )
        b, p = self.write(bom=bom + "\n")
        with self.assertRaisesRegex(KicadError, "has an LCSC number"):
            self.quiet(jlc.write, b, p, self.dir)

    def test_lcsc_part_without_placement(self):
        pos = "\n".join(line for line in POS.splitlines() if '"D1"' not in line)
        b, p = self.write(pos=pos + "\n")
        with self.assertRaisesRegex(KicadError, r"no SMD placement: \['D1'\]"):
            self.quiet(jlc.write, b, p, self.dir)
        code, out = self.quiet(jlc.main, [str(b), str(p), str(self.dir)])
        self.assertEqual(code, 1)
        self.assertIn("FAIL: LCSC parts with no SMD placement", out)

    def test_missing_column(self):
        b, p = self.write(bom=BOM.replace('"LCSC"', '"Part"', 1))
        code, out = self.quiet(jlc.main, [str(b), str(p), str(self.dir)])
        self.assertEqual(code, 1)
        self.assertIn("column 'LCSC' missing", out)


class FabTests(unittest.TestCase):
    """`kicad.fab` on a temp project, with kicad-cli replaced by canned exports."""

    def setUp(self):
        tmp = tempfile.TemporaryDirectory()
        self.addCleanup(tmp.cleanup)
        self.project = Path(tmp.name) / "demo"
        kicad_dir = self.project / "hardware/carrier/demo-carrier"
        kicad_dir.mkdir(parents=True)
        (self.project / "hardware/carrier/board.toml").write_text(
            '[project]\nname = "demo-carrier"\n'
        )
        for suffix in (".kicad_pcb", ".kicad_sch"):
            (kicad_dir / f"demo-carrier{suffix}").touch()
        self.asm = kicad_dir / "output/assembly"

    def fab(self, bom: str) -> tuple[int, str]:
        def run(argv):
            out = Path(argv[argv.index("-o") + 1])
            if argv[1:4] == ["sch", "export", "bom"]:
                out.write_text(bom)
            elif argv[1:4] == ["pcb", "export", "pos"]:
                out.write_text(POS)

        with (
            mock.patch.object(fab, "kicad_cli", return_value="kicad-cli"),
            mock.patch.object(fab, "run", side_effect=run),
            contextlib.redirect_stdout(io.StringIO()) as out,
        ):
            code = fab.main([str(self.project)])
        return code, out.getvalue()

    def test_writes_the_jlc_files(self):
        code, out = self.fab(BOM)
        self.assertEqual(code, 0, out)
        self.assertEqual((self.asm / "bom-jlc.csv").read_bytes(), BOM_JLC.encode())
        self.assertEqual((self.asm / "cpl-jlc.csv").read_bytes(), CPL_JLC.encode())

    def test_missing_column_fails(self):
        code, out = self.fab(BOM.replace('"LCSC"', '"Part"', 1))
        self.assertEqual(code, 1, out)
        self.assertIn("FAIL: ", out)
        self.assertIn("column 'LCSC' missing from bom.csv or pos.csv", out)


if __name__ == "__main__":
    unittest.main()
