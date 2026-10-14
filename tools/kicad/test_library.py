"""Tests for the library resolver (spec §3.4).

Hermetic: the installed-KiCad steps run against temp trees passed in as
`search_roots`, `env` and `bases`, so the suite behaves the same on a Mac with
KiCad and on a CI runner without it, and never reads /Applications or
/usr/share.

    python3 -m unittest discover -s tools/kicad -t tools
"""

from __future__ import annotations

import tempfile
import unittest
from pathlib import Path

from kicad import library
from kicad.errors import KicadError


def touch(path: Path, text: str = "x") -> Path:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(text)
    return path


class VendoredTests(unittest.TestCase):
    def test_real_vendored_parts_resolve(self):
        self.assertEqual(
            library.symbol_lib("Device"),
            library.VENDORED / "kicad-stock/symbols/Device.kicad_sym",
        )
        self.assertEqual(
            library.footprint_file(
                "Resistor_SMD:R_0805_2012Metric_Pad1.20x1.40mm_HandSolder"
            ),
            library.VENDORED
            / "kicad-stock/footprints/Resistor_SMD.pretty"
            / "R_0805_2012Metric_Pad1.20x1.40mm_HandSolder.kicad_mod",
        )

    def test_project_nickname_has_its_own_subtree(self):
        self.assertEqual(
            library.vendored_symbol_lib("Seeed_XIAO"),
            library.VENDORED / "seeed-xiao/Seeed_XIAO.kicad_sym",
        )
        self.assertEqual(
            library.vendored_footprint_file("Seeed_XIAO:XIAO-RP2350-Socket"),
            library.VENDORED
            / "seeed-xiao/Seeed_XIAO.pretty/XIAO-RP2350-Socket.kicad_mod",
        )

    def test_missing_vendored_file_names_the_vendor_recipe(self):
        with self.assertRaisesRegex(KicadError, r"just kicad::vendor"):
            library.symbol_lib("Amplifier_Audio")
        with self.assertRaisesRegex(KicadError, r"just kicad::vendor"):
            library.footprint_file("Resistor_SMD:R_0402_1005Metric")

    def test_vendored_only_never_reads_an_installed_copy(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            touch(root / "fp/Resistor_SMD.pretty/R_0402_1005Metric.kicad_mod")
            with self.assertRaises(KicadError):
                library.footprint_file(
                    "Resistor_SMD:R_0402_1005Metric", search_roots=[root / "fp"]
                )

    def test_bad_library_id(self):
        for bad in ("Device", ":R", "Device:", "a:b:c"):
            with self.subTest(bad=bad), self.assertRaises(KicadError):
                library.split_id(bad)


class InstalledOrderTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        root = Path(self.tmp.name)
        self.env_sym = root / "env-symbols"
        self.env_fp = root / "env-footprints"
        self.mac = root / "mac/SharedSupport"
        self.linux = root / "usr/share/kicad"
        self.vendored = root / "vendored"
        for base in (self.mac, self.linux):
            touch(base / "symbols/Device.kicad_sym", str(base))
            touch(base / "footprints/Diode_SMD.pretty/D_SOD-123.kicad_mod", str(base))
        touch(self.env_sym / "Device.kicad_sym", "env")
        touch(self.env_fp / "Diode_SMD.pretty/D_SOD-123.kicad_mod", "env")
        self.env = {
            "KICAD_SYMBOL_DIR": str(self.env_sym),
            "KICAD_FOOTPRINT_DIR": str(self.env_fp),
        }

    def roots(self, kind, env=None):
        return library.installed_roots(
            kind, env=env or {}, bases=(self.mac, self.linux)
        )

    def test_order_is_env_then_mac_then_linux(self):
        self.assertEqual(
            self.roots("symbols", self.env),
            [self.env_sym, self.mac / "symbols", self.linux / "symbols"],
        )
        self.assertEqual(
            self.roots("footprints", self.env),
            [self.env_fp, self.mac / "footprints", self.linux / "footprints"],
        )

    def test_missing_directories_are_skipped(self):
        env = {"KICAD_SYMBOL_DIR": str(Path(self.tmp.name) / "nowhere")}
        self.assertEqual(
            self.roots("symbols", env), [self.mac / "symbols", self.linux / "symbols"]
        )
        with self.assertRaises(KicadError):
            library.installed_roots("models", env={}, bases=())

    def test_first_root_holding_the_file_wins(self):
        sym = library.symbol_lib(
            "Device",
            vendored_only=False,
            search_roots=self.roots("symbols", self.env),
            vendored=self.vendored,
        )
        self.assertEqual(sym.read_text(), "env")
        fp = library.footprint_file(
            "Diode_SMD:D_SOD-123",
            vendored_only=False,
            search_roots=self.roots("footprints"),
            vendored=self.vendored,
        )
        self.assertEqual(fp.read_text(), str(self.mac))
        only_linux = [self.linux / "footprints"]
        fp = library.footprint_file(
            "Diode_SMD:D_SOD-123",
            False,
            search_roots=only_linux,
            vendored=self.vendored,
        )
        self.assertEqual(fp.read_text(), str(self.linux))

    def test_vendored_copy_wins_over_installed(self):
        touch(self.vendored / "kicad-stock/symbols/Device.kicad_sym", "vendored")
        sym = library.symbol_lib(
            "Device",
            False,
            search_roots=self.roots("symbols", self.env),
            vendored=self.vendored,
        )
        self.assertEqual(sym.read_text(), "vendored")

    def test_not_found_anywhere(self):
        with self.assertRaisesRegex(KicadError, "not in any installed KiCad"):
            library.footprint_file(
                "Diode_SMD:D_SOD-323",
                False,
                search_roots=self.roots("footprints", self.env),
                vendored=self.vendored,
            )


if __name__ == "__main__":
    unittest.main()
