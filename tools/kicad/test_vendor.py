"""Tests for the vendored KiCad library subset and `kicad.vendor`.

Mutations run on a temp copy of the subtree; the committed files are only read.
The installed-KiCad comparisons run against temp roots, so the suite needs no
KiCad. A temp root is a git work tree: the referenced set includes the swap
footprints of git-tracked perfboard.toml files.

    python3 -m unittest discover -s tools/kicad -t tools
"""

from __future__ import annotations

import contextlib
import io
import re
import shutil
import subprocess
import tempfile
import unittest
from pathlib import Path

from kicad import symbols, vendor
from kicad.paths import REPO_ROOT

LIB = REPO_ROOT / "tools/kicad/lib"
STOCK = REPO_ROOT / vendor.STOCK_REL
SEEED = REPO_ROOT / vendor.SEEED_REL
PARTS = REPO_ROOT / "docs/reference/parts"
PERFBOARD_TOML = "packages/robotics/balancebot/hardware/perfboard/perfboard.toml"
# sha256 of Seeed-Studio/OSHW-XIAO-Series LICENSE at cc7f3bb (MIT).
SEEED_LICENSE_SHA256 = (
    "df584ae476b849794bd8462e8c5ea8f33c0fbca400acb83688a331731a30207b"
)
# sha256 of kicad-symbols and kicad-footprints 10.0.6 LICENSE.md (identical files).
UPSTREAM_LICENSE_SHA256 = (
    "45d2bce75e5a4208f5afb01b8fb2c406e700371c4fe2b5f5cd5c443d46db4d8f"
)
FOOTPRINT = "footprints/Resistor_SMD.pretty/R_0805_2012Metric_Pad1.20x1.40mm_HandSolder.kicad_mod"


def git(root: Path, *args: str) -> None:
    subprocess.run(["git", *args], cwd=root, check=True, capture_output=True)


class CommittedSubtreeTests(unittest.TestCase):
    def test_check_passes(self):
        self.assertEqual(vendor.check(), [])

    def test_vendored_set_is_the_referenced_set(self):
        want, have = vendor.referenced(), vendor.vendored_ids(STOCK)
        self.assertEqual(have.symbols, want.symbols)
        self.assertEqual(have.footprints, want.footprints)
        self.assertIn("power:PWR_FLAG", have.symbols)
        self.assertEqual((len(have.symbols), len(have.footprints)), (17, 21))

    def test_licence_and_readme(self):
        self.assertEqual(vendor.sha256(STOCK / "LICENSE.md"), UPSTREAM_LICENSE_SHA256)
        readme = (STOCK / "README.md").read_text()
        for needle in (
            "10.0.6",
            "KiCad Libraries contributors",
            "unchanged",
            "just kicad::vendor",
        ):
            self.assertIn(needle, readme)
        self.assertIn("LICENSE.md\t", (STOCK / "MANIFEST.tsv").read_text())

    def test_subset_libraries_have_no_extends(self):
        for lib in sorted((STOCK / "symbols").glob("*.kicad_sym")):
            text = lib.read_text()
            self.assertNotIn("(extends ", text, lib.name)
            self.assertTrue(
                symbols.header(text).startswith("(kicad_symbol_lib\n"), lib.name
            )
            self.assertTrue(text.endswith("\n)\n"), lib.name)

    def test_manifest_is_reproducible(self):
        self.assertEqual(
            vendor.render_manifest(STOCK), (STOCK / "MANIFEST.tsv").read_text()
        )

    def test_main_check(self):
        out = io.StringIO()
        with contextlib.redirect_stdout(out):
            self.assertEqual(vendor.main(["--check"]), 0)
        self.assertIn("17 symbols, 21 footprints, 28 files pinned", out.getvalue())
        self.assertIn(
            "seeed-xiao: 6 files pinned at OSHW-XIAO-Series cc7f3bb", out.getvalue()
        )


class SeeedSubtreeTests(unittest.TestCase):
    def test_pinned(self):
        self.assertEqual(vendor.check_pinned(REPO_ROOT, vendor.SEEED_TREE), ([], True))

    def test_licence_readme_and_derive_toml_are_present(self):
        self.assertEqual(vendor.sha256(SEEED / "LICENSE"), SEEED_LICENSE_SHA256)
        readme = (SEEED / "README.md").read_text()
        for needle in ("cc7f3bb", "MIT", "derived", "kicad.derive_xiao"):
            self.assertIn(needle, readme)
        self.assertTrue((SEEED / "derive.toml").is_file())

    def test_manifest_is_reproducible(self):
        self.assertEqual(
            vendor.render_manifest(SEEED, vendor.SEEED_TAG, vendor.SEEED_TREE.unpinned),
            (SEEED / "MANIFEST.tsv").read_text(),
        )
        self.assertEqual(
            vendor.subtree_files(SEEED),
            sorted(
                [
                    "LICENSE",
                    "MANIFEST.tsv",
                    "README.md",
                    "Seeed_XIAO.kicad_sym",
                    "Seeed_XIAO.pretty/XIAO-ESP32-S3-DIP.kicad_mod",
                    "Seeed_XIAO.pretty/XIAO-ESP32-S3-Socket.kicad_mod",
                    "Seeed_XIAO.pretty/XIAO-RP2350-DIP.kicad_mod",
                    "Seeed_XIAO.pretty/XIAO-RP2350-Socket.kicad_mod",
                    "derive.toml",
                ]
            ),
        )

    def test_subtree_at(self):
        self.assertIs(vendor.subtree_at(SEEED), vendor.SEEED_TREE)
        self.assertIs(vendor.subtree_at(STOCK), vendor.STOCK_TREE)
        with self.assertRaisesRegex(Exception, "not a vendored subtree"):
            vendor.subtree_at(LIB)


class MutationTests(unittest.TestCase):
    """Each mutation of a temp copy must make `--check` fail, naming the cause."""

    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.root = Path(self.tmp.name)
        shutil.copytree(LIB, self.root / "tools/kicad/lib")
        shutil.copytree(PARTS, self.root / "docs/reference/parts")
        git(self.root, "init", "-q")
        self.stock = self.root / vendor.STOCK_REL
        self.seeed = self.root / vendor.SEEED_REL
        self.assertEqual(vendor.check(self.root), [])

    def errors(self):
        return "\n".join(vendor.check(self.root))

    def test_flipped_byte_fails_on_the_manifest_hash(self):
        path = self.stock / FOOTPRINT
        data = bytearray(path.read_bytes())
        data[100] ^= 0x01
        path.write_bytes(bytes(data))
        self.assertRegex(
            self.errors(), r"R_0805_.*sha256 .* does not match MANIFEST\.tsv"
        )
        out = io.StringIO()
        with contextlib.redirect_stdout(out):
            self.assertEqual(vendor.main(["--check"], repo_root=self.root), 1)
        self.assertIn("FAIL:", out.getvalue())

    def test_flipped_byte_in_the_seeed_subtree_fails(self):
        path = self.seeed / "Seeed_XIAO.pretty/XIAO-RP2350-DIP.kicad_mod"
        data = bytearray(path.read_bytes())
        data[100] ^= 0x01
        path.write_bytes(bytes(data))
        errors = self.errors()
        # A vendor file: regenerating is not the fix (it refuses), so the hint
        # says restore, and names --update-vendor only for re-vendoring.
        self.assertRegex(
            errors,
            r"seeed-xiao/Seeed_XIAO\.pretty/XIAO-RP2350-DIP\.kicad_mod: sha256 .* "
            r"does not match MANIFEST\.tsv.*a vendor file.*restore it to the upstream "
            r"bytes \(cc7f3bb\).*--update-vendor",
        )
        self.assertNotIn("run `PYTHONPATH=tools python3 -m kicad.derive_xiao`", errors)

    def test_seeed_hint_names_the_file_kind(self):
        seeed_files = {
            "Seeed_XIAO.pretty/XIAO-RP2350-Socket.kicad_mod": "a derived file",
            "Seeed_XIAO.kicad_sym": "holds vendor and derived symbols",
            "LICENSE": "a vendor file",
        }
        for rel, needle in seeed_files.items():
            with self.subTest(rel=rel):
                path = self.seeed / rel
                original = path.read_bytes()
                path.write_bytes(original + b"\n")
                self.assertRegex(self.errors(), rf"{re.escape(rel)}: sha256 .*{needle}")
                path.write_bytes(original)

    def test_seeed_orphan_missing_licence_and_wrong_tag(self):
        (self.seeed / "Seeed_XIAO.pretty/XIAO-X.kicad_mod").write_text("(footprint)")
        (self.seeed / "LICENSE").unlink()
        manifest = self.seeed / "MANIFEST.tsv"
        manifest.write_text(manifest.read_text().replace("\tcc7f3bb\n", "\tdeadbee\n"))
        errors = self.errors()
        self.assertIn("XIAO-X.kicad_mod: not in MANIFEST.tsv (orphan file)", errors)
        self.assertIn("seeed-xiao/LICENSE: missing", errors)
        self.assertIn("tag 'deadbee', expected 'cc7f3bb'", errors)

    def test_no_seeed_subtree(self):
        shutil.rmtree(self.seeed)
        self.assertRegex(self.errors(), "seeed-xiao: no vendored library")

    def test_edited_licence_fails(self):
        with (self.stock / "LICENSE.md").open("a") as f:
            f.write("extra terms\n")
        self.assertRegex(self.errors(), r"LICENSE\.md: sha256")

    def test_missing_licence_or_readme(self):
        (self.stock / "README.md").unlink()
        (self.stock / "LICENSE.md").unlink()
        errors = self.errors()
        self.assertIn("README.md: missing", errors)
        self.assertIn("LICENSE.md: missing", errors)

    def test_orphan_file(self):
        (self.stock / "footprints/Resistor_SMD.pretty/R_0402.kicad_mod").write_text(
            "(footprint)"
        )
        errors = self.errors()
        self.assertIn("R_0402.kicad_mod: not in MANIFEST.tsv (orphan file)", errors)
        self.assertIn(
            "footprint Resistor_SMD:R_0402: vendored but not referenced", errors
        )

    def test_missing_file(self):
        (self.stock / FOOTPRINT).unlink()
        errors = self.errors()
        self.assertIn("listed in MANIFEST.tsv but missing", errors)
        self.assertIn("referenced but not vendored", errors)

    def test_referenced_but_not_vendored(self):
        want = vendor.referenced()
        want = vendor.Referenced(want.symbols | {"Device:L"}, want.footprints)
        self.assertEqual(
            vendor.check(self.root, want),
            ["symbol Device:L: referenced but not vendored"],
        )

    def test_perfboard_swap_footprint_is_referenced(self):
        # A [swaps.REF] footprint in a tracked perfboard.toml must be vendored:
        # the perfboard generator reads vendored footprints only.
        toml = self.root / "packages/robotics/demo/hardware/perfboard/perfboard.toml"
        toml.parent.mkdir(parents=True)
        swap = "Connector_PinHeader_2.54mm:PinHeader_1x09_P2.54mm_Vertical"
        toml.write_text(
            (REPO_ROOT / PERFBOARD_TOML).read_text()
            + f'\n[swaps.J2]\nfootprint = "{swap}"\n'
        )
        # Untracked, it is not a project yet (control).
        self.assertNotIn(swap, vendor.referenced(self.root).footprints)
        git(self.root, "add", str(toml))
        self.assertIn(swap, vendor.referenced(self.root).footprints)
        self.assertEqual(
            vendor.check(self.root), [f"footprint {swap}: referenced but not vendored"]
        )

    def test_vendored_but_not_referenced(self):
        want = vendor.referenced()
        want = vendor.Referenced(want.symbols - {"Device:R"}, want.footprints)
        self.assertEqual(
            vendor.check(self.root, want),
            ["symbol Device:R: vendored but not referenced"],
        )

    def test_unsorted_or_malformed_manifest(self):
        manifest = self.stock / "MANIFEST.tsv"
        lines = manifest.read_text().splitlines(keepends=True)
        manifest.write_text("".join(reversed(lines)))
        self.assertIn("not sorted", self.errors())
        manifest.write_text("LICENSE.md abc 10.0.6\n")
        self.assertIn("expected 'path<TAB>sha256<TAB>tag'", self.errors())

    def test_wrong_tag(self):
        manifest = self.stock / "MANIFEST.tsv"
        manifest.write_text(manifest.read_text().replace("\t10.0.6\n", "\t9.0.0\n", 1))
        self.assertIn("tag '9.0.0', expected '10.0.6'", self.errors())

    def test_extends_symbol_is_rejected(self):
        # A derived symbol fails even when the manifest is regenerated to match.
        lib = self.stock / "symbols/Device.kicad_sym"
        text = lib.read_text()
        lib.write_text(
            text[: text.rindex(")")]
            + '\t(symbol "R_Small"\n\t\t(extends "R")\n\t)\n)\n'
        )
        (self.stock / "MANIFEST.tsv").write_text(vendor.render_manifest(self.stock))
        self.assertIn("uses (extends ...)", self.errors())

    def test_no_subtree(self):
        shutil.rmtree(self.stock)
        self.assertRegex(self.errors(), "no vendored library")


class InstalledTests(unittest.TestCase):
    """`--check-installed` and `--update` against a fake installed KiCad."""

    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        base = Path(self.tmp.name)
        # The vendored subset is itself a valid installed layout.
        self.sym_root = base / "installed/symbols"
        self.fp_root = base / "installed/footprints"
        shutil.copytree(STOCK / "symbols", self.sym_root)
        shutil.copytree(STOCK / "footprints", self.fp_root)
        self.root = base / "repo"
        shutil.copytree(LIB, self.root / "tools/kicad/lib")
        shutil.copytree(PARTS, self.root / "docs/reference/parts")
        git(self.root, "init", "-q")
        self.stock = self.root / vendor.STOCK_REL

    def drift(self):
        return vendor.check_installed(self.root, [self.sym_root], [self.fp_root])

    def test_identical_install_has_no_drift(self):
        self.assertEqual(self.drift(), [])

    def test_changed_footprint_is_drift(self):
        path = self.fp_root / FOOTPRINT.removeprefix("footprints/")
        path.write_text(path.read_text().replace("(size 1.2 1.4)", "(size 1.3 1.4)"))
        self.assertEqual(
            [d.split(":")[0] + ":" + d.split(":")[1] for d in self.drift()],
            ["footprint Resistor_SMD:R_0805_2012Metric_Pad1.20x1.40mm_HandSolder"],
        )

    def test_changed_symbol_block_and_header_are_drift(self):
        lib = self.sym_root / "Device.kicad_sym"
        text = lib.read_text()
        text = text.replace("(version 20251024)", "(version 20261231)")
        text = text.replace(
            "(pin passive line\n\t\t\t\t(at 0 3.81 270)",
            "(pin input line\n\t\t\t\t(at 0 3.81 270)",
            1,
        )
        lib.write_text(text)
        drift = self.drift()
        self.assertIn("header differs", drift[0])
        self.assertTrue(
            any(d.startswith("symbol Device:") and "differs" in d for d in drift[1:])
        )

    def test_missing_from_install_is_drift(self):
        (self.sym_root / "power.kicad_sym").unlink()
        self.assertIn("'power': not in any installed KiCad", self.drift()[0])

    def test_update_reproduces_the_committed_subtree(self):
        shutil.rmtree(self.stock / "symbols")
        shutil.rmtree(self.stock / "footprints")
        (self.stock / "MANIFEST.tsv").unlink()
        # Extra parts in the install are not copied.
        (self.fp_root / "Extra.pretty").mkdir()
        (self.fp_root / "Extra.pretty/X.kicad_mod").write_text("(footprint)")
        vendor.update(self.root, [self.sym_root], [self.fp_root])
        for rel in vendor.subtree_files(STOCK):
            self.assertEqual(
                (self.stock / rel).read_bytes(), (STOCK / rel).read_bytes(), rel
            )
        self.assertEqual(vendor.subtree_files(self.stock), vendor.subtree_files(STOCK))
        self.assertEqual(vendor.check(self.root), [])

    def test_update_needs_the_licence(self):
        (self.stock / "LICENSE.md").unlink()
        with self.assertRaisesRegex(Exception, "LICENSE.md: missing"):
            vendor.update(self.root, [self.sym_root], [self.fp_root])


if __name__ == "__main__":
    unittest.main()
