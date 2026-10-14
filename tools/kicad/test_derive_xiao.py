"""Tests for `kicad.derive_xiao`: the socket XIAO parts derived from Seeed's DIP ones.

The refactor proof is the first test: the generalised module reproduces the
committed socket footprint and symbol library byte for byte. Mutations run on a
temp copy of tools/kicad/lib; the committed files are only read.

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

from kicad import derive_xiao, library, symbols, vendor
from kicad.errors import KicadError
from kicad.paths import REPO_ROOT

LIB = REPO_ROOT / "tools/kicad/lib"
SEEED = LIB / "seeed-xiao"
SOCKET_FP = SEEED / "Seeed_XIAO.pretty/XIAO-RP2350-Socket.kicad_mod"
DIP_FP = SEEED / "Seeed_XIAO.pretty/XIAO-RP2350-DIP.kicad_mod"
SYM_LIB = SEEED / "Seeed_XIAO.kicad_sym"
RP2350 = derive_xiao.Socket(
    "XIAO-RP2350-DIP", "XIAO-RP2350-DIP", "XIAO-RP2350-Socket", 1, 14
)


def run(argv, repo_root):
    out = io.StringIO()
    with contextlib.redirect_stdout(out):
        code = derive_xiao.main(argv, repo_root=repo_root)
    return code, out.getvalue()


def pad_numbers(text: str) -> list[tuple[int, str]]:
    return sorted(
        (int(n), kind) for n, kind in re.findall(r'\(pad "(\d+)" (\w+)', text)
    )


class CommittedLibraryTests(unittest.TestCase):
    def test_reproduces_the_committed_files_byte_for_byte(self):
        out = derive_xiao.render("Seeed_XIAO")
        self.assertEqual(sorted(out), sorted([SOCKET_FP, SYM_LIB]))
        self.assertEqual(out[SOCKET_FP], SOCKET_FP.read_text(encoding="utf-8"))
        self.assertEqual(out[SYM_LIB], SYM_LIB.read_text(encoding="utf-8"))

    def test_main_check_passes(self):
        code, out = run(["--check"], REPO_ROOT)
        self.assertEqual(code, 0, out)
        self.assertNotIn("wrote", out)

    def test_config(self):
        self.assertEqual(derive_xiao.load_config(SEEED / "derive.toml"), [RP2350])
        self.assertEqual(derive_xiao.libraries(), [("Seeed_XIAO", SEEED)])

    def test_socket_footprint_is_the_cut(self):
        text = SOCKET_FP.read_text(encoding="utf-8")
        self.assertNotIn("Edge.Cuts", text)
        self.assertEqual(pad_numbers(text), [(n, "thru_hole") for n in range(1, 15)])
        self.assertTrue(text.startswith('(footprint "XIAO-RP2350-Socket"\n'))
        dip = pad_numbers(DIP_FP.read_text(encoding="utf-8"))
        self.assertIn((20, "thru_hole"), dip)
        self.assertIn((1, "smd"), dip)

    def test_symbol_library_keeps_the_vendor_block(self):
        blocks = symbols.blocks(SYM_LIB.read_text(encoding="utf-8"))
        self.assertEqual(list(blocks), ["XIAO-RP2350-DIP", "XIAO-RP2350-Socket"])
        socket = blocks["XIAO-RP2350-Socket"]
        self.assertIn('(property "Footprint" "Seeed_XIAO:XIAO-RP2350-Socket"', socket)
        numbers = sorted(int(n) for n in re.findall(r'\(number "(\d+)"', socket))
        self.assertEqual(numbers, list(range(1, 15)))
        dip = sorted(
            int(n) for n in re.findall(r'\(number "(\d+)"', blocks["XIAO-RP2350-DIP"])
        )
        self.assertEqual(dip, list(range(1, 25)))


class DeriveTests(unittest.TestCase):
    def test_pads_outside_the_range_raise(self):
        # nRF54LM20A-style numbering: the vendor footprint has no pads 1-14.
        pads = "\n\t".join(
            f'(pad "{n}" thru_hole circle (at 0 {n}) (size 1.6 1.6) (drill 1))'
            for n in range(15, 46)
        )
        text = f'(footprint "XIAO-nRF54LM20A-DIP"\n\t(layer "F.Cu")\n\t{pads}\n)\n'
        with self.assertRaisesRegex(
            KicadError,
            r"expected .* each of 1-14, got \[\] .*through-hole pads: \[15, 16,",
        ):
            derive_xiao.derive_footprint(text, RP2350)

    def test_a_missing_kept_pad_raises(self):
        text = DIP_FP.read_text(encoding="utf-8")
        text = re.sub(r'\(pad "7" thru_hole', '(pad "70" thru_hole', text)
        with self.assertRaisesRegex(KicadError, "each of 1-14"):
            derive_xiao.derive_footprint(text, RP2350)

    def test_a_missing_vendor_symbol_raises(self):
        lib = SYM_LIB.read_text(encoding="utf-8")
        socket = derive_xiao.Socket(
            "XIAO-RP2350-DIP", "XIAO-X-DIP", "XIAO-X-Socket", 1, 14
        )
        with self.assertRaisesRegex(KicadError, "no vendor symbol 'XIAO-X-DIP'"):
            derive_xiao.derive_library(lib, [socket], "Seeed_XIAO", "lib")

    def test_a_symbol_without_the_kept_pins_raises(self):
        block = symbols.blocks(SYM_LIB.read_text(encoding="utf-8"))["XIAO-RP2350-DIP"]
        socket = derive_xiao.Socket("A", "XIAO-RP2350-DIP", "B", 1, 30)
        with self.assertRaisesRegex(KicadError, "expected 1-30"):
            derive_xiao.derive_symbol(block, socket, "Seeed_XIAO")

    def test_config_rejects(self):
        good = (
            'vendor_footprint = "A-DIP"\nvendor_symbol = "A-DIP"\n'
            'derived = "A-Socket"\npads = [1, 14]\n'
        )
        cases = {
            "": "no \\[\\[socket\\]\\] tables",
            "[[sockets]]\n": "unknown key",
            "[[socket]]\n" + good + "extra = 1\n": r"unknown key\(s\) \['extra'\]",
            "[[socket]]\n" + good.replace('derived = "A-Socket"\n', ""): "missing key",
            "[[socket]]\n" + good.replace("[1, 14]", "[14, 1]"): "pads must be",
            "[[socket]]\n" + good.replace("[1, 14]", "[0, 14]"): "pads must be",
            "[[socket]]\n" + good.replace("[1, 14]", "[1]"): "pads must be",
            "[[socket]]\n" + good.replace('"A-Socket"', '"A-DIP"'): "overwrite",
            "[[socket]]\n" + good.replace('"A-Socket"', '"a b"'): "part name",
            "[[socket]]\n" + good + "[[socket]]\n" + good: "more than once",
            "[[socket\n": "derive.toml",
        }
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "derive.toml"
            for text, msg in cases.items():
                with self.subTest(text=text):
                    path.write_text(text)
                    with self.assertRaisesRegex(KicadError, msg):
                        derive_xiao.load_config(path)
            path.unlink()
            with self.assertRaisesRegex(KicadError, "missing"):
                derive_xiao.load_config(path)


class CliTests(unittest.TestCase):
    """`main` on a temp copy of tools/kicad/lib and the parts registry."""

    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.root = Path(self.tmp.name)
        shutil.copytree(LIB, self.root / "tools/kicad/lib")
        shutil.copytree(
            REPO_ROOT / "docs/reference/parts", self.root / "docs/reference/parts"
        )
        # A git work tree: vendor.check lists git-tracked perfboard.toml files.
        subprocess.run(["git", "init", "-q"], cwd=self.root, check=True)
        self.seeed = self.root / "tools/kicad/lib/seeed-xiao"
        self.socket = self.seeed / SOCKET_FP.relative_to(SEEED)
        self.assertEqual(vendor.check(self.root), [])

    def test_check_fails_on_a_stale_derived_file_and_writes_nothing(self):
        text = self.socket.read_text()
        stale = text.replace('(pad "1" ', '(pad "1"  ', 1)
        self.assertNotEqual(stale, text)
        self.socket.write_text(stale)
        code, out = run(["--check"], self.root)
        self.assertEqual(code, 1)
        self.assertIn("FAIL: tools/kicad/lib/seeed-xiao/Seeed_XIAO.pretty/", out)
        self.assertEqual(self.socket.read_text(), stale)

    def test_regenerating_restores_the_file_and_the_manifest(self):
        self.socket.write_text("(footprint)\n")
        (self.seeed / "MANIFEST.tsv").write_text(
            vendor.render_manifest(
                self.seeed, vendor.SEEED_TAG, vendor.SEEED_TREE.unpinned
            )
        )
        code, out = run([], self.root)
        self.assertEqual(code, 0, out)
        self.assertIn("wrote tools/kicad/lib/seeed-xiao/Seeed_XIAO.pretty/", out)
        self.assertEqual(self.socket.read_bytes(), SOCKET_FP.read_bytes())
        self.assertEqual(
            (self.seeed / "MANIFEST.tsv").read_bytes(),
            (SEEED / "MANIFEST.tsv").read_bytes(),
        )
        self.assertEqual(vendor.check(self.root), [])
        self.assertEqual(run(["--check"], self.root)[0], 0)

    def _edit(self, rel: str, old: str, new: str) -> Path:
        path = self.seeed / rel
        text = path.read_text(encoding="utf-8")
        self.assertIn(old, text)
        path.write_text(text.replace(old, new, 1), encoding="utf-8")
        return path

    def test_refuses_to_pin_an_edited_vendor_file(self):
        # A hand edit to each vendor-origin file: regenerating must not re-pin
        # it (vendor --check would then pass), and must write nothing.
        edits = {
            "Seeed_XIAO.pretty/XIAO-RP2350-DIP.kicad_mod": ("(layer", "(layer  "),
            "LICENSE": ("MIT License", "MIT Licence"),
            # The vendor symbol block, not the derived one.
            "Seeed_XIAO.kicad_sym": (
                '(symbol "XIAO-RP2350-DIP"',
                '(symbol "XIAO-RP2350-DIP"\n\t\t(exclude_from_sim yes)',
            ),
        }
        manifest = self.seeed / "MANIFEST.tsv"
        for rel, (old, new) in edits.items():
            with self.subTest(rel=rel):
                path = self.seeed / rel
                original = path.read_bytes()
                pinned = manifest.read_bytes()
                socket = self.socket.read_bytes()
                self._edit(rel, old, new)
                edited = path.read_bytes()
                code, out = run([], self.root)
                self.assertEqual(code, 1, out)
                self.assertIn(f"{rel}: differs from its MANIFEST.tsv pin", out)
                self.assertIn("--update-vendor", out)
                self.assertIn("FAIL: wrote nothing", out)
                self.assertEqual(manifest.read_bytes(), pinned)
                self.assertEqual(path.read_bytes(), edited)
                self.assertEqual(self.socket.read_bytes(), socket)
                self.assertRegex(
                    "\n".join(vendor.check(self.root)), rf"{re.escape(rel)}: sha256"
                )
                path.write_bytes(original)

    def test_regenerates_over_an_edited_derived_symbol(self):
        # Only the derived block is off: regenerating restores the pinned bytes.
        self._edit(
            "Seeed_XIAO.kicad_sym",
            '(symbol "XIAO-RP2350-Socket"',
            '(symbol "XIAO-RP2350-Socket"\n\t\t(exclude_from_sim yes)',
        )
        code, out = run([], self.root)
        self.assertEqual(code, 0, out)
        self.assertEqual(
            (self.seeed / "Seeed_XIAO.kicad_sym").read_bytes(), SYM_LIB.read_bytes()
        )
        self.assertEqual(
            (self.seeed / "MANIFEST.tsv").read_bytes(),
            (SEEED / "MANIFEST.tsv").read_bytes(),
        )

    def test_update_vendor_repins_a_revendored_file(self):
        self._edit("Seeed_XIAO.pretty/XIAO-RP2350-DIP.kicad_mod", "(layer", "(layer  ")
        code, out = run(["--update-vendor"], self.root)
        self.assertEqual(code, 0, out)
        self.assertIn("wrote tools/kicad/lib/seeed-xiao/MANIFEST.tsv", out)
        self.assertEqual(vendor.check(self.root), [])

    def test_a_new_vendor_file_needs_update_vendor(self):
        manifest = self.seeed / "MANIFEST.tsv"
        manifest.write_text(
            "".join(
                line + "\n"
                for line in manifest.read_text().splitlines()
                if not line.startswith("LICENSE\t")
            )
        )
        code, out = run([], self.root)
        self.assertEqual(code, 1, out)
        self.assertIn(
            "LICENSE: not in MANIFEST.tsv; pin a newly vendored file with "
            "--update-vendor",
            out,
        )
        manifest.unlink()
        code, out = run([], self.root)
        self.assertEqual(code, 1, out)
        self.assertIn("a first pin needs --update-vendor", out)
        self.assertEqual(run(["--update-vendor"], self.root)[0], 0)
        self.assertEqual(vendor.check(self.root), [])

    def test_check_and_update_vendor_are_exclusive(self):
        with self.assertRaises(SystemExit), contextlib.redirect_stderr(io.StringIO()):
            run(["--check", "--update-vendor"], self.root)

    def test_reads_only_the_vendored_copy(self):
        (self.seeed / DIP_FP.relative_to(SEEED)).unlink()
        code, out = run(["--check"], self.root)
        self.assertEqual(code, 1)
        self.assertIn("not vendored", out)
        self.assertIn(library.VENDOR_HINT, out)

    def test_no_derive_toml_fails(self):
        (self.seeed / "derive.toml").unlink()
        code, out = run(["--check"], self.root)
        self.assertEqual(code, 1)
        self.assertIn("FAIL: no vendored library", out)


if __name__ == "__main__":
    unittest.main()
