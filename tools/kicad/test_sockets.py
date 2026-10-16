"""Tests for `kicad.sockets`: breakout socket footprints from recorded pad centres.

Positions are checked against the board pages read independently of the
module under test, or, for a socket whose page records no pad centres,
against the `pad_xy` coordinates its sockets.toml table lists. Mutations run on a temp copy of tools/kicad/lib and
docs/reference/boards; the committed files are only read.

    python3 -m unittest discover -s tools/kicad -t tools
"""

from __future__ import annotations

import contextlib
import io
import re
import shutil
import tempfile
import tomllib
import unittest
from pathlib import Path

from hardware import board_pad_centres

from kicad import footprints, library, libtables, sockets, vendor
from kicad.errors import KicadError
from kicad.paths import REPO_ROOT

LIB = REPO_ROOT / "tools/kicad/lib"
BOARDS = REPO_ROOT / "docs/reference/boards"
SUBTREE = REPO_ROOT / vendor.SOCKETS_REL
PRETTY = SUBTREE / f"{sockets.NICK}.pretty"
CONFIG = SUBTREE / sockets.CONFIG
PIN_SOCKET = (
    LIB / "kicad-stock/footprints/Connector_PinSocket_2.54mm.pretty/"
    "PinSocket_1x08_P2.54mm_Vertical.kicad_mod"
)
# Pad counts from issue #750's plan, confirmed against the board pages.
EXPECTED_COUNTS = {
    "SparkFun_TB6612FNG_Socket": 16,
    "Adafruit_MAX98357A_Socket": 7,
    "Adafruit_TCA9548A_Socket": 24,
    "Adafruit_PCA9685_Socket": 12,
}


def run(argv, repo_root):
    out = io.StringIO()
    with contextlib.redirect_stdout(out):
        code = sockets.main(argv, repo_root=repo_root)
    return code, out.getvalue()


def expected_centres(board: str, pads: list[str]) -> list[tuple[float, float]]:
    """The page's centres for `pads`, in footprint order, read without `sockets`."""
    centres = board_pad_centres(board)
    out = []
    for entry in pads:
        element, _, pad = entry.partition(".")
        found = centres[element]
        if pad:
            out += [(c.x, c.y) for c in found if c.pad == pad]
        else:
            out += [(c.x, c.y) for c in sorted(found, key=lambda c: int(c.pad))]
    return out


def expected_socket_centres(socket: dict) -> list[tuple[float, float]]:
    """A config table's pad centres in footprint order: listed, or the page's."""
    if "pad_xy" in socket:
        return [tuple(xy) for xy in socket["pad_xy"]]
    return expected_centres(socket["board"], socket["pads"])


def page_outline_size(page: Path) -> tuple[float, float]:
    """The "the outline W × H mm" a board page states in its prose."""
    m = re.search(r"the outline\s+([\d.]+) × ([\d.]+) mm", page.read_text())
    assert m, f"{page}: no outline size"
    return float(m.group(1)), float(m.group(2))


def fab_rects(text: str) -> list[tuple[float, float, float, float]]:
    """(x0, y0, x1, y1) of each fp_rect on F.Fab."""
    out = []
    for m in re.finditer(
        r"\(fp_rect\s*\(start (\S+) (\S+)\)\s*\(end (\S+) (\S+)\)(.*?)\(layer \"([^\"]+)\"\)",
        text,
        re.S,
    ):
        if m.group(6) == "F.Fab":
            out.append(tuple(float(m.group(i)) for i in range(1, 5)))
    return out


class CommittedFootprintTests(unittest.TestCase):
    def setUp(self):
        self.config = tomllib.loads(CONFIG.read_text())["socket"]

    def test_check_passes_on_the_committed_tree(self):
        code, out = run(["--check"], REPO_ROOT)
        self.assertEqual(code, 0, out)

    def test_pad_counts(self):
        counts = {
            s["name"]: len(footprints.parse(PRETTY / f"{s['name']}.kicad_mod").pads)
            for s in self.config
        }
        self.assertEqual(counts, EXPECTED_COUNTS)

    def test_pads_sit_on_the_pages_recorded_centres(self):
        for s in self.config:
            with self.subTest(socket=s["name"]):
                want = expected_socket_centres(s)
                fp = footprints.parse(PRETTY / f"{s['name']}.kicad_mod")
                x0, y0 = want[0]
                for n, (x, y) in enumerate(want, 1):
                    pad = fp.pads[str(n)]
                    # Pad 1 is the footprint origin; the page is already y-down.
                    self.assertAlmostEqual(pad.x, x - x0, places=6, msg=f"pad {n}")
                    self.assertAlmostEqual(pad.y, y - y0, places=6, msg=f"pad {n}")

    def test_pad_shape_and_drill_come_from_pin_socket(self):
        stock = footprints.parse(PIN_SOCKET)
        text = PIN_SOCKET.read_text()
        drill = "(drill 1)"
        size = "(size 1.7 1.7)"
        self.assertIn(drill, text)
        self.assertIn(size, text)
        for s in self.config:
            with self.subTest(socket=s["name"]):
                path = PRETTY / f"{s['name']}.kicad_mod"
                fp = footprints.parse(path)
                body = path.read_text()
                self.assertEqual(fp.pads["1"].shape, stock.pads["1"].shape)
                self.assertEqual(
                    {p.shape for n, p in fp.pads.items() if n != "1"},
                    {stock.pads["2"].shape},
                )
                self.assertEqual({p.kind for p in fp.pads.values()}, {"thru_hole"})
                self.assertEqual(body.count(drill), len(fp.pads))
                self.assertEqual(body.count(size), len(fp.pads))
                self.assertEqual(fp.attr, "through_hole")

    def test_courtyard_and_silkscreen_enclose_every_pad(self):
        stock = footprints.parse(PIN_SOCKET)
        xs = [p.x for p in stock.pads.values()]
        margin = min(xs) - stock.courtyard[0]
        self.assertAlmostEqual(margin, sockets.COURTYARD_MARGIN)
        for s in self.config:
            with self.subTest(socket=s["name"]):
                path = PRETTY / f"{s['name']}.kicad_mod"
                fp = footprints.parse(path)  # raises without an F.CrtYd
                px = [p.x for p in fp.pads.values()]
                py = [p.y for p in fp.pads.values()]
                xmin, ymin, xmax, ymax = fp.courtyard
                # The courtyard is the box over the headers' margin and the
                # module outline's (test_module_outline_...), so it is at
                # least the headers' own.
                self.assertLessEqual(xmin, min(px) - margin + 1e-9)
                self.assertLessEqual(ymin, min(py) - margin + 1e-9)
                self.assertGreaterEqual(xmax, max(px) + margin - 1e-9)
                self.assertGreaterEqual(ymax, max(py) + margin - 1e-9)
                self.assertIn('(layer "F.SilkS")', path.read_text())

    def test_module_outline_is_the_pages_and_the_courtyard_covers_it(self):
        # The breakout sits above the carrier on its headers, so its whole
        # board, not only the headers, must stay clear of other parts (#750).
        for s in self.config:
            with self.subTest(socket=s["name"]):
                page = BOARDS / f"{s['board']}.md"
                w, h = page_outline_size(page)
                x0, y0, x1, y1 = s["outline"]
                self.assertAlmostEqual(x1 - x0, w, places=2)
                self.assertAlmostEqual(y1 - y0, h, places=2)
                px, py = expected_socket_centres(s)[0]
                path = PRETTY / f"{s['name']}.kicad_mod"
                fab = fab_rects(path.read_text())
                want = (x0 - px, y0 - py, x1 - px, y1 - py)
                self.assertEqual(len(fab), 1, fab)
                for got, exp in zip(fab[0], want, strict=True):
                    self.assertAlmostEqual(got, exp, places=4)
                cx0, cy0, cx1, cy1 = footprints.parse(path).courtyard
                m = sockets.MODULE_MARGIN
                self.assertLessEqual(cx0, want[0] - m + 1e-9)
                self.assertLessEqual(cy0, want[1] - m + 1e-9)
                self.assertGreaterEqual(cx1, want[2] + m - 1e-9)
                self.assertGreaterEqual(cy1, want[3] + m - 1e-9)

    def test_render_is_byte_stable(self):
        first = sockets.render(REPO_ROOT)
        second = sockets.render(REPO_ROOT)
        self.assertEqual(first, second)
        for path, text in first.items():
            self.assertEqual(path.read_text(encoding="utf-8"), text, path)

    def test_no_footprint_outside_the_config(self):
        self.assertEqual(
            sorted(p.stem for p in PRETTY.glob("*.kicad_mod")),
            sorted(s["name"] for s in self.config),
        )

    def test_vendor_pins_the_subtree(self):
        self.assertEqual(
            vendor.check_pinned(REPO_ROOT, vendor.SOCKETS_TREE), ([], True)
        )
        self.assertIs(vendor.subtree_at(SUBTREE), vendor.SOCKETS_TREE)
        self.assertIn(vendor.SOCKETS_TREE, vendor.SUBTREES)

    def test_library_and_tables_know_the_nickname(self):
        self.assertEqual(
            library.footprint_file(f"{sockets.NICK}:Adafruit_PCA9685_Socket"),
            PRETTY / "Adafruit_PCA9685_Socket.kicad_mod",
        )
        descr = libtables.descr(sockets.NICK)
        for needle in ("CC-BY-SA", "Adafruit", "SparkFun", "PinSocket"):
            self.assertIn(needle, descr)


class ConfigTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.path = Path(self.tmp.name) / "sockets.toml"

    def load(self, text):
        self.path.write_text(text)
        return sockets.load_config(self.path)

    def test_valid(self):
        got = self.load(
            '[[socket]]\nname = "A_Socket"\nboard = "b"\npads = ["JP1", "JP2.3"]\n'
            "outline = [0, -10, 20, 0]\n"
        )
        self.assertEqual(
            got,
            [sockets.Socket("A_Socket", "b", ("JP1", "JP2.3"), (0, -10, 20, 0))],
        )

    def test_valid_explicit_pad_positions(self):
        got = self.load(
            '[[socket]]\nname = "A_Socket"\nboard = "b"\n'
            "pad_xy = [[0, 0], [2.54, 0]]\noutline = [-1, -1, 4, 1]\n"
        )
        self.assertEqual(
            got,
            [sockets.Socket("A_Socket", "b", (), (-1, -1, 4, 1), ((0, 0), (2.54, 0)))],
        )

    def test_rejects(self):
        ok = "outline = [0, -1, 1, 0]\n"
        cases = {
            "unknown key": '[[socket]]\nname = "A"\nboard = "b"\npads = ["JP1"]\nx = 1\n'
            + ok,
            "no outline": '[[socket]]\nname = "A"\nboard = "b"\npads = ["JP1"]\n',
            "outline not 4 numbers": '[[socket]]\nname = "A"\nboard = "b"\npads = ["JP1"]\n'
            "outline = [0, 1, 2]\n",
            "outline inverted": '[[socket]]\nname = "A"\nboard = "b"\npads = ["JP1"]\n'
            "outline = [2, 0, 0, -1]\n",
            "missing key": '[[socket]]\nname = "A"\nboard = "b"\n',
            "bad name": '[[socket]]\nname = "a b"\nboard = "b"\npads = ["JP1"]\n',
            "empty pads": '[[socket]]\nname = "A"\nboard = "b"\npads = []\n',
            "pads not strings": '[[socket]]\nname = "A"\nboard = "b"\npads = [1]\n',
            "no tables": "other = 1\n",
            "pads and pad_xy": '[[socket]]\nname = "A"\nboard = "b"\npads = ["JP1"]\n'
            "pad_xy = [[0, 0]]\n" + ok,
            "pad_xy empty": '[[socket]]\nname = "A"\nboard = "b"\npad_xy = []\n' + ok,
            "pad_xy not pairs": '[[socket]]\nname = "A"\nboard = "b"\n'
            "pad_xy = [[0, 0, 1]]\n" + ok,
            "pad_xy not numbers": '[[socket]]\nname = "A"\nboard = "b"\n'
            'pad_xy = [["0", 0]]\n' + ok,
            "duplicate name": (
                '[[socket]]\nname = "A"\nboard = "b"\npads = ["JP1"]\n'
                '[[socket]]\nname = "A"\nboard = "c"\npads = ["JP1"]\n'
            ),
        }
        for label, text in cases.items():
            with self.subTest(label), self.assertRaises(KicadError):
                self.load(text)


class MutationTests(unittest.TestCase):
    """--check and regeneration on a temp copy of the library and board pages."""

    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.root = Path(self.tmp.name)
        shutil.copytree(LIB, self.root / "tools/kicad/lib")
        shutil.copytree(BOARDS, self.root / "docs/reference/boards")
        self.subtree = self.root / vendor.SOCKETS_REL
        self.pretty = self.subtree / f"{sockets.NICK}.pretty"
        self.assertEqual(run(["--check"], self.root)[0], 0)

    def test_hand_edited_footprint_is_stale(self):
        path = self.pretty / "Adafruit_MAX98357A_Socket.kicad_mod"
        path.write_text(path.read_text().replace("(drill 1)", "(drill 0.8)", 1))
        code, out = run(["--check"], self.root)
        self.assertEqual(code, 1)
        self.assertIn("Adafruit_MAX98357A_Socket.kicad_mod: stale", out)
        self.assertEqual(run([], self.root)[0], 0)
        self.assertEqual(run(["--check"], self.root)[0], 0)
        self.assertEqual(vendor.check_pinned(self.root, vendor.SOCKETS_TREE)[0], [])

    def test_moved_page_centre_makes_the_footprint_stale(self):
        page = self.root / "docs/reference/boards/sparkfun-tb6612fng.md"
        page.write_text(
            page.read_text().replace(
                "| JP2.8 | 17.78 | -1.27 |", "| JP2.8 | 17.78 | -1.30 |"
            )
        )
        code, out = run(["--check"], self.root)
        self.assertEqual(code, 1, out)
        self.assertIn("SparkFun_TB6612FNG_Socket.kicad_mod: stale", out)

    def test_orphan_footprint_fails_check_and_regeneration_removes_it(self):
        orphan = self.pretty / "Old_Socket.kicad_mod"
        orphan.write_text('(footprint "Old_Socket")\n')
        code, out = run(["--check"], self.root)
        self.assertEqual(code, 1)
        self.assertIn("Old_Socket.kicad_mod: not generated by", out)
        self.assertEqual(run([], self.root)[0], 0)
        self.assertFalse(orphan.exists())
        self.assertEqual(vendor.check_pinned(self.root, vendor.SOCKETS_TREE)[0], [])

    def test_unknown_element_or_pad_fails(self):
        config = self.subtree / sockets.CONFIG
        text = config.read_text()
        for bad in ('"JP9"', '"JP6.99"'):
            with self.subTest(bad):
                # The comment quotes "JP6.10" too; target the pads line.
                target = '["JP3", "JP6.10"'
                self.assertEqual(text.count(target), 1)
                config.write_text(text.replace(target, f'["JP3", {bad}'))
                code, out = run(["--check"], self.root)
                self.assertEqual(code, 1)
                self.assertIn("FAIL:", out)
                self.assertIn(bad.strip('"'), out)
        config.write_text(text)

    def test_outline_disagreeing_with_the_page_fails(self):
        config = self.subtree / sockets.CONFIG
        text = config.read_text()
        target = "outline = [0, -30.48, 17.78, 0]"
        self.assertEqual(text.count(target), 1)
        config.write_text(text.replace(target, "outline = [0, -30.48, 17.5, 0]"))
        code, out = run(["--check"], self.root)
        self.assertEqual(code, 1, out)
        self.assertIn("Adafruit_TCA9548A_Socket", out)
        self.assertIn("17.78", out)

    def test_pad_outside_the_outline_fails(self):
        config = self.subtree / sockets.CONFIG
        text = config.read_text()
        target = "outline = [0, -30.48, 17.78, 0]"
        config.write_text(text.replace(target, "outline = [5, -30.48, 22.78, 0]"))
        code, out = run(["--check"], self.root)
        self.assertEqual(code, 1, out)
        self.assertIn("outside the outline", out)

    def add_explicit_socket(self, pad_xy: str) -> Path:
        """Append a `pad_xy` socket on the MAX98357A page; its footprint path."""
        config = self.subtree / sockets.CONFIG
        config.write_text(
            config.read_text()
            + '\n[[socket]]\nname = "Listed_Socket"\nboard = "adafruit-max98357a"\n'
            f"pad_xy = {pad_xy}\noutline = [-1, -1, 16.78, 18.05]\n"
        )
        return self.pretty / "Listed_Socket.kicad_mod"

    def test_explicit_pads_render_at_their_listed_positions(self):
        path = self.add_explicit_socket("[[2.54, 2.54], [2.54, 5.08], [17.78 , 5.08]]")
        code, out = run([], self.root)
        self.assertEqual(code, 1, out)  # the third pad is outside the outline
        self.assertIn("outside the outline", out)
        config = self.subtree / sockets.CONFIG
        config.write_text(config.read_text().replace("[17.78 , 5.08]", "[15.24, 5.08]"))
        code, out = run([], self.root)
        self.assertEqual(code, 0, out)
        fp = footprints.parse(path)
        got = {n: (p.x, p.y) for n, p in fp.pads.items()}
        self.assertEqual(got, {"1": (0, 0), "2": (0, 2.54), "3": (12.7, 2.54)})
        self.assertIn("positions listed in sockets.toml", path.read_text())
        self.assertEqual(run(["--check"], self.root)[0], 0)
        config.write_text(config.read_text().replace("[15.24, 5.08]", "[15.24, 5.1]"))
        code, out = run(["--check"], self.root)
        self.assertEqual(code, 1, out)
        self.assertIn("Listed_Socket.kicad_mod: stale", out)

    def test_explicit_pad_listed_twice_fails(self):
        self.add_explicit_socket("[[0, 0], [2.54, 0], [0, 0]]")
        code, out = run(["--check"], self.root)
        self.assertEqual(code, 1, out)
        self.assertIn("Listed_Socket: [0, 0] listed twice", out)

    def test_duplicate_pad_fails(self):
        config = self.subtree / sockets.CONFIG
        config.write_text(
            config.read_text().replace('"JP6.10", "JP6.7"', '"JP3.1", "JP6.7"')
        )
        code, out = run(["--check"], self.root)
        self.assertEqual(code, 1)
        self.assertIn("JP3.1 listed twice", out)


if __name__ == "__main__":
    unittest.main()
