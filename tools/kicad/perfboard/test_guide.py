"""Tests for the printable step guide, on the frozen balancebot layout.

A step page is only useful if it carries that step's wires and checks and
shows that step's panel: every wire once, on its own step's page; every net's
continuity row on its step's page; and each crop of perfboard.svg holding its
own panel's grid and legend and nothing of a neighbour's titles.

    python3 -m unittest discover -s tools/kicad -t tools
"""

from __future__ import annotations

import re
import unittest

from kicad import testfixture
from kicad.perfboard import __main__ as cli
from kicad.perfboard import config, render_guide, render_svg
from kicad.perfboard.render_svg import GRID, text_box
from kicad.perfboard.test_render import ATTR, RECT, TEXT, elements, intersects

ROOT, PROJECT = testfixture.shared()
LAYOUT = cli.generate(PROJECT, ROOT)
TEXTS = {path.name: text for path, text in LAYOUT.outputs.items()}
GUIDE, SVG = TEXTS[render_guide.FILENAME], TEXTS["perfboard.svg"]
CFG = config.load(config.perfboard_dir(PROJECT) / config.FILENAME)
PAGES = GUIDE.split("\n#pagebreak()\n")
STEP_PAGES = PAGES[1 : 1 + len(CFG.steps)]
PANEL = re.compile(r"panel\(\d+, ([\d.]+), ([\d.]+), ([\d.]+), ([\d.]+)\)")
# A wire row: the check box, then the wire's number.
WIRE_ROW = re.compile(r'^  check, "(\d+)", "([^"]+)"', re.M)
# A continuity row: the check box, then the net.
CHECK_ROW = re.compile(r'^  check, "([^"]+)"', re.M)


def wire_steps():
    """Wire number -> step index, from the wire list (perfboard.md)."""
    steps, step = {}, -1
    for line in TEXTS["perfboard.md"].splitlines():
        if line.startswith("## Step "):
            step += 1
        elif line.startswith("| ☐ |") and step >= 0:
            steps[int(line.split("|")[2])] = step
    return steps


class PageTests(unittest.TestCase):
    def test_component_side_then_one_page_per_step(self):
        # The power-up page only when perfboard.toml names isolated rails.
        self.assertEqual(len(PAGES), 1 + len(CFG.steps) + bool(CFG.isolated))
        self.assertIn('"Component side"', PAGES[0])
        self.assertEqual('"Before first power-up"' in GUIDE, bool(CFG.isolated))

    def test_every_wire_once_on_its_own_steps_page(self):
        expected = wire_steps()
        self.assertEqual(len(expected), LAYOUT.wires)
        seen = {}
        for page_no, page in enumerate(STEP_PAGES):
            for n, _ in WIRE_ROW.findall(page.split("== Continuity")[0]):
                self.assertNotIn(int(n), seen, f"wire {n} on two pages")
                seen[int(n)] = page_no
        self.assertEqual(seen, expected)

    def test_every_steps_nets_have_their_continuity_row(self):
        for step, (st, page) in enumerate(zip(CFG.steps, STEP_PAGES, strict=True)):
            checks = page.split("== Continuity")[1]
            with self.subTest(step=step + 1):
                self.assertEqual(CHECK_ROW.findall(checks), list(st.nets))


class CropTests(unittest.TestCase):
    """Each page's panel(...) box against the drawing it crops."""

    @classmethod
    def setUpClass(cls):
        cls.boxes = []
        for page in PAGES[: 1 + len(CFG.steps)]:
            x, y, w, h = map(float, PANEL.search(page).groups())
            cls.boxes.append((x, y, x + w, y + h))
        cls.grids = [
            (
                float(a["x"]),
                float(a["y"]),
                float(a["x"]) + float(a["width"]),
                float(a["y"]) + float(a["height"]),
            )
            for a, _ in elements(RECT, SVG)
            if a.get("stroke") == GRID
        ]
        cls.titles = [
            text_box(
                float(a["x"]), float(a["y"]), t, size=float(a.get("font-size", 10))
            )
            for a, t in elements(TEXT, SVG)
            if a.get("font-size") == "13" or t.startswith(("solder side", "parts as"))
        ]

    def test_each_crop_holds_its_own_panels_grid(self):
        self.assertEqual(len(self.boxes), len(self.grids))
        for i, (box, grid) in enumerate(zip(self.boxes, self.grids, strict=True)):
            with self.subTest(panel=i):
                self.assertLessEqual(box[0], grid[0])
                self.assertLessEqual(box[1], grid[1])
                self.assertGreaterEqual(box[2], grid[2])
                self.assertGreaterEqual(box[3], grid[3])

    def test_no_crop_shows_any_panels_title_or_subtitle(self):
        # Subtitles run on into the next panel's cell; the crop starts below them.
        self.assertEqual(len(self.titles), 2 * len(self.boxes))
        for i, box in enumerate(self.boxes):
            with self.subTest(panel=i):
                self.assertFalse(any(intersects(box, t) for t in self.titles))

    def test_control_whole_cell_shows_a_title(self):
        x, y, w, h = render_svg.cell(1, CFG.board)
        self.assertTrue(any(intersects((x, y, x + w, y + h), t) for t in self.titles))

    def test_the_size_the_crop_rewrites_is_the_drawings_own(self):
        # panel() narrows the viewBox by replacing the root's width and height;
        # if they do not match, Typst would print the whole drawing on every page.
        width, height = render_svg.size(CFG.board, CFG.steps)
        root = dict(ATTR.findall(SVG.split(">", 1)[0]))
        self.assertEqual((root["width"], root["height"]), (str(width), str(height)))
        self.assertIn(f'width=\\"{width}\\" height=\\"{height}\\"', GUIDE)

    def test_each_panel_is_one_flat_group_holding_its_grid(self):
        # panel(i, ...) takes the text from `<g id="panel-i">` to the next
        # `</g>`: a nested group would cut the panel short.
        groups = re.findall(r'<g id="panel-(\d+)">(.*?)</g>', SVG, re.S)
        self.assertEqual([int(i) for i, _ in groups], list(range(len(self.boxes))))
        self.assertEqual(SVG.count("<g"), len(groups))
        for (i, body), grid in zip(groups, self.grids, strict=True):
            with self.subTest(panel=i):
                self.assertIn(f'x="{grid[0]:g}" y="{grid[1]:g}"', body)
        for i, page in enumerate(PAGES[: len(groups)]):
            self.assertIn(f"panel({i}, ", page)


if __name__ == "__main__":
    unittest.main()
