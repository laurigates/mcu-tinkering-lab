"""Tests for the perfboard drawing's text layout, on the frozen balancebot layout.

The component-side panel names each part with a label. No label may cover the
column letters above the hole grid or the row numbers beside it: those are
how a builder finds a hole. Boxes are `render_svg.text_box`'s, which
over-estimate the drawn text.

    python3 -m unittest discover -s tools/kicad -t tools
"""

from __future__ import annotations

import re
import unittest

from kicad import testfixture
from kicad.perfboard import __main__ as cli
from kicad.perfboard import render_svg
from kicad.perfboard.render_svg import GRID, MUTED, NUMBERS_DX, PART_INK, text_box

ROOT, PROJECT = testfixture.shared()
TEXT = re.compile(r"<text ([^>]*)>([^<]*)</text>")
ATTR = re.compile(r'([\w-]+)="([^"]*)"')
RECT = re.compile(r"<rect ([^>]*)/>")


def elements(pattern, svg):
    for m in pattern.finditer(svg):
        yield dict(ATTR.findall(m.group(1))), (m.group(2) if m.lastindex == 2 else "")


def intersects(a, b):
    return a[0] < b[2] and b[0] < a[2] and a[1] < b[3] and b[1] < a[3]


def hull(boxes):
    xs0, ys0, xs1, ys1 = zip(*boxes, strict=True)
    return min(xs0), min(ys0), max(xs1), max(ys1)


class ComponentPanel:
    """The component-side panel of a drawing: its grid, letters, numbers, labels."""

    def __init__(self, svg: str):
        # The first grid rect is the component side's (panel 0).
        grid = next(a for a, _ in elements(RECT, svg) if a.get("stroke") == GRID)
        self.left, self.top = float(grid["x"]), float(grid["y"])
        self.right = self.left + float(grid["width"])
        self.bottom = self.top + float(grid["height"])
        letters, numbers, self.labels = [], [], {}
        for a, text in elements(TEXT, svg):
            x, y = float(a["x"]), float(a["y"])
            anchor = a.get("text-anchor", "start")
            box = text_box(x, y, text, anchor)
            if a.get("fill") == PART_INK:
                self.labels[text] = box
            elif a.get("fill") != MUTED:
                continue
            elif anchor == "middle" and self.left <= x <= self.right and y < self.top:
                letters.append(box)
            elif anchor == "start" and x == self.right + NUMBERS_DX:
                numbers.append(box)
        self.letter_row = hull(letters)
        self.number_column = hull(numbers)
        self.n_letters, self.n_numbers = len(letters), len(numbers)

    def collisions(self, labels=None):
        return [
            f"{text} covers the {name}"
            for text, box in (labels or self.labels).items()
            for name, band in (
                ("column letters", self.letter_row),
                ("row numbers", self.number_column),
            )
            if intersects(box, band)
        ]


class LabelTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        layout = cli.generate(PROJECT, ROOT)
        svg = next(t for p, t in layout.outputs.items() if p.suffix == ".svg")
        cls.svg = svg
        cls.panel = ComponentPanel(svg)

    def test_panel_is_parsed(self):
        # 18 x 24 holes, 15 parts: the bands below are the real ones.
        self.assertEqual((self.panel.n_letters, self.panel.n_numbers), (18, 24))
        self.assertEqual(len(self.panel.labels), 15)

    def test_no_label_covers_the_letters_or_numbers(self):
        self.assertEqual(self.panel.collisions(), [])

    def test_control_label_inside_u1_covers_the_letters(self):
        # U1's courtyard reaches above row 1. Its label where every other part
        # has it, 10 px into the body, is the C2 regression: on the letters.
        u1 = next(
            a
            for a, _ in elements(RECT, self.svg)
            if a.get("stroke") == PART_INK
            and float(a["y"]) < self.panel.top
            and float(a["x"]) < self.panel.left + 20
        )
        text = "U1 XIAO RP2350"
        old = text_box(float(u1["x"]) + 3, float(u1["y"]) + 10, text)
        self.assertEqual(
            self.panel.collisions({text: old}), [f"{text} covers the column letters"]
        )

    def test_modules_above_the_grid_are_labelled_above_the_letters(self):
        for ref in ("U1 XIAO RP2350", "J3 GY-521 MPU6050"):
            with self.subTest(label=ref):
                self.assertLess(self.panel.labels[ref][3], self.panel.letter_row[1])


class LabelAtTests(unittest.TestCase):
    """`label_at` on its own: the three placements."""

    TOP, RIGHT = 100.0, 400.0

    def test_body_on_the_grid_keeps_the_label_inside(self):
        self.assertEqual(
            render_svg.label_at(50.0, 120.0, "R1 10k", self.TOP, self.RIGHT),
            (53.0, 130.0),
        )

    def test_body_above_the_grid_lifts_the_label_over_the_letters(self):
        x, y = render_svg.label_at(50.0, 95.0, "U1", self.TOP, self.RIGHT)
        letters = text_box(60.0, self.TOP - render_svg.LETTERS_DY, "A", "middle")
        self.assertEqual(x, 53.0)
        self.assertLess(text_box(x, y, "U1")[3], letters[1])

    def test_label_past_the_grid_is_pulled_clear_of_the_numbers(self):
        text = "J5 a long motor connector name"
        x, y = render_svg.label_at(self.RIGHT - 40, 300.0, text, self.TOP, self.RIGHT)
        numbers_left = self.RIGHT + render_svg.NUMBERS_DX
        self.assertLess(x, self.RIGHT - 40 + 3)
        self.assertLess(text_box(x, y, text)[2], numbers_left)


if __name__ == "__main__":
    unittest.main()
