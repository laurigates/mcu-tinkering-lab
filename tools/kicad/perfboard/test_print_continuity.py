"""Tests for the 1:1 print and the continuity checklist, on the frozen balancebot layout.

The print is only useful if a printer at 100 % puts its holes on a real
board's: the document must be sized in mm with 1 mm user units, the holes
2.54 mm apart, every pad on its hole, and the solder side mirrored. The
checklist is only useful if probing it proves every net whole: each net once,
its probe and targets exactly the net's pads, named by their holes.

    python3 -m unittest discover -s tools/kicad -t tools
"""

from __future__ import annotations

import dataclasses
import re
import tomllib
import unittest
import xml.etree.ElementTree as ET
from itertools import combinations
from pathlib import Path

from kicad import library, netlist, registry, testfixture
from kicad.errors import KicadError
from kicad.perfboard import __main__ as cli
from kicad.perfboard import config, continuity, place, render_print
from kicad.perfboard.grid import PITCH, hole_name

ROOT, PROJECT = testfixture.shared()
TOML = config.perfboard_dir(PROJECT) / config.FILENAME
NS = {"s": "http://www.w3.org/2000/svg"}
PAD = re.compile(r"(\w+)\.(\w+) \(([A-Z]+\d+)\)")
EPS = 1e-6


def frozen():
    """The frozen layout's parts, nets, config and generated outputs by name."""
    nl = netlist.build(PROJECT, ROOT)
    cfg = config.load(TOML)
    specs = {c.registry: registry.load(c.registry, ROOT) for c in nl.comps}
    skip = cfg.skip | place.pinless(nl)
    parts, _, _ = place.build_parts(
        nl, cfg, specs, lambda f: library.footprint_file(f, vendored_only=True), skip
    )
    layout = cli.generate(PROJECT, ROOT)
    texts = {path.name: text for path, text in layout.outputs.items()}
    return parts, place.nets_of(nl, skip), cfg, layout, texts


PARTS, NETS, CFG, LAYOUT, TEXTS = frozen()


class PrintTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.root = ET.fromstring(TEXTS["perfboard-print.svg"])
        cls.groups = {g.get("id"): g for g in cls.root.iter(f"{{{NS['s']}}}g")}

    def outline(self, side):
        cut = self.groups[side].find("s:rect[@class='cut']", NS)
        return float(cut.get("x")), float(cut.get("y"))

    def centre(self, el):
        if el.get("cx") is not None:
            return float(el.get("cx")), float(el.get("cy"))
        return (
            float(el.get("x")) + float(el.get("width")) / 2,
            float(el.get("y")) + float(el.get("height")) / 2,
        )

    def test_document_is_sized_in_mm_with_mm_user_units(self):
        w, h = self.root.get("width"), self.root.get("height")
        self.assertTrue(w.endswith("mm") and h.endswith("mm"), (w, h))
        box = [float(v) for v in self.root.get("viewBox").split()]
        self.assertEqual(box, [0, 0, float(w[:-2]), float(h[:-2])])
        self.assertLessEqual(box[2], 190, "fits A4 portrait with 10 mm margins")

    def test_holes_are_2_54_mm_apart(self):
        for side in ("component", "solder"):
            holes = self.groups[side].findall("s:circle[@class='hole']", NS)
            self.assertEqual(len(holes), CFG.cols * CFG.rows)
            xs = sorted({round(self.centre(h)[0], 6) for h in holes})
            ys = sorted({round(self.centre(h)[1], 6) for h in holes})
            self.assertEqual((len(xs), len(ys)), (CFG.cols, CFG.rows))
            for axis in (xs, ys):
                for a, b in zip(axis, axis[1:]):
                    self.assertAlmostEqual(b - a, PITCH, delta=EPS)

    def test_pads_land_on_their_holes_and_the_solder_side_mirrors(self):
        for side, mirror in (("component", False), ("solder", True)):
            x0, y0 = self.outline(side)
            drawn = {
                el.get("data-pad"): self.centre(el)
                for el in self.groups[side].iter()
                if el.get("data-pad")
            }
            want = {
                f"{p.ref}.{n}": (
                    x0 + ((CFG.cols - 1 - c) if mirror else c) * PITCH + PITCH / 2,
                    y0 + r * PITCH + PITCH / 2,
                )
                for p in PARTS.values()
                for n, (c, r) in p.pads.items()
            }
            self.assertEqual(set(drawn), set(want), side)
            for name, (x, y) in want.items():
                with self.subTest(side=side, pad=name):
                    self.assertAlmostEqual(drawn[name][0], x, delta=EPS)
                    self.assertAlmostEqual(drawn[name][1], y, delta=EPS)

    def test_cut_line_is_the_hole_grid(self):
        cut = self.groups["component"].find("s:rect[@class='cut']", NS)
        self.assertAlmostEqual(float(cut.get("width")), CFG.cols * PITCH, delta=EPS)
        self.assertAlmostEqual(float(cut.get("height")), CFG.rows * PITCH, delta=EPS)

    def test_corner_marks_cross_the_corner_holes(self):
        x0, y0 = self.outline("component")
        corners = {
            (round(x0 + (c + 0.5) * PITCH, 3), round(y0 + (r + 0.5) * PITCH, 3))
            for c in (0, CFG.cols - 1)
            for r in (0, CFG.rows - 1)
        }
        marks = set()
        for path in self.groups["component"].findall("s:path[@class='corner']", NS):
            # "M{x-ARM} {y}H{x+ARM}M{x} {y-ARM}V{y+ARM}": the second move is the centre
            x, y = re.findall(r"M([\d.]+) ([\d.]+)", path.get("d"))[1]
            marks.add((round(float(x), 3), round(float(y) + render_print.ARM, 3)))
        self.assertEqual(marks, corners)
        # Arms that reached the cut line's corner (half a pitch out) made the
        # cross and the corner read as one mark: measured, the crosses then
        # seemed 60.96 mm apart (the outline) instead of 58.42 mm.
        self.assertLess(render_print.ARM, PITCH / 2)

    def test_note_names_the_crosshair_spacing(self):
        text = "".join(self.root.itertext())
        self.assertIn(f"{(CFG.cols - 1) * PITCH:.2f} mm across", text)
        self.assertIn(f"{(CFG.rows - 1) * PITCH:.2f} mm down", text)

    def test_scale_bars_are_50_mm_across_and_down(self):
        bars = {
            b.get("data-axis"): [float(b.get(k)) for k in ("x1", "y1", "x2", "y2")]
            for b in self.root.findall("s:line[@class='scale']", NS)
        }
        self.assertEqual(set(bars), {"across", "down"})
        x1, y1, x2, y2 = bars["across"]
        self.assertEqual((x2 - x1, y2 - y1), (50, 0))
        x1, y1, x2, y2 = bars["down"]
        self.assertEqual((x2 - x1, y2 - y1), (0, 50))
        width = float(self.root.get("width")[:-2])
        self.assertLess(x1, width - 2, "the down bar sits on the page")

    def test_every_wire_is_drawn(self):
        wires = self.groups["solder"].findall("s:polyline[@class='wire']", NS)
        self.assertEqual(len(wires), LAYOUT.wires)


def rows(text, header):
    """The table rows of the section headed `header`, cells stripped."""
    out, on = [], False
    for line in text.splitlines():
        if line.startswith("## "):
            on = line.startswith(header)
        elif on and line.startswith("| ☐ |"):
            out.append([c.strip() for c in line.strip("|").split("|")])
    return out


class ContinuityTests(unittest.TestCase):
    def test_every_net_once_with_exactly_its_pads(self):
        text = TEXTS["continuity.md"]
        seen = {}
        for row in rows(text, "## Step"):
            _, net, probe, targets = row
            self.assertNotIn(net, seen)
            seen[net] = PAD.findall(probe) + PAD.findall(targets)
        self.assertEqual(set(seen), set(NETS))
        for net, pads in seen.items():
            with self.subTest(net=net):
                self.assertEqual(sorted((r, p) for r, p, _ in pads), sorted(NETS[net]))
                for ref, pin, hole in pads:
                    self.assertEqual(hole, hole_name(*PARTS[ref].pads[pin]))

    def test_nets_follow_their_build_step(self):
        text = TEXTS["continuity.md"]
        for i, st in enumerate(CFG.steps, 1):
            got = [r[1] for r in rows(text, f"## Step {i}:")]
            self.assertEqual(got, list(st.nets))

    def test_no_rails_no_isolation_section(self):
        self.assertEqual(CFG.isolated, ())
        self.assertNotIn("Before first power-up", TEXTS["continuity.md"])

    def test_isolation_pairs(self):
        rails = ("VBAT", "+3V3", "GND")
        text = continuity.checklist(
            PARTS, NETS, dataclasses.replace(CFG, isolated=rails)
        )
        got = [tuple(r[1:3]) for r in rows(text, "## Before first power-up")]
        self.assertEqual(got, list(combinations(rails, 2)))
        for _, a, b, pa, pb in rows(text, "## Before first power-up"):
            (ra, na, _), (rb, nb, _) = PAD.findall(pa)[0], PAD.findall(pb)[0]
            self.assertIn((ra, na), NETS[a])
            self.assertIn((rb, nb), NETS[b])


def config_from(text: str) -> config.PerfboardConfig:
    return config.parse(tomllib.loads(text), Path("perfboard.toml"))


class IsolatedConfigTests(unittest.TestCase):
    BASE = TOML.read_text(encoding="utf-8")

    def test_parsed(self):
        cfg = config_from(self.BASE + '\n[continuity]\nisolated = ["VBAT", "GND"]\n')
        self.assertEqual(cfg.isolated, ("VBAT", "GND"))

    def test_rejects(self):
        cases = {
            "one rail": ('isolated = ["GND"]', "two or more distinct"),
            "a rail twice": ('isolated = ["GND", "GND"]', "two or more distinct"),
            "not a list": ('isolated = "GND"', "expected a list of net names"),
            "unknown key": ("rails = []", r"unknown key\(s\) \['rails'\]"),
        }
        for name, (body, msg) in cases.items():
            with self.subTest(name), self.assertRaisesRegex(KicadError, msg):
                config_from(self.BASE + f"\n[continuity]\n{body}\n")

    def test_rail_must_be_a_net(self):
        cfg = dataclasses.replace(CFG, isolated=("VBAT", "VBATT"))
        self.assertEqual(
            config.check_isolated(cfg, NETS),
            [f"{cfg.path} [continuity] isolated: not a net: ['VBATT']"],
        )


if __name__ == "__main__":
    unittest.main()
