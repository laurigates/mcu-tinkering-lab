"""Tests for tools/labels: layout invariants, data validation, cut files, candidates.

python3 -m unittest discover -s tools/labels
"""

import json
import re
import subprocess
import sys
import tempfile
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path

import labels

HERE = Path(__file__).resolve().parent
DATA = HERE / "data" / "drawers-2026-10.toml"
SVG = "{http://www.w3.org/2000/svg}"
INK = "{http://www.inkscape.org/namespaces/inkscape}"
# sendto_silhouette.py's regmark_settings_from_group() pattern (driver f376a08).
DRIVER_NOTES_RE = re.compile(
    r"Left\s*=\s*([-+0-9.eE]+)\s*mm.*?Top\s*=\s*([-+0-9.eE]+)\s*mm.*?"
    r"X\s*=\s*([-+0-9.eE]+)\s*mm.*?Y\s*=\s*([-+0-9.eE]+)\s*mm",
    re.DOTALL,
)


def write_data(text: str) -> Path:
    d = tempfile.mkdtemp()
    p = Path(d) / "d.toml"
    p.write_text(text)
    return p


def synthetic(n_labels: int, n_lids: int, n_strips: int) -> dict:
    lab = {
        "cat": "C",
        "color": "#123456",
        "value": "V",
        "spec": ["s"],
        "foot": "f",
        "render": None,
        "stock": "",
    }
    labs = [
        dict(lab, lid={"value": "V", "spec": ["s"]}) if i < n_lids else dict(lab)
        for i in range(n_labels)
    ]
    strips = [{"color": "#123456", "title": "T", "detail": "D"}] * n_strips
    return {
        "title": "t",
        "regmarks": dict(labels.DEFAULT_REG),
        "gap": labels.DEFAULT_GAP,
        "labels": labs,
        "strips": strips,
    }


class LayoutInvariants(unittest.TestCase):
    def assert_valid(self, data, pages):
        reg, gap = data["regmarks"], data["gap"]
        area = labels.Rect(reg["x"], reg["y"], reg["x"] + reg["w"], reg["y"] + reg["l"])
        blocked = labels.keepouts(reg)
        for page in pages:
            for i, p in enumerate(page):
                r = p.rect
                self.assertTrue(
                    area.x0 <= r.x0
                    and r.x1 <= area.x1
                    and area.y0 <= r.y0
                    and r.y1 <= area.y1,
                    p,
                )
                for b in blocked:
                    self.assertFalse(b.overlaps(r), f"{p} enters a mark keep-out {b}")
                for q in page[i + 1 :]:
                    grown = labels.Rect(
                        r.x0 - gap + 1e-6,
                        r.y0 - gap + 1e-6,
                        r.x1 + gap - 1e-6,
                        r.y1 + gap - 1e-6,
                    )
                    self.assertFalse(
                        grown.overlaps(q.rect), f"{p} and {q} closer than {gap} mm"
                    )

    def test_real_data(self):
        data = labels.load(DATA)
        pages = labels.layout(data)
        self.assert_valid(data, pages)
        self.assertEqual(sum(len(p) for p in pages), 55 + 19 + 8)

    def test_many_stickers_paginate(self):
        data = synthetic(120, 60, 30)
        pages = labels.layout(data)
        self.assertGreater(len(pages), 3)
        self.assert_valid(data, pages)
        self.assertEqual(sum(len(p) for p in pages), 120 + 60 + 30)

    def test_order_kept_and_kind_starts_row(self):
        pages = labels.layout(synthetic(2, 1, 1))
        kinds = [p.kind for p in pages[0]]
        self.assertEqual(kinds, ["label", "label", "lid", "strip"])
        lid, last_label = pages[0][2], pages[0][1]
        self.assertGreater(lid.y, last_label.y, "a new kind starts a new row")

    def test_clear_of_the_printed_marks(self):
        # Mark geometry stated independently of labels.keepouts(), from
        # render_silhouette_regmarks.py: a 5 mm square at (10, 10); 20 mm L arms
        # meeting at (200, 10) and (10, 287). Every sticker stays 2 mm off them.
        marks = [
            labels.Rect(10, 10, 15, 15),
            labels.Rect(180, 10, 200, 10),
            labels.Rect(200, 10, 200, 30),
            labels.Rect(10, 287, 30, 287),
            labels.Rect(10, 267, 10, 287),
        ]
        # The square's whole arm-length corner is the sensor's search window.
        marks.append(labels.Rect(10, 10, 30, 30))
        for page in labels.layout(labels.load(DATA)) + labels.layout(
            synthetic(120, 60, 30)
        ):
            for p in page:
                r = p.rect
                near = labels.Rect(r.x0 - 2, r.y0 - 2, r.x1 + 2, r.y1 + 2)
                for m in marks:
                    hit = (
                        near.x0 < m.x1
                        and m.x0 < near.x1
                        and near.y0 < m.y1
                        and m.y0 < near.y1
                    )
                    self.assertFalse(
                        hit, f"{p.kind} at ({p.x}, {p.y}) within 2 mm of a mark {m}"
                    )

    def test_sizes(self):
        self.assertEqual(
            labels.SIZES["label"][:2], (35.0, 35.0)
        )  # 2.5 mm in from a 40 mm face
        self.assertEqual(
            labels.SIZES["lid"][:2], (18.0, 18.0)
        )  # 1 mm in from a 20 mm face
        self.assertEqual(labels.SIZES["strip"][:2], (80.0, 20.0))  # full size

    def test_inside_portrait2_cut_width(self):
        # Graphtec.py declares the Portrait 2 at 203 mm cutting width.
        pages = labels.layout(labels.load(DATA))
        self.assertLessEqual(max(p.rect.x1 for page in pages for p in page), 203)


class DataValidation(unittest.TestCase):
    BASE = 'schema = 1\n[colors]\nc = "#112233"\n'

    def test_unknown_colour(self):
        p = write_data(
            self.BASE
            + '[[label]]\ncat="a"\ncolor="nope"\nvalue="v"\nspec="s"\nfoot="f"\n'
        )
        with self.assertRaisesRegex(labels.DataError, "not in \\[colors\\]"):
            labels.load(p)

    def test_unknown_render_kind(self):
        p = write_data(
            self.BASE
            + '[[label]]\ncat="a"\ncolor="c"\nvalue="v"\nspec="s"\nfoot="f"\nrender={kind="blob"}\n'
        )
        with self.assertRaisesRegex(labels.DataError, "render kind"):
            labels.load(p)

    def test_missing_field(self):
        p = write_data(
            self.BASE + '[[label]]\ncat="a"\ncolor="c"\nvalue="v"\nfoot="f"\n'
        )
        with self.assertRaisesRegex(labels.DataError, "missing spec"):
            labels.load(p)

    def test_schema(self):
        with self.assertRaisesRegex(labels.DataError, "schema"):
            labels.load(write_data("schema = 2\n"))

    def test_lid_defaults_to_label_value(self):
        p = write_data(
            self.BASE
            + '[[label]]\ncat="a"\ncolor="c"\nvalue="10 nF"\nspec="s"\nfoot="f"\nlid={spec="x"}\n'
        )
        self.assertEqual(
            labels.load(p)["labels"][0]["lid"], {"value": "10 nF", "spec": ["x"]}
        )

    def test_actual_size_default(self):
        p = write_data(
            self.BASE
            + '[[label]]\ncat="a"\ncolor="c"\nvalue="v"\nspec="s"\nfoot="f"\nrender={kind="chip",l=2,w=1,term=0.4}\n'
            '[[label]]\ncat="a"\ncolor="c"\nvalue="v"\nspec="s"\nfoot="f"\nrender={kind="module",l=20,w=10,pins=0,pcb="#000000"}\n'
        )
        a, b = labels.load(p)["labels"]
        self.assertTrue(a["render"]["actual"])
        self.assertFalse(b["render"]["actual"])

    def test_template_draws_every_kind(self):
        src = (HERE / "render.typ").read_text()
        for kind in labels.RENDER_KINDS:
            self.assertIn(f'r.kind == "{kind}"', src, f"render.typ cannot draw {kind}")


class CutFiles(unittest.TestCase):
    def setUp(self):
        self.out = Path(tempfile.mkdtemp())
        self.pages = labels.build(DATA, self.out)

    def test_one_file_per_page_one_path_per_sticker(self):
        self.assertEqual(len(list(self.out.glob("cut-p*.svg"))), len(self.pages))
        for n, page in enumerate(self.pages, 1):
            root = ET.parse(self.out / f"cut-p{n}.svg").getroot()
            self.assertEqual(root.get("width"), "210mm")
            self.assertEqual(root.get("viewBox"), "0 0 210 297")
            cut = [g for g in root.iter(f"{SVG}g") if g.get(f"{INK}label") == "Cut"][0]
            paths = cut.findall(f"{SVG}path")
            self.assertEqual(len(paths), len(page))
            for path, p in zip(paths, page):
                d = path.get("d")
                self.assertTrue(d.endswith("Z"), "cut paths are closed")
                x, y = (float(v) for v in d.split()[1].split(","))
                self.assertAlmostEqual(x, p.x + p.r)
                self.assertAlmostEqual(y, p.y)

    def test_regmark_layer_matches_driver(self):
        root = ET.parse(self.out / "cut-p1.svg").getroot()
        layer = [g for g in root.iter(f"{SVG}g") if g.get(f"{INK}label") == "Regmarks"][
            0
        ]
        ids = {e.get("id") for e in layer}
        self.assertTrue({"regmark-tl", "regmark-tr", "regmark-bl"} <= ids)
        notes = "".join(layer.find(f"{SVG}text").itertext())
        m = DRIVER_NOTES_RE.search(notes)
        self.assertIsNotNone(
            m, "the driver must be able to read the mark geometry back"
        )
        self.assertEqual(
            tuple(float(v) for v in m.groups()), (10.0, 10.0, 190.0, 277.0)
        )

    def test_plan_and_cut_share_positions(self):
        plan = json.loads((self.out / "plan.json").read_text())
        self.assertEqual(len(plan["pages"]), len(self.pages))
        for page_plan, page in zip(plan["pages"], self.pages):
            self.assertEqual(
                [(s["x"], s["y"], s["w"], s["h"]) for s in page_plan],
                [(p.x, p.y, p.w, p.h) for p in page],
            )
        self.assertEqual(plan["regmarks"]["arm"], labels.REG_ARM)

    def test_limit_cuts_first_stickers_only(self):
        out = Path(tempfile.mkdtemp())
        labels.build(DATA, out, limit=1)
        root = ET.parse(out / "cut-p1.svg").getroot()
        self.assertEqual(
            len(
                [
                    p
                    for p in root.iter(f"{SVG}path")
                    if p.get("id", "").startswith("cut-")
                ]
            ),
            1,
        )

    def test_rebuild_drops_stale_pages(self):
        (self.out / "cut-p9.svg").write_text("stale")
        labels.build(DATA, self.out)
        self.assertFalse((self.out / "cut-p9.svg").exists())


class Candidates(unittest.TestCase):
    def setUp(self):
        self.stock = Path(tempfile.mkdtemp())
        (self.stock / "items" / "capacitor").mkdir(parents=True)
        (self.stock / "items" / "capacitor" / "c-labelled.toml").write_text(
            'schema = 1\n[item]\nname = "x"\ncategory = "capacitor"\n[stock]\nmode = "count"\nqty = 1\nverified = "2026-11-01"\n'
        )
        (self.stock / "items" / "capacitor" / "c0603-1u.toml").write_text(
            'schema = 1\n[item]\nname = "0603 ceramic capacitor, 1 µF"\ncategory = "capacitor"\npackage = "0603"\n'
            '[stock]\nmode = "count"\nqty = 50\nverified = ""\n'
            '[specs]\nvalue = "1uF"\ndielectric = "X5R"\nunknown = ["voltage"]\n'
            '[[sources]]\norder = "aliexpress/2026-09-19-123"\nqty = 50\n'
        )
        self.data = write_data(
            'schema = 1\n[colors]\nc = "#112233"\n'
            '[[label]]\ncat="a"\ncolor="c"\nvalue="v"\nspec="s"\nfoot="f"\nstock="c-labelled"\n'
        )

    def test_only_unlabelled_items(self):
        out = labels.candidates(self.data, self.stock)
        self.assertIn("1 of 2 parts-stock items", out)
        self.assertNotIn('stock = "c-labelled"', out)
        self.assertIn('stock = "c0603-1u"', out)

    def test_draft_content(self):
        out = labels.candidates(self.data, self.stock)
        self.assertIn('value = "1uF"', out)
        self.assertIn('spec = ["0603 · X5R"]', out)
        self.assertIn('foot = "50 pcs · Aliexpress 09/2026"', out)
        self.assertIn("listing does not state: voltage", out)
        self.assertIn("not counted", out)

    def test_drafts_parse_as_data(self):
        out = labels.candidates(self.data, self.stock).replace(
            'color = "mech"', 'color = "c"'
        )
        p = write_data('schema = 1\n[colors]\nc = "#112233"\n' + out)
        self.assertEqual(len(labels.load(p)["labels"]), 1)

    def test_empty_stock_is_an_error(self):
        with self.assertRaisesRegex(labels.DataError, "no item files"):
            labels.candidates(self.data, Path(tempfile.mkdtemp()))


class DryCut(unittest.TestCase):
    def test_refuses_a_live_run(self):
        r = subprocess.run(
            [
                sys.executable,
                str(HERE / "dry_cut.py"),
                "/nonexistent/sendto_silhouette.py",
                "x.svg",
            ],
            capture_output=True,
            text=True,
        )
        self.assertNotEqual(r.returncode, 0)
        self.assertIn("only runs dry", r.stderr)


if __name__ == "__main__":
    unittest.main()
