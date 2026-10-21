"""kicad.measure on small synthetic board dumps (no KiCad needed).

The layer test is the one that matters: a path script that joined a B.Cu
track end to an F.Cu track it merely crossed reported "no via" for a path
that runs through a via, and the number reached a README. `LayerBlindTests`
shows the check fails when the layer is ignored.

The resistance test needs numpy and scipy and skips without them, as in
`just kicad::test`; run it with

    uv run --no-project --with numpy --with scipy python -m unittest kicad.test_measure

from tools/.
"""

from __future__ import annotations

import importlib.util
import json
import tempfile
import unittest
from contextlib import redirect_stdout
from io import StringIO
from pathlib import Path
from unittest import mock

from kicad import measure
from kicad.errors import KicadError

HAVE_NUMERIC = all(importlib.util.find_spec(m) for m in ("numpy", "scipy"))


def rect(cx, cy, w, h):
    return [
        [cx - w / 2, cy - h / 2],
        [cx + w / 2, cy - h / 2],
        [cx + w / 2, cy + h / 2],
        [cx - w / 2, cy + h / 2],
    ]


def pad(ref, num, net, at, size=(1.0, 1.0), layers=("F.Cu",), tht=False, drill=0.0):
    shape = {"outline": rect(at[0], at[1], *size), "holes": []}
    return {
        "ref": ref,
        "num": num,
        "net": net,
        "at": list(at),
        "tht": tht,
        "drill": drill,
        "layers": list(layers),
        "shapes": {layer: [shape] for layer in layers},
    }


def track(net, layer, a, b, width=0.25):
    return {"net": net, "layer": layer, "width": width, "a": list(a), "b": list(b)}


def via(net, at, size=0.6, drill=0.3):
    return {
        "net": net,
        "at": list(at),
        "size": size,
        "drill": drill,
        "layers": ["F.Cu", "B.Cu"],
    }


def zone(net, layer, *outlines):
    return {
        "net": net,
        "layer": layer,
        "priority": 0,
        "islands": [{"outline": o, "holes": []} for o in outlines],
    }


def board(tracks=(), vias=(), pads=(), zones=(), holes=(), spec=None):
    return {
        "tracks": list(tracks),
        "vias": list(vias),
        "pads": list(pads),
        "zones": list(zones),
        "holes": list(holes),
        "spec": spec or {},
    }


def through_via():
    """A:1 (F.Cu) -> F.Cu track -> via -> B.Cu track -> B:1 (B.Cu)."""
    return board(
        tracks=[
            track("N", "F.Cu", (0, 0), (5, 0)),
            track("N", "B.Cu", (5, 0), (10, 0)),
        ],
        vias=[via("N", (5, 0))],
        pads=[
            pad("A", "1", "N", (0, 0)),
            pad("B", "1", "N", (10, 0), layers=("B.Cu",)),
        ],
    )


def crossing():
    """An F.Cu track from A:1 and a B.Cu track to B:1 that cross at (5, 0) with
    no via there. A layer-blind join connects them; copper does not."""
    return board(
        tracks=[
            track("N", "F.Cu", (0, 0), (10, 0)),
            track("N", "B.Cu", (5, -5), (5, 5)),
        ],
        pads=[
            pad("A", "1", "N", (0, 0)),
            pad("B", "1", "N", (5, 5), layers=("B.Cu",)),
        ],
    )


class PathTests(unittest.TestCase):
    def test_a_path_through_a_via_counts_one_via(self):
        r = measure.path(through_via(), "N", "A:1", "B:1")
        self.assertTrue(r["connected"])
        self.assertEqual(r["vias"], 1)
        self.assertAlmostEqual(r["length_mm"], 10.0)
        self.assertEqual(r["layers"], ["B.Cu", "F.Cu"])

    def test_crossing_tracks_on_two_layers_do_not_connect(self):
        r = measure.path(crossing(), "N", "A:1", "B:1")
        self.assertFalse(r["connected"])

    def test_a_via_at_the_crossing_connects_them(self):
        b = crossing()
        b["vias"].append(via("N", (5, 0)))
        r = measure.path(b, "N", "A:1", "B:1")
        self.assertTrue(r["connected"])
        self.assertEqual(r["vias"], 1)

    def test_narrowest_and_the_shorter_route(self):
        b = through_via()
        # a longer detour on F.Cu with a thinner track, no via
        b["tracks"] += [
            track("N", "F.Cu", (0, 0), (0, 8), 0.15),
            track("N", "F.Cu", (0, 8), (10, 8), 0.15),
            track("N", "F.Cu", (10, 8), (10, 0), 0.15),
        ]
        b["pads"][1] = pad(
            "B", "1", "N", (10, 0), layers=("F.Cu", "B.Cu"), tht=True, drill=0.8
        )
        r = measure.path(b, "N", "A:1", "B:1")
        self.assertAlmostEqual(r["length_mm"], 10.0)
        self.assertEqual(r["narrowest_mm"], 0.25)

    def test_a_pad_on_another_net_is_refused(self):
        with self.assertRaises(KicadError):
            measure.path(through_via(), "OTHER", "A:1", "B:1")


class LayerBlindTests(unittest.TestCase):
    """The crossing test must fail against a layer-blind join."""

    def test_layer_blind_adjacency_would_join_the_crossing(self):
        real = measure.adjacency

        def blind(its):
            return real([measure.Item(it.key, "F.Cu", it.shape) for it in its])

        with mock.patch.object(measure, "adjacency", blind):
            r = measure.path(crossing(), "N", "A:1", "B:1")
        self.assertTrue(r["connected"], "the control: layer-blind joins the crossing")
        self.assertFalse(measure.path(crossing(), "N", "A:1", "B:1")["connected"])


class CutTests(unittest.TestCase):
    def test_the_only_via_is_a_cut(self):
        rows = measure.cuts(through_via(), "N", "A:1")
        self.assertEqual(len(rows), 1)
        self.assertEqual(rows[0]["via_at"], [5, 0])
        self.assertEqual(rows[0]["disconnects"], ["B:1"])

    def test_a_second_via_removes_the_cut(self):
        b = through_via()
        b["tracks"] += [
            track("N", "F.Cu", (5, 0), (7, 0)),
            track("N", "B.Cu", (7, 0), (10, 0)),
        ]
        b["vias"].append(via("N", (7, 0)))
        self.assertEqual(measure.cuts(b, "N", "A:1"), [])

    def test_the_pour_carries_current(self):
        b = through_via()
        b["vias"].append(via("N", (1, 0)))
        b["zones"].append(zone("N", "B.Cu", rect(5, 0, 12, 2)))
        self.assertEqual(measure.cuts(b, "N", "A:1"), [])


class ViaInPadTests(unittest.TestCase):
    def setUp(self):
        self.b = board(
            pads=[pad("U1", "3", "GND", (0, 0)), pad("U1", "4", "GND", (5, 0))],
            vias=[
                via(
                    "GND", (0.75, 0)
                ),  # ring reaches the pad edge at 0.5, drill does not
                via("GND", (5.4, 0)),  # drill reaches the pad
                via("GND", (10, 0)),  # clear
            ],
        )

    def test_ring_only_and_hole_in_pad(self):
        rows = {r["pad"]: r for r in measure.vias_in_pads(self.b)}
        self.assertEqual(sorted(rows), ["U1:3", "U1:4"])
        self.assertEqual(rows["U1:3"]["overlap"], "ring-only")
        self.assertEqual(rows["U1:4"]["overlap"], "hole-in-pad")

    def test_in_pad_entries_are_left_out(self):
        self.b["spec"] = {"fanout": {"in_pad": ["U1:4"]}}
        self.assertEqual([r["pad"] for r in measure.vias_in_pads(self.b)], ["U1:3"])

    def test_through_hole_pads_are_left_out(self):
        self.b["pads"] = [
            pad("J1", "1", "GND", (0, 0), tht=True, layers=("F.Cu", "B.Cu"))
        ]
        self.assertEqual(measure.vias_in_pads(self.b), [])


class HoleTests(unittest.TestCase):
    def test_nearest_copper_to_each_hole(self):
        b = board(
            tracks=[track("N", "B.Cu", (0, 2), (10, 2), 0.2)],
            zones=[zone("GND", "F.Cu", rect(20, 0, 4, 4))],
            holes=[
                {"ref": "MB1", "num": "", "at": [5, 0], "drill": 0.6},
                {"ref": "MB2", "num": "", "at": [20, 3], "drill": 0.6},
            ],
        )
        rows = {r["hole"]: r for r in measure.clearance_to_holes(b)}
        self.assertAlmostEqual(rows["MB1"]["clearance_mm"], 2 - 0.1 - 0.3)
        self.assertEqual(rows["MB1"]["layer"], "B.Cu")
        self.assertAlmostEqual(rows["MB2"]["clearance_mm"], 1 - 0.3)
        self.assertEqual(rows["MB2"]["nearest"], "GND fill")


class PlaneTests(unittest.TestCase):
    def split(self):
        """GND pour in two pieces on B.Cu, joined only by an F.Cu track
        between two vias: the right piece is tracks-only."""
        return board(
            zones=[
                zone("GND", "B.Cu", rect(5, 0, 10, 10), rect(25, 0, 10, 10)),
            ],
            vias=[via("GND", (8, 0)), via("GND", (22, 0))],
            tracks=[track("GND", "F.Cu", (8, 0), (22, 0))],
            spec={"pour": {"regions": [{"net": "GND"}]}},
        )

    def test_a_split_joined_by_a_track_is_flagged(self):
        rows = measure.plane(self.split())
        planes = [r for r in rows if r["kind"] == "plane"]
        self.assertEqual([r["status"] for r in planes], ["main", "tracks-only"])
        self.assertAlmostEqual(planes[0]["area_mm2"], 100.0)
        summary = rows[-1]
        self.assertEqual(summary["components"], 2)
        self.assertTrue(summary["flag"])

    def test_an_unjoined_piece_is_isolated(self):
        b = self.split()
        b["tracks"] = []
        statuses = [r["status"] for r in measure.plane(b) if r["kind"] == "plane"]
        self.assertEqual(statuses, ["main", "isolated"])

    def test_a_via_pair_through_a_poured_layer_joins_the_plane(self):
        b = self.split()
        b["zones"].append(zone("GND", "F.Cu", rect(15, 0, 20, 2)))
        b["tracks"] = []
        rows = measure.plane(b)
        self.assertEqual(rows[-1]["components"], 1)
        self.assertFalse(rows[-1]["flag"])

    def test_small_pieces_are_counted_not_listed(self):
        b = self.split()
        b["zones"][0]["islands"][1]["outline"] = rect(25, 0, 2, 2)
        rows = measure.plane(b)
        # the 4 mm2 island, and the via at x 22 it no longer reaches
        self.assertEqual(rows[-1]["below_min_area"], 2)
        self.assertEqual(rows[-1]["listed"], 1)
        self.assertFalse(rows[-1]["flag"])

    def test_region_area_subtracts_holes(self):
        r = measure.Region(
            tuple(map(tuple, rect(0, 0, 10, 10))),
            (tuple(map(tuple, rect(0, 0, 2, 2))),),
        )
        self.assertAlmostEqual(r.area, 96.0)


@unittest.skipUnless(HAVE_NUMERIC, "resistance needs numpy and scipy")
class ResistanceTests(unittest.TestCase):
    def test_a_straight_track_matches_its_squares(self):
        # 1 mm wide, 9 mm between the pads' inner edges: 9 squares of 1 oz copper
        b = board(
            tracks=[track("N", "F.Cu", (0, 0), (10, 0), 1.0)],
            pads=[pad("A", "1", "N", (0, 0)), pad("B", "1", "N", (10, 0))],
        )
        r = measure.resistance(b, "N", "A:1", "B:1")
        expect = 9 * measure.RHO_CU / measure.CU_1OZ_M * 1e3
        self.assertTrue(r["estimate"])
        self.assertAlmostEqual(r["milliohm"], expect, delta=0.1 * expect)

    def test_a_narrow_track_is_resolved(self):
        # 0.25 mm wide: at a 0.1 mm raster this read 26 % high
        b = board(
            tracks=[track("N", "F.Cu", (0, 0.0123), (10, 0.0123), 0.25)],
            pads=[pad("A", "1", "N", (0, 0)), pad("B", "1", "N", (10, 0))],
        )
        r = measure.resistance(b, "N", "A:1", "B:1")
        expect = 9 / 0.25 * measure.RHO_CU / measure.CU_1OZ_M * 1e3
        self.assertAlmostEqual(r["milliohm"], expect, delta=0.03 * expect)

    def test_crossing_layers_without_a_via_are_open(self):
        r = measure.resistance(crossing(), "N", "A:1", "B:1")
        self.assertFalse(r["connected"])


class CliTests(unittest.TestCase):
    def run_cli(self, b, *args):
        with tempfile.TemporaryDirectory() as tmp:
            p = Path(tmp) / "b.json"
            p.write_text(json.dumps(b), encoding="utf-8")
            out = StringIO()
            with redirect_stdout(out):
                self.assertEqual(measure.main([str(p), *args]), 0)
            return out.getvalue()

    def test_text_is_one_line_per_result(self):
        out = self.run_cli(through_via(), "path", "N", "A:1", "B:1")
        self.assertEqual(out.count("\n"), 1)
        self.assertIn("1 via,", out)

    def test_json(self):
        rows = json.loads(self.run_cli(through_via(), "cuts", "N", "A:1", "--json"))
        self.assertEqual(rows[0]["disconnects"], ["B:1"])

    def test_an_unknown_pad_fails_loudly(self):
        with tempfile.TemporaryDirectory() as tmp:
            p = Path(tmp) / "b.json"
            p.write_text(json.dumps(through_via()), encoding="utf-8")
            out = StringIO()
            with redirect_stdout(out):
                code = measure.main([str(p), "path", "N", "A:1", "Z:9"])
        self.assertEqual(code, 1)
        self.assertIn("FAIL: no pad Z:9", out.getvalue())

    def test_review_section_does_not_gate(self):
        out = self.run_cli(PlaneTests().split(), "plane", "--review")
        self.assertIn("1 plane flag(s)", out)
        self.assertIn("does not gate", out)


if __name__ == "__main__":
    unittest.main()
