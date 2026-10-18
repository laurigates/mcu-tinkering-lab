"""Placement repeats: build_board.py twice on one input exports the same DSN (#763).

KiCad 10.0.6 saves a board's footprints in UUID order, and its Specctra DSN
export follows that order: images in the order their first footprint appears,
each image's places in UUID order. build_board.py therefore takes each
footprint's UUID from the board input (uuid5 of the carrier directory and
`fp/<ref>`) instead of letting KiCad draw one, and these tests pin that two
placement runs export byte-identical DSN, which also equals
`testdata/balancebot-carrier.dsn` (line 1, the output path, normalised).

They need KiCad 10.0.6's bundled Python ($KICAD_PYTHON, else the macOS
bundle's, as in tools/kicad.just) and skip without it, or when its pcbnew
reports another version (the fixture is 10.0.6's export), so CI, which has no
KiCad, and a machine with another KiCad skip them; run them locally after
touching `pcb/build_board.py` or the board input. The
input is the frozen balancebot (`kicad.testfixture`), not the live project.

    python3 -m unittest discover -s tools/kicad -t tools
"""

from __future__ import annotations

import json
import os
import re
import subprocess
import tempfile
import unittest
from pathlib import Path

from kicad import boardinput, testfixture

HERE = Path(__file__).resolve().parent
FIXTURE = HERE / "testdata/balancebot-carrier.dsn"
BUILD = HERE / "pcb/build_board.py"
MACOS_PYTHON = (
    "/Applications/KiCad/KiCad.app/Contents/Frameworks/"
    "Python.framework/Versions/Current/bin/python3"
)
KICAD_PYTHON = os.environ.get("KICAD_PYTHON", MACOS_PYTHON)
KICAD_VERSION = "10.0.6"
EXPORT = (
    "import pcbnew, sys\n"
    "print(pcbnew.Version())\n"
    "sys.exit(not pcbnew.ExportSpecctraDSN(pcbnew.LoadBoard(sys.argv[1]), sys.argv[2]))\n"
)


def skip_reason(python: str) -> str | None:
    """Why RepeatTests cannot run under `python`, or None when its pcbnew is 10.0.6."""
    need = f"needs KiCad {KICAD_VERSION}'s bundled Python ($KICAD_PYTHON or {MACOS_PYTHON})"
    if not Path(python).is_file():
        return f"{need}; {python} is absent"
    done = subprocess.run(
        [python, "-c", "import pcbnew; print(pcbnew.Version())"],
        capture_output=True,
        text=True,
    )
    lines = done.stdout.strip().splitlines()
    if done.returncode != 0 or not lines:
        return f"{need}; {python} has no pcbnew: {done.stderr.strip()[-200:]}"
    found = lines[-1].strip()
    if found != KICAD_VERSION:
        return f"{need}; found {found} at {python}"
    return None


def run(argv: list[str]) -> str:
    done = subprocess.run(argv, capture_output=True, text=True)
    if done.returncode != 0:
        raise AssertionError(
            f"{argv[1]} exit {done.returncode}:\n{done.stdout}{done.stderr}"
        )
    return done.stdout


def normalise(dsn: str) -> str:
    """The export with line 1, which carries the output path, as the fixture has it.

    A distribution build of 10.0.6 also names its package in `host_version`
    (Ubuntu's: "10.0.6-10.0.6~ubuntu24.04.1"); that suffix is cut back to the
    fixture's "10.0.6". Another version stays as it is and fails the comparison.
    """
    first, rest = dsn.split("\n", 1)
    if not first.startswith("(pcb "):
        raise AssertionError(f"DSN line 1 is {first!r}, not '(pcb <path>'")
    rest = re.sub(
        rf'\(host_version "{re.escape(KICAD_VERSION)}-[^"]*"\)',
        f'(host_version "{KICAD_VERSION}")',
        rest,
    )
    return "(pcb carrier.dsn\n" + rest


def place_and_export(spec: dict, scratch: Path) -> tuple[str, str]:
    """build_board.py on `spec` into `scratch`, then the DSN export; (version, dsn)."""
    spec = dict(spec, pcb=str(scratch / "carrier.kicad_pcb"))
    board_input = scratch / "board-input.json"
    board_input.write_text(json.dumps(spec))
    run([KICAD_PYTHON, str(BUILD), str(board_input)])
    out = scratch / "carrier.dsn"
    version = run([KICAD_PYTHON, "-c", EXPORT, spec["pcb"], str(out)]).strip()
    return version.splitlines()[-1], normalise(out.read_text())


SKIP = skip_reason(KICAD_PYTHON)


@unittest.skipIf(SKIP is not None, SKIP or "")
class RepeatTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        root, project = testfixture.shared()
        spec = boardinput.build(project, root)
        cls.exports = []
        for _ in range(2):
            with tempfile.TemporaryDirectory(prefix="kicad-763-") as tmp:
                cls.exports.append(place_and_export(spec, Path(tmp)))

    def test_kicad_version(self):
        for version, _ in self.exports:
            self.assertEqual(version, KICAD_VERSION, "the fixture is KiCad 10.0.6's")

    def test_two_builds_export_the_same_dsn(self):
        (_, first), (_, second) = self.exports
        self.assertEqual(first, second)

    def test_export_equals_the_fixture(self):
        self.assertEqual(self.exports[0][1], FIXTURE.read_text())


@unittest.skipIf(SKIP is not None, SKIP or "")
class KeepoutTests(unittest.TestCase):
    """A board.toml keepout reaches the board as a rule area and Freerouting's DSN (#750).

    The frozen balancebot input with one keepout band and one cut line added;
    the board is never routed here, so the placement overlapping the band is
    irrelevant.
    """

    BAND = {"name": "cut band", "rect": [40.0, 0.0, 43.0, 62.0]}

    @classmethod
    def setUpClass(cls):
        root, project = testfixture.shared()
        spec = boardinput.build(project, root)
        spec["keepouts"] = [cls.BAND]
        spec["silk"] = dict(spec["silk"], cut_lines=[[41.5, 0.5, 41.5, 61.5]])
        with tempfile.TemporaryDirectory(prefix="kicad-keepout-") as tmp:
            _, cls.dsn = place_and_export(spec, Path(tmp))
            cls.pcb = (Path(tmp) / "carrier.kicad_pcb").read_text()

    def test_rule_area_on_every_copper_layer(self):
        start = self.pcb.rindex("\n\t(zone\n", 0, self.pcb.index('(name "cut band")'))
        zone = self.pcb[start : self.pcb.index("\n\t)\n", start)]
        for needle in (
            '(layers "F.Cu" "B.Cu")',
            "(tracks not_allowed)",
            "(vias not_allowed)",
            "(pads not_allowed)",
            "(copperpour not_allowed)",
            "(footprints not_allowed)",
            "(xy 140 100) (xy 143 100) (xy 143 162) (xy 140 162)",
        ):
            self.assertIn(needle, zone)

    def test_dsn_carries_the_keepout_on_both_layers(self):
        # Specctra coordinates are um with y up; the band is x 140..143 mm. The
        # export names it "", and a structure `keepout` bars wires and vias.
        structure = self.dsn[: self.dsn.index("  (placement")]
        for layer in ("F.Cu", "B.Cu"):
            with self.subTest(layer=layer):
                self.assertIn(
                    f'(keepout "" (polygon {layer} 0  140000 -100000  143000 -100000'
                    "  143000 -162000  140000 -162000",
                    structure,
                )

    def test_cut_line_and_label_on_both_silkscreens(self):
        for layer in ("F.SilkS", "B.SilkS"):
            with self.subTest(layer=layer):
                self.assertIn(
                    f"(start 141.5 100.5)\n\t\t(end 141.5 161.5)\n\t\t(stroke\n"
                    f"\t\t\t(width 0.15)\n\t\t\t(type default)\n\t\t)\n"
                    f'\t\t(layer "{layer}")',
                    self.pcb,
                )
        self.assertEqual(self.pcb.count('(gr_text "CUT"'), 2)


@unittest.skipIf(SKIP is not None, SKIP or "")
class CutoutTests(unittest.TestCase):
    """A board.toml cutout is an Edge.Cuts circle and a keepout in the DSN (#750).

    The frozen balancebot input with one 2 mm cutout added; never routed.
    """

    @classmethod
    def setUpClass(cls):
        root, project = testfixture.shared()
        spec = boardinput.build(project, root)
        spec["cutouts"] = [{"name": "relief", "center": [41.5, 30.0], "diameter": 2.0}]
        with tempfile.TemporaryDirectory(prefix="kicad-cutout-") as tmp:
            _, cls.dsn = place_and_export(spec, Path(tmp))
            cls.pcb = (Path(tmp) / "carrier.kicad_pcb").read_text()

    def test_circle_on_edge_cuts(self):
        self.assertEqual(self.pcb.count("(gr_circle"), 1)
        start = self.pcb.index("(gr_circle")
        circle = self.pcb[start : self.pcb.index("(uuid", start)]
        for needle in ("(center 141.5 130)", "(end 142.5 130)", '(layer "Edge.Cuts")'):
            self.assertIn(needle, circle)

    def test_dsn_keeps_the_outline_and_keeps_routing_out_of_the_hole(self):
        # Specctra um, y up: the boundary stays the outline rectangle, and the
        # hole is a keepout polygon on every signal layer around (141.5, 130) mm.
        structure = self.dsn[: self.dsn.index("  (placement")]
        self.assertIn(
            "(path pcb 0  190000 -162000  100000 -162000  100000 -100000  190000 -100000",
            structure,
        )
        start = structure.index('(keepout "" (polygon signal 0 ')
        body = structure[start : structure.index(")", start)].split("signal 0", 1)[1]
        xy = [int(v) for v in body.split()]
        points = list(zip(xy[0::2], xy[1::2], strict=True))
        self.assertGreater(len(points), 8)
        for x, y in points:
            r = ((x - 141500) ** 2 + (y + 130000) ** 2) ** 0.5
            self.assertTrue(990 <= r <= 1010, (x, y, r))


@unittest.skipIf(SKIP is not None, SKIP or "")
class BackSideTests(unittest.TestCase):
    """A placement's fourth element "back" mounts the footprint on B.Cu (#750).

    robocar-unified's head carries the IDC ribbon header on the back face.
    """

    @classmethod
    def setUpClass(cls):
        root, project = testfixture.shared()
        spec = boardinput.build(project, root)
        for comp in spec["components"]:
            if comp["ref"] == "R2":
                cls.at = comp["at"]
                comp["at"] = [*comp["at"], "back"]
        # A reference position on a back-side part is where the label lands.
        cls.ref_at = [cls.at[0] + 3.0, cls.at[1] + 4.0]
        spec["silk"] = dict(
            spec["silk"],
            ref_positions=dict(spec["silk"]["ref_positions"], R2=cls.ref_at),
        )
        with tempfile.TemporaryDirectory(prefix="kicad-back-") as tmp:
            _, cls.dsn = place_and_export(spec, Path(tmp))
            cls.pcb = (Path(tmp) / "carrier.kicad_pcb").read_text()

    def test_dsn_places_it_on_the_back_at_the_same_point(self):
        # Specctra gives a back-side part's rotation as seen from below, so only
        # the side and the point are compared here.
        x, y, _ = self.at
        origin = 100.0  # the frozen balancebot outline's origin, both axes
        self.assertIn(
            f"(place R2 {round((origin + x) * 1000)} {-round((origin + y) * 1000)} back ",
            self.dsn,
        )

    def test_board_footprint_is_on_b_cu_with_the_given_rotation(self):
        x, y, rot = self.at
        start = self.pcb.rindex(
            "\n\t(footprint ", 0, self.pcb.index('"Reference" "R2"')
        )
        head = self.pcb[start : self.pcb.index('"Reference" "R2"')]
        self.assertIn('(layer "B.Cu")', head)
        at = f"(at {100 + x:g} {100 + y:g}" + (f" {rot:g})" if rot else ")")
        self.assertIn(at, head)

    def test_reference_position_is_not_mirrored_by_the_flip(self):
        # KiCad stores a field's position relative to its footprint.
        dx = self.ref_at[0] - self.at[0]
        dy = self.ref_at[1] - self.at[1]
        self.assertIn(f'(property "Reference" "R2"\n\t\t\t(at {dx:g} {dy:g} ', self.pcb)

    def test_every_other_part_stays_on_the_front(self):
        backs = [line for line in self.dsn.splitlines() if " back " in line]
        self.assertEqual(len(backs), 1, backs)


class SkipTests(unittest.TestCase):
    """RepeatTests skips unless $KICAD_PYTHON's pcbnew is exactly KiCad 10.0.6."""

    def stub(self, scratch: Path, version: str) -> tuple[Path, Path]:
        """An executable stand-in for KiCad's Python that reports `version`."""
        mark = scratch / "stub-ran"
        stub = scratch / "python3"
        stub.write_text(f"#!/bin/sh\ntouch '{mark}'\necho '{version}'\n")
        stub.chmod(0o755)
        self.assertTrue(os.access(stub, os.X_OK), "the stub must be executable")
        return stub, mark

    def test_another_version_skips_naming_both(self):
        with tempfile.TemporaryDirectory(prefix="kicad-skip-") as tmp:
            stub, mark = self.stub(Path(tmp), "9.0.1")
            reason = skip_reason(str(stub))
            self.assertTrue(mark.exists(), "the stub, not a real KiCad, answered")
        self.assertIsNotNone(reason)
        self.assertIn("9.0.1", reason)
        self.assertIn(KICAD_VERSION, reason)

    def test_the_pinned_version_runs(self):
        with tempfile.TemporaryDirectory(prefix="kicad-skip-") as tmp:
            stub, mark = self.stub(Path(tmp), KICAD_VERSION)
            self.assertIsNone(skip_reason(str(stub)))
            self.assertTrue(mark.exists())

    def test_missing_python_skips(self):
        reason = skip_reason("/nonexistent/kicad/python3")
        self.assertIsNotNone(reason)
        self.assertIn(KICAD_VERSION, reason)

    def test_module_skips_under_another_version(self):
        """$KICAD_PYTHON reaches the decorator: the whole module passes, RepeatTests skipped."""
        with tempfile.TemporaryDirectory(prefix="kicad-skip-") as tmp:
            stub, mark = self.stub(Path(tmp), "9.0.1")
            env = dict(os.environ, KICAD_PYTHON=str(stub), PYTHONPATH=str(HERE.parent))
            done = subprocess.run(
                [
                    "python3",
                    "-m",
                    "unittest",
                    "-v",
                    "kicad.test_build_board.RepeatTests",
                ],
                capture_output=True,
                text=True,
                env=env,
                cwd=HERE.parent.parent,
            )
            self.assertTrue(mark.exists(), "the stub, not a real KiCad, answered")
        self.assertEqual(done.returncode, 0, done.stderr)
        self.assertIn("skipped", done.stderr)
        self.assertIn("found 9.0.1", done.stderr)


if __name__ == "__main__":
    unittest.main()
