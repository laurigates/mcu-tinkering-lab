"""Placement repeats: build_board.py twice on one input exports the same DSN (#763).

KiCad 10.0.6 saves a board's footprints in UUID order, and its Specctra DSN
export follows that order: images in the order their first footprint appears,
each image's places in UUID order. build_board.py therefore takes each
footprint's UUID from the board input (uuid5 of the carrier directory and
`fp/<ref>`) instead of letting KiCad draw one, and these tests pin that two
placement runs export byte-identical DSN, which also equals
`testdata/balancebot-carrier.dsn` (line 1, the output path, normalised).

They need KiCad's bundled Python ($KICAD_PYTHON, else the macOS bundle's, as in
tools/kicad.just) and skip without it, so CI, which has no KiCad, skips them;
run them locally after touching `pcb/build_board.py` or the board input. The
input is the frozen balancebot (`kicad.testfixture`), not the live project.

    python3 -m unittest discover -s tools/kicad -t tools
"""

from __future__ import annotations

import json
import os
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


def run(argv: list[str]) -> str:
    done = subprocess.run(argv, capture_output=True, text=True)
    if done.returncode != 0:
        raise AssertionError(
            f"{argv[1]} exit {done.returncode}:\n{done.stdout}{done.stderr}"
        )
    return done.stdout


def normalise(dsn: str) -> str:
    """The export with line 1, which carries the output path, as the fixture has it."""
    first, rest = dsn.split("\n", 1)
    if not first.startswith("(pcb "):
        raise AssertionError(f"DSN line 1 is {first!r}, not '(pcb <path>'")
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


@unittest.skipUnless(
    Path(KICAD_PYTHON).is_file(),
    f"needs KiCad {KICAD_VERSION}'s bundled Python ($KICAD_PYTHON or {MACOS_PYTHON})",
)
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


if __name__ == "__main__":
    unittest.main()
