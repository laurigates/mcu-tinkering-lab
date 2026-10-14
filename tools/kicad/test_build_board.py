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
