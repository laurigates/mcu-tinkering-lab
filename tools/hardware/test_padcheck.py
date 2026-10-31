"""Tests for tools/hardware/padcheck.py — the pad-centre re-derivation check (#750).

Offline: the network fetch and the breakout-pinout.py run are the recipe's job
(`just hardware::pad-centres-check`); these cover what it compares, and that a
failed fetch or run (mocked) ends in one `error:` line rather than a traceback.

    python3 -m unittest discover -s tools/hardware -t tools
"""

from __future__ import annotations

import contextlib
import email.message
import io
import subprocess
import tempfile
import textwrap
import unittest
import urllib.error
from pathlib import Path
from unittest import mock

from hardware import HardwareError
from hardware.layout import BOARDS_DIR
from hardware.padcheck import (
    centre_pages,
    diff,
    expected_centres,
    main,
    raw_url,
    source,
    table,
)

BREAKOUTS = (
    "adafruit-max98357a",
    "adafruit-pca9685",
    "adafruit-tca9548a",
    "sparkfun-tb6612fng",
)

# The shape `tools/breakout-pinout.py --json` prints: Eagle coordinates, +y up,
# rounded to 0.01 mm, so a pad on y=0 can come out as -0.0.
PINOUT_JSON = {
    "elements": [
        {
            "element": "JP1",
            "pins": [
                {"pad": "2", "x": 0.0, "y": 2.54, "net": "GND"},
                {"pad": "1", "x": -0.0, "y": -0.0, "net": "VCC"},
            ],
        },
        {
            "element": "U1",
            "pins": [{"pad": "1", "x": 5.0, "y": 5.0, "net": "-"}],
        },
    ],
    "silkscreen": [],
}


class ExpectedCentresTest(unittest.TestCase):
    def test_y_is_flipped_into_kicad_convention(self):
        got = expected_centres(PINOUT_JSON, {"JP1"})
        self.assertEqual(got[("JP1", "2")], (0.0, -2.54))

    def test_negative_zero_is_normalised(self):
        # "-0.00" in a page would read as a different number to a diff.
        x, y = expected_centres(PINOUT_JSON, {"JP1"})[("JP1", "1")]
        self.assertEqual(f"{x:.2f} {y:.2f}", "0.00 0.00")

    def test_only_the_requested_headers_are_kept(self):
        self.assertNotIn(("U1", "1"), expected_centres(PINOUT_JSON, {"JP1"}))

    def test_a_requested_header_missing_from_the_board_file_is_an_error(self):
        with self.assertRaisesRegex(HardwareError, "JP7"):
            expected_centres(PINOUT_JSON, {"JP1", "JP7"})


class DiffTest(unittest.TestCase):
    def test_identical_centres_have_no_diff(self):
        centres = {("JP1", "1"): (0.0, 0.0)}
        self.assertEqual(diff(centres, dict(centres)), [])

    def test_a_moved_pad_is_reported_with_both_positions(self):
        lines = diff({("JP1", "1"): (0.0, 0.0)}, {("JP1", "1"): (0.0, 2.54)})
        self.assertEqual(len(lines), 1)
        self.assertIn("JP1.1", lines[0])
        self.assertIn("2.54", lines[0])

    def test_missing_and_extra_pads_are_reported(self):
        lines = diff({("JP1", "1"): (0.0, 0.0)}, {("JP1", "2"): (0.0, 0.0)})
        self.assertEqual(len(lines), 2)
        self.assertTrue(any("JP1.1" in line for line in lines))
        self.assertTrue(any("JP1.2" in line for line in lines))

    def test_a_sub_hundredth_difference_is_not_a_diff(self):
        # The page records 0.01 mm, the resolution breakout-pinout.py prints.
        self.assertEqual(
            diff({("JP1", "1"): (2.54, 0.0)}, {("JP1", "1"): (2.5400001, 0.0)}),
            [],
        )


class TableTest(unittest.TestCase):
    def test_table_rows_parse_back_to_the_same_centres(self):
        # The authoring output must be exactly what the reader accepts.
        from hardware import parse_pad_centres

        centres = expected_centres(PINOUT_JSON, {"JP1"})
        page = (
            "| Pin | Side | Pos | Header |\n|---|---|---|---|\n"
            "| VCC | L | 1 | JP1.1 |\n| GND | L | 2 | JP1.2 |\n\n" + table(centres)
        )
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "part.md"
            path.write_text(page)
            parsed = parse_pad_centres(path)
        self.assertEqual(
            {(c.element, c.pad): (c.x, c.y) for row in parsed.values() for c in row},
            centres,
        )


class SourceTest(unittest.TestCase):
    def write(self, tmp: str, text: str) -> Path:
        path = Path(tmp) / "part.md"
        path.write_text(textwrap.dedent(text))
        return path

    def test_every_breakout_page_names_one_repo_file_and_commit(self):
        for slug in BREAKOUTS:
            with self.subTest(slug):
                src = source(BOARDS_DIR / f"{slug}.md")
                self.assertRegex(src.repo, r"^[\w.-]+/[\w.-]+$")
                self.assertTrue(src.path.endswith(".brd"))
                self.assertRegex(src.commit, r"^[0-9a-f]{40}$")

    def test_the_url_pins_the_commit_and_quotes_the_path(self):
        src = source(BOARDS_DIR / "adafruit-pca9685.md")
        url = raw_url(src)
        self.assertIn(src.commit, url)
        self.assertIn("Adafruit%20PCA9685%20rev%20C.brd", url)
        self.assertTrue(url.startswith("https://raw.githubusercontent.com/adafruit/"))

    def test_a_page_with_two_commits_in_its_source_is_rejected(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = self.write(
                tmp,
                f"""\
                ## Source

                [`a/b`](https://github.com/a/b), `x.brd` at commit `{"1" * 40}`,
                or `{"2" * 40}`.
                """,
            )
            with self.assertRaisesRegex(HardwareError, "commit"):
                source(path)

    def test_a_page_with_no_source_section_is_rejected(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = self.write(tmp, "# nothing\n")
            with self.assertRaisesRegex(HardwareError, "Source"):
                source(path)


class CentrePagesTest(unittest.TestCase):
    def test_the_four_robocar_breakouts_are_the_pages_with_centres(self):
        self.assertEqual(tuple(p.stem for p in centre_pages()), BREAKOUTS)


class OfflineTest(unittest.TestCase):
    """A fetch or parser failure is a one-line `error:` exit, not a traceback."""

    def run_main(self, argv: list[str]) -> tuple[int, str, str]:
        out, err = io.StringIO(), io.StringIO()
        with contextlib.redirect_stdout(out), contextlib.redirect_stderr(err):
            code = main(argv)
        return code, out.getvalue(), err.getvalue()

    def assert_one_error_line(self, argv: list[str], needle: str) -> None:
        code, _, err = self.run_main(argv)
        self.assertEqual(code, 1)
        lines = err.splitlines()
        self.assertEqual(len(lines), 1, err)
        self.assertTrue(lines[0].startswith("error: "), err)
        self.assertIn(needle, lines[0])

    def test_network_failures(self):
        url = raw_url(source(BOARDS_DIR / f"{BREAKOUTS[0]}.md"))
        http_error = urllib.error.HTTPError(
            url, 404, "Not Found", email.message.Message(), None
        )
        # Python 3.15 warns when an unclosed HTTPError is collected, and the
        # warning lands in whichever later test captures stderr when GC runs.
        self.addCleanup(http_error.close)
        failures = {
            "URLError": urllib.error.URLError("nodename nor servname provided"),
            "HTTPError": http_error,
            "timeout": TimeoutError("timed out"),
        }
        for name, exc in failures.items():
            for argv in (["--print", BREAKOUTS[0]], []):
                with (
                    self.subTest(failure=name, argv=argv),
                    mock.patch(
                        "hardware.padcheck.urllib.request.urlopen", side_effect=exc
                    ),
                ):
                    self.assert_one_error_line(argv, "could not fetch")

    def test_parser_that_cannot_start(self):
        resp = mock.MagicMock()
        resp.__enter__.return_value.read.return_value = b"<eagle/>"
        with (
            mock.patch("hardware.padcheck.urllib.request.urlopen", return_value=resp),
            mock.patch(
                "hardware.padcheck.subprocess.run",
                side_effect=FileNotFoundError(2, "No such file or directory"),
            ),
        ):
            self.assert_one_error_line(
                ["--print", BREAKOUTS[0]], "could not run breakout-pinout.py"
            )

    def test_parser_that_times_out(self):
        resp = mock.MagicMock()
        resp.__enter__.return_value.read.return_value = b"<eagle/>"
        with (
            mock.patch("hardware.padcheck.urllib.request.urlopen", return_value=resp),
            mock.patch(
                "hardware.padcheck.subprocess.run",
                side_effect=subprocess.TimeoutExpired("breakout-pinout.py", 60),
            ),
        ):
            self.assert_one_error_line(
                ["--print", BREAKOUTS[0]], "could not run breakout-pinout.py"
            )


if __name__ == "__main__":
    unittest.main()
