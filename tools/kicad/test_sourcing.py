"""Tests for kicad.sourcing: every carrier BOM part has a route onto the board.

python3 -m unittest discover -s tools/kicad -t tools
"""

from __future__ import annotations

import contextlib
import io
import tomllib
import unittest
from pathlib import Path
from unittest import mock

from kicad import paths, registry, sourcing
from kicad.errors import KicadError
from kicad.paths import REPO_ROOT
from kicad.sourcing import Row

ENTRY = """\
[part]
name = "x"
source = "test"
symbol = "Device:R"
footprint = "Resistor_SMD:R_0805_2012Metric"
buy = "{buy}"

[pins]
A = 1
B = 2
"""


class ClassifyTests(unittest.TestCase):
    def test_each_route_and_its_absence(self):
        report = sourcing.classify(
            [
                Row("R1", "10k", True, "C17414", "aliexpress:32901883276"),
                Row("Q1", "ZXMP4A16G", True, "C141324", ""),
                Row("J1", "XT30", True, "", "aliexpress:1005012170140774"),
                Row("J2", "header", True, "", ""),
                Row("H1", "M3", False, "", ""),
            ]
        )
        self.assertEqual([r.ref for r in report.unsourced], ["J2"])
        self.assertEqual([r.ref for r in report.jlc_only], ["Q1"])


class RegistryBuyTests(unittest.TestCase):
    def parse(self, buy: str) -> registry.PartSpec:
        text = ENTRY.format(buy=buy)
        return registry.parse(tomllib.loads(text), "x", Path("x.toml"))

    def test_each_form_parses(self):
        for buy in ("aliexpress:1000007373786", "lcsc:C141324", "owned:a reel"):
            with self.subTest(buy=buy):
                self.assertEqual(self.parse(buy).buy, buy)

    def test_another_form_fails(self):
        for buy in ("ali:1", "C141324", "lcsc:141324", "owned:", "aliexpress:12a"):
            with self.subTest(buy=buy), self.assertRaisesRegex(KicadError, "buy"):
                self.parse(buy)


class CarrierTests(unittest.TestCase):
    """The gate itself, over every tracked carrier project."""

    def test_every_bom_part_on_every_carrier_has_a_source(self):
        projects = paths.tracked_projects(REPO_ROOT)
        self.assertTrue(projects)
        for project in projects:
            with self.subTest(project=project.name):
                report = sourcing.classify(sourcing.rows(project))
                self.assertEqual([r.ref for r in report.unsourced], [])

    def test_balancebot_leaves_only_q1_and_f1_to_jlcpcb(self):
        # 2026-10-30: AliExpress sells neither the ZXMP4A16G nor a rated 24 V
        # 1812 PTC, so a hand-built balancebot must have JLCPCB or LCSC supply
        # them (.claude/rules/part-sourcing.md). Adding `buy` to either, or a
        # new part with only an LCSC number, changes this list on purpose.
        project = REPO_ROOT / "packages/robotics/balancebot"
        report = sourcing.classify(sourcing.rows(project))
        self.assertEqual(sorted(r.ref for r in report.jlc_only), ["F1", "Q1"])

    def test_the_cli_exit_code_follows_the_unsourced_parts(self):
        project = "packages/robotics/balancebot"
        # The failing run names the part; a part with `buy` is not listed.
        for found, code in (
            ([Row("J2", "header", True, "", "")], 1),
            ([Row("J1", "XT30", True, "", "aliexpress:1")], 0),
        ):
            out = io.StringIO()
            with (
                self.subTest(code=code),
                mock.patch.object(sourcing, "rows", return_value=found),
                contextlib.redirect_stdout(out),
            ):
                self.assertEqual(sourcing.main([project]), code)
                self.assertEqual(found[0].ref in out.getvalue(), code == 1)


if __name__ == "__main__":
    unittest.main()
