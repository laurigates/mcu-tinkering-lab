"""Tests for tools/hardware/blocks.py: circuit blocks expanded into hardware.toml.

python3 -m unittest discover -s tools/hardware -t tools
"""

from __future__ import annotations

import tempfile
import textwrap
import unittest
from pathlib import Path

from hardware import HardwareError
from hardware.blocks import BLOCKS, expand, load_sidecar

REPO_ROOT = Path(__file__).resolve().parents[2]

DIVIDER = """
[block]
name = "Divider"
source = "test"

[ports]
TOP = ["top.A"]
MID = ["top.B", "bot.A"]
GND = ["bot.B", "cap.B"]

[parts.top]
name = "top resistor"
kind = "resistor"
part = "r0805"
value = "10k"

[parts.bot]
name = "bottom resistor"
kind = "resistor"
part = "r0805"
value = "10k"
lcsc = "C17414"

[parts.cap]
name = "filter cap"
kind = "capacitor"
part = "c0805"
value = "100nF"

[[rails]]
name = "FILT"
from = "bot.A"
to = ["cap.A"]
"""


class BlockCase(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.root = Path(self.tmp.name)
        self.write_block("divider", DIVIDER)

    def tearDown(self):
        self.tmp.cleanup()

    def write_block(self, slug: str, text: str) -> None:
        path = self.root / BLOCKS / f"{slug}.toml"
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(textwrap.dedent(text), encoding="utf-8")

    def instance(self, **extra) -> dict:
        inst = {
            "block": "divider",
            "net_prefix": "DIV",
            "refs": {"top": "R1", "bot": "R2", "cap": "C1"},
        }
        inst.update(extra)
        return inst

    def data(self, inst: dict | None = None, rails: list | None = None) -> dict:
        return {
            "parts": {"src": {"name": "source", "kind": "connector"}},
            "blocks": {"div": inst or self.instance()},
            "rails": rails
            if rails is not None
            else [
                {"name": "VIN", "from": "src.1", "to": ["div.TOP"]},
                {"name": "SENSE", "from": "div.MID", "to": ["src.2"]},
                {"name": "GND", "from": "src.3", "to": ["div.GND"]},
            ],
        }

    def expand(self, data: dict) -> dict:
        return expand(data, "hardware.toml", repo_root=self.root)


class TestExpand(BlockCase):
    def test_parts_take_instance_keys_and_refs(self):
        out = self.expand(self.data())
        self.assertNotIn("blocks", out)
        self.assertEqual(list(out["parts"]), ["src", "div_top", "div_bot", "div_cap"])
        self.assertEqual(out["parts"]["div_top"]["ref"], "R1")
        self.assertEqual(out["parts"]["div_bot"]["lcsc"], "C17414")
        self.assertEqual(out["parts"]["div_cap"]["part"], "c0805")

    def test_port_in_to_becomes_its_pins(self):
        out = self.expand(self.data())
        vin = next(r for r in out["rails"] if r["name"] == "VIN")
        self.assertEqual(vin, {"name": "VIN", "from": "src.1", "to": ["div_top.A"]})
        gnd = next(r for r in out["rails"] if r["name"] == "GND")
        self.assertEqual(gnd["to"], ["div_bot.B", "div_cap.B"])

    def test_multi_pin_port_in_from_moves_the_rest_to_to(self):
        out = self.expand(self.data())
        sense = next(r for r in out["rails"] if r["name"] == "SENSE")
        self.assertEqual(sense["from"], "div_top.B")
        self.assertEqual(sense["to"], ["div_bot.A", "src.2"])

    def test_internal_rail_gets_prefix_and_local_pins(self):
        out = self.expand(self.data())
        filt = next(r for r in out["rails"] if r["name"] == "DIV_FILT")
        self.assertEqual(
            filt, {"name": "DIV_FILT", "from": "div_bot.A", "to": ["div_cap.A"]}
        )

    def test_net_names_override_an_internal_name(self):
        out = self.expand(self.data(self.instance(net_names={"FILT": "VSENSE_F"})))
        self.assertIn("VSENSE_F", [r["name"] for r in out["rails"]])
        self.assertNotIn("DIV_FILT", [r["name"] for r in out["rails"]])

    def test_part_overrides(self):
        inst = self.instance(parts={"top": {"value": "56k", "lcsc": "C17756"}})
        out = self.expand(self.data(inst))
        self.assertEqual(out["parts"]["div_top"]["value"], "56k")
        self.assertEqual(out["parts"]["div_top"]["lcsc"], "C17756")
        self.assertEqual(out["parts"]["div_top"]["name"], "top resistor")

    def test_build_applies_to_every_entry(self):
        out = self.expand(self.data(self.instance(build="carrier")))
        for key in ("div_top", "div_bot", "div_cap"):
            self.assertEqual(out["parts"][key]["build"], "carrier")
        filt = next(r for r in out["rails"] if r["name"] == "DIV_FILT")
        self.assertEqual(filt["build"], "carrier")

    def test_ports_in_other_tables(self):
        data = self.data()
        data["nets"] = [{"role": "PIN_SENSE", "to": "div.MID"}]
        with self.assertRaisesRegex(HardwareError, "MID.*2 pins"):
            self.expand(data)
        self.write_block(
            "one", DIVIDER.replace('MID = ["top.B", "bot.A"]', 'MID = ["bot.A"]')
        )
        data = self.data(self.instance(block="one"))
        data["nets"] = [{"role": "PIN_SENSE", "to": "div.MID"}]
        self.assertEqual(self.expand(data)["nets"][0]["to"], "div_bot.A")

    def test_no_blocks_is_unchanged(self):
        data = {"parts": {"a": {"name": "a"}}, "rails": []}
        self.assertEqual(self.expand(data), data)

    def test_input_is_not_mutated(self):
        data = self.data()
        before = repr(data)
        self.expand(data)
        self.assertEqual(repr(data), before)


class TestErrors(BlockCase):
    def assertFails(self, data: dict, pattern: str) -> None:
        with self.assertRaisesRegex(HardwareError, pattern):
            self.expand(data)

    def test_unknown_block(self):
        self.assertFails(self.data(self.instance(block="nope")), "nope.*not found")

    def test_missing_ref(self):
        self.assertFails(
            self.data(self.instance(refs={"top": "R1", "bot": "R2"})), "no ref for.*cap"
        )

    def test_extra_ref(self):
        refs = {"top": "R1", "bot": "R2", "cap": "C1", "led": "D1"}
        self.assertFails(self.data(self.instance(refs=refs)), "led")

    def test_bad_ref(self):
        refs = {"top": "R1", "bot": "r2", "cap": "C1"}
        self.assertFails(self.data(self.instance(refs=refs)), "r2")

    def test_unknown_instance_key(self):
        self.assertFails(self.data(self.instance(colour="red")), "colour")

    def test_override_of_unknown_part(self):
        self.assertFails(self.data(self.instance(parts={"led": {"value": "x"}})), "led")

    def test_override_of_a_fixed_key(self):
        self.assertFails(
            self.data(self.instance(parts={"top": {"part": "c0805"}})), "part"
        )

    def test_unknown_port(self):
        rails = [
            {"name": "VIN", "from": "src.1", "to": ["div.TOP", "div.OUT"]},
            {"name": "SENSE", "from": "div.MID", "to": ["src.2"]},
            {"name": "GND", "from": "src.3", "to": ["div.GND"]},
        ]
        self.assertFails(self.data(rails=rails), "OUT")

    def test_unused_port(self):
        rails = [
            {"name": "VIN", "from": "src.1", "to": ["div.TOP"]},
            {"name": "GND", "from": "src.3", "to": ["div.GND"]},
        ]
        self.assertFails(self.data(rails=rails), "MID.*not connected")

    def test_part_key_collision(self):
        data = self.data()
        data["parts"]["div_top"] = {"name": "clash"}
        self.assertFails(data, "div_top")

    def test_rail_name_collision(self):
        data = self.data()
        data["rails"].append({"name": "DIV_FILT", "from": "src.4", "to": ["src.5"]})
        self.assertFails(data, "DIV_FILT")

    def test_net_names_for_unknown_rail(self):
        self.assertFails(self.data(self.instance(net_names={"NOPE": "X"})), "NOPE")

    def test_block_pin_on_unknown_part(self):
        self.write_block(
            "bad", DIVIDER.replace('GND = ["bot.B", "cap.B"]', 'GND = ["led.K"]')
        )
        self.assertFails(self.data(self.instance(block="bad")), "led")

    def test_block_part_with_a_ref(self):
        self.write_block(
            "bad", DIVIDER.replace('part = "c0805"', 'part = "c0805"\nref = "C9"')
        )
        self.assertFails(self.data(self.instance(block="bad")), "ref")


class TestLoadSidecar(BlockCase):
    def test_reads_and_expands(self):
        sidecar = self.root / "hardware.toml"
        sidecar.write_text(
            textwrap.dedent(
                """
                [parts.src]
                name = "source"

                [blocks.div]
                block = "divider"
                net_prefix = "DIV"
                refs = { top = "R1", bot = "R2", cap = "C1" }

                [[rails]]
                name = "VIN"
                from = "src.1"
                to = ["div.TOP"]

                [[rails]]
                name = "SENSE"
                from = "div.MID"
                to = ["src.2"]

                [[rails]]
                name = "GND"
                from = "src.3"
                to = ["div.GND"]
                """
            ),
            encoding="utf-8",
        )
        data = load_sidecar(sidecar, repo_root=self.root)
        self.assertIn("div_cap", data["parts"])

    def test_bad_toml(self):
        sidecar = self.root / "hardware.toml"
        sidecar.write_text("[parts\n", encoding="utf-8")
        with self.assertRaisesRegex(HardwareError, "hardware.toml"):
            load_sidecar(sidecar, repo_root=self.root)


class TestRepoBlocks(unittest.TestCase):
    """Every committed block parses, and every port pin names one of its parts."""

    def test_every_block_is_well_formed(self):
        from hardware.blocks import load_block

        paths = sorted((REPO_ROOT / BLOCKS).glob("*.toml"))
        self.assertTrue(paths, f"no blocks under {BLOCKS}")
        for path in paths:
            with self.subTest(block=path.stem):
                block = load_block(path.stem, repo_root=REPO_ROOT)
                self.assertTrue(block.ports)


if __name__ == "__main__":
    unittest.main()
