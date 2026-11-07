"""Every pin a circuit block names exists on the registry part it names.

hardware.blocks checks a block's own structure (ports and rails name block
parts). Whether `fet.G` is a pin of the part behind `fet` is only checked when
a project expands the block and the netlist resolves it, so a block no carrier
uses yet would carry a misspelt pin until its first board. This closes that.

    python3 -m unittest discover -s tools/kicad -t tools
"""

from __future__ import annotations

import unittest
from pathlib import Path

from hardware.blocks import BLOCKS, Block, load_block

from kicad import registry
from kicad.paths import REPO_ROOT


def block_pin_errors(block: Block, repo_root: Path = REPO_ROOT) -> list[str]:
    """Each '<key>.<pin>' in the block's ports and rails whose part lacks the pin."""
    names: list[str] = [p for pins in block.ports.values() for p in pins]
    for rail in block.rails:
        names += [rail["from"], *rail.get("to", [])]
    errors = []
    for ref in names:
        key, pin = ref.split(".", 1)
        spec = registry.load(block.parts[key]["part"], repo_root)
        if pin not in spec.pins:
            errors.append(
                f"{block.slug}: {ref}: {spec.slug} has no pin {pin!r}; "
                f"its pins are {sorted(spec.pins)}"
            )
    return errors


class TestBlockPins(unittest.TestCase):
    def test_every_block_pin_is_a_registry_pin(self):
        paths = sorted((REPO_ROOT / BLOCKS).glob("*.toml"))
        self.assertTrue(paths, f"no blocks under {BLOCKS}")
        for path in paths:
            with self.subTest(block=path.stem):
                self.assertEqual(block_pin_errors(load_block(path.stem)), [])

    def test_a_misspelt_pin_fails(self):
        block = load_block("pack-input-pfet-ptc")
        bad = Block(
            block.slug,
            block.path,
            {**block.ports, "GND": ["gate.C"]},
            block.parts,
            block.rails,
        )
        errors = block_pin_errors(bad)
        self.assertEqual(len(errors), 1)
        self.assertIn("gate.C", errors[0])
        self.assertIn("r0805 has no pin 'C'", errors[0])


if __name__ == "__main__":
    unittest.main()
