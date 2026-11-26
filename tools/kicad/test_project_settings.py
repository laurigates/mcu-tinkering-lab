"""Hand-kept carrier project files: `.kicad_dru` rules the `.kicad_pro` overrides.

KiCad 10's starved-thermal check reads the `.kicad_pro`'s `min_resolved_spokes`
and ignores a lower `min_resolved_spokes` constraint in the `.kicad_dru`
(carrier-pcb skill, trap 61; #1280, #1302). A rule asking for fewer spokes than
the project setting therefore does nothing, and a route that leaves the rule's
count fails DRC. Hold the two equal for every committed carrier.
"""

import json
import re
import subprocess
import unittest
from pathlib import Path

from kicad.paths import REPO_ROOT

SPOKES = re.compile(r"\(constraint\s+min_resolved_spokes\s+(\d+)\)")


def rule_spokes(text: str) -> set[int]:
    """Every `min_resolved_spokes` constraint value in a `.kicad_dru`."""
    return {int(n) for n in SPOKES.findall(text)}


def project_spokes(text: str) -> int:
    """The `.kicad_pro`'s board design setting."""
    return json.loads(text)["board"]["design_settings"]["rules"]["min_resolved_spokes"]


def committed_rule_files(repo_root: Path = REPO_ROOT) -> list[Path]:
    out = subprocess.run(
        ["git", "ls-files", "-z", "*.kicad_dru"],
        cwd=repo_root,
        capture_output=True,
        check=True,
        text=True,
    ).stdout
    return [
        repo_root / p
        for p in out.split("\0")
        if p and not p.startswith("tools/kicad/testdata/")
    ]


class ParseTests(unittest.TestCase):
    def test_rule_spokes(self):
        text = (
            '(rule "a"\n\t(condition "A.Type == \'Pad\'")\n'
            "\t(constraint min_resolved_spokes 1))\n"
        )
        self.assertEqual(rule_spokes(text), {1})
        self.assertEqual(rule_spokes("(version 1)\n"), set())

    def test_project_spokes(self):
        pro = {"board": {"design_settings": {"rules": {"min_resolved_spokes": 2}}}}
        self.assertEqual(project_spokes(json.dumps(pro)), 2)


class CommittedCarrierTests(unittest.TestCase):
    def test_some_carriers_found(self):
        self.assertTrue(committed_rule_files())

    def test_project_spokes_match_the_rule(self):
        for dru in committed_rule_files():
            want = rule_spokes(dru.read_text())
            if not want:
                continue
            pro = dru.with_suffix(".kicad_pro")
            with self.subTest(carrier=dru.stem):
                self.assertEqual(len(want), 1, f"{dru}: rules disagree: {sorted(want)}")
                self.assertEqual(
                    project_spokes(pro.read_text()),
                    min(want),
                    f"{pro.name}: min_resolved_spokes must equal {dru.name}'s rule "
                    "(KiCad 10 applies the project value, #1302)",
                )


if __name__ == "__main__":
    unittest.main()
