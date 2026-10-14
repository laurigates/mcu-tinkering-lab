"""Tests for the Specctra DSN net-class rewrite (tools/kicad/pcb/dsn.py).

`pcb/` is a directory of scripts for KiCad's bundled Python, not a package, so
`unittest discover` does not reach it; this test loads `pcb/dsn.py` by path.

The fixture `testdata/balancebot-carrier.dsn` is `pcbnew.ExportSpecctraDSN` of
the unrouted balancebot board that `kicad.boardinput` + `pcb/build_board.py`
place (line 1, which carries the output path, normalised). The golden
`balancebot-carrier.rewritten.dsn` is what `rewrite_dsn` in the balancebot
`route_board.py` wrote from it before the script moved to tools/kicad, with
its hard-coded classes. Here the classes come from board.toml through
`kicad.boardinput`, so the test also pins that board.toml says what the
script's constants said.

    python3 -m unittest discover -s tools/kicad -t tools
"""

from __future__ import annotations

import copy
import importlib.util
import unittest
from pathlib import Path

from kicad import boardinput
from kicad.paths import REPO_ROOT

HERE = Path(__file__).resolve().parent
TESTDATA = HERE / "testdata"
PROJECT = REPO_ROOT / "packages/robotics/balancebot"


def load_dsn():
    spec = importlib.util.spec_from_file_location("kicad_pcb_dsn", HERE / "pcb/dsn.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


dsn = load_dsn()


class RewriteTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        spec = boardinput.build(PROJECT)
        cls.classes = spec["classes"]
        cls.signal = spec["signal"]
        cls.fixture = (TESTDATA / "balancebot-carrier.dsn").read_text()
        cls.golden = (TESTDATA / "balancebot-carrier.rewritten.dsn").read_text()

    def rewrite(self, classes=None, signal=None, text=None):
        return dsn.rewrite(
            self.fixture if text is None else text,
            self.classes if classes is None else classes,
            self.signal if signal is None else signal,
        )

    def test_equals_the_golden(self):
        self.assertEqual(self.rewrite(), self.golden)

    def test_patterns_expand_against_the_dsn(self):
        classes = copy.deepcopy(self.classes)
        classes[0]["nets"] = ["VBAT", "GND", "MOT?_*"]
        self.assertEqual(self.rewrite(classes=classes), self.golden)

    def test_absent_net(self):
        classes = copy.deepcopy(self.classes)
        classes[1]["nets"].append("VMOT")
        with self.assertRaisesRegex(dsn.DsnError, "'VMOT' is not in the board"):
            self.rewrite(classes=classes)

    def test_pattern_matching_nothing(self):
        classes = copy.deepcopy(self.classes)
        classes[0]["nets"].append("MOTX_*")
        with self.assertRaisesRegex(dsn.DsnError, "'MOTX_\\*' matches no net"):
            self.rewrite(classes=classes)

    def test_net_in_two_classes(self):
        classes = copy.deepcopy(self.classes)
        classes[1]["nets"].append("GND")
        with self.assertRaisesRegex(
            dsn.DsnError, "'GND' is in classes 'power' and 'logic_power'"
        ):
            self.rewrite(classes=classes)

    def test_default_via_mismatch(self):
        signal = dict(self.signal, via_um=[700, 350])
        with self.assertRaisesRegex(
            dsn.DsnError, "default via 'Via\\[0-1\\]_600:300_um'"
        ):
            self.rewrite(signal=signal)

    def test_no_default_class(self):
        text = self.fixture.replace("(class kicad_default", "(class other", 1)
        with self.assertRaisesRegex(dsn.DsnError, "no kicad_default class"):
            self.rewrite(text=text)

    def test_shared_via_is_listed_once(self):
        classes = copy.deepcopy(self.classes)
        classes[1]["via_um"] = list(classes[0]["via_um"])
        out = self.rewrite(classes=classes)
        self.assertEqual(out.count('(padstack "Via[0-1]_1000:500_um"'), 1)
        self.assertNotIn('(padstack "Via[0-1]_800:400_um"', out)
        self.assertIn('(via "Via[0-1]_600:300_um" "Via[0-1]_1000:500_um")', out)

    def test_via_name(self):
        self.assertEqual(dsn.via_name([1000, 500]), "Via[0-1]_1000:500_um")


if __name__ == "__main__":
    unittest.main()
