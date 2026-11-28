"""kicad.cartcheck: the PreToolUse nudge before a PCB goes into a fab cart.
KiCad-free; projects are throwaway directories."""

from __future__ import annotations

import csv
import hashlib
import io
import json
import subprocess
import tempfile
import unittest
from contextlib import redirect_stdout
from pathlib import Path
from unittest import mock

from kicad import cartcheck, provenance, revision

SHA = "0123456789abcdef0123456789abcdef01234567"
PCB = "(kicad_pcb\n\t(version 20260206)\n)\n"
PCB_SHA = hashlib.sha256(PCB.encode()).hexdigest()

CDP = {"command": "node .claude/skills/jlcpcb-order/cdp.mjs eval cart.js"}
SAVE = {
    "action": "left_click",
    "text": "Save to Cart",
    "url": "https://jlcpcb.com/quote",
}


class Relevance(unittest.TestCase):
    def test_fab_cart_calls_are_relevant(self):
        self.assertEqual(cartcheck.relevant(CDP), (True, False))
        self.assertEqual(
            cartcheck.relevant({"url": "https://cart.jlcpcb.com/shopcart"}),
            (True, False),
        )
        self.assertEqual(cartcheck.relevant(SAVE), (True, True))
        self.assertEqual(
            cartcheck.relevant({"url": "https://jlcpcb.com/checkout"}), (True, True)
        )

    def test_other_calls_are_not(self):
        for tool_input in (
            {"command": "just robocar::build"},
            {"url": "https://www.aliexpress.com/cart"},
            {"url": "https://jlcpcb.com/parts"},
            {},
        ):
            with self.subTest(tool_input=tool_input):
                self.assertEqual(cartcheck.relevant(tool_input), (False, False))


class Project(unittest.TestCase):
    def setUp(self):
        self._tmp = tempfile.TemporaryDirectory()
        self.root = Path(self._tmp.name)
        self.project = self.root / "demo"
        self.kicad = self.project / "hardware/carrier/demo"
        self.kicad.mkdir(parents=True)
        (self.kicad / "demo.kicad_pcb").write_text(PCB)
        (self.project / "hardware/carrier/board.toml").write_text(
            '[project]\nname = "demo"\n[fab]\nrevision = 2\nprovenance_at = [1.0, 1.0]\n'
        )
        self.state = self.root / "state"
        self.state.mkdir()

    def tearDown(self):
        self._tmp.cleanup()

    def export(self, dirty: str = "no", pcb: str = PCB_SHA, rev: int = 2) -> None:
        out = self.kicad / "output"
        out.mkdir(exist_ok=True)
        row = dict.fromkeys(provenance.HANDOFF_FIELDS, "")
        row.update(
            board="demo",
            revision=str(rev),
            git_sha=SHA,
            dirty=dirty,
            silk_text=f"r{rev} 0123456 {pcb[:8]}",
            pcb_sha256=pcb,
            zip=f"out/demo-r{rev}-fab-0123456.zip",
            zip_sha256="9" * 64,
        )
        with open(out / f"demo-r{rev}-fab-0123456.csv", "w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=provenance.HANDOFF_FIELDS)
            w.writeheader()
            w.writerow(row)

    def hook(self, tool_input, session="s1"):
        return cartcheck.hook(
            {"session_id": session, "tool_input": tool_input},
            [self.project],
            self.root,
            self.state,
        )

    def test_silent_without_an_export(self):
        self.assertIsNone(self.hook(CDP))

    def test_names_the_cart_line_and_silkscreen(self):
        self.export()
        answer = self.hook(CDP)
        ctx = answer["hookSpecificOutput"]["additionalContext"]
        self.assertEqual(answer["hookSpecificOutput"]["hookEventName"], "PreToolUse")
        self.assertIn("`demo-r2-fab-0123456_Y<n>`", ctx)
        self.assertIn(f"`r2 0123456 {PCB_SHA[:8]}`", ctx)
        self.assertIn("EVERY PCB line", ctx)
        self.assertIn("kicad.revision record", ctx)
        self.assertNotIn("NOT ORDERABLE", ctx)

    def test_flags_dirty_stale_and_unbumped_exports(self):
        self.export(dirty="yes", pcb="ab" * 32, rev=1)
        revision.append_order(
            self.project / "hardware/carrier" / revision.LEDGER,
            {
                "revision": 2,
                "date": "2026-11-01",
                "vendor": "JLCPCB",
                "order": "W1",
                "git_sha": SHA,
                "pcb_sha256": "cd" * 32,
            },
        )
        ctx = self.hook(CDP)["hookSpecificOutput"]["additionalContext"]
        self.assertIn("NOT ORDERABLE", ctx)
        self.assertIn("-dirty", ctx)
        self.assertIn("carrier-fab", ctx)
        self.assertIn("board.toml says 2", ctx)
        self.assertIn("revision = 3", ctx)

    def test_once_per_session_but_every_commit_step(self):
        self.export()
        self.assertIsNotNone(self.hook(CDP))
        self.assertIsNone(self.hook(CDP))
        self.assertIsNotNone(self.hook(SAVE))
        self.assertIsNotNone(self.hook(SAVE))
        self.assertIsNotNone(self.hook(CDP, session="s2"))

    def test_irrelevant_call_is_silent(self):
        self.export()
        self.assertIsNone(self.hook({"command": "ls"}))

    def test_main_never_fails(self):
        with (
            mock.patch("sys.stdin", io.StringIO("not json")),
            redirect_stdout(io.StringIO()) as out,
        ):
            self.assertEqual(cartcheck.main(), 0)
        self.assertEqual(out.getvalue(), "")

    def test_main_prints_the_answer(self):
        self.export()
        event = json.dumps({"session_id": "m", "tool_input": SAVE})
        with (
            mock.patch("sys.stdin", io.StringIO(event)),
            mock.patch.object(
                cartcheck.paths, "tracked_projects", return_value=[self.project]
            ),
            redirect_stdout(io.StringIO()) as out,
        ):
            self.assertEqual(cartcheck.main(), 0)
        self.assertIn(
            "additionalContext", json.loads(out.getvalue())["hookSpecificOutput"]
        )


class Wiring(unittest.TestCase):
    ROOT = Path(__file__).resolve().parents[2]
    SCRIPT = ROOT / ".claude/hooks/fab-cart-check.sh"

    def test_settings_run_the_hook_before_browser_and_bash_calls(self):
        settings = json.loads((self.ROOT / ".claude/settings.json").read_text())
        (entry,) = [
            e
            for e in settings["hooks"]["PreToolUse"]
            if any("fab-cart-check.sh" in h["command"] for h in e["hooks"])
        ]
        for tool in ("Bash", "mcp__claude-in-chrome__", "mcp__Claude_Browser__"):
            self.assertIn(tool, entry["matcher"])

    def test_script_is_silent_for_other_calls(self):
        done = subprocess.run(
            ["bash", str(self.SCRIPT)],
            input='{"tool_input": {"command": "ls"}}',
            capture_output=True,
            text=True,
        )
        self.assertEqual((done.returncode, done.stdout), (0, ""))


if __name__ == "__main__":
    unittest.main()
