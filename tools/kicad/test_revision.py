"""kicad.revision: the board revision in board.toml, its agreement with the
committed silkscreen, and the order ledger that forbids two different boards
under one revision. KiCad-free."""

from __future__ import annotations

import csv
import hashlib
import io
import tempfile
import tomllib
import unittest
from contextlib import redirect_stdout
from pathlib import Path

import gitenv

from kicad import paths, provenance, revision
from kicad.errors import KicadError

setUpModule = gitenv.isolate

SHA = "0123456789abcdef0123456789abcdef01234567"
PCB = '(kicad_pcb\n\t(version 20260206)\n\t(generator "pcbnew")\n)\n'
PCB_SHA = hashlib.sha256(PCB.encode()).hexdigest()


def cfg(data: dict) -> paths.BoardConfig:
    return paths.BoardConfig(Path("board.toml"), "demo", "A4", data)


def order(rev: int, pcb: str = PCB_SHA, **extra) -> dict:
    return {
        "revision": rev,
        "date": "2026-11-28",
        "vendor": "JLCPCB",
        "order": "W1",
        "git_sha": SHA,
        "pcb_sha256": pcb,
        **extra,
    }


class Number(unittest.TestCase):
    def test_revision_is_read_from_fab(self):
        self.assertEqual(revision.number(cfg({"fab": {"revision": 3}})), 3)

    def test_missing_or_bad_revision_is_refused(self):
        for data in (
            {},
            {"fab": {}},
            {"fab": {"revision": 0}},
            {"fab": {"revision": "3"}},
            {"fab": {"revision": True}},
            {"fab": {"revision": 2.0}},
        ):
            with self.subTest(data=data), self.assertRaises(KicadError) as e:
                revision.number(cfg(data))
            self.assertIn("[fab] revision", str(e.exception))

    def test_label(self):
        self.assertEqual(revision.label(3), "r3")


class SilkAgreement(unittest.TestCase):
    def test_a_rev_text_must_name_the_revision(self):
        data = {"silk": {"texts": [{"text": "rev3 2026-11", "at": [0, 0]}]}}
        self.assertEqual(revision.silk_errors(cfg(data), 3), [])
        (err,) = revision.silk_errors(cfg(data), 4)
        self.assertIn("rev3 2026-11", err)
        self.assertIn("4", err)

    def test_other_texts_and_no_silk_are_fine(self):
        data = {"silk": {"texts": [{"text": "revision", "at": [0, 0]}, {"text": "5V"}]}}
        self.assertEqual(revision.silk_errors(cfg(data), 1), [])
        self.assertEqual(revision.silk_errors(cfg({}), 1), [])

    def test_r_label_on_silk_is_checked_too(self):
        data = {"silk": {"texts": [{"text": "r2", "at": [0, 0]}]}}
        self.assertEqual(len(revision.silk_errors(cfg(data), 3)), 1)


class Ledger(unittest.TestCase):
    def test_no_orders_allows_any_revision(self):
        self.assertEqual(revision.order_errors([], 1, PCB_SHA), [])

    def test_reordering_the_same_board_is_fine(self):
        self.assertEqual(revision.order_errors([order(3)], 3, PCB_SHA), [])

    def test_a_changed_board_under_an_ordered_revision_must_bump(self):
        (err,) = revision.order_errors([order(3, pcb="ab" * 32)], 3, PCB_SHA)
        self.assertIn("r3", err)
        self.assertIn("abababab", err)
        self.assertIn("revision = 4", err)

    def test_revision_may_not_go_below_an_ordered_one(self):
        (err,) = revision.order_errors([order(4, pcb="ab" * 32)], 3, PCB_SHA)
        self.assertIn("r4", err)

    def test_a_bumped_revision_passes(self):
        self.assertEqual(
            revision.order_errors([order(3, pcb="ab" * 32)], 4, PCB_SHA), []
        )

    def test_ledger_round_trip(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "orders.toml"
            self.assertEqual(revision.load_orders(path), [])
            revision.append_order(path, order(1, note='say "hi"'))
            revision.append_order(path, order(2, pcb="cd" * 32))
            got = revision.load_orders(path)
            self.assertEqual([o["revision"] for o in got], [1, 2])
            self.assertEqual(got[0]["note"], 'say "hi"')
            self.assertTrue(path.read_text().startswith("#"))
            tomllib.loads(path.read_text())

    def test_ledger_entries_are_validated(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "orders.toml"
            path.write_text("[[order]]\nrevision = 1\n")
            with self.assertRaises(KicadError) as e:
                revision.load_orders(path)
            self.assertIn("pcb_sha256", str(e.exception))


class Project(unittest.TestCase):
    """check() and record() against a throwaway project directory."""

    def setUp(self):
        self._tmp = tempfile.TemporaryDirectory()
        self.root = Path(self._tmp.name)
        carrier = self.root / "hardware" / "carrier"
        (carrier / "demo").mkdir(parents=True)
        (carrier / "demo" / "demo.kicad_pcb").write_text(PCB)
        self.board_toml = carrier / "board.toml"
        self.write_board(3)
        self.ledger = carrier / revision.LEDGER

    def tearDown(self):
        self._tmp.cleanup()

    def write_board(self, rev: int, silk: str = "rev3 2026-11") -> None:
        self.board_toml.write_text(
            '[project]\nname = "demo"\n\n'
            f'[silk]\ntexts = [{{ text = "{silk}", at = [1.0, 2.0] }}]\n\n'
            f"[fab]\nrevision = {rev}\nprovenance_at = [1.0, 1.0]\n"
        )

    def test_check_passes_a_consistent_project(self):
        self.assertEqual(revision.check(self.root), [])

    def test_check_reports_silk_and_ledger(self):
        revision.append_order(self.ledger, order(3, pcb="ab" * 32))
        self.write_board(3, silk="rev2")
        errors = revision.check(self.root)
        self.assertEqual(len(errors), 2, errors)

    def handoff(self, rev: int = 3, dirty: str = "no") -> Path:
        out = self.root / "hardware" / "carrier" / "demo" / "output"
        out.mkdir(exist_ok=True)
        row = dict.fromkeys(provenance.HANDOFF_FIELDS, "x")
        row.update(
            board="demo",
            revision=str(rev),
            git_sha=SHA,
            dirty=dirty,
            silk_text=f"r{rev} 0123456 {PCB_SHA[:8]}",
            pcb_sha256=PCB_SHA,
            zip=f"demo-r{rev}-fab-0123456.zip",
        )
        path = out / f"demo-r{rev}-fab-0123456.csv"
        with open(path, "w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=provenance.HANDOFF_FIELDS)
            w.writeheader()
            w.writerow(row)
        return path

    def test_record_appends_the_handoff_row(self):
        self.handoff()
        with redirect_stdout(io.StringIO()):
            revision.record(self.root, "JLCPCB", "W2026", "2026-11-28")
        (got,) = revision.load_orders(self.ledger)
        self.assertEqual(got["revision"], 3)
        self.assertEqual(got["git_sha"], SHA)
        self.assertEqual(got["pcb_sha256"], PCB_SHA)
        self.assertEqual(got["order"], "W2026")
        self.assertEqual(got["zip"], "demo-r3-fab-0123456.zip")

    def test_record_refuses_a_dirty_export(self):
        self.handoff(dirty="yes")
        with self.assertRaises(KicadError) as e:
            revision.record(self.root, "JLCPCB", "W1", "2026-11-28")
        self.assertIn("dirty", str(e.exception))

    def test_record_refuses_a_stale_export(self):
        self.handoff()
        (self.root / "hardware/carrier/demo/demo.kicad_pcb").write_text(PCB + " ")
        with self.assertRaises(KicadError) as e:
            revision.record(self.root, "JLCPCB", "W1", "2026-11-28")
        self.assertIn("carrier-fab", str(e.exception))

    def test_record_refuses_no_or_several_exports(self):
        with self.assertRaises(KicadError):
            revision.record(self.root, "JLCPCB", "W1", "2026-11-28")
        self.handoff(3)
        self.handoff(2)
        with self.assertRaises(KicadError):
            revision.record(self.root, "JLCPCB", "W1", "2026-11-28")


class TrackedCarriers(unittest.TestCase):
    def test_every_carrier_board_passes(self):
        # What the kicad-carrier hook and `just kicad::check` run.
        for project in paths.tracked_projects():
            if not paths.load_board_config(project).has_board:
                continue
            with self.subTest(project=project.name):
                self.assertEqual(revision.check(project), [])


if __name__ == "__main__":
    unittest.main()
