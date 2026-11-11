"""kicad.provenance: the commit on the silkscreen, in the file names and in the
hand-off row, and the refusal to export from a dirty tree. KiCad-free."""

from __future__ import annotations

import csv
import hashlib
import json
import subprocess
import tempfile
import unittest
from pathlib import Path

from kicad import paths, provenance
from kicad.errors import KicadError

SHA = "0123456789abcdef0123456789abcdef01234567"
PCB_SHA = "fedcba9876543210" * 4
BOARD = '(kicad_pcb\n\t(version 20260206)\n\t(generator "pcbnew")\n)\n'


def cfg(data: dict) -> paths.BoardConfig:
    return paths.BoardConfig(Path("board.toml"), "demo", "A4", data)


def git(root: Path, *args: str) -> None:
    subprocess.run(
        ["git", "-c", "user.name=t", "-c", "user.email=t@t", *args],
        cwd=root,
        check=True,
        capture_output=True,
    )


class Naming(unittest.TestCase):
    def test_clean_tag_is_the_short_sha(self):
        self.assertEqual(provenance.tag(SHA, False), "0123456")

    def test_dirty_tag_says_so(self):
        self.assertEqual(provenance.tag(SHA, True), "0123456-dirty")

    def test_short_sha_is_refused(self):
        with self.assertRaises(KicadError):
            provenance.tag("abc", False)

    def test_silk_text_is_commit_then_board_hash_prefix(self):
        self.assertEqual(provenance.silk_text(SHA, False, PCB_SHA), "0123456 fedcba98")

    def test_dirty_silk_text_marks_the_separator(self):
        self.assertEqual(provenance.silk_text(SHA, True, PCB_SHA), "0123456*fedcba98")

    def test_silk_text_width_does_not_depend_on_dirty(self):
        # A dirty fab must fit wherever the clean one passed DRC: '-dirty' grew
        # the balancebot text into J3's INT label. '*' takes the space's place,
        # and KiCad's stroke font gives both the same advance.
        clean = provenance.silk_text(SHA, False, PCB_SHA)
        dirty = provenance.silk_text(SHA, True, PCB_SHA)
        self.assertEqual(len(clean), provenance.SILK_LEN)
        self.assertEqual(len(dirty), provenance.SILK_LEN)
        self.assertEqual(provenance.SILK_LEN, 16)
        self.assertEqual(
            [i for i, (a, b) in enumerate(zip(clean, dirty, strict=True)) if a != b],
            [provenance.SHA_LEN],
        )

    def test_silk_text_refuses_short_inputs(self):
        with self.assertRaises(KicadError):
            provenance.silk_text("abc", False, PCB_SHA)
        with self.assertRaises(KicadError):
            provenance.silk_text(SHA, False, "fedc")

    def test_every_output_name_carries_the_tag(self):
        n = provenance.names("balancebot-carrier", "0123456")
        self.assertEqual(n.zip, "balancebot-carrier-fab-0123456.zip")
        self.assertEqual(n.bom, "balancebot-carrier-bom-jlc-0123456.csv")
        self.assertEqual(n.cpl, "balancebot-carrier-cpl-jlc-0123456.csv")
        self.assertEqual(n.handoff, "balancebot-carrier-fab-0123456.csv")
        dirty = provenance.names("x", "0123456-dirty")
        for name in (dirty.zip, dirty.bom, dirty.cpl, dirty.handoff):
            self.assertIn("0123456-dirty", name)


class SilkText(unittest.TestCase):
    def test_text_is_added_on_front_silk_at_board_coordinates(self):
        out = provenance.add_text(BOARD, "0123456 fedcba98", (144.0, 150.2), 1.0)
        self.assertTrue(out.startswith(BOARD.rstrip()[:-1]))
        self.assertTrue(out.endswith("\t)\n)\n"))
        self.assertIn('(gr_text "0123456 fedcba98"', out)
        self.assertIn("(at 144 150.2 0)", out)
        self.assertIn('(layer "F.SilkS")', out)
        self.assertIn("(size 1 1)", out)
        self.assertIn("(thickness 0.16)", out)
        self.assertEqual(out.count("("), out.count(")"))

    def test_same_text_gives_the_same_board(self):
        a = provenance.add_text(BOARD, "t 1", (1, 2), 1.0)
        self.assertEqual(a, provenance.add_text(BOARD, "t 1", (1, 2), 1.0))

    def test_not_a_board_is_refused(self):
        with self.assertRaises(KicadError):
            provenance.add_text("(kicad_sch)\n", "t", (0, 0), 1.0)
        with self.assertRaises(KicadError):
            provenance.add_text(BOARD, 'a "quoted" text', (0, 0), 1.0)

    def test_origin_relative_point_becomes_absolute(self):
        c = cfg({"outline": {"origin": [100.0, 100.0], "size": [90, 62]}})
        self.assertEqual(provenance.absolute(c, [44.0, 50.2]), (144.0, 150.2))


class Spec(unittest.TestCase):
    def test_position_and_size(self):
        c = cfg({"fab": {"provenance_at": [1, 2], "provenance_size": 1.2}})
        self.assertEqual(provenance.spec(c), ([1.0, 2.0], 1.2))

    def test_size_defaults_to_one_mm(self):
        self.assertEqual(
            provenance.spec(cfg({"fab": {"provenance_at": [1, 2]}}))[1], 1.0
        )

    def test_missing_position_fails_the_run(self):
        for data in ({}, {"fab": {}}):
            with self.subTest(data=data), self.assertRaises(KicadError) as e:
                provenance.spec(cfg(data))
            self.assertIn("provenance_at", str(e.exception))

    def test_boardcheck_validation(self):
        size = [90, 62]
        ok = {"fab": {"provenance_at": [44.0, 50.2], "provenance_size": 1.0}}
        self.assertEqual(provenance.validate("b", ok, size, 1.0), [])
        self.assertEqual(provenance.validate("b", {}, size, 1.0), [])
        outside = {"fab": {"provenance_at": [95.0, 50.2]}}
        (err,) = provenance.validate("b", outside, size, 1.0)
        self.assertIn("not inside [outline]", err)
        small = {"fab": {"provenance_at": [4, 5], "provenance_size": 0.8}}
        (err,) = provenance.validate("b", small, size, 1.0)
        self.assertIn("floor is 1.0 mm", err)
        self.assertEqual(provenance.validate("b", small, size, 0.8), [])
        bad = {"fab": {"provenance_at": "here"}}
        (err,) = provenance.validate("b", bad, size, 1.0)
        self.assertIn("[x, y]", err)

    def test_every_carrier_board_names_its_position(self):
        # carrier-fab fails without [fab] provenance_at; catch it in CI instead.
        for project in paths.tracked_projects():
            c = paths.load_board_config(project)
            if not c.has_board:
                continue
            with self.subTest(project=project.name):
                _at, _size = provenance.spec(c)
                errors = provenance.validate(
                    str(c.path), c.data, c.data["outline"]["size"], 0.8
                )
                self.assertEqual(errors, [])


class DirtyTree(unittest.TestCase):
    def test_porcelain_paths(self):
        out = " M a/hardware.toml\0?? a/hardware/new.toml\0R  a/new\0a/old\0"
        self.assertEqual(
            provenance.dirty_paths(out),
            ["a/hardware.toml", "a/hardware/new.toml", "a/new", "a/old"],
        )
        self.assertEqual(provenance.dirty_paths(""), [])

    def test_clean_tree_passes(self):
        state = provenance.GitState(SHA, ())
        self.assertFalse(provenance.require_clean(state, False))

    def test_dirty_tree_is_refused_and_says_why(self):
        state = provenance.GitState(SHA, ("p/hardware/carrier/board.toml",))
        with self.assertRaises(KicadError) as e:
            provenance.require_clean(state, False)
        msg = str(e.exception)
        self.assertIn("uncommitted", msg)
        self.assertIn("p/hardware/carrier/board.toml", msg)
        self.assertIn(provenance.ALLOW_DIRTY_ENV, msg)

    def test_override_marks_the_run_dirty(self):
        state = provenance.GitState(SHA, ("tools/kicad/fab.py",))
        self.assertTrue(provenance.require_clean(state, True))

    def test_override_comes_from_the_environment(self):
        self.assertTrue(
            provenance.allow_dirty_from_env({"CARRIER_FAB_ALLOW_DIRTY": "1"})
        )
        self.assertFalse(
            provenance.allow_dirty_from_env({"CARRIER_FAB_ALLOW_DIRTY": "0"})
        )
        self.assertFalse(provenance.allow_dirty_from_env({}))


class GitStateInARepo(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.root = Path(self.tmp.name)
        self.project = self.root / "packages/demo/p"
        files = {
            "packages/demo/p/hardware.toml": "a",
            "packages/demo/p/hardware/carrier/board.toml": "b",
            "packages/demo/p/main/main.c": "c",
            "tools/kicad/fab.py": "d",
            "docs/reference/parts/x.toml": "e",
            ".gitignore": "output/\n",
        }
        for rel, text in files.items():
            path = self.root / rel
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text(text)
        git(self.root, "init", "-q")
        git(self.root, "add", "-A")
        git(self.root, "commit", "-q", "-m", "init")

    def state(self) -> provenance.GitState:
        return provenance.git_state(self.project, self.root)

    def test_clean_checkout(self):
        state = self.state()
        self.assertEqual(len(state.sha), 40)
        self.assertEqual(state.dirty, ())

    def test_changes_that_feed_the_board_are_dirty(self):
        (self.project / "hardware.toml").write_text("changed")
        (self.project / "hardware/carrier/new.toml").write_text("untracked")
        (self.root / "tools/kicad/fab.py").write_text("changed")
        (self.root / "docs/reference/parts/x.toml").unlink()
        self.assertEqual(
            sorted(self.state().dirty),
            [
                "docs/reference/parts/x.toml",
                "packages/demo/p/hardware.toml",
                "packages/demo/p/hardware/carrier/new.toml",
                "tools/kicad/fab.py",
            ],
        )

    def test_firmware_and_ignored_output_are_not(self):
        (self.project / "main/main.c").write_text("changed")
        out = self.project / "hardware/carrier/demo/output"
        out.mkdir(parents=True)
        (out / "demo-fab-0123456.zip").write_text("zip")
        self.assertEqual(self.state().dirty, ())


class StampAndHandoff(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.dir = Path(self.tmp.name)

    def test_stamp_must_vouch_for_the_board(self):
        pcb = BOARD.encode()
        stamp = self.dir / "demo.check.json"
        stamp.write_text(json.dumps({"board_sha256": provenance.sha256(pcb)}))
        self.assertEqual(
            provenance.stamped_board_sha256(stamp, pcb), provenance.sha256(pcb)
        )
        with self.assertRaises(KicadError) as e:
            provenance.stamped_board_sha256(stamp, pcb + b" ")
        self.assertIn("carrier-check", str(e.exception))

    def test_handoff_row_holds_the_commit_and_the_zip_sha256(self):
        z = self.dir / "demo-fab-0123456.zip"
        z.write_bytes(b"gerbers")
        bom, cpl = self.dir / "b.csv", self.dir / "c.csv"
        state = provenance.GitState(SHA, ())
        row = provenance.handoff_row(
            "demo", state, False, "0123456 fedcba98", PCB_SHA, z, bom, cpl, self.dir
        )
        self.assertEqual(row["git_sha"], SHA)
        self.assertEqual(row["dirty"], "no")
        self.assertEqual(row["zip"], "demo-fab-0123456.zip")
        self.assertEqual(row["zip_sha256"], hashlib.sha256(b"gerbers").hexdigest())
        path = self.dir / "demo-fab-0123456.csv"
        provenance.write_handoff(path, row)
        with open(path, newline="") as f:
            (read,) = list(csv.DictReader(f))
        self.assertEqual(read, row)
        self.assertEqual(list(read), list(provenance.HANDOFF_FIELDS))


if __name__ == "__main__":
    unittest.main()
