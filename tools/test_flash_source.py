"""Tests for tools/flash-source.py: which firmware a flash writes, and the SHA gate.

The pure parts are imported from the script; the git parts run against a
throwaway repository so the checkout-SHA rule matches what the CMakeLists
computes (HEAD, plus "-dirty" for uncommitted tracked changes).
"""

from __future__ import annotations

import importlib.util
import shlex
import subprocess
import tempfile
import unittest
from pathlib import Path

try:
    from tools import gitenv
except ImportError:  # run as a script: python3 tools/test_flash_source.py
    import gitenv

setUpModule = gitenv.isolate

_SPEC = importlib.util.spec_from_file_location(
    "flash_source", Path(__file__).with_name("flash-source.py")
)
assert _SPEC is not None and _SPEC.loader is not None
fs = importlib.util.module_from_spec(_SPEC)
_SPEC.loader.exec_module(fs)

SHA = "0123456789abcdef0123456789abcdef01234567"
OTHER = "fedcba9876543210fedcba9876543210fedcba98"


def image(*shas: str) -> bytes:
    return b"\xe9 app header " + b"".join(s.encode() + b"\x00 rodata " for s in shas)


class EmbeddedSha(unittest.TestCase):
    def test_one_clean_sha(self):
        self.assertEqual(fs.embedded_sha(image(SHA)), SHA)

    def test_dirty_sha_is_its_own_value(self):
        self.assertEqual(fs.embedded_sha(image(SHA + "-dirty")), SHA + "-dirty")

    def test_none_or_several_refuse(self):
        with self.assertRaises(fs.FlashSourceError):
            fs.embedded_sha(b"no sha in here")
        with self.assertRaises(fs.FlashSourceError):
            fs.embedded_sha(image(SHA, OTHER))

    def test_unterminated_hex_is_not_a_sha(self):
        with self.assertRaises(fs.FlashSourceError):
            fs.embedded_sha(SHA.encode() + b"x")


class CheckSha(unittest.TestCase):
    def test_match_passes(self):
        fs.check_sha(SHA, SHA, "image")

    def test_mismatch_refuses(self):
        with self.assertRaises(fs.FlashSourceError):
            fs.check_sha(OTHER, SHA, "image")

    def test_dirty_build_is_not_the_clean_commit(self):
        with self.assertRaises(fs.FlashSourceError):
            fs.check_sha(SHA + "-dirty", SHA, "image")


class TagCandidates(unittest.TestCase):
    P = "robocar-v"

    def test_version_forms(self):
        for ref in ("v0.2.24", "0.2.24", "robocar-v0.2.24"):
            self.assertEqual(fs.tag_candidates(ref, self.P), ["robocar-v0.2.24"])

    def test_a_tag_from_before_the_rename_is_not_a_release_tag(self):
        # No legacy prefix: robocar-unified-v0.2.17 is resolved as a plain git
        # ref, and its commit is then refused for lacking the project directory.
        self.assertEqual(fs.tag_candidates("robocar-unified-v0.2.17", self.P), [])

    def test_branches_and_shas_are_not_tags(self):
        for ref in ("main", "fix/robocar-drive-reverse", SHA, "vacation"):
            self.assertEqual(fs.tag_candidates(ref, self.P), [])


class ReleaseParts(unittest.TestCase):
    def test_every_build_part_has_a_prefixed_asset(self):
        parts = fs.release_parts("robocar")
        self.assertEqual(
            parts,
            {
                "build/bootloader/bootloader.bin": "robocar-bootloader.bin",
                "build/partition_table/partition-table.bin": "robocar-partition-table.bin",
                "build/ota_data_initial.bin": "robocar-ota_data_initial.bin",
                "build/robocar.bin": "robocar.bin",
            },
        )


class Assignments(unittest.TestCase):
    def test_eval_safe(self):
        out = fs.assignments(Path("/tmp/a dir/x"), SHA, "release robocar-v1")
        parsed = dict(line.split("=", 1) for line in out.splitlines())
        self.assertEqual(shlex.split(parsed["SRC_DIR"]), ["/tmp/a dir/x"])
        self.assertEqual(shlex.split(parsed["SOURCE"]), ["release robocar-v1"])


class CheckoutSha(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.root = Path(self.tmp.name)

        def run(*a: str) -> None:
            subprocess.run(
                ["git", "-C", str(self.root), *a], check=True, capture_output=True
            )

        run("init", "-q")
        run("config", "user.email", "t@example.invalid")
        run("config", "user.name", "t")
        (self.root / "f.txt").write_text("one\n")
        run("add", "f.txt")
        run("commit", "-q", "-m", "one")
        self.head = subprocess.run(
            ["git", "-C", str(self.root), "rev-parse", "HEAD"],
            capture_output=True,
            text=True,
            check=True,
        ).stdout.strip()

    def tearDown(self):
        self.tmp.cleanup()

    def test_clean_tree_is_head(self):
        self.assertEqual(fs.checkout_sha(self.root), self.head)

    def test_tracked_change_is_dirty(self):
        (self.root / "f.txt").write_text("two\n")
        self.assertEqual(fs.checkout_sha(self.root), self.head + "-dirty")

    def test_a_commit_without_the_project_directory_is_refused(self):
        # A commit from before the project moved to its current directory:
        # nothing is built, and the error says how to flash it.
        with self.assertRaises(fs.FlashSourceError) as caught:
            fs.build_commit(
                self.root, "packages/robotics/robocar", "robocar", self.head
            )
        self.assertIn("has no packages/robotics/robocar", str(caught.exception))
        self.assertIn("checkout of that commit", str(caught.exception))

    def test_untracked_file_is_not_dirty(self):
        (self.root / "new.txt").write_text("x\n")
        self.assertEqual(fs.checkout_sha(self.root), self.head)

    def test_resolve_commit(self):
        self.assertEqual(fs.resolve_commit(self.root, "HEAD"), self.head)
        with self.assertRaises(fs.FlashSourceError):
            fs.resolve_commit(self.root, "no-such-branch")


if __name__ == "__main__":
    unittest.main()
