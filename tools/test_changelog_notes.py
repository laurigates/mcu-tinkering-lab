"""Tests for changelog-notes.py, run against the shipped script.

Stdlib unittest only, so the pre-commit hook runs it with nothing but python3:

    python3 -m unittest tools/test_changelog_notes.py

The script is executed as a subprocess, never imported, so the tests exercise
the same entry point generate-flasher-manifests.sh calls.
"""

from __future__ import annotations

import json
import subprocess
import sys
import tempfile
import textwrap
import unittest
from pathlib import Path

SCRIPT = Path(__file__).with_name("changelog-notes.py")

# The shape release-please writes (packages/robocar/unified/CHANGELOG.md).
CHANGELOG = textwrap.dedent(
    """\
    # Changelog

    ## [0.2.18](https://github.com/o/r/compare/robocar-unified-v0.2.17...robocar-unified-v0.2.18) (2026-11-08)


    ### Features

    * **robocar-unified:** announce an update before installing it ([abc1234](https://github.com/o/r/commit/abc1234))
    * **mechanism:** animate the clash sweep ([def5678](https://github.com/o/r/commit/def5678))


    ### Bug Fixes

    * **robocar-unified:** never speak a reply's thought parts ([1111111](https://github.com/o/r/commit/1111111))


    ### Documentation

    * **robocar-unified:** propose the Live design ([2222222](https://github.com/o/r/commit/2222222))

    ## [0.2.17](https://github.com/o/r/compare/robocar-unified-v0.2.16...robocar-unified-v0.2.17) (2026-11-03)


    ### Features

    * **robocar-unified:** fit the TCA9548A clone on the carrier ([3333333](https://github.com/o/r/commit/3333333))
    """
)


def run(
    changelog: str | None, version: str, *extra: str
) -> tuple[int, list | None, str]:
    with tempfile.TemporaryDirectory() as tmp:
        path = Path(tmp) / "CHANGELOG.md"
        if changelog is not None:
            path.write_text(changelog, encoding="utf-8")
        proc = subprocess.run(
            [sys.executable, str(SCRIPT), str(path), version, *extra],
            capture_output=True,
            text=True,
            check=False,
        )
    notes = json.loads(proc.stdout) if proc.stdout.strip() else None
    return proc.returncode, notes, proc.stderr


class ChangelogNotesTest(unittest.TestCase):
    def test_features_and_fixes_of_the_version_only(self):
        rc, notes, _ = run(CHANGELOG, "0.2.18")
        self.assertEqual(rc, 0)
        self.assertEqual(
            notes,
            [
                "robocar-unified: announce an update before installing it",
                "mechanism: animate the clash sweep",
                "robocar-unified: never speak a reply's thought parts",
            ],
        )

    def test_documentation_and_other_versions_are_left_out(self):
        _, notes, _ = run(CHANGELOG, "0.2.18")
        joined = " ".join(notes)
        self.assertNotIn("Live design", joined)  # Documentation section
        self.assertNotIn("TCA9548A", joined)  # 0.2.17

    def test_an_older_version_reads_its_own_section(self):
        _, notes, _ = run(CHANGELOG, "0.2.17")
        self.assertEqual(
            notes, ["robocar-unified: fit the TCA9548A clone on the carrier"]
        )

    def test_a_version_prefix_does_not_match_a_longer_version(self):
        # "0.2.1" must not read the 0.2.17 or 0.2.18 sections.
        rc, notes, _ = run(CHANGELOG, "0.2.1")
        self.assertEqual((rc, notes), (0, []))

    def test_unknown_version_gives_an_empty_list(self):
        # A missing section must not fail the release: the firmware falls back
        # to a template line when notes are empty.
        self.assertEqual(run(CHANGELOG, "9.9.9")[:2], (0, []))

    def test_missing_changelog_gives_an_empty_list(self):
        self.assertEqual(run(None, "0.2.18")[:2], (0, []))

    def test_item_count_and_length_are_capped(self):
        items = "\n".join(
            f"* **x:** {'word ' * 60}item {i} ([{i:07x}](u))" for i in range(20)
        )
        log = f"## [1.0.0](u) (2026-01-01)\n\n### Features\n\n{items}\n"
        _, notes, _ = run(log, "1.0.0", "--max-items", "4", "--max-chars", "80")
        self.assertEqual(len(notes), 4)
        self.assertTrue(all(len(n) <= 80 for n in notes))
        self.assertTrue(all(n.endswith("…") for n in notes))

    def test_default_notes_fit_the_old_firmware_manifest_buffer(self):
        # robocar-unified <= 0.2.17 reads the whole manifest into 2048 bytes and
        # cannot parse a truncated one, so it would never update again. The
        # notes are held under 800 encoded bytes; the rest of the manifest is
        # about 600 bytes as written.
        items = "\n".join(
            f"* **x:** {'pitkä sana ' * 30}item {i} ([{i:07x}](u))" for i in range(20)
        )
        log = f"## [1.0.0](u) (2026-01-01)\n\n### Features\n\n{items}\n"
        rc, notes, _ = run(log, "1.0.0")
        self.assertEqual(rc, 0)
        self.assertGreater(len(notes), 0)
        encoded = json.dumps(notes, ensure_ascii=False).encode("utf-8")
        self.assertLessEqual(len(encoded), 800)

    def test_max_bytes_drops_trailing_entries_whole(self):
        log = (
            "## [1.0.0](u) (2026-01-01)\n\n### Features\n\n"
            "* **a:** one ([abc1234](u))\n* **b:** two ([abc1235](u))\n"
        )
        _, notes, _ = run(log, "1.0.0", "--max-bytes", "12")
        self.assertEqual(notes, ["a: one"])

    def test_utf8_is_kept_unescaped(self):
        log = "## [1.0.0](u) (2026-01-01)\n\n### Features\n\n* **robocar:** sano päivää ([abc1234](u))\n"
        rc, notes, _ = run(log, "1.0.0")
        self.assertEqual((rc, notes), (0, ["robocar: sano päivää"]))


if __name__ == "__main__":
    unittest.main()
