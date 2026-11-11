"""Tests for the repo-root ruff.toml.

Stdlib unittest only, so the pre-commit hook runs it with nothing but python3:

    python3 -m unittest tools/test_ruff_config.py

TOML puts every key after a table header into that table. `select` and
`fixable` once sat below `[lint.per-file-ignores]`, so ruff read them as two
per-file ignores for files named `select` and `fixable`, and the effective
rule set fell back to ruff's version defaults (issue #761). Ruff accepts that
file without a warning, so these tests hold the parsed structure instead:

- `lint.select` and `lint.fixable` are keys of `[lint]`.
- Every `lint.per-file-ignores` key is a glob that matches a tracked file, and
  every value is a list of rule codes.
- E501 is not ignored, the lint limit is 100 columns and the formatter's
  line length stays at 88.
"""

from __future__ import annotations

import fnmatch
import re
import subprocess
import tomllib
import unittest
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent
RUFF_TOML = REPO_ROOT / "ruff.toml"

SELECT = {"E", "W", "F", "I", "UP", "B", "SIM", "RUF"}
RULE_CODE = re.compile(r"^[A-Z]+[0-9]*$")


def tracked_files() -> list[str]:
    out = subprocess.run(
        ["git", "ls-files"], cwd=REPO_ROOT, capture_output=True, text=True, check=True
    ).stdout
    return out.splitlines()


def glob_matches(pattern: str, path: str) -> bool:
    """Ruff's per-file-ignores rule: a pattern with no `/` also matches the basename."""
    if "/" not in pattern and fnmatch.fnmatchcase(path.rsplit("/", 1)[-1], pattern):
        return True
    return fnmatch.fnmatchcase(path, pattern)


class RuffConfigTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        with RUFF_TOML.open("rb") as f:
            cls.config = tomllib.load(f)
        cls.lint = cls.config.get("lint", {})
        cls.files = tracked_files()

    def test_select_is_a_lint_key(self):
        self.assertIn("select", self.lint, "lint.select is missing from [lint]")
        self.assertEqual(set(self.lint["select"]), SELECT)

    def test_fixable_is_a_lint_key(self):
        self.assertEqual(self.lint.get("fixable"), ["ALL"])

    def test_every_per_file_ignore_glob_matches_a_tracked_file(self):
        for pattern in self.lint.get("per-file-ignores", {}):
            with self.subTest(pattern=pattern):
                self.assertTrue(
                    any(glob_matches(pattern, f) for f in self.files),
                    f"per-file-ignores key {pattern!r} matches no tracked file",
                )

    def test_every_per_file_ignore_value_is_a_list_of_rule_codes(self):
        for pattern, codes in self.lint.get("per-file-ignores", {}).items():
            with self.subTest(pattern=pattern):
                self.assertIsInstance(codes, list)
                for code in codes:
                    self.assertRegex(code, RULE_CODE)

    def test_e501_is_not_ignored(self):
        # E501 is enforced: a blanket ignore lets prose and comments grow
        # without bound, since the formatter wraps only code.
        self.assertNotIn("E501", self.lint.get("ignore", []))

    def test_lint_line_limit_is_100(self):
        pycodestyle = self.lint.get("pycodestyle", {})
        self.assertEqual(pycodestyle.get("max-line-length"), 100)

    def test_formatter_line_length_stays_88(self):
        # line-length unset means ruff's default of 88; the lint limit above
        # must not be applied to the formatter.
        self.assertEqual(self.config.get("line-length", 88), 88)

    def test_glob_matching_control(self):
        # The stray keys from #761 must fail the glob check; a real glob passes.
        self.assertFalse(any(glob_matches("select", f) for f in self.files))
        self.assertFalse(any(glob_matches("fixable", f) for f in self.files))
        self.assertTrue(any(glob_matches("tools/**", f) for f in self.files))


if __name__ == "__main__":
    unittest.main()
