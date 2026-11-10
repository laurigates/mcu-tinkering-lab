"""Drift tests for .github/issue-labels.json, the repo's label taxonomy.

Stdlib unittest only, so the pre-commit hook can run it on a CI runner that
has nothing but python3 and git:

    python3 -m unittest tools/test_issue_labels_taxonomy.py

The taxonomy is read by git-plugin/scripts/issue-labels.py in
laurigates/claude-plugins, which .github/workflows/issue-labels.yml fetches at
a pinned commit. That script is not in this repo, so two of its rules are
restated here:

- ``glob_regex`` is copied from the script at commit 2a46797. ``*`` and ``?``
  stay inside one path segment, ``**`` crosses segments, and a trailing ``/**``
  also matches the directory itself. If a later script changes glob matching,
  change this copy with the workflow's ``plugins-ref``.
- An entry's key is one of its scopes, and scopes are compared lowercased with
  whitespace runs collapsed. The script's built-in ``core`` keys exist even
  when the file does not list them.

What the tests hold:

- Every ``paths`` glob matches at least one path git tracks. A glob left
  behind by a moved or deleted directory routes nothing and says nothing.
- Every scope selects one entry. The script lets one scope select several
  labels; this repo keeps them apart so a title scope names one label.

Whether each ``build:`` and ``area:`` name, colour and description matches the
live labels needs the network, so it is not tested here. The gate for that is
``issue-labels.py provision --dry-run``, which must plan no change.
"""

from __future__ import annotations

import json
import re
import subprocess
import unittest
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent
TAXONOMY = REPO_ROOT / ".github" / "issue-labels.json"

SECTIONS = ("build", "area", "core")
# The script's DEFAULT_CORE keys: entries that exist without being listed.
BUILTIN_CORE = ("ci", "agent", "build", "quality", "docs")


def glob_regex(glob: str) -> re.Pattern[str]:
    glob = glob.lstrip("/")
    out: list[str] = []
    i = 0
    while i < len(glob):
        if glob.startswith("**/", i):
            out.append("(?:.*/)?")
            i += 3
        elif glob.startswith("/**", i) and i + 3 == len(glob):
            out.append("(?:/.*)?")
            i += 3
        elif glob.startswith("**", i):
            out.append(".*")
            i += 2
        elif glob[i] == "*":
            out.append("[^/]*")
            i += 1
        elif glob[i] == "?":
            out.append("[^/]")
            i += 1
        else:
            out.append(re.escape(glob[i]))
            i += 1
    return re.compile("".join(out) + r"\Z")


def norm(text: str) -> str:
    return " ".join(text.lower().split())


def load_taxonomy(path: Path = TAXONOMY) -> dict:
    with path.open(encoding="utf-8") as f:
        return json.load(f)


def entries(tax: dict) -> list[tuple[str, dict]]:
    """(``section.key``, entry) for every entry, built-in core keys included."""
    core = {key: {} for key in BUILTIN_CORE} | (tax.get("core") or {})
    sections = {"build": tax.get("build") or {}, "area": tax.get("area") or {}}
    sections["core"] = core
    return [
        (f"{section}.{key}", entry)
        for section in SECTIONS
        for key, entry in sections[section].items()
    ]


def unmatched_globs(tax: dict, tracked: list[str]) -> list[str]:
    """``section.key: glob`` for each glob that matches no tracked path."""
    missing = []
    for where, entry in entries(tax):
        for glob in entry.get("paths", []):
            pattern = glob_regex(glob)
            if not any(pattern.match(path) for path in tracked):
                missing.append(f"{where}: {glob}")
    return missing


def shared_scopes(tax: dict) -> list[str]:
    """``scope: owner, owner`` for each scope more than one entry answers to."""
    owners: dict[str, list[str]] = {}
    for where, entry in entries(tax):
        key = where.split(".", 1)[1]
        for scope in [key, *entry.get("scopes", [])]:
            owners.setdefault(norm(scope), []).append(where)
    return [
        f"{scope}: {', '.join(names)}"
        for scope, names in sorted(owners.items())
        if len(names) > 1
    ]


def tracked_paths() -> list[str]:
    out = subprocess.run(
        ["git", "ls-files", "-z"],
        cwd=REPO_ROOT,
        check=True,
        capture_output=True,
        text=True,
    ).stdout
    return [path for path in out.split("\0") if path]


class GlobRegex(unittest.TestCase):
    """The copied matcher behaves as the script documents it."""

    def matches(self, path: str, glob: str) -> bool:
        return bool(glob_regex(glob).match(path))

    def test_star_stays_inside_one_segment(self) -> None:
        self.assertTrue(self.matches("tools/esp32-idf.just", "tools/esp32*.just"))
        self.assertFalse(self.matches("tools/kicad/x.just", "tools/*.just"))
        self.assertFalse(self.matches("tools/esp32.just", "*.just"))

    def test_trailing_double_star_matches_below_and_the_directory(self) -> None:
        self.assertTrue(self.matches("tools/labels/a/b.py", "tools/labels/**"))
        self.assertTrue(self.matches("tools/labels", "tools/labels/**"))
        self.assertFalse(self.matches("tools/labels-old/a.py", "tools/labels/**"))

    def test_segment_wildcards_before_a_fixed_tail(self) -> None:
        glob = "packages/*/*/hardware/carrier/**"
        self.assertTrue(
            self.matches("packages/robocar/unified/hardware/carrier/b.toml", glob)
        )
        self.assertFalse(
            self.matches("packages/robocar/unified/x/hardware/carrier/b.toml", glob)
        )

    def test_a_plain_name_matches_only_itself(self) -> None:
        self.assertTrue(self.matches("justfile", "justfile"))
        self.assertFalse(self.matches("tools/labels/justfile", "justfile"))


class Checks(unittest.TestCase):
    """The two checks report a fault when given one."""

    def test_a_glob_with_no_tracked_path_is_reported(self) -> None:
        tax = {"build": {"robot": {"paths": ["packages/robot/**", "gone/**"]}}}
        self.assertEqual(
            unmatched_globs(tax, ["packages/robot/main.c"]), ["build.robot: gone/**"]
        )

    def test_a_scope_in_two_entries_is_reported(self) -> None:
        tax = {
            "build": {"robot": {"scopes": ["Robot  FW"]}},
            "area": {"ota": {"scopes": ["robot fw"]}},
        }
        self.assertEqual(shared_scopes(tax), ["robot fw: build.robot, area.ota"])

    def test_a_scope_equal_to_another_entry_key_is_reported(self) -> None:
        tax = {"build": {"robot": {"scopes": ["ota"]}}, "area": {"ota": {}}}
        self.assertEqual(shared_scopes(tax), ["ota: build.robot, area.ota"])

    def test_a_scope_equal_to_a_builtin_core_key_is_reported(self) -> None:
        tax = {"area": {"pipeline": {"scopes": ["ci"]}}}
        self.assertEqual(shared_scopes(tax), ["ci: area.pipeline, core.ci"])


class Taxonomy(unittest.TestCase):
    """The committed file against the tracked tree."""

    @classmethod
    def setUpClass(cls) -> None:
        cls.tax = load_taxonomy()
        cls.tracked = tracked_paths()

    def test_only_the_three_sections_are_defined(self) -> None:
        sections = [s for s in self.tax if not s.startswith(("$", "_"))]
        self.assertEqual(sorted(sections), sorted(SECTIONS))

    def test_the_tree_was_read(self) -> None:
        # An empty listing would make every glob read as stale, and a listing
        # of some other directory would make the result mean nothing.
        self.assertIn(".github/issue-labels.json", self.tracked)

    def test_every_glob_matches_a_tracked_path(self) -> None:
        self.assertEqual(unmatched_globs(self.tax, self.tracked), [])

    def test_every_scope_selects_one_entry(self) -> None:
        self.assertEqual(shared_scopes(self.tax), [])


if __name__ == "__main__":
    unittest.main()
