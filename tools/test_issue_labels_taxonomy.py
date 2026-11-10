"""Drift tests for .github/issue-labels.json, the repo's label taxonomy.

Stdlib unittest only, so the pre-commit hook can run it on a CI runner that
has nothing but python3 and git:

    python3 -m unittest tools/test_issue_labels_taxonomy.py

The taxonomy is read by git-plugin/scripts/issue-labels.py in
laurigates/claude-plugins, which .github/workflows/issue-labels.yml fetches at
a pinned commit. That script is not in this repo, so the parts of it these
tests depend on are copied from claude-plugins commit 2a46797 (2a46797490f5)
and must be changed with the workflow's ``plugins-ref``:

- ``glob_regex``, verbatim. ``*`` and ``?`` stay inside one path segment,
  ``**`` crosses segments, and a trailing ``/**`` also matches the directory
  itself.
- ``DEFAULT_CORE``, the built-in ``core`` globs, and the merge
  ``routing_entries`` applies: each ``core`` entry in the file is laid over the
  built-in entry of the same key field by field, so an entry without
  ``paths`` keeps the built-in globs.
- ``check_taxonomy``'s rules as ``schema_problems``: the sections, the fields
  each section allows, the key pattern, ``paths``/``scopes`` as lists of
  strings, and for ``build``/``area`` a quoted six-digit hex colour (required,
  as ``provision`` requires it), a description of at most 100 characters and
  a ``renamed_from`` that is a name or null and renames a label only once.
- An entry's key is one of its scopes, and scopes are compared lowercased with
  whitespace runs collapsed.

What the tests hold:

- The file passes the schema rules. The script refuses a file that fails
  them, so every run of the workflow would stop at the taxonomy.
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
PROVISIONED_SECTIONS = ("build", "area")
# Copied from issue-labels.py at 2a46797: the built-in core entries, which
# exist without being listed and supply any field a listed core entry omits.
DEFAULT_CORE: dict[str, dict] = {
    "ci": {"paths": [".github/**"]},
    "agent": {"paths": [".claude/**", ".mcp.json"]},
    "build": {"paths": ["justfile", "*.just"]},
    "quality": {"paths": [".pre-commit-config.yaml"]},
    "docs": {"paths": ["docs/**"]},
}
ENTRY_FIELDS = {
    "build": {"description", "color", "paths", "scopes", "renamed_from"},
    "area": {"description", "color", "paths", "scopes", "renamed_from"},
    "core": {"paths", "scopes"},
}
KEY = re.compile(r"^[a-z0-9][a-z0-9._-]*$")
COLOR = re.compile(r"^[0-9a-fA-F]{6}$")
MAX_DESCRIPTION = 100


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
    """(``section.key``, entry) for every entry, merged as routing_entries does:
    built-in core entries included, and each listed core entry laid over the
    built-in one of the same key."""
    core = {key: dict(entry) for key, entry in DEFAULT_CORE.items()}
    for key, entry in (tax.get("core") or {}).items():
        core[key] = {**core.get(key, {}), **entry}
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


def schema_problems(tax: object) -> list[str]:
    """What check_taxonomy(tax, for_provision=True) at 2a46797 rejects.

    The script stops classify, provision and audit on any of these, and a
    misspelt field (``path`` for ``paths``) would otherwise switch an entry's
    matching off without a word.
    """
    if not isinstance(tax, dict):
        return ["taxonomy is not a JSON object"]
    problems: list[str] = []
    renamed: dict[str, str] = {}
    for section, section_entries in tax.items():
        if section.startswith(("$", "_")):
            continue
        if section not in SECTIONS:
            problems.append(f"section '{section}' is not one of {SECTIONS}")
            continue
        if not isinstance(section_entries, dict):
            problems.append(f"section '{section}' is not an object")
            continue
        for key, entry in section_entries.items():
            where = f"{section}.{key}"
            if not KEY.match(key):
                problems.append(f"{where}: key must match {KEY.pattern}")
            if not isinstance(entry, dict):
                problems.append(f"{where} is not an object")
                continue
            for field in sorted(set(entry) - ENTRY_FIELDS[section]):
                problems.append(f"{where}: field '{field}' is not allowed")
            for field in ("paths", "scopes"):
                value = entry.get(field, [])
                if not isinstance(value, list) or not all(
                    isinstance(v, str) and v.strip() for v in value
                ):
                    problems.append(f"{where}.{field} must be a list of strings")
            if section not in PROVISIONED_SECTIONS:
                continue
            color = entry.get("color")
            if color is None:
                problems.append(f"{where}.color is required")
            elif not (isinstance(color, str) and COLOR.match(color)):
                problems.append(f"{where}.color must be six hex digits, quoted")
            description = entry.get("description", "")
            if not isinstance(description, str) or len(description) > MAX_DESCRIPTION:
                problems.append(
                    f"{where}.description must be a string of at most "
                    f"{MAX_DESCRIPTION} characters"
                )
            old = entry.get("renamed_from")
            if old is not None and not (isinstance(old, str) and old.strip()):
                problems.append(f"{where}.renamed_from must be a label name or null")
            elif old is not None:
                first = renamed.setdefault(norm(old), where)
                if first != where:
                    problems.append(f"{where}.renamed_from: already renamed by {first}")
    return problems


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
        missing = unmatched_globs(tax, ["packages/robot/main.c"])
        # The built-in core globs match nothing in this tree either.
        self.assertEqual(
            [m for m in missing if m.startswith("build.")], ["build.robot: gone/**"]
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


class CoreMerge(unittest.TestCase):
    """Core entries are merged over the built-in ones, as the script does."""

    def test_a_core_entry_without_paths_keeps_the_builtin_globs(self) -> None:
        tax = {"core": {"docs": {"scopes": ["prd"]}}}
        self.assertIn("core.docs: docs/**", unmatched_globs(tax, ["README.md"]))

    def test_a_core_entry_with_paths_replaces_the_builtin_globs(self) -> None:
        tax = {"core": {"docs": {"paths": ["README.md"]}}}
        missing = unmatched_globs(tax, ["README.md"])
        self.assertNotIn("core.docs: docs/**", missing)
        self.assertNotIn("core.docs: README.md", missing)

    def test_builtin_core_entries_are_checked_when_not_listed(self) -> None:
        self.assertIn("core.ci: .github/**", unmatched_globs({}, ["README.md"]))


class Schema(unittest.TestCase):
    """Each rule of the script's check_taxonomy rejects a taxonomy breaking it."""

    GOOD = {"description": "Robot", "color": "1d76db", "paths": ["robot/**"]}

    def rejects(self, tax: object, fragment: str) -> None:
        problems = schema_problems(tax)
        self.assertTrue(
            any(fragment in p for p in problems), f"{fragment!r} not in {problems}"
        )

    def test_a_sound_taxonomy_passes(self) -> None:
        tax = {
            "$comment": "ignored",
            "build": {"robot": dict(self.GOOD, scopes=["bot"], renamed_from=None)},
            "core": {"ci": {"paths": [".github/**"], "scopes": ["release"]}},
        }
        self.assertEqual(schema_problems(tax), [])

    def test_an_unknown_section_is_rejected(self) -> None:
        self.rejects({"needs": {"x": {}}}, "section 'needs'")

    def test_a_misspelt_field_is_rejected(self) -> None:
        entry = {"color": "1d76db", "path": ["robot/**"]}
        self.rejects({"build": {"robot": entry}}, "field 'path'")

    def test_a_colour_or_description_on_a_core_entry_is_rejected(self) -> None:
        self.rejects({"core": {"ci": {"color": "1d76db"}}}, "field 'color'")
        self.rejects({"core": {"ci": {"description": "x"}}}, "field 'description'")

    def test_an_uppercase_key_is_rejected(self) -> None:
        self.rejects({"area": {"OTA": dict(self.GOOD)}}, "key must match")

    def test_paths_and_scopes_must_be_lists_of_strings(self) -> None:
        self.rejects({"area": {"ota": dict(self.GOOD, paths="ota/**")}}, ".paths")
        self.rejects({"area": {"ota": dict(self.GOOD, scopes=[""])}}, ".scopes")

    def test_a_colour_with_a_hash_is_rejected(self) -> None:
        self.rejects({"area": {"ota": dict(self.GOOD, color="#0e8a16")}}, ".color")

    def test_an_unquoted_colour_is_rejected(self) -> None:
        self.rejects({"area": {"ota": dict(self.GOOD, color=123456)}}, ".color")

    def test_a_missing_colour_is_rejected(self) -> None:
        entry = {"description": "OTA"}
        self.rejects({"area": {"ota": entry}}, ".color is required")

    def test_a_long_description_is_rejected(self) -> None:
        entry = dict(self.GOOD, description="x" * (MAX_DESCRIPTION + 1))
        self.rejects({"area": {"ota": entry}}, ".description")

    def test_a_description_at_the_limit_passes(self) -> None:
        entry = dict(self.GOOD, description="x" * MAX_DESCRIPTION)
        self.assertEqual(schema_problems({"area": {"ota": entry}}), [])

    def test_an_empty_renamed_from_is_rejected(self) -> None:
        entry = dict(self.GOOD, renamed_from=" ")
        self.rejects({"area": {"ota": entry}}, ".renamed_from")

    def test_one_label_renamed_twice_is_rejected(self) -> None:
        tax = {
            "build": {"robot": dict(self.GOOD, renamed_from="Robot")},
            "area": {"ota": dict(self.GOOD, renamed_from="robot")},
        }
        self.rejects(tax, "already renamed by build.robot")


class Taxonomy(unittest.TestCase):
    """The committed file against the tracked tree."""

    @classmethod
    def setUpClass(cls) -> None:
        cls.tax = load_taxonomy()
        cls.tracked = tracked_paths()

    def test_only_the_three_sections_are_defined(self) -> None:
        sections = [s for s in self.tax if not s.startswith(("$", "_"))]
        self.assertEqual(sorted(sections), sorted(SECTIONS))

    def test_the_file_passes_the_scripts_schema(self) -> None:
        self.assertEqual(schema_problems(self.tax), [])

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
