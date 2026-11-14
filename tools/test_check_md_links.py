"""Unit tests for check-md-links.py (issue #1147).

Stdlib unittest only, so the pre-commit hook runs it with nothing but python3:

    python3 -m unittest tools/test_check_md_links.py

The scan takes the tracked-file set and a reader as arguments, so most tests
pass an in-memory tree and create no git repository. The last class runs the
check over this repository.
"""

from __future__ import annotations

import contextlib
import importlib.util
import io
import sys
import tempfile
import unittest
from pathlib import Path
from unittest import mock

_SPEC = importlib.util.spec_from_file_location(
    "check_md_links", Path(__file__).with_name("check-md-links.py")
)
cml = importlib.util.module_from_spec(_SPEC)
assert _SPEC.loader is not None
sys.modules[_SPEC.name] = cml
_SPEC.loader.exec_module(cml)

REPO_ROOT = Path(__file__).resolve().parent.parent


def scan(tree: dict[str, str], excludes=()) -> tuple[int, int, list[str]]:
    """Scan an in-memory tree: {path: text}, every path tracked."""
    return cml.scan(set(tree), tree.__getitem__, excludes)


class TargetsTest(unittest.TestCase):
    def targets(self, text: str) -> list[tuple[int, str]]:
        return list(cml.targets(text))

    def test_inline_link_image_and_angle_target(self):
        text = '[a](x.md) ![b](img/y.png "t")\n[c](<with space.md>)\n'
        self.assertEqual(
            self.targets(text), [(1, "x.md"), (1, "img/y.png"), (2, "<with space.md>")]
        )

    def test_link_text_with_one_nested_bracket(self):
        self.assertEqual(
            self.targets("[![badge](b.svg)](page.md)"), [(1, "b.svg"), (1, "page.md")]
        )

    def test_reference_definition(self):
        self.assertEqual(
            self.targets('[id]: ../a.csv\n[t]: b.md "title"\n'),
            [(1, "../a.csv"), (2, "b.md")],
        )

    def test_running_prose_after_a_bracket_is_not_a_definition(self):
        # docs/reference/jlc-blog/3dp-enclosure-mechanism.md wraps a line this way.
        self.assertEqual(self.targets("[4]: SLA M3 hole 4.1-4.2), and so on"), [])

    def test_backtick_and_tilde_fences_are_skipped(self):
        text = "```md\n[a](gone.md)\n```\n~~~\n[b](gone.md)\n~~~\n[c](kept.md)\n"
        self.assertEqual(self.targets(text), [(7, "kept.md")])

    def test_a_shorter_fence_inside_a_longer_one_does_not_close_it(self):
        text = "````\n```\n[a](gone.md)\n```\n````\n[b](kept.md)\n"
        self.assertEqual(self.targets(text), [(6, "kept.md")])

    def test_inline_code_is_skipped(self):
        self.assertEqual(
            self.targets("`[a](gone.md)` and ``[b](gone.md)`` [c](kept.md)"),
            [(1, "kept.md")],
        )


class ResolveTest(unittest.TestCase):
    def test_relative_to_the_linking_file(self):
        self.assertEqual(cml.resolve("docs/a/b.md", "../c.md"), "docs/c.md")

    def test_leading_slash_is_the_repo_root(self):
        self.assertEqual(cml.resolve("docs/a/b.md", "/tools/x.py"), "tools/x.py")

    def test_anchor_query_and_percent_escape(self):
        self.assertEqual(cml.resolve("d/a.md", "b%20c.md#sec?x"), "d/b c.md")

    def test_names_no_local_path(self):
        for raw in (
            "https://x.org/a.md",
            "mailto:a@b",
            "//host/a",
            "#anchor",
            "../{name}.md",
            "a/.../b.md",
            "$(x).md",
        ):
            with self.subTest(raw=raw):
                self.assertIsNone(cml.resolve("d/a.md", raw))

    def test_climbing_above_the_root(self):
        self.assertTrue(cml.resolve("a.md", "../x.md").startswith("<outside repo>/"))


class ScanTest(unittest.TestCase):
    def test_wrong_dot_dot_count_is_broken_and_the_right_one_is_not(self):
        tree = {
            "docs/boards/b.md": "[ok](../../decisions/A.md) [bad](../decisions/A.md)",
            "decisions/A.md": "",
        }
        docs, checked, broken = scan(tree)
        self.assertEqual((docs, checked), (2, 2))
        self.assertEqual(
            broken, ["docs/boards/b.md:1: ../decisions/A.md -> docs/decisions/A.md"]
        )

    def test_missing_file_and_broken_reference_definition(self):
        tree = {"p/README.md": "[x](GONE.md)\n\n[r]: ../../old/p.csv\n"}
        _, checked, broken = scan(tree)
        self.assertEqual(checked, 2)
        self.assertEqual(
            broken,
            [
                "p/README.md:1: GONE.md -> p/GONE.md",
                "p/README.md:3: ../../old/p.csv -> <outside repo>/../../old/p.csv",
            ],
        )

    def test_directory_holding_tracked_files_resolves(self):
        tree = {
            "docs/a.md": "[d](../packages/x/) [e](../packages/x)",
            "packages/x/main/m.c": "",
        }
        self.assertEqual(scan(tree)[2], [])

    def test_untracked_target_is_broken(self):
        # The reader can see a file git does not track; the scan must not.
        files = {"a.md": "[x](local.md)", "local.md": ""}
        _, _, broken = cml.scan({"a.md"}, files.__getitem__, ())
        self.assertEqual(broken, ["a.md:1: local.md -> local.md"])

    def test_excluded_doc_is_not_scanned(self):
        tree = {
            "tools/kicad/testdata/x/b.md": "[x](gone.md)",
            "a.md": "[y](b.md)",
            "b.md": "",
        }
        docs, checked, broken = scan(tree, cml.EXCLUDES)
        self.assertEqual((docs, checked, broken), (2, 1, []))

    def test_non_markdown_files_are_not_scanned(self):
        self.assertEqual(
            scan({"a.txt": "[x](gone.md)", "a.md": "[y](a.txt)"}), (1, 1, [])
        )


class MainTest(unittest.TestCase):
    def run_main(self, tree: dict[str, str]) -> tuple[int, str, str]:
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            for rel, text in tree.items():
                (root / rel).parent.mkdir(parents=True, exist_ok=True)
                (root / rel).write_text(text, encoding="utf-8")
            out, err = io.StringIO(), io.StringIO()
            with (
                mock.patch.object(cml, "git_ls", return_value=list(tree)),
                contextlib.redirect_stdout(out),
                contextlib.redirect_stderr(err),
            ):
                rc = cml.main(["--root", tmp])
        return rc, out.getvalue(), err.getvalue()

    def test_clean(self):
        rc, out, _ = self.run_main({"a.md": "[b](b.md)", "b.md": ""})
        self.assertEqual(rc, 0)
        self.assertIn("links_checked=1 broken=0", out)

    def test_broken(self):
        rc, _, err = self.run_main({"a.md": "[b](gone.md)"})
        self.assertEqual(rc, 1)
        self.assertIn("a.md:1: gone.md -> gone.md", err)

    def test_nothing_checked_is_not_a_pass(self):
        rc, _, err = self.run_main({"a.md": "no links, only https://x.org"})
        self.assertEqual(rc, 2)
        self.assertIn("nothing was checked", err)


class RepositoryTest(unittest.TestCase):
    def test_every_relative_link_resolves(self):
        docs, checked, broken = cml.check(REPO_ROOT)
        self.assertGreater(checked, 0)
        self.assertEqual(broken, [], f"{len(broken)} of {checked} links in {docs} docs")

    def test_each_exclude_has_a_reason_and_still_hides_a_broken_link(self):
        # An exclude that hides nothing has outlived its reason: drop it.
        tracked = set(cml.git_ls(REPO_ROOT))
        for glob, reason in cml.EXCLUDES.items():
            with self.subTest(glob=glob):
                self.assertTrue(reason.strip())
                others = {g: r for g, r in cml.EXCLUDES.items() if g != glob}
                _, _, broken = cml.scan(
                    tracked,
                    lambda d: (REPO_ROOT / d).read_text(
                        encoding="utf-8", errors="replace"
                    ),
                    others,
                )
                hidden = [
                    b for b in broken if cml.excluded(b.split(":", 1)[0], {glob: ""})
                ]
                self.assertTrue(hidden, f"{glob} excludes no broken link")

    def test_template_examples_in_inline_code_are_not_links(self):
        # #1147: the two example paths stay as they are, and need no exclude.
        doc = (
            REPO_ROOT
            / "packages/networking/it-troubleshooter/.claude/rules/document-management.md"
        )
        text = doc.read_text(encoding="utf-8")
        self.assertIn("(../adrs/0001-title.md)", text)
        self.assertIn("(../prds/feature.md)", text)
        raws = [raw for _, raw in cml.targets(text)]
        self.assertNotIn("../adrs/0001-title.md", raws)
        self.assertNotIn("../prds/feature.md", raws)


if __name__ == "__main__":
    unittest.main()
