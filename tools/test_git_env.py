"""Every test suite that builds a throwaway git repository scrubs GIT_* first.

During a `git commit` from a linked worktree, git exports GIT_DIR and an
absolute GIT_INDEX_FILE to the pre-commit hooks. A suite run as a hook that
then spawns `git init`, `git add` or `git commit` in a temporary directory
acts on the repository being committed instead: stray commits, staged files,
core.bare set to true (issue #1123). The code under test spawns git too, so
the scrub covers the whole test process, not each call:

- a test module calls ``setUpModule = gitenv.isolate`` (tools/gitenv.py);
- a pytest project deletes every GIT_* variable in an autouse fixture in its
  conftest.py, as docs/schematics/conftest.py does.

This module fails when a tracked test module or conftest.py that creates a
repository does neither. A suite that only reads the real repository
(``git ls-files``, ``git tag``) is left alone: during a commit the inherited
index is the one it should read.

Run: python3 -m unittest tools/test_git_env.py
"""

from __future__ import annotations

import os
import re
import subprocess
import sys
import tempfile
import textwrap
import unittest
from pathlib import Path, PurePosixPath

REPO_ROOT = Path(__file__).resolve().parent.parent
TOOLS = REPO_ROOT / "tools"

# A quoted git argv element (also the argument of shutil.which) together with
# a quoted init argument, or a quoted command string that starts with
# `git init`. Creating a repository is what makes a suite a writer: every one
# of them calls init. (This comment and the synthetic sources below avoid
# those quoted forms, or this file would read as one that creates a
# repository.)
GIT_ARGV = re.compile(r"""["']git["']""")
INIT_ARG = re.compile(r"""["']init["']""")
INIT_STRING = re.compile(r"""["']git\s+init\b""")
MODULE_SCRUB = re.compile(r"^setUpModule = gitenv\.isolate$", re.MULTILINE)
CONFTEST_SCRUB = (
    re.compile(r"""startswith\(\s*["']GIT_["']\s*\)"""),
    re.compile(r"monkeypatch\.delenv\("),
    re.compile(r"autouse\s*=\s*True"),
)


def creates_repository(source: str) -> bool:
    """The source spawns `git init` (and so the git calls that follow it)."""
    return bool(
        (GIT_ARGV.search(source) and INIT_ARG.search(source))
        or INIT_STRING.search(source)
    )


def module_scrubs(source: str) -> bool:
    return bool(MODULE_SCRUB.search(source))


def conftest_scrubs(source: str) -> bool:
    return all(p.search(source) for p in CONFTEST_SCRUB)


def tracked_test_files() -> list[str]:
    out = subprocess.run(
        [
            "git",
            "ls-files",
            "-z",
            "--",
            ":(glob)**/test_*.py",
            ":(glob)**/conftest.py",
        ],
        cwd=REPO_ROOT,
        capture_output=True,
        text=True,
        check=True,
    ).stdout
    return sorted(p for p in out.split("\0") if p)


def unscrubbed(sources: dict[str, str]) -> list[str]:
    """Paths in ``sources`` (repo-relative path -> text) that create a
    repository with neither their own scrub nor a scrubbing conftest.py in
    their directory or above it."""
    scrubbing_dirs = {
        PurePosixPath(path).parent
        for path, text in sources.items()
        if PurePosixPath(path).name == "conftest.py" and conftest_scrubs(text)
    }
    found = []
    for path, text in sorted(sources.items()):
        if not creates_repository(text):
            continue
        p = PurePosixPath(path)
        if p.name == "conftest.py":
            if not conftest_scrubs(text):
                found.append(path)
            continue
        if module_scrubs(text):
            continue
        if any(d == p.parent or d in p.parents for d in scrubbing_dirs):
            continue
        found.append(path)
    return found


def read_sources(paths: list[str]) -> dict[str, str]:
    return {p: (REPO_ROOT / p).read_text(encoding="utf-8") for p in paths}


# Synthetic sources are built by concatenation so that this file does not
# itself read as a module that creates a repository.
GIT = "gi" + "t"
INIT = "in" + "it"
SPAWN = f'subprocess.run(["{GIT}", "{INIT}"], cwd=tmp)\n'
ISOLATE_LINE = "setUp" + "Module = gitenv.isolate\n"
READ_ONLY = f'subprocess.run(["{GIT}", "ls-files"], cwd=ROOT)\n'


class GitEnvScanTest(unittest.TestCase):
    def setUp(self) -> None:
        self.paths = tracked_test_files()
        self.sources = read_sources(self.paths)

    def test_every_suite_that_creates_a_repository_scrubs(self) -> None:
        self.assertEqual(
            unscrubbed(self.sources),
            [],
            "these tests create a git repository with the inherited GIT_* "
            "environment; add `setUpModule = gitenv.isolate` (tools/gitenv.py) "
            "or, in a pytest project, an autouse conftest fixture that deletes "
            "every GIT_* variable (#1123)",
        )

    def test_scan_matches_known_repository_creators(self) -> None:
        # Control: a pattern that stops matching would pass every file.
        creators = {p for p, t in self.sources.items() if creates_repository(t)}
        for known in (
            "tools/test_post_edit_lint_hook.py",
            "tools/test_flash_source.py",
            "tools/test_workflows.py",
            "tools/hardware/test_docs.py",
            "tools/kicad/test_vendor.py",
            "docs/schematics/test_freshness.py",
        ):
            self.assertIn(known, creators)
        for read_only in (
            "tools/test_python_suites.py",
            "tools/test_ruff_config.py",
            "tools/test_issue_labels_taxonomy.py",
            "tools/test_check_flash_recipes.py",
        ):
            self.assertIn(read_only, self.sources)
            self.assertNotIn(read_only, creators)

    def test_scan_sees_the_test_files(self) -> None:
        self.assertIn("tools/test_git_env.py", self.paths)
        self.assertIn("docs/schematics/conftest.py", self.paths)

    def test_verdicts_on_synthetic_sources(self) -> None:
        conftest = (
            "@pytest.fixture(autouse=True)\n"
            "def _no_git_env(monkeypatch):\n"
            "    for key in [k for k in os.environ if k.startswith('GIT_')]:\n"
            "        monkeypatch.delenv(key)\n"
        )
        cases = {
            "a/test_bare.py": SPAWN,
            "a/test_isolated.py": ISOLATE_LINE + SPAWN,
            "a/test_reads.py": READ_ONLY,
            "a/test_shell.py": f"subprocess.run('{GIT} {INIT} -q', shell=True)\n",
            "b/conftest.py": conftest,
            "b/sub/test_covered.py": SPAWN,
            "c/conftest.py": "import pytest\n",
            "c/test_uncovered.py": SPAWN,
            "d/conftest.py": SPAWN,
        }
        self.assertEqual(
            unscrubbed(cases),
            [
                "a/test_bare.py",
                "a/test_shell.py",
                "c/test_uncovered.py",
                "d/conftest.py",
            ],
        )


class IsolateTest(unittest.TestCase):
    def test_isolate_scrubs_for_the_module_and_restores_after(self) -> None:
        module = textwrap.dedent(
            f"""\
            import os, sys, unittest
            sys.path.insert(0, {str(TOOLS)!r})
            import gitenv

            setUpModule = gitenv.isolate

            class T(unittest.TestCase):
                def test_scrubbed(self):
                    self.assertNotIn("GIT_DIR", os.environ)
                    self.assertNotIn("GIT_INDEX_FILE", os.environ)
                    self.assertEqual(os.environ.get("KEEP_ME"), "1")

            if __name__ == "__main__":
                prog = unittest.main(exit=False)
                print("after:", os.environ.get("GIT_DIR"))
                sys.exit(0 if prog.result.wasSuccessful() else 1)
            """
        )
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "probe_isolate.py"
            path.write_text(module)
            result = subprocess.run(
                [sys.executable, str(path)],
                capture_output=True,
                text=True,
                env={
                    **os.environ,
                    "GIT_DIR": "/nonexistent/decoy/.git",
                    "GIT_INDEX_FILE": "/nonexistent/decoy/.git/index",
                    "KEEP_ME": "1",
                },
                timeout=60,
            )
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn("after: /nonexistent/decoy/.git", result.stdout)


if __name__ == "__main__":
    unittest.main()
