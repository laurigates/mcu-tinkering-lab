"""Every `kicad.*` CLI fails loudly when it finds nothing to process.

A per-project CLI run with no project argument processes every git-tracked
`packages/**/hardware/carrier/board.toml`. In a repo with none it must print
`0 project(s)` and exit 1, unless given `--allow-none`; otherwise a renamed or
not-yet-added board.toml turns every gate into a silent pass.

A new CLI module must be listed in PROJECT_CLIS or REPO_CLIS below:
`test_every_cli_is_listed` fails until it is.

    python3 -m unittest discover -s tools/kicad -t tools
"""

from __future__ import annotations

import argparse
import contextlib
import importlib
import io
import pkgutil
import subprocess
import tempfile
import unittest
from pathlib import Path

import kicad
from kicad import paths
from kicad.errors import KicadError

# Modules whose main() takes project directories (paths.add_project_args).
PROJECT_CLIS: tuple[str, ...] = ()
# Modules whose main() works on the repo as a whole; with an empty repo root
# each must exit 1. `vendor` checks tools/kicad/lib/ and `derive_xiao` reads its
# derive.toml files; neither has projects.
REPO_CLIS = ("vendor", "derive_xiao")
REPO_CLI_ARGS = {"vendor": ["--check"], "derive_xiao": ["--check"]}

BOARD_TOML = '[project]\nname = "demo-carrier"\n'


def git(root: Path, *args: str) -> None:
    subprocess.run(["git", *args], cwd=root, check=True, capture_output=True)


def run(main, argv, repo_root):
    out = io.StringIO()
    with contextlib.redirect_stdout(out):
        code = main(argv, repo_root=repo_root)
    return code, out.getvalue()


def demo_main(argv=None, repo_root=paths.REPO_ROOT) -> int:
    """The shape every per-project CLI's main() has."""
    parser = argparse.ArgumentParser(prog="kicad.demo")
    paths.add_project_args(parser)
    args = parser.parse_args(argv)
    try:
        paths.select_projects(args.projects, args.allow_none, repo_root)
    except KicadError as e:
        print(f"FAIL: {e}")
        return 1
    return 0


class EmptyRepoTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.root = Path(self.tmp.name)
        git(self.root, "init", "-q")

    def test_every_cli_is_listed(self):
        found = set()
        for mod in pkgutil.iter_modules(kicad.__path__):
            if mod.name.startswith("test_") or mod.ispkg:
                continue
            if hasattr(importlib.import_module(f"kicad.{mod.name}"), "main"):
                found.add(mod.name)
        self.assertEqual(found, set(PROJECT_CLIS) | set(REPO_CLIS))

    def test_project_clis_fail_on_zero_projects(self):
        for name in (*PROJECT_CLIS, "demo"):
            main = (
                demo_main
                if name == "demo"
                else importlib.import_module(f"kicad.{name}").main
            )
            with self.subTest(cli=name):
                code, out = run(main, [], self.root)
                self.assertEqual(code, 1)
                self.assertIn("0 project(s)", out)
                code, out = run(main, ["--allow-none"], self.root)
                self.assertEqual(code, 0)
                self.assertIn("0 project(s)", out)

    def test_repo_clis_fail_on_an_empty_repo(self):
        for name in REPO_CLIS:
            main = importlib.import_module(f"kicad.{name}").main
            with self.subTest(cli=name):
                code, out = run(main, REPO_CLI_ARGS[name], self.root)
                self.assertEqual(code, 1)
                self.assertIn("FAIL:", out)

    def test_not_a_git_work_tree(self):
        with tempfile.TemporaryDirectory() as tmp, self.assertRaises(KicadError):
            paths.tracked_projects(Path(tmp))


class TrackedProjectTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.root = Path(self.tmp.name)
        git(self.root, "init", "-q")
        self.project = self.root / "packages/robotics/demo"
        self.board = paths.carrier_dir(self.project) / "board.toml"
        self.board.parent.mkdir(parents=True)
        self.board.write_text(BOARD_TOML)

    def test_untracked_board_toml_is_not_a_project(self):
        self.assertEqual(paths.tracked_projects(self.root), [])

    def test_tracked_board_toml_is_a_project(self):
        git(self.root, "add", str(self.board))
        self.assertEqual(paths.tracked_projects(self.root), [self.project])
        code, out = run(demo_main, [], self.root)
        self.assertEqual(code, 0)
        self.assertIn("1 project(s): packages/robotics/demo", out)

    def test_deleted_board_toml_is_not_a_project(self):
        git(self.root, "add", str(self.board))
        self.board.unlink()
        self.assertEqual(paths.tracked_projects(self.root), [])

    def test_explicit_project_needs_no_tracking(self):
        code, out = run(demo_main, [str(self.project)], self.root)
        self.assertEqual(code, 0)
        self.assertIn("1 project(s): packages/robotics/demo", out)

    def test_board_config(self):
        cfg = paths.load_board_config(self.project)
        self.assertEqual((cfg.name, cfg.paper), ("demo-carrier", "A3"))
        self.assertEqual(
            paths.kicad_project_dir(self.project), self.board.parent / "demo-carrier"
        )

    def test_board_config_rejects(self):
        cases = {
            "[project]\nname = 'x'\n[placment]\n": "unknown table",
            "[project]\nname = 'x'\npapr = 'A4'\n": "unknown key",
            "[outline]\n": r"\[project\] is required",
            "[project]\nname = 'a/b'\n": "file stem",
            "[project]\nname = 'x'\npaper = 3\n": "paper",
            "[project\n": "board.toml",
        }
        for text, msg in cases.items():
            with self.subTest(text=text):
                self.board.write_text(text)
                with self.assertRaisesRegex(KicadError, msg):
                    paths.load_board_config(self.project)
        self.board.unlink()
        with self.assertRaisesRegex(KicadError, "no board.toml"):
            paths.load_board_config(self.project)


if __name__ == "__main__":
    unittest.main()
