"""Every `kicad.*` CLI fails loudly when it finds nothing to process.

A per-project CLI run with no project argument processes every git-tracked
`packages/**/hardware/carrier/board.toml`. In a repo with none it must print
`0 project(s)` and exit 1, unless given `--allow-none`; otherwise a renamed or
not-yet-added board.toml turns every gate into a silent pass.

A new CLI module, in `kicad` or any subpackage of it, must be listed in
PROJECT_CLIS, REPO_CLIS or FILE_CLIS below: `test_every_cli_is_listed` fails
until it is.

    python3 -m unittest discover -s tools/kicad -t tools
"""

from __future__ import annotations

import argparse
import contextlib
import importlib
import importlib.util
import io
import pkgutil
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

import kicad
from kicad import paths
from kicad.errors import KicadError

# Modules whose main() takes project directories (paths.add_project_args).
PROJECT_CLIS: tuple[str, ...] = (
    "area",
    "boardcheck",
    "boardinput",
    "check",
    "fab",
    "netlist",
    "parity",
    "perfboard",
    "schematic",
    "stamp",
)
# Modules whose main() works on the repo as a whole; with an empty repo root
# each must exit 1. `vendor` checks tools/kicad/lib/, `derive_xiao` reads its
# derive.toml files and `sockets` its sockets.toml; none has projects.
REPO_CLIS = ("vendor", "derive_xiao", "sockets")
REPO_CLI_ARGS = {
    "vendor": ["--check"],
    "derive_xiao": ["--check"],
    "sockets": ["--check"],
}
# Modules whose main() takes file arguments, not projects; without them each
# must exit 1. (`jlc` cuts a BOM/CPL pair; kicad.fab calls it per project.
# `lcsc_rotation` takes an LCSC number and a footprint, or --check. `measure`
# takes pcb/dump.py's JSON; carrier-measure writes it per project.)
FILE_CLIS = ("jlc", "lcsc_rotation", "measure")

BOARD_TOML = '[project]\nname = "demo-carrier"\n'


def cli_modules(path, prefix: str) -> set[str]:
    """Dotted names (without `prefix`) of every module with a main(), at any depth.

    A package counts as a CLI when its `__main__` has a main() (`python3 -m
    kicad.perfboard`); test modules are skipped at every level. `pcb/` has no
    __init__.py, so it is not a package: its scripts run under KiCad's Python
    and are guarded by test_pcb_py39.
    """
    found = set()
    for mod in pkgutil.walk_packages(path, prefix):
        leaf = mod.name.rsplit(".", 1)[-1]
        if leaf.startswith("test_") or leaf == "__main__":
            continue  # a __main__ is counted as its package
        name = f"{mod.name}.__main__" if mod.ispkg else mod.name
        if mod.ispkg and importlib.util.find_spec(name) is None:
            continue
        if hasattr(importlib.import_module(name), "main"):
            found.add(mod.name.removeprefix(prefix))
    return found


def cli_main(name: str):
    """`kicad.<name>`'s main(); a package's is in its `__main__`."""
    module = importlib.import_module(f"kicad.{name}")
    if hasattr(module, "__path__"):
        module = importlib.import_module(f"kicad.{name}.__main__")
    return module.main


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
        found = cli_modules(kicad.__path__, "kicad.")
        self.assertEqual(found, set(PROJECT_CLIS) | set(REPO_CLIS) | set(FILE_CLIS))

    def test_cli_scan_descends_into_subpackages(self):
        # Control for the scan above: a CLI in a subpackage (like perfboard/),
        # as a module or as the package's __main__, must be found.
        files = {
            "fakecli/__init__.py": "",
            "fakecli/top.py": "def main(): pass\n",
            "fakecli/helper.py": "X = 1\n",
            "fakecli/test_top.py": "def main(): pass\n",
            "fakecli/sub/__init__.py": "",
            "fakecli/sub/tool.py": "def main(): pass\n",
            "fakecli/sub/test_tool.py": "def main(): pass\n",
            "fakecli/runnable/__init__.py": "",
            "fakecli/runnable/__main__.py": "def main(): pass\n",
        }
        with tempfile.TemporaryDirectory() as tmp:
            for rel, text in files.items():
                path = Path(tmp) / rel
                path.parent.mkdir(parents=True, exist_ok=True)
                path.write_text(text)
            sys.path.insert(0, tmp)
            try:
                pkg = importlib.import_module("fakecli")
                found = cli_modules(pkg.__path__, "fakecli.")
            finally:
                sys.path.remove(tmp)
                for name in [m for m in sys.modules if m.split(".")[0] == "fakecli"]:
                    del sys.modules[name]
        self.assertEqual(found, {"top", "sub.tool", "runnable"})

    def test_project_clis_fail_on_zero_projects(self):
        for name in (*PROJECT_CLIS, "demo"):
            main = demo_main if name == "demo" else cli_main(name)
            with self.subTest(cli=name):
                code, out = run(main, [], self.root)
                self.assertEqual(code, 1)
                self.assertIn("0 project(s)", out)
                code, out = run(main, ["--allow-none"], self.root)
                self.assertEqual(code, 0)
                self.assertIn("0 project(s)", out)

    def test_repo_clis_fail_on_an_empty_repo(self):
        for name in REPO_CLIS:
            main = cli_main(name)
            with self.subTest(cli=name):
                code, out = run(main, REPO_CLI_ARGS[name], self.root)
                self.assertEqual(code, 1)
                self.assertIn("FAIL:", out)

    def test_file_clis_fail_without_files(self):
        for name in FILE_CLIS:
            main = cli_main(name)
            with self.subTest(cli=name):
                out = io.StringIO()
                with contextlib.redirect_stdout(out):
                    code = main([])
                self.assertEqual(code, 1)
                self.assertIn("FAIL:", out.getvalue())

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

    def test_marker_selects_the_project_kind(self):
        # A perfboard project is found by its perfboard.toml, two levels down,
        # and a carrier's board.toml does not make it one.
        marker = "hardware/perfboard/perfboard.toml"
        git(self.root, "add", str(self.board))
        self.assertEqual(paths.tracked_projects(self.root, marker), [])
        toml = self.project / marker
        toml.parent.mkdir(parents=True)
        toml.write_text("")
        git(self.root, "add", str(toml))
        self.assertEqual(paths.tracked_projects(self.root, marker), [self.project])

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
        self.assertEqual(cfg.stage, paths.STAGE_BOARD)
        self.assertTrue(cfg.has_board)
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
            "[project]\nname = 'x'\nstage = 'routed'\n": "stage",
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


class SchematicStageTests(unittest.TestCase):
    """A carrier at `stage = "schematic"` has no board yet, and says so.

    The board gates (boardcheck, the stamp) skip it with a line naming the
    stage instead of failing on the missing .kicad_pcb; a carrier at the
    default stage with no board still fails.
    """

    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.root = Path(self.tmp.name)
        self.project = self.root / "packages/robotics/demo"
        self.board = paths.carrier_dir(self.project) / paths.BOARD_TOML
        self.board.parent.mkdir(parents=True)
        (self.root / "justfile").write_text("mod demo 'packages/robotics/demo'\n")

    def test_stage_schematic_has_no_board(self):
        self.board.write_text(BOARD_TOML + 'stage = "schematic"\n')
        cfg = paths.load_board_config(self.project)
        self.assertEqual(cfg.stage, paths.STAGE_SCHEMATIC)
        self.assertFalse(cfg.has_board)

    def test_board_gates_skip_a_schematic_stage_carrier(self):
        self.board.write_text(BOARD_TOML + 'stage = "schematic"\n')
        for name in ("boardcheck", "stamp"):
            with self.subTest(cli=name):
                code, out = run(cli_main(name), [str(self.project)], self.root)
                self.assertEqual(code, 0, out)
                self.assertIn('SKIP: packages/robotics/demo: stage = "schematic"', out)

    def test_board_gates_fail_a_board_stage_carrier_without_a_board(self):
        self.board.write_text(BOARD_TOML)
        for name in ("boardcheck", "stamp"):
            with self.subTest(cli=name):
                code, out = run(cli_main(name), [str(self.project)], self.root)
                self.assertEqual(code, 1, out)
                self.assertNotIn("SKIP", out)


class JustModuleTests(unittest.TestCase):
    """A hint names the root justfile's module, not the project directory (#766)."""

    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.root = Path(self.tmp.name)

    def justfile(self, text: str) -> None:
        (self.root / "justfile").write_text(text)

    def test_module_name_differs_from_directory(self):
        self.justfile(
            "mod robocar 'packages/robocar/docs'\n"
            "mod robocar-unified 'packages/robocar/unified'\n"
        )
        project = self.root / "packages/robocar/unified"
        self.assertEqual(paths.just_module(project, self.root), "robocar-unified")

    def test_mod_forms(self):
        self.justfile(
            '[group: "x"]\n'
            'mod a "packages/a"\n'
            "mod? b 'packages/b/'\n"
            "mod c 'packages/c/justfile'\n"
            "mod d\n"
            "mod f 'packages/f/mod.just'\n"
            "# mod e 'packages/e'\n"
        )
        for name, rel in (
            ("a", "packages/a"),
            ("b", "packages/b"),
            ("c", "packages/c"),
            ("d", "d"),
            ("f", "packages/f"),
        ):
            with self.subTest(name=name):
                self.assertEqual(paths.just_module(self.root / rel, self.root), name)
        for rel in ("packages/e", "packages"):
            with (
                self.subTest(rel=rel),
                self.assertRaisesRegex(KicadError, "no `mod` line"),
            ):
                paths.just_module(self.root / rel, self.root)

    def test_no_mod_line_fails(self):
        self.justfile("mod other 'packages/other'\n")
        with self.assertRaisesRegex(KicadError, r"no `mod` line.*packages/demo"):
            paths.just_module(self.root / "packages/demo", self.root)

    def test_no_justfile_fails(self):
        with self.assertRaisesRegex(KicadError, "justfile"):
            paths.just_module(self.root / "packages/demo", self.root)

    def test_repo_justfile(self):
        self.assertEqual(
            paths.just_module(paths.REPO_ROOT / "packages/robocar/unified"),
            "robocar-unified",
        )
        self.assertEqual(
            paths.just_module(paths.REPO_ROOT / "packages/robotics/balancebot"),
            "balancebot",
        )


if __name__ == "__main__":
    unittest.main()
