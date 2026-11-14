"""Tests for .claude/hooks/post-edit-lint.sh, the PostToolUse Edit/Write hook.

The hook formats the file an Edit or Write just touched. Issue #1111 found two
ways it cost work:

1. It formatted files that are not in this repository (a checkout of another
   project under the session scratchpad), rewriting them in ruff's defaults.
2. Its `ruff check --fix` removed an import added in one Edit before the next
   Edit used it, and a `# noqa` comment the same way.

Each test builds a throwaway project repository with a copy of the hook, so
nothing here reads or writes the real checkout. A stub `ruff` on PATH records
its arguments, which shows whether the hook ran at all. The last test runs the
real ruff when one is on PATH, as the hook does.

Git calls here and the hook itself run without inherited GIT_* variables.
InheritedGitEnvTest re-runs every test with the GIT_DIR and GIT_INDEX_FILE a
pre-commit hook gets during a commit from a linked worktree, aimed at a decoy
repository that must stay unchanged.

Run: python3 -m unittest tools/test_post_edit_lint_hook.py
"""

from __future__ import annotations

import json
import os
import shutil
import subprocess
import tempfile
import unittest
from pathlib import Path
from unittest import mock

try:
    from tools import gitenv
except ImportError:  # run as a script: python3 tools/test_post_edit_lint_hook.py
    import gitenv

setUpModule = gitenv.isolate

REPO = Path(__file__).resolve().parent.parent
HOOK = REPO / ".claude" / "hooks" / "post-edit-lint.sh"
RUFF_TOML = REPO / "ruff.toml"

STUB_RUFF = """#!/usr/bin/env bash
printf '%s\\n' "$*" >>"$RUFF_LOG"
"""


def git(cwd: Path, *args: str) -> str:
    result = subprocess.run(
        ["git", "-C", str(cwd), *args],
        check=True,
        capture_output=True,
        text=True,
        env={
            **gitenv.clean_env(),
            "GIT_CONFIG_GLOBAL": os.devnull,
            "GIT_CONFIG_NOSYSTEM": "1",
        },
    )
    return result.stdout


def init_repo(path: Path) -> Path:
    path.mkdir(parents=True, exist_ok=True)
    git(path, "init", "-q")
    return path


class PostEditLintHookTest(unittest.TestCase):
    def setUp(self) -> None:
        self._tmp = tempfile.TemporaryDirectory()
        self.tmp = Path(self._tmp.name).resolve()
        self.addCleanup(self._tmp.cleanup)

        self.project = init_repo(self.tmp / "project")
        hooks = self.project / ".claude" / "hooks"
        hooks.mkdir(parents=True)
        shutil.copy2(HOOK, hooks / HOOK.name)
        shutil.copy2(RUFF_TOML, self.project / "ruff.toml")
        self.hook = hooks / HOOK.name

        self.bin = self.tmp / "bin"
        self.bin.mkdir()
        stub = self.bin / "ruff"
        stub.write_text(STUB_RUFF)
        stub.chmod(0o755)
        self.log = self.tmp / "ruff.log"

    def run_hook(
        self,
        file_path: Path,
        *,
        stub_ruff: bool = True,
        extra_env: dict[str, str] | None = None,
    ) -> None:
        env = {
            **gitenv.clean_env(),
            "CLAUDE_PROJECT_DIR": str(self.project),
            "RUFF_LOG": str(self.log),
            **(extra_env or {}),
        }
        if stub_ruff:
            env["PATH"] = f"{self.bin}{os.pathsep}{env['PATH']}"
            # The stub must be the ruff the hook finds, or a real ruff runs and
            # the log stays empty for the wrong reason.
            found = shutil.which("ruff", path=env["PATH"])
            self.assertEqual(found, str(self.bin / "ruff"))
        payload = json.dumps(
            {"tool_name": "Edit", "tool_input": {"file_path": str(file_path)}}
        )
        result = subprocess.run(
            ["bash", str(self.hook)],
            input=payload,
            capture_output=True,
            text=True,
            env=env,
            timeout=60,
        )
        self.assertEqual(result.returncode, 0, result.stderr)

    def ruff_calls(self) -> list[str]:
        if not self.log.exists():
            return []
        return self.log.read_text().splitlines()

    def write_py(self, path: Path) -> Path:
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text("x = 1\n")
        return path

    # -- which files the hook touches ------------------------------------

    def test_untracked_file_in_project_is_formatted(self) -> None:
        """Control: a new file in the project reaches ruff."""
        path = self.write_py(self.project / "tools" / "new_tool.py")
        self.run_hook(path)
        calls = self.ruff_calls()
        self.assertTrue(any(c.startswith("format ") for c in calls), calls)

    def test_file_outside_any_repo_is_left_alone(self) -> None:
        path = self.write_py(self.tmp / "scratchpad" / "notes.py")
        self.run_hook(path)
        self.assertEqual(self.ruff_calls(), [])

    def test_file_in_another_repo_outside_project_is_left_alone(self) -> None:
        """The #1111 case: a checkout of another project in the scratchpad."""
        other = init_repo(self.tmp / "scratchpad" / "inkscape-silhouette")
        path = self.write_py(other / "silhouette" / "Graphtec.py")
        self.run_hook(path)
        self.assertEqual(self.ruff_calls(), [])

    def test_foreign_clone_inside_project_is_left_alone(self) -> None:
        """A clone under the project's tmp/ is still someone else's code."""
        other = init_repo(self.project / "tmp" / "upstream")
        path = self.write_py(other / "module.py")
        self.run_hook(path)
        self.assertEqual(self.ruff_calls(), [])

    def test_c_file_outside_project_is_left_alone(self) -> None:
        other = init_repo(self.tmp / "scratchpad" / "other-fw")
        cq = other / "tools" / "cquality.sh"
        cq.parent.mkdir(parents=True)
        marker = self.tmp / "cquality-ran"
        cq.write_text(f'#!/usr/bin/env bash\ntouch "{marker}"\n')
        cq.chmod(0o755)
        src = other / "main" / "main.c"
        src.parent.mkdir(parents=True)
        src.write_text("int main(void){return 0;}\n")
        self.run_hook(src)
        self.assertFalse(marker.exists())

    def test_git_dir_in_hook_env_does_not_widen_scope(self) -> None:
        """GIT_DIR set in the hook's own environment must not adopt foreign files.

        With GIT_DIR pointing at the project, every path would resolve to the
        project's git directory and the scope check would pass for a file in
        another repository.
        """
        other = init_repo(self.tmp / "scratchpad" / "other-project")
        path = self.write_py(other / "module.py")
        self.run_hook(
            path,
            extra_env={
                "GIT_DIR": str(self.project / ".git"),
                "GIT_WORK_TREE": str(self.project),
            },
        )
        self.assertEqual(self.ruff_calls(), [])

    def test_worktree_of_project_is_formatted(self) -> None:
        """Agents edit in worktrees under .claude/worktrees/; those are this repo."""
        git(self.project, "add", "-A")
        git(
            self.project,
            "-c",
            "user.name=t",
            "-c",
            "user.email=t@example.invalid",
            "commit",
            "-q",
            "-m",
            "init",
        )
        worktree = self.project / ".claude" / "worktrees" / "agent-1"
        git(self.project, "worktree", "add", "-q", "--detach", str(worktree))
        path = self.write_py(worktree / "tools" / "edited.py")
        self.run_hook(path)
        calls = self.ruff_calls()
        self.assertTrue(any(c.startswith("format ") for c in calls), calls)

    # -- what the hook may change in a file ------------------------------

    def test_fix_leaves_unused_imports_and_noqa_alone(self) -> None:
        path = self.write_py(self.project / "tools" / "new_tool.py")
        self.run_hook(path)
        checks = [c for c in self.ruff_calls() if c.startswith("check ")]
        self.assertEqual(len(checks), 1, self.ruff_calls())
        args = checks[0].split()
        self.assertIn("--unfixable", args)
        unfixable = args[args.index("--unfixable") + 1].split(",")
        self.assertIn("F401", unfixable)
        self.assertIn("RUF100", unfixable)

    @unittest.skipUnless(
        shutil.which("ruff"), "no ruff on PATH (the hook skips Python then too)"
    )
    def test_not_yet_used_import_survives_real_ruff(self) -> None:
        """An Edit that adds imports before the Edit that uses them keeps them."""
        path = self.project / "tools" / "test_labels.py"
        path.parent.mkdir(parents=True)
        source = "import io\nimport types\n\nx = 1  # noqa: E501\n"
        path.write_text(source)
        self.run_hook(path, stub_ruff=False)
        after = path.read_text()
        self.assertIn("import io\n", after)
        self.assertIn("import types\n", after)
        self.assertIn("# noqa: E501", after)


class InheritedGitEnvTest(PostEditLintHookTest):
    """Every test above, under the git environment of a commit from a worktree.

    pre-commit runs this suite as a hook. During a `git commit` from a linked
    worktree it inherits GIT_DIR=<common>/worktrees/<name> and an absolute
    GIT_INDEX_FILE. Here both point at a decoy repository, which must come out
    of each test unchanged.
    """

    def setUp(self) -> None:
        decoy_tmp = tempfile.TemporaryDirectory()
        self.addCleanup(decoy_tmp.cleanup)
        root = Path(decoy_tmp.name).resolve()
        self.decoy = init_repo(root / "decoy")
        (self.decoy / "real.txt").write_text("real\n")
        git(self.decoy, "add", "real.txt")
        git(
            self.decoy,
            "-c",
            "user.name=t",
            "-c",
            "user.email=t@example.invalid",
            "commit",
            "-q",
            "-m",
            "real",
        )
        self.decoy_wt = root / "decoy-wt"
        git(self.decoy, "worktree", "add", "-q", "--detach", str(self.decoy_wt))
        self.decoy_before = self.decoy_state()

        wt_git = self.decoy / ".git" / "worktrees" / self.decoy_wt.name
        patcher = mock.patch.dict(
            os.environ,
            {"GIT_DIR": str(wt_git), "GIT_INDEX_FILE": str(wt_git / "index")},
        )
        patcher.start()
        self.addCleanup(patcher.stop)
        super().setUp()

    def decoy_state(self) -> dict[str, str]:
        return {
            "core.bare": git(self.decoy, "config", "--get", "core.bare").strip(),
            "HEAD": git(self.decoy_wt, "rev-parse", "HEAD").strip(),
            "status": git(self.decoy_wt, "status", "--porcelain"),
        }

    def tearDown(self) -> None:
        self.assertEqual(self.decoy_state(), self.decoy_before)


if __name__ == "__main__":
    unittest.main()
