"""Tests for the check-credentials pre-commit hook (.pre-commit-config.yaml).

The hook refuses a commit that stages a credential file: credentials.h,
wifi_config.h, or a name ending in .key, .secret or .token. Issue #1145: its
first two names were unanchored, so the tracked template
main/credentials.h.example was refused too.

Each case stages files in a throwaway repository and runs the hook's entry,
read from .pre-commit-config.yaml and split the way pre-commit splits it, so
the test exercises the shipped command rather than a copy of its pattern.

Run: python3 -m unittest tools/test_check_credentials.py
"""

from __future__ import annotations

import os
import shlex
import subprocess
import tempfile
import unittest
from pathlib import Path

try:
    from tools import gitenv
except ImportError:  # run as a script: python3 tools/test_check_credentials.py
    import gitenv

setUpModule = gitenv.isolate

REPO_ROOT = Path(__file__).resolve().parent.parent
PRE_COMMIT = REPO_ROOT / ".pre-commit-config.yaml"
HOOK_ID = "check-credentials"
REFUSED = "ERROR - Attempted to commit credential files!"


def hook_entry(config: str, hook_id: str) -> str:
    """The `entry:` of ``hook_id``, a plain or folded (``>-``) scalar.

    Read as text: the system python3 that runs this hook need not have
    PyYAML. A folded scalar's lines are joined with spaces, which is what
    YAML does for lines at one indentation with no blank line between them.
    """
    lines = config.splitlines()
    start = lines.index(f"      - id: {hook_id}")
    for i in range(start + 1, len(lines)):
        line = lines[i]
        if line.lstrip().startswith("- id: "):
            break
        stripped = line.strip()
        if not stripped.startswith("entry:"):
            continue
        value = stripped[len("entry:") :].strip()
        if value not in (">-", ">", "|", "|-"):
            return value
        indent = len(line) - len(line.lstrip())
        body = []
        for cont in lines[i + 1 :]:
            if cont.strip() and len(cont) - len(cont.lstrip()) <= indent:
                break
            body.append(cont.strip())
        assert value.startswith(">"), f"{hook_id}: literal block entry not handled"
        return " ".join(b for b in body if b)
    raise AssertionError(f"{hook_id} has no entry in {PRE_COMMIT}")


ENTRY = hook_entry(PRE_COMMIT.read_text(), HOOK_ID)


def git(cwd: Path, *args: str) -> None:
    subprocess.run(
        ["git", "-C", str(cwd), *args],
        check=True,
        capture_output=True,
        env={**os.environ, "GIT_CONFIG_GLOBAL": os.devnull, "GIT_CONFIG_NOSYSTEM": "1"},
    )


class CheckCredentialsHookTest(unittest.TestCase):
    def run_hook(self, *staged: str) -> subprocess.CompletedProcess[str]:
        with tempfile.TemporaryDirectory() as tmp:
            repo = Path(tmp)
            git(repo, "init", "-q")
            for name in staged:
                path = repo / name
                path.parent.mkdir(parents=True, exist_ok=True)
                path.write_text("x\n")
            git(repo, "add", "--", *staged)
            return subprocess.run(
                shlex.split(ENTRY),
                cwd=repo,
                capture_output=True,
                text=True,
                timeout=60,
            )

    def assert_refused(self, name: str) -> None:
        result = self.run_hook(name)
        self.assertEqual(result.returncode, 1, (name, result.stdout, result.stderr))
        self.assertIn(REFUSED, result.stdout)

    def assert_passes(self, name: str) -> None:
        result = self.run_hook(name)
        self.assertEqual(result.returncode, 0, (name, result.stdout, result.stderr))
        self.assertNotIn(REFUSED, result.stdout)

    def test_entry_is_the_staged_name_check(self) -> None:
        self.assertTrue(ENTRY.startswith("bash -c "), ENTRY)
        self.assertIn("git diff --cached --name-only", ENTRY)

    def test_credential_headers_are_refused(self) -> None:
        for name in ("credentials.h", "main/credentials.h", "wifi_config.h"):
            with self.subTest(name=name):
                self.assert_refused(name)

    def test_templates_pass(self) -> None:
        for name in ("credentials.h.example", "main/credentials.h.example"):
            with self.subTest(name=name):
                self.assert_passes(name)

    def test_key_secret_token_suffixes_are_refused(self) -> None:
        for name in ("server.key", "certs/a.secret", "bot.token"):
            with self.subTest(name=name):
                self.assert_refused(name)

    def test_names_merely_containing_a_suffix_pass(self) -> None:
        for name in ("server.key.example", "notes.token.md", "main/main.c"):
            with self.subTest(name=name):
                self.assert_passes(name)

    def test_one_credential_among_others_is_refused(self) -> None:
        result = self.run_hook(
            "main/main.c", "main/credentials.h.example", "main/credentials.h"
        )
        self.assertEqual(result.returncode, 1, result.stdout)


if __name__ == "__main__":
    unittest.main()
