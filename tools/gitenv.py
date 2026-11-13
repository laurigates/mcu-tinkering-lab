"""Keep a test module's git calls out of the repository being committed.

During a `git commit` from a linked worktree, git exports GIT_DIR and an
absolute GIT_INDEX_FILE to the pre-commit hooks, and several test suites run
as hooks. Inherited, those variables send the suite's `git init`, `git add`
and `git commit` in a temporary directory to the repository being committed
(issue #1123): stray commits, staged files, core.bare set to true. The code
under test spawns git too, so the scrub covers the whole test process rather
than each subprocess call.

A test module that creates a throwaway repository sets, after its imports::

    setUpModule = gitenv.isolate

tools/test_git_env.py fails for one that does not.
"""

from __future__ import annotations

import os
import unittest
from unittest import mock


def clean_env() -> dict[str, str]:
    """The current environment without any GIT_* variable."""
    return {k: v for k, v in os.environ.items() if not k.startswith("GIT_")}


def isolate() -> None:
    """Remove every GIT_* variable from os.environ until the module finishes.

    Used as ``setUpModule``: the original environment comes back in the
    module cleanup, so the next module in the same run starts unchanged.
    Variables a test sets afterwards (GIT_AUTHOR_NAME, a decoy GIT_DIR) stay
    under the test's own control.
    """
    patcher = mock.patch.dict(os.environ, clean_env(), clear=True)
    patcher.start()
    unittest.addModuleCleanup(patcher.stop)
