"""Tests for refresh-idf-locks.py, run against the shipped script.

Stdlib unittest only, so the pre-commit hook runs it on a runner with nothing
but python3:

    python3 -m unittest tools/test_refresh_idf_locks.py

The script is executed as a subprocess, never imported and re-implemented. A
stub `idf.py` is put first on PATH; it records how it was called and mimics the
real `update-dependencies` (delete the lock, reconfigure, write a new one), or
fails, depending on STUB_MODE.
"""

from __future__ import annotations

import json
import os
import shutil
import stat
import subprocess
import sys
import tempfile
import textwrap
import unittest
from pathlib import Path

SCRIPT = Path(__file__).with_name("refresh-idf-locks.py")

REGISTRY_LOCK = textwrap.dedent(
    """\
    dependencies:
      espressif/mdns:
        component_hash: aaaa
        source:
          registry_url: https://components.espressif.com/
          type: service
        version: 1.14.0
      idf:
        source:
          type: idf
        version: 5.4.0
    direct_dependencies:
    - espressif/mdns
    - idf
    manifest_hash: 1111
    target: {target}
    version: 2.0.0
    """
)

IDF_ONLY_LOCK = textwrap.dedent(
    """\
    dependencies:
      idf:
        source:
          type: idf
        version: 5.4.0
    direct_dependencies:
    - idf
    manifest_hash: 2222
    target: esp32s3
    version: 2.0.0
    """
)

STUB = textwrap.dedent(
    """\
    #!/usr/bin/env python3
    import json, os, sys
    from pathlib import Path

    args = sys.argv[1:]
    project = Path(args[args.index("-C") + 1])
    with open(os.environ["STUB_LOG"], "a") as log:
        log.write(json.dumps({"argv": args,
                              "env_target": os.environ.get("IDF_TARGET")}) + "\\n")
    mode = os.environ.get("STUB_MODE", "write")
    lock = project / "dependencies.lock"
    lock.unlink(missing_ok=True)  # update-dependencies deletes the lock first
    if mode == "fail":
        sys.exit(2)
    if mode == "write":
        lock.write_text("regenerated\\n")
    """
)


class RefreshIdfLocksTest(unittest.TestCase):
    def setUp(self) -> None:
        self.tmp = Path(tempfile.mkdtemp())
        self.addCleanup(shutil.rmtree, self.tmp)
        bindir = self.tmp / "bin"
        bindir.mkdir()
        stub = bindir / "idf.py"
        stub.write_text(STUB)
        stub.chmod(stub.stat().st_mode | stat.S_IXUSR)
        self.log = self.tmp / "calls.jsonl"
        self.env = dict(os.environ)
        self.env["PATH"] = f"{bindir}{os.pathsep}{self.env['PATH']}"
        self.env["STUB_LOG"] = str(self.log)
        self.env.pop("IDF_TARGET", None)
        # A non-executable stub is skipped by PATH lookup and the real idf.py
        # would run instead; prove the stub is the one that resolves.
        self.assertEqual(shutil.which("idf.py", path=self.env["PATH"]), str(stub))

    def project(self, name: str, lock: str) -> Path:
        path = self.tmp / name
        path.mkdir()
        (path / "dependencies.lock").write_text(lock)
        return path

    def run_script(self, *args: str, mode: str = "write", cwd: Path | None = None):
        env = dict(self.env, STUB_MODE=mode)
        return subprocess.run(
            [sys.executable, str(SCRIPT), *args],
            env=env,
            cwd=cwd or self.tmp,
            capture_output=True,
            text=True,
        )

    def calls(self) -> list[dict]:
        if not self.log.exists():
            return []
        return [json.loads(line) for line in self.log.read_text().splitlines()]

    def test_regenerates_with_the_target_the_lock_records(self) -> None:
        proj = self.project("cam", REGISTRY_LOCK.format(target="esp32s3"))
        result = self.run_script(str(proj))
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        self.assertEqual((proj / "dependencies.lock").read_text(), "regenerated\n")
        (call,) = self.calls()
        self.assertIn("update-dependencies", call["argv"])
        self.assertIn("-DIDF_TARGET=esp32s3", call["argv"])
        # idf.py refuses a -D target that disagrees with IDF_TARGET in the
        # environment (esp-idf-ci-action exports one), so both must agree.
        self.assertEqual(call["env_target"], "esp32s3")
        self.assertIn("changed", result.stdout)

    def test_leaves_no_build_dir_or_sdkconfig_in_the_project(self) -> None:
        proj = self.project("cam", REGISTRY_LOCK.format(target="esp32"))
        result = self.run_script(str(proj))
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        (call,) = self.calls()
        argv = call["argv"]
        build_dir = Path(argv[argv.index("-B") + 1])
        self.assertFalse(build_dir.is_relative_to(proj), build_dir)
        sdkconfig = [a for a in argv if a.startswith("-DSDKCONFIG=")]
        self.assertEqual(len(sdkconfig), 1, argv)
        self.assertFalse(
            Path(sdkconfig[0].split("=", 1)[1]).is_relative_to(proj), sdkconfig
        )

    def test_skips_a_lock_with_no_registry_dependency(self) -> None:
        proj = self.project("synth", IDF_ONLY_LOCK)
        result = self.run_script(str(proj))
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        self.assertEqual(self.calls(), [])
        self.assertEqual((proj / "dependencies.lock").read_text(), IDF_ONLY_LOCK)
        self.assertIn("skip", result.stdout)

    def test_failure_restores_the_lock_and_still_refreshes_the_rest(self) -> None:
        original = REGISTRY_LOCK.format(target="esp32")
        bad = self.project("bad", original)
        result = self.run_script(str(bad), mode="fail")
        self.assertNotEqual(result.returncode, 0)
        self.assertEqual((bad / "dependencies.lock").read_text(), original)
        self.assertIn(str(bad), result.stderr)

    def test_one_failure_does_not_stop_the_next_project(self) -> None:
        a = self.project("a", REGISTRY_LOCK.format(target="esp32"))
        b = self.project("b", REGISTRY_LOCK.format(target="esp32s3"))
        # Only the first call fails: switch the stub to "write" after it.
        failing_once = STUB.replace(
            'mode = os.environ.get("STUB_MODE", "write")',
            'mode = "fail" if project.name == "a" else "write"',
        )
        stub = Path(shutil.which("idf.py", path=self.env["PATH"]))
        stub.write_text(failing_once)
        result = self.run_script(str(a), str(b))
        self.assertNotEqual(result.returncode, 0)
        self.assertEqual(len(self.calls()), 2)
        self.assertEqual((b / "dependencies.lock").read_text(), "regenerated\n")
        self.assertEqual(
            (a / "dependencies.lock").read_text(), REGISTRY_LOCK.format(target="esp32")
        )

    def test_success_without_a_new_lock_is_a_failure(self) -> None:
        original = REGISTRY_LOCK.format(target="esp32")
        proj = self.project("cam", original)
        result = self.run_script(str(proj), mode="nolock")
        self.assertNotEqual(result.returncode, 0)
        self.assertEqual((proj / "dependencies.lock").read_text(), original)

    def test_lock_without_a_target_is_a_failure(self) -> None:
        lock = REGISTRY_LOCK.format(target="esp32").replace("target: esp32\n", "")
        proj = self.project("cam", lock)
        result = self.run_script(str(proj))
        self.assertNotEqual(result.returncode, 0)
        self.assertEqual(self.calls(), [])

    def test_no_project_arguments_is_a_usage_error(self) -> None:
        result = self.run_script()
        self.assertNotEqual(result.returncode, 0)
        self.assertEqual(self.calls(), [])

    def test_list_prints_only_tracked_locks(self) -> None:
        repo = self.tmp / "repo"
        for rel in ("packages/a/tracked", "packages/b/ignored"):
            (repo / rel).mkdir(parents=True)
            (repo / rel / "dependencies.lock").write_text(IDF_ONLY_LOCK)
        (repo / ".gitignore").write_text("dependencies.lock\n")
        (repo / "packages/c").mkdir(parents=True)
        (repo / "packages/c/main.c").write_text("int main(void) { return 0; }\n")
        git = ["git", "-c", "user.email=t@t", "-c", "user.name=t"]
        subprocess.run([*git, "init", "-q"], cwd=repo, check=True)
        subprocess.run(
            [*git, "add", "-f", "packages/a/tracked/dependencies.lock"],
            cwd=repo,
            check=True,
        )
        # A tracked file that is not a lock must not be listed either.
        subprocess.run([*git, "add", ".gitignore", "packages/c"], cwd=repo, check=True)
        result = self.run_script("--list", cwd=repo)
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(result.stdout.split(), ["packages/a/tracked"])


if __name__ == "__main__":
    unittest.main()
