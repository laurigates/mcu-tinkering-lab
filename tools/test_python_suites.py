"""Tests for tools/python-suites.py and the CI setup it belongs to.

Stdlib unittest only, so CI can run it with nothing but python3 and git:

    python3 -m unittest tools/test_python_suites.py

`.github/python-suites.json` lists the uv projects whose test suites the
`Python Suites` job in `.github/workflows/test.yml` runs, and the paths that
select each one. `tools/python-suites.py` turns the changed-file list into
that job's matrix. What these tests hold:

- Selection: `**` crosses path segments, `*` stays inside one, a change to the
  CI definition itself selects every suite, and an unrelated change selects
  none.
- The manifest: its fields, a tracked uv.lock in each `dir`, globs that each
  match a tracked file, and `just <module>::<recipe>` commands that name a
  module and recipe that exist.
- Lock currency (#457): every git-tracked uv.lock is installed with
  `uv sync --locked` by some workflow, or is listed as not run in CI and has
  no tests. No workflow installs with `--frozen`, a plain `uv sync`, or
  `uv pip install`.
- One install path for uv and just (#835): only
  `.github/actions/setup-tools/action.yml` names `astral-sh/setup-uv` or
  `taiki-e/install-action`, nothing names `extractions/setup-just` (it asks
  the Releases API at run time), the action pins both versions literally, and
  the Renovate regex that bumps just still matches the pin.
- Tool suites (#838): every `test` recipe in a `tools/*/justfile` runs in CI,
  through the suites job, a workflow that runs its command and lists its
  directory in its trigger paths, or a pre-commit hook (which `test.yml`'s
  Code Quality job runs on any `tools/**` change).
- Component host suites (#1077): every `components/<name>/test/` directory
  under packages/ has a Makefile that `just components::test` finds, and a
  change anywhere in that component, or in the shared stubs, selects the
  `components` suite. A suite with no uv project (no `dir`) skips the uv
  install step.
"""

from __future__ import annotations

import importlib.util
import json
import os
import re
import subprocess
import tempfile
import unittest
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent
MANIFEST = REPO_ROOT / ".github" / "python-suites.json"
WORKFLOWS = REPO_ROOT / ".github" / "workflows"
SETUP_ACTION = REPO_ROOT / ".github" / "actions" / "setup-tools" / "action.yml"
TEST_WORKFLOW = WORKFLOWS / "test.yml"
PRE_COMMIT = REPO_ROOT / ".pre-commit-config.yaml"
RENOVATE = REPO_ROOT / "renovate.json"

# uv projects that a workflow other than the suites job installs. Each named
# workflow must run `uv sync --locked` with this working directory.
OTHER_CONSUMERS: dict[str, tuple[str, ...]] = {
    "docs/schematics": ("schematics-check.yml",),
    "packages/audio/melody-detector/training": ("test-melody-train.yml",),
    "packages/robotics/robocar/simulation": ("test.yml", "build-python-simulation.yml"),
}
# uv projects no workflow installs. Each must have no tests: once one gains a
# test, it needs a suite.
NOT_IN_CI: dict[str, str] = {
    "tools/switch-controller-usb-test": "a Raspberry Pi USB-gadget probe with no tests",
}

INSTALLERS_ONLY_IN_SETUP_ACTION = ("astral-sh/setup-uv", "taiki-e/install-action")
FORBIDDEN_ACTIONS = ("extractions/setup-just",)


def load_script():
    spec = importlib.util.spec_from_file_location(
        "python_suites", REPO_ROOT / "tools" / "python-suites.py"
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def tracked(*pathspecs: str) -> list[str]:
    out = subprocess.run(
        ["git", "ls-files", "--", *pathspecs],
        cwd=REPO_ROOT,
        check=True,
        capture_output=True,
        text=True,
    ).stdout
    return [line for line in out.splitlines() if line]


def ci_files() -> list[Path]:
    return sorted(WORKFLOWS.glob("*.yml")) + sorted(
        (REPO_ROOT / ".github" / "actions").glob("*/action.yml")
    )


def code_lines(path: Path) -> list[tuple[int, str]]:
    """Non-comment lines, with their 1-based numbers."""
    lines = []
    for n, line in enumerate(path.read_text().splitlines(), 1):
        if line.lstrip().startswith("#"):
            continue
        lines.append((n, line))
    return lines


def just_modules() -> dict[str, str]:
    """Root justfile `mod name 'path'` lines, as name -> path."""
    text = (REPO_ROOT / "justfile").read_text()
    return dict(re.findall(r"^mod\s+([\w-]+)\s+'([^']+)'", text, re.MULTILINE))


def python_suites_hook_files() -> re.Pattern:
    """The `files:` regex of the pre-commit hook that runs this suite."""
    text = PRE_COMMIT.read_text()
    block = re.search(r"- id: python-suites\n((?:        .*\n)+)", text)
    return re.compile(re.search(r"files: '([^']*)'", block.group(1)).group(1))


def recipe_body(justfile: Path, recipe: str) -> list[str] | None:
    """The command lines of `recipe`, or None if the justfile has no such recipe."""
    lines = justfile.read_text().splitlines()
    head = re.compile(rf"^{re.escape(recipe)}(\s[^:]*)?:(?!=)")
    for i, line in enumerate(lines):
        if head.match(line):
            body = []
            for cont in lines[i + 1 :]:
                if not cont.startswith((" ", "\t")):
                    break
                body.append(cont.strip())
            return [b for b in body if b]
    return None


class Selection(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.mod = load_script()
        cls.suites = cls.mod.load_manifest(MANIFEST)
        cls.names = [s["name"] for s in cls.suites]

    def selected(self, changed):
        return [s["name"] for s in self.mod.select(self.suites, changed)]

    def test_all_selects_every_suite(self):
        self.assertEqual(self.selected(None), self.names)

    def test_file_in_one_suite_selects_only_that_suite(self):
        self.assertEqual(
            self.selected(["tools/enclosure/enclosure/case.py"]), ["enclosure"]
        )

    def test_shared_input_selects_every_suite_that_reads_it(self):
        self.assertEqual(
            self.selected(["tools/cadstd/fits.py"]),
            ["enclosure", "mechanism", "cadstd"],
        )

    def test_double_star_crosses_segments(self):
        changed = ["packages/robotics/balancebot/hardware/enclosure/enclosure.toml"]
        self.assertEqual(self.selected(changed), ["enclosure"])

    def test_star_stays_in_one_segment(self):
        regex = self.mod.glob_regex("tools/*.py")
        self.assertTrue(regex.fullmatch("tools/python-suites.py"))
        self.assertFalse(regex.fullmatch("tools/kicad/sexpr.py"))

    def test_unrelated_change_selects_nothing(self):
        self.assertEqual(self.selected(["docs/decisions/ADR-001.md", "README.md"]), [])

    def test_ci_definition_change_selects_every_suite(self):
        for path in (
            ".github/python-suites.json",
            ".github/actions/setup-tools/action.yml",
            ".github/workflows/test.yml",
            "tools/python-suites.py",
        ):
            with self.subTest(path=path):
                self.assertEqual(self.selected([path]), self.names)

    def test_matrix_shape(self):
        mx = self.mod.matrix(
            self.mod.select(self.suites, ["tools/glitchtip/bridge/x.py"])
        )
        self.assertEqual(
            mx,
            {
                "include": [
                    {
                        "name": "glitchtip-bridge",
                        "dir": "tools/glitchtip/bridge",
                        "run": "just glitchtip::test",
                        "apt": "",
                    }
                ]
            },
        )

    def test_cli_writes_github_output(self):
        with tempfile.TemporaryDirectory() as tmp:
            out = Path(tmp) / "out"
            out.write_text("")
            rc = self.mod.main(
                ["--changed-json", json.dumps(["tools/mechanism/mechanism/check.py"])],
                github_output=out,
            )
            self.assertEqual(rc, 0)
            lines = dict(line.split("=", 1) for line in out.read_text().splitlines())
            self.assertEqual(lines["any"], "true")
            names = [e["name"] for e in json.loads(lines["matrix"])["include"]]
            self.assertEqual(names, ["mechanism"])

    def test_workflow_step_selects_from_a_large_file_list(self):
        """Run test.yml's own Select step, with dorny's output substituted in.

        3000 paths is past the 128 KiB a single env value can carry, which is
        why the step takes the list through a heredoc. One name holds a quote,
        a $() and an escaped newline, which must reach the script as data.
        """
        text = TEST_WORKFLOW.read_text()
        m = re.search(
            r"- name: Select Python suites\n(?:.*\n)*?\s+run: \|\n((?: {10}.*\n|\n)+)",
            text,
        )
        self.assertIsNotNone(m, "no Select Python suites step with a run: block")
        script = "\n".join(line[10:] for line in m.group(1).splitlines())
        files = [f"docs/bulk/{n:05d}-{'x' * 40}.md" for n in range(3000)]
        files += ['tools/glitchtip/bridge/a"$(touch pwned)\nEOF.py']
        payload = json.dumps(files)
        self.assertGreater(len(payload), 128 * 1024)
        self.assertNotIn("\n", payload)
        script = script.replace("${{ github.event_name }}", "pull_request")
        script = script.replace(
            "${{ steps.filter.outputs.all_files || '[]' }}", payload
        )
        self.assertNotIn("${{", script)
        with tempfile.TemporaryDirectory() as tmp:
            out = Path(tmp) / "out"
            out.write_text("")
            # A file, not `bash -c`: one argv string over 128 KiB (Linux
            # MAX_ARG_STRLEN) fails exec with E2BIG before bash starts.
            step = Path(tmp) / "step.sh"
            step.write_text(script)
            subprocess.run(
                ["bash", "-e", str(step)],
                cwd=REPO_ROOT,
                check=True,
                env={**os.environ, "GITHUB_OUTPUT": str(out)},
                capture_output=True,
            )
            self.assertFalse((REPO_ROOT / "pwned").exists())
            lines = dict(line.split("=", 1) for line in out.read_text().splitlines())
            names = [e["name"] for e in json.loads(lines["matrix"])["include"]]
            self.assertEqual(names, ["glitchtip-bridge"])

    def test_cli_reports_no_suites(self):
        with tempfile.TemporaryDirectory() as tmp:
            out = Path(tmp) / "out"
            out.write_text("")
            self.mod.main(["--changed-json", "[]"], github_output=out)
            lines = dict(line.split("=", 1) for line in out.read_text().splitlines())
            self.assertEqual(lines["any"], "false")
            self.assertEqual(json.loads(lines["matrix"]), {"include": []})


class Manifest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.mod = load_script()
        cls.suites = cls.mod.load_manifest(MANIFEST)

    def write(self, data) -> Path:
        with tempfile.NamedTemporaryFile("w", suffix=".json", delete=False) as tmp:
            json.dump(data, tmp)
        self.addCleanup(Path(tmp.name).unlink)
        return Path(tmp.name)

    def test_schema_rejects_bad_entries(self):
        good = {"name": "a", "dir": "d", "run": "true", "paths": ["d/**"]}
        for bad in (
            [{**good, "extra": 1}],
            [{k: v for k, v in good.items() if k != "paths"}],
            [good, good],
            [{**good, "paths": []}],
            [{**good, "apt": ["libx"]}],
        ):
            with self.subTest(bad=bad), self.assertRaises(ValueError):
                self.mod.load_manifest(self.write(bad))

    def test_dir_is_optional(self):
        """A suite with no uv project (a make-only C suite) omits `dir`."""
        suite = {"name": "c", "run": "true", "paths": ["d/**"]}
        loaded = self.mod.load_manifest(self.write([suite]))
        self.assertEqual(
            self.mod.matrix(loaded),
            {"include": [{"name": "c", "dir": "", "run": "true", "apt": ""}]},
        )

    def test_suites_job_skips_uv_install_without_a_dir(self):
        text = TEST_WORKFLOW.read_text()
        step = re.search(
            r"- name: Install \$\{\{ matrix\.dir \}\} from its uv\.lock\n((?:\s{8}\S.*\n)+)",
            text,
        )
        self.assertIsNotNone(step, "no uv install step in the suites job")
        self.assertIn("if: matrix.dir != ''", step.group(1))

    def test_each_dir_has_a_tracked_lock(self):
        locks = set(tracked("*uv.lock"))
        for s in self.suites:
            if "dir" not in s:
                continue
            with self.subTest(suite=s["name"]):
                self.assertIn(f"{s['dir']}/uv.lock", locks)

    def test_each_suite_is_selected_by_its_own_lock(self):
        for s in self.suites:
            if "dir" not in s:
                continue
            with self.subTest(suite=s["name"]):
                picked = self.mod.select([s], [f"{s['dir']}/uv.lock"])
                self.assertEqual(picked, [s], "a lock change would not run this suite")

    def test_every_glob_matches_a_tracked_file(self):
        files = tracked()
        for s in self.suites:
            for glob in s["paths"]:
                with self.subTest(suite=s["name"], glob=glob):
                    regex = self.mod.glob_regex(glob)
                    self.assertTrue(
                        any(regex.fullmatch(f) for f in files),
                        "matches no tracked path, so it selects nothing",
                    )

    def test_just_commands_name_real_recipes(self):
        modules = just_modules()
        for s in self.suites:
            m = re.fullmatch(r"just ([\w-]+)::([\w-]+)", s["run"])
            if not m:
                continue
            module, recipe = m.groups()
            with self.subTest(suite=s["name"]):
                self.assertIn(
                    module, modules, f"no `mod {module}` in the root justfile"
                )
                body = recipe_body(REPO_ROOT / modules[module] / "justfile", recipe)
                self.assertIsNotNone(
                    body, f"{modules[module]}/justfile has no {recipe} recipe"
                )


class LockCurrency(unittest.TestCase):
    def test_every_tracked_lock_is_installed_locked_or_has_no_tests(self):
        suite_dirs = {
            s["dir"] for s in load_script().load_manifest(MANIFEST) if "dir" in s
        }
        for lock in tracked("*uv.lock"):
            d = str(Path(lock).parent)
            with self.subTest(project=d):
                if d in suite_dirs:
                    continue
                if d in OTHER_CONSUMERS:
                    for wf in OTHER_CONSUMERS[d]:
                        text = (WORKFLOWS / wf).read_text()
                        self.assertIn(f"working-directory: {d}", text, wf)
                        self.assertIn("uv sync --locked", text, wf)
                    continue
                if d in NOT_IN_CI:
                    tests = [
                        f
                        for f in tracked(d)
                        if re.search(
                            r"(^|/)(tests?/|test_[^/]*\.py$|[^/]*_test\.py$)", f
                        )
                    ]
                    self.assertEqual(
                        tests, [], f"{d} has tests now; add it to {MANIFEST.name}"
                    )
                    continue
                self.fail(
                    f"{lock}: no workflow installs it. Add a suite to {MANIFEST.name}, "
                    "a consumer to OTHER_CONSUMERS, or a reason to NOT_IN_CI."
                )

    def test_suites_job_asserts_each_lock(self):
        text = TEST_WORKFLOW.read_text()
        self.assertIn("working-directory: ${{ matrix.dir }}", text)
        self.assertIn("UV_LOCKED: '1'", text)

    def test_uv_sync_is_always_locked(self):
        for path in ci_files():
            for n, line in code_lines(path):
                if re.search(r"\buv sync\b", line):
                    with self.subTest(file=path.name, line=n):
                        self.assertIn("--locked", line)

    def test_no_unlocked_installs(self):
        for path in ci_files():
            for n, line in code_lines(path):
                for bad in ("--frozen", "uv pip install", "pip install uv"):
                    with self.subTest(file=path.name, line=n, bad=bad):
                        self.assertNotIn(bad, line)


class SetupAction(unittest.TestCase):
    def test_installers_only_in_setup_action(self):
        for path in ci_files():
            if path == SETUP_ACTION:
                continue
            for n, line in code_lines(path):
                for action in INSTALLERS_ONLY_IN_SETUP_ACTION + FORBIDDEN_ACTIONS:
                    with self.subTest(file=path.name, line=n, action=action):
                        self.assertNotIn(action, line)

    def test_setup_action_pins_versions(self):
        text = SETUP_ACTION.read_text()
        code = "\n".join(line for _, line in code_lines(SETUP_ACTION))
        self.assertNotIn("extractions/setup-just", code)
        self.assertRegex(
            text,
            r"uses: astral-sh/setup-uv@v\d+\.\d+\.\d+\n\s+with:\n\s+version: '\d+\.\d+\.\d+'",
        )
        self.assertRegex(text, r"uses: taiki-e/install-action@v\d+\.\d+\.\d+")
        self.assertRegex(text, r"tool: just@\d+\.\d+\.\d+\n")

    def test_renovate_regex_matches_the_just_pin(self):
        managers = json.loads(RENOVATE.read_text()).get("customManagers", [])
        mine = [
            m
            for m in managers
            if any(
                re.fullmatch(p.strip("/"), ".github/actions/setup-tools/action.yml")
                for p in m.get("managerFilePatterns", [])
            )
        ]
        self.assertEqual(
            len(mine), 1, "one Renovate custom manager reads the setup action"
        )
        mgr = mine[0]
        self.assertEqual(mgr.get("packageNameTemplate"), "casey/just")
        self.assertEqual(mgr.get("datasourceTemplate"), "github-releases")
        text = SETUP_ACTION.read_text()
        # Renovate's named groups are JavaScript's (?<name>); Python's (?P<name>.
        py = [p.replace("(?<", "(?P<") for p in mgr["matchStrings"]]
        hits = [m for p in py for m in re.finditer(p, text)]
        self.assertEqual(len(hits), 1)
        pinned = re.search(r"tool: just@(\S+)", text).group(1)
        self.assertEqual(hits[0].group("currentValue"), pinned)

    def test_workflows_running_just_install_it(self):
        for path in sorted(WORKFLOWS.glob("*.yml")):
            text = path.read_text()
            runs_just = re.search(r"^\s+(run: )?just [\w-]+::", text, re.MULTILINE)
            if path == TEST_WORKFLOW:
                runs_just = True  # the suites job runs `just ...` from the manifest
            if runs_just:
                with self.subTest(file=path.name):
                    self.assertRegex(
                        text,
                        r"uses: \./\.github/actions/setup-tools\n"
                        r"\s+with:\n(\s+\S.*\n)*?\s+just: true",
                    )


class ToolSuiteCoverage(unittest.TestCase):
    def test_every_tool_test_recipe_runs_in_ci(self):
        suite_runs = {s["run"] for s in load_script().load_manifest(MANIFEST)}
        workflows = {p.name: p.read_text() for p in WORKFLOWS.glob("*.yml")}
        pre_commit = PRE_COMMIT.read_text()
        for module, path in sorted(just_modules().items()):
            if not path.startswith("tools/"):
                continue
            body = recipe_body(REPO_ROOT / path / "justfile", "test")
            if body is None:
                continue
            with self.subTest(module=module):
                if f"just {module}::test" in suite_runs:
                    continue
                commands = [
                    re.sub(r"^cd (\.\./\.\.|\{\{_root\}\}) && ", "", b) for b in body
                ]
                in_hook = all(c in pre_commit for c in commands)
                in_workflow = [
                    name
                    for name, text in workflows.items()
                    if all(c in text for c in commands) and f"'{path}/**'" in text
                ]
                self.assertTrue(
                    in_hook or in_workflow,
                    f"`just {module}::test` runs in no workflow: add a suite to {MANIFEST.name}",
                )

    def test_code_quality_runs_hooks_on_tools_changes(self):
        text = TEST_WORKFLOW.read_text()
        any_code = re.search(r"any-code:\n((?:\s+- .*\n)+)", text)
        self.assertIsNotNone(any_code)
        self.assertIn("'tools/**'", any_code.group(1))


# A component's host suite: packages/<...>/components/<name>/test/, run by
# `just components::test` through the `components` suite (#1077).
COMPONENT_TEST_DIR = re.compile(r"^(packages/(?:.+/)?components/[^/]+)/(tests?)/")
COMPONENTS_SUITE = "components"
COMPONENTS_RUN = "just components::test"
COMPONENTS_JUSTFILE = REPO_ROOT / "packages" / "components" / "justfile"
SHARED_STUBS = "packages/components/host-test"


class ComponentHostSuites(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.mod = load_script()
        cls.suites = cls.mod.load_manifest(MANIFEST)
        cls.files = tracked("packages")
        cls.test_dirs = sorted(
            {
                f"{m.group(1)}/{m.group(2)}"
                for f in cls.files
                if (m := COMPONENT_TEST_DIR.match(f))
            }
        )

    def suite(self) -> dict:
        found = [s for s in self.suites if s["name"] == COMPONENTS_SUITE]
        self.assertEqual(
            len(found), 1, f"no {COMPONENTS_SUITE!r} suite in {MANIFEST.name}"
        )
        return found[0]

    def test_the_pattern_finds_the_known_suites(self):
        # Control: a pattern that matched nothing would pass every test below.
        self.assertIn("packages/components/thinkpack-protocol/test", self.test_dirs)
        self.assertIn(
            "packages/input-gaming/xbox-switch-bridge/components/status_led/test",
            self.test_dirs,
        )
        self.assertNotIn(
            "packages/robotics/robocar/test",
            self.test_dirs,
            "a project, not a component",
        )

    def test_suite_runs_the_recipe(self):
        self.assertEqual(self.suite()["run"], COMPONENTS_RUN)
        self.assertNotIn("dir", self.suite(), "a make-only suite has no uv project")

    def test_every_component_test_dir_is_run_by_the_recipe(self):
        """The recipe's own pathspec lists one Makefile per component test dir.

        A `tests/` directory, or a `test/` with no Makefile, fails here: the
        recipe runs `make` in each directory its pathspec lists, and nothing
        else.
        """
        body = recipe_body(COMPONENTS_JUSTFILE, "test")
        self.assertIsNotNone(body, f"{COMPONENTS_JUSTFILE} has no test recipe")
        specs = re.findall(r"'(:\(glob\)[^']+)'", "\n".join(body))
        self.assertEqual(len(specs), 1, "want one ':(glob)...' pathspec in the recipe")
        found = sorted(str(Path(f).parent) for f in tracked(specs[0]))
        self.assertEqual(
            found,
            self.test_dirs,
            "every packages/**/components/<name>/test/ needs a Makefile, and no "
            "component may keep its host suite anywhere else",
        )

    def test_a_change_to_a_tested_component_selects_the_suite(self):
        suite = self.suite()
        components = {str(Path(d).parent) for d in self.test_dirs}
        watched = [
            f
            for f in self.files
            if any(f.startswith(c + "/") for c in components | {SHARED_STUBS})
        ]
        self.assertTrue(any(f.startswith(SHARED_STUBS + "/") for f in watched))
        for f in watched:
            with self.subTest(file=f):
                self.assertEqual(
                    self.mod.select([suite], [f]),
                    [suite],
                    f"add the component to the {COMPONENTS_SUITE!r} suite's paths",
                )

    def test_the_hook_runs_on_a_component_suite_change(self):
        # The python-suites hook's files: must take these, or a suite the
        # recipe would not run fails only in CI (#1103).
        files = python_suites_hook_files()
        inputs = [
            f
            for f in self.files
            if COMPONENT_TEST_DIR.match(f) or f.startswith(SHARED_STUBS + "/")
        ]
        self.assertTrue(inputs)
        missed = [f for f in inputs if not files.search(f)]
        self.assertEqual(missed, [], "extend the python-suites hook's files:")

    def test_no_stub_is_copied_between_suites(self):
        by_content: dict[bytes, list[str]] = {}
        for f in self.files:
            in_suite = COMPONENT_TEST_DIR.match(f) or f.startswith(SHARED_STUBS + "/")
            if in_suite and f.endswith(".h"):
                by_content.setdefault((REPO_ROOT / f).read_bytes(), []).append(f)
        copies = [sorted(fs) for fs in by_content.values() if len(fs) > 1]
        self.assertEqual(copies, [], f"move shared test headers to {SHARED_STUBS}/")


if __name__ == "__main__":
    unittest.main()
