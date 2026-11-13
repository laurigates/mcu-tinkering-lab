"""Structural tests for .github/workflows/*.yml.

Needs PyYAML, so the pre-commit hook runs it through uv:

    uv run --no-project --with pyyaml==6.0.3 python -m unittest tools/test_workflows.py

What the tests hold:

- No ``run:`` script, and no ``with.command`` (espressif/esp-idf-ci-action
  pastes it into its own script), expands ``inputs``, ``github.event`` or
  ``github.head_ref`` with ``${{ }}``, whether dotted (``inputs.x``), through
  a function (``format('{0}', inputs.x)``, ``toJSON(github.event)``) or by
  bracket (``github.event['pull_request']``). Actions substitutes the text
  into the script before the shell parses it, so a value carrying ``$(...)``
  or a quote runs as code in a job that may hold ``contents: write`` (issue
  #855). Pass such a value through ``env:`` and read it as ``"$VAR"``; for
  esp-idf-ci-action, also forward it into its container with
  ``extra_docker_args: -e NAME``, which the suite checks. A ``boolean`` or
  ``number`` input is exempt, since GitHub rejects any other value. Other contexts
  (``matrix``, ``needs``, ``steps``, ``env``, ``github.repository``) are not
  covered: their values come from this repo's files or from GitHub itself.
- build-firmware.yml checks the release tag before any job uses it: every
  other job needs ``resolve-tag``, directly or through another job. The check
  itself, run here, passes every tag in the clone and a robocar tag from each
  side of its rename, and refuses shell text, a pre-release suffix and a bare
  version.
- build-firmware.yml's Pages assembly step, run here in a scratch tree, copies
  ``firmware/robocar`` to ``_site/firmware/unified`` (the one OTA hop for
  robots on robocar-unified <= 0.2.23) and leaves ``firmware/`` as it was,
  which the release gates count and walk. A tree with ``firmware/unified``, or
  with neither directory (a replayed older tag), is assembled with no copy.
- build-firmware.yml builds Pico SDK projects by calling
  ``_ci-build-pico.yml``, the reusable workflow build.yml calls, at the
  release tag, and copies none of its steps (issue #1065). Its select step,
  run here against the repo's own files, picks balancebot, skips an ESP-IDF
  component, and selects nothing for a tag older than project-matrix.json.
- build.yml's discovery step, run here against the repo's own
  project-matrix.json, does not select the robocar firmware for a change
  inside one of the four projects nested in its directory (bringup,
  integrated, simulation, twin), still selects it and robocar-bringup for a
  firmware change, alone or in the same pull request as a nested-project
  change, and keeps ``extra_paths`` and ``exclude_paths`` out of the job
  matrix.
- auto-fix.yml passes the release-please App to the reusable auto-fix, so the
  fix commits it pushes start CI (issue #866).
- security-secrets.yml runs one secret scan, the gitleaks job, and no longer
  calls the reusable Claude scan (issue #860).
- Every workflow declares workflow-level ``permissions:``, except the
  reusable callers listed in ``INHERITS_REPO_DEFAULT`` (none now). Each
  laurigates/.github reusable caller is read-only at workflow level and its
  caller job grants exactly the scopes the reusable declares (issue #1103).
- A pre-commit hook that watches CI configuration (``.github/``,
  ``.claude/hooks/``, any root file but Markdown) runs on a PR
  that changes only that file: test.yml's ``any-code`` filter covers the file,
  or an always-on job in test.yml runs the hook (issue #1089). Every job but
  the advisory C/C++ lint reaches Test Summary's failure check.
- No gate reads git's output where a failing git reads as clean: inside
  ``[ ]`` or ``test``, through ``< <(...)``, or with ``git diff --quiet``,
  which does not see untracked files (issue #1091). build-guide-check.yml's
  drift step, run here with stub ``typst`` and ``git``, fails when
  ``git status`` or ``git ls-files`` fails.

Whether the workflows run as intended needs GitHub; actionlint and these tests
only read them.
"""

import json
import os
import re
import shutil
import subprocess
import tempfile
import unittest
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parent.parent
WORKFLOWS = ROOT / ".github" / "workflows"
PRE_COMMIT = ROOT / ".pre-commit-config.yaml"

# The body of each ${{ }} expression, up to the first }} (a format string's
# '{0}' holds a single brace, so it does not end the match).
EXPRESSION = re.compile(r"\$\{\{(.*?)\}\}", re.DOTALL)
# A reference to a context whose value a user or a PR author chooses. The
# lookbehinds keep `steps.x.outputs.inputs` and `foo_github` out; `\b` after
# `event` keeps `github.event_name` out.
UNTRUSTED_REFERENCE = re.compile(
    r"(?<![\w.])(inputs\.[\w-]+|inputs\s*\["
    r"|github\s*\.\s*event\b|github\s*\[\s*['\"]event['\"]"
    r"|github\s*\.\s*head_ref\b|github\s*\[\s*['\"]head_ref['\"])"
)
PICO_REUSABLE = "./.github/workflows/_ci-build-pico.yml"
ESP_IDF_ACTION = "espressif/esp-idf-ci-action@"
PICO_SELECT_STEP = "Match the tag's component to a Pico SDK project"
RESOLVE_TAG_STEP = "Match the release-please tag pattern"
PAGES_STEP = "Assemble Pages site"
DISCOVER_STEP = "Filter buildable projects by change scope"
MATRIX = ROOT / ".github" / "project-matrix.json"
ROBOCAR = "packages/robotics/robocar"
DRIFT_STEP_SUFFIX = "check for drift"

# Reusable-workflow callers with no workflow-level permissions: their one job
# runs a laurigates/.github reusable with the repo's default token, and a
# block here would cap the scopes that reusable declares (a startup_failure
# if it is narrower). Give the caller job the reusable's scopes, as
# auto-fix.yml does, then drop the file from this list.
INHERITS_REPO_DEFAULT: set[str] = set()

# CI configuration: files whose hooks must run on a PR that changes nothing
# else (issue #1089): .github/, the Claude Code hooks, and every root file but
# Markdown. Root files rather than a list of names: a list of four missed
# release-please-config.json, which the OTA-agreement hooks read (#1148), and
# ruff.toml. Docs (*.md, docs/) stay out on purpose: a docs-only PR should not
# pay for Code Quality Checks.
CI_CONFIG = re.compile(r"^(\.github/|\.claude/hooks/|[^/]+(?<!\.md)$)")
CONFIG_HOOKS_JOB = "config-hooks"
# Advisory: its result is reported but never fails the summary.
ADVISORY_JOBS = {"lint-c"}

# Gate shapes in which a failing git reads as clean or as unchanged. set -e
# does not fire for a command substitution inside [ ] or test, nor for a
# process substitution; `git diff --quiet` ignores untracked files.
HIDDEN_GIT_FAILURE = re.compile(
    r"\[\[?[^\]\n]*\$\(\s*git\b"
    r"|\btest\b[^\n;]*\$\(\s*git\b"
    r"|<\s*<\(\s*git\b"
    r"|\bgit\b[^\n|;]*\bdiff\b[^\n|;]*--(quiet|exit-code)\b"
)


def load(name: str) -> dict:
    return yaml.safe_load((WORKFLOWS / name).read_text())


def untrusted_references(script: str) -> list[str]:
    """Each untrusted context referenced inside a ``${{ }}`` in ``script``."""
    return [
        ref.group(1)
        for expression in EXPRESSION.finditer(script)
        for ref in UNTRUSTED_REFERENCE.finditer(expression.group(1))
    ]


def workflow_files() -> list[Path]:
    return sorted(WORKFLOWS.glob("*.yml"))


def steps_of(workflow: dict):
    for job_id, job in (workflow.get("jobs") or {}).items():
        for index, step in enumerate(job.get("steps") or []):
            yield job_id, index, step


def scripts_of(step: dict):
    """Each piece of a step that a shell parses: ``run:``, and ``with.command``,
    which espressif/esp-idf-ci-action pastes into its own script and runs
    through ``bash -c`` the same way."""
    for key, script in (
        ("run", step.get("run")),
        ("with.command", (step.get("with") or {}).get("command")),
    ):
        if isinstance(script, str):
            yield key, script


def typed_inputs(workflow: dict) -> set[str]:
    """``inputs.<name>`` for each boolean or number input: GitHub checks the
    value against the type, so it cannot carry shell text."""
    triggers = workflow.get(True) or workflow.get("on") or {}
    if not isinstance(triggers, dict):
        return set()
    names = set()
    for event in ("workflow_dispatch", "workflow_call"):
        for name, spec in ((triggers.get(event) or {}).get("inputs") or {}).items():
            if (spec or {}).get("type") in ("boolean", "number"):
                names.add(f"inputs.{name}")
    return names


def needs_of(job: dict) -> set[str]:
    needs = job.get("needs") or []
    return {needs} if isinstance(needs, str) else set(needs)


class UntrustedContextInRunTest(unittest.TestCase):
    def test_pattern_catches_the_shapes_it_targets(self):
        for text in (
            'TAG="${{ github.event.release.tag_name || inputs.release_tag }}"',
            'PR="${{ inputs.pr_number }}"',
            'B="${{ github.event.workflow_run.head_branch }}"',
            'echo "${{ github.head_ref }}"',
            "T=\"${{ format('{0}', inputs.release_tag) }}\"",
            "T=\"${{ format('{0}-{1}', matrix.x, github.event.release.name) }}\"",
            "echo '${{ toJSON(github.event) }}'",
            "T=\"${{ github.event['pull_request']['title'] }}\"",
            "P=\"${{ inputs['pr-number'] }}\"",
            "B=\"${{ github['head_ref'] }}\"",
            'B="${{ github . head_ref }}"',
        ):
            self.assertNotEqual(untrusted_references(text), [], text)
        for text in (
            'if [ "${{ github.event_name }}" = "push" ]',
            'repo="${{ github.repository }}"',
            'cd "${{ matrix.project_path }}"',
            'TAG="$RELEASE_TAG"',
            'H="${{ steps.hash.outputs.inputs_digest }}"',
            "N=\"${{ format('{0}', github.run_id) }}\"",
            # Outside ${{ }}, the text is the shell's, not an expansion.
            'echo "github.event.release.tag_name and inputs.x"',
        ):
            self.assertEqual(untrusted_references(text), [], text)

    def test_no_run_script_expands_untrusted_context(self):
        found = []
        for path in workflow_files():
            workflow = load(path.name)
            safe = typed_inputs(workflow)
            for job_id, index, step in steps_of(workflow):
                for key, script in scripts_of(step):
                    for ref in untrusted_references(script):
                        if ref in safe:
                            continue
                        label = step.get("name", f"step {index}")
                        found.append(f"{path.name}: {job_id} / {label} ({key}): {ref}")
        self.assertEqual(found, [], 'move these to env: and read them as "$VAR"')

    def test_esp_idf_command_variables_reach_the_container(self):
        # esp-idf-ci-action runs `command` inside `docker run`, which does not
        # inherit the step's env: each variable needs `-e NAME` in
        # extra_docker_args, or the command reads it as empty.
        missing = []
        for path in workflow_files():
            for job_id, index, step in steps_of(load(path.name)):
                if not str(step.get("uses", "")).startswith(ESP_IDF_ACTION):
                    continue
                with_ = step.get("with") or {}
                command = str(with_.get("command", ""))
                forwarded = set(
                    re.findall(r"-e\s+(\w+)", str(with_.get("extra_docker_args", "")))
                )
                for name in step.get("env") or {}:
                    used = re.search(rf"\$\{{?{name}\b", command)
                    if used and name not in forwarded:
                        label = step.get("name", f"step {index}")
                        missing.append(f"{path.name}: {job_id} / {label}: {name}")
        self.assertEqual(missing, [], "add -e NAME to extra_docker_args")


class BuildFirmwareTest(unittest.TestCase):
    def setUp(self):
        self.jobs = load("build-firmware.yml")["jobs"]

    def test_every_job_waits_for_the_tag_check(self):
        self.assertIn("resolve-tag", list(self.jobs))
        unchecked = []
        for job_id in self.jobs:
            if job_id == "resolve-tag":
                continue
            seen, frontier = set(), needs_of(self.jobs[job_id])
            while frontier:
                seen |= frontier
                frontier = (
                    set().union(*(needs_of(self.jobs[j]) for j in frontier)) - seen
                )
            if "resolve-tag" not in seen:
                unchecked.append(job_id)
        self.assertEqual(unchecked, [])

    def run_resolve_tag(self, tag: str) -> tuple[int, dict]:
        """Run the shipped tag check with ``tag`` as the release tag."""
        (step,) = [
            s
            for _, _, s in steps_of({"jobs": self.jobs})
            if s.get("name") == RESOLVE_TAG_STEP
        ]
        with tempfile.TemporaryDirectory() as scratch:
            output = Path(scratch) / "github-output"
            output.touch()
            env = {**os.environ, "TAG": tag, "GITHUB_OUTPUT": str(output)}
            proc = subprocess.run(
                ["bash", "-e", "-c", step["run"]], env=env, capture_output=True
            )
            values = dict(
                line.split("=", 1) for line in output.read_text().splitlines() if line
            )
        return proc.returncode, values

    def test_resolve_tag_passes_every_release_tag_in_the_clone(self):
        tags = subprocess.run(
            ["git", "-C", str(ROOT), "tag", "--list", "*-v*"],
            capture_output=True,
            text=True,
            check=True,
        ).stdout.split()
        # test.yml's CI Config Hooks job fetches the tags so this runs in CI;
        # a local clone without them skips.
        if not tags:
            self.skipTest("this clone has no release tags")
        refused = [t for t in tags if self.run_resolve_tag(t)[0] != 0]
        self.assertEqual(refused, [])

    def test_resolve_tag_passes_robocar_tags_from_both_sides_of_the_rename(self):
        # Needs no tag in the clone. Releases up to 0.2.23 are tagged
        # robocar-unified-v*, and a workflow_dispatch replay may still name one.
        for tag, component in (
            ("robocar-v0.2.24", "robocar"),
            ("robocar-unified-v0.2.23", "robocar-unified"),
            ("robocar-unified-v0.2.2", "robocar-unified"),
        ):
            with self.subTest(tag=tag):
                code, values = self.run_resolve_tag(tag)
                self.assertEqual(code, 0)
                self.assertEqual(values, {"tag": tag, "component": component})

    def test_resolve_tag_refuses_anything_else(self):
        for tag in (
            "",
            "v1.2.3",
            "robocar-unified-v1.2.3-rc1",
            "robocar-unified-v1.2",
            "Robocar-v1.2.3",
            "-v1.2.3",
            "robocar-unified-v1.2.3\nother=x",
            'x-v1.2.3"; echo pwned; "',
            "x-v1.2.3$(id)",
            "../x-v1.2.3",
        ):
            with self.subTest(tag=tag):
                code, values = self.run_resolve_tag(tag)
                self.assertNotEqual(code, 0)
                self.assertEqual(values, {}, "a refused tag must write no output")

    def assemble_pages(self, projects: list[str]) -> tuple[int, str, dict, list]:
        """Run the shipped Pages assembly step in a scratch tree holding one
        firmware/<project>/ per name. Returns the exit code, stderr, the files
        under _site/firmware by relative path, and firmware/'s entries after
        the step."""
        (step,) = [
            s
            for _, _, s in steps_of({"jobs": self.jobs})
            if s.get("name") == PAGES_STEP
        ]
        with tempfile.TemporaryDirectory() as scratch:
            tree = Path(scratch)
            files = {
                "docs/flasher/index.html": "flasher",
                "workflow-tools/docs/site/robocar/index.html": "page",
                "firmware/projects.json": "[]",
            }
            for project in projects:
                files[f"firmware/{project}/manifest.json"] = f"manifest of {project}"
                files[f"firmware/{project}/{project}.bin"] = f"image of {project}"
            for rel, text in files.items():
                (tree / rel).parent.mkdir(parents=True, exist_ok=True)
                (tree / rel).write_text(text)
            proc = subprocess.run(
                ["bash", "-eo", "pipefail", "-c", step["run"]],
                cwd=tree,
                capture_output=True,
                text=True,
            )
            site = tree / "_site" / "firmware"
            published = {
                str(f.relative_to(site)): f.read_text()
                for f in sorted(site.rglob("*"))
                if f.is_file()
            }
            firmware = sorted(os.listdir(tree / "firmware"))
        return proc.returncode, proc.stderr, published, firmware

    def test_pages_publishes_robocar_under_the_old_ota_directory_too(self):
        code, stderr, published, firmware = self.assemble_pages(["robocar", "other"])
        self.assertEqual(code, 0, stderr)
        self.assertEqual(published["robocar/manifest.json"], "manifest of robocar")
        for name in ("manifest.json", "robocar.bin"):
            with self.subTest(name=name):
                self.assertEqual(
                    published.get(f"unified/{name}"), published[f"robocar/{name}"]
                )
        # The copy is in _site only: the release gates count firmware/*.
        self.assertEqual(firmware, ["other", "projects.json", "robocar"])

    def test_pages_copies_nothing_for_a_tag_from_before_the_move(self):
        for projects in (["unified", "other"], ["main", "camera"]):
            with self.subTest(projects=projects):
                code, stderr, published, firmware = self.assemble_pages(projects)
                self.assertEqual(code, 0, stderr)
                self.assertEqual(
                    {Path(name).parts[0] for name in published},
                    {*projects, "projects.json"},
                )
                self.assertEqual(firmware, sorted([*projects, "projects.json"]))

    def test_pico_projects_build_with_the_ci_reusable_at_the_tag(self):
        callers = [j for j in self.jobs.values() if j.get("uses") == PICO_REUSABLE]
        self.assertEqual(len(callers), 1, f"expected one job calling {PICO_REUSABLE}")
        self.assertIn("needs.resolve-tag.outputs.tag", callers[0]["with"]["ref"])
        reusable_inputs = load("_ci-build-pico.yml")[True]["workflow_call"]["inputs"]
        self.assertIn("ref", reusable_inputs)

    def run_pico_select(self, component: str, with_files: bool) -> tuple[int, dict]:
        """Run the shipped select step in a scratch tree, as the job runs it
        after its sparse checkout of the tag."""
        (step,) = [
            s
            for _, _, s in steps_of({"jobs": self.jobs})
            if s.get("name") == PICO_SELECT_STEP
        ]
        with tempfile.TemporaryDirectory() as tree:
            tree = Path(tree)
            if with_files:
                (tree / ".github").mkdir()
                for rel in (
                    "release-please-config.json",
                    ".github/project-matrix.json",
                ):
                    shutil.copy(ROOT / rel, tree / rel)
            output = tree / "github-output"
            output.touch()
            env = {**os.environ, "COMPONENT": component, "GITHUB_OUTPUT": str(output)}
            proc = subprocess.run(
                ["bash", "-c", step["run"]], cwd=tree, env=env, capture_output=True
            )
            values = dict(
                line.split("=", 1) for line in output.read_text().splitlines() if line
            )
        return proc.returncode, values

    @unittest.skipUnless(shutil.which("jq"), "the select step needs jq")
    def test_pico_select_picks_balancebot(self):
        code, values = self.run_pico_select("balancebot", with_files=True)
        self.assertEqual(code, 0)
        self.assertEqual(values.get("count"), "1")
        self.assertIn('"project":"balancebot"', values.get("matrix", ""))

    @unittest.skipUnless(shutil.which("jq"), "the select step needs jq")
    def test_pico_select_skips_an_esp_idf_component(self):
        code, values = self.run_pico_select("robocar", with_files=True)
        self.assertEqual((code, values.get("count")), (0, "0"))

    def test_pico_select_passes_a_tag_older_than_the_matrix(self):
        # 43 tags (robocar-unified-v0.1.2, balancebot-v0.1.1, ...) predate
        # .github/project-matrix.json; a workflow_dispatch replay of one must
        # select nothing, not fail the run.
        code, values = self.run_pico_select("balancebot", with_files=False)
        self.assertEqual(code, 0)
        self.assertEqual(values, {"matrix": '{"include":[]}', "count": "0"})

    def test_no_pico_build_step_is_copied(self):
        copied = [
            f"{job_id} / {step.get('name', index)}"
            for job_id, index, step in steps_of({"jobs": self.jobs})
            if re.search(
                r"gcc-arm-none-eabi|pico-sdk|picotool", str(step.get("run", ""))
            )
        ]
        self.assertEqual(copied, [])


@unittest.skipUnless(shutil.which("jq"), "the discovery step needs jq")
class BuildDiscoveryTest(unittest.TestCase):
    """build.yml's shipped discovery step, run on a pull request that changes
    the given file or files, in a scratch repository holding the files the
    step reads."""

    def discover(
        self, changed: str | list[str], matrix: list | None = None
    ) -> list[dict]:
        (step,) = [
            s
            for _, _, s in steps_of(load("build.yml"))
            if s.get("name") == DISCOVER_STEP
        ]
        with tempfile.TemporaryDirectory() as scratch:
            tree = Path(scratch) / "tree"
            (tree / ".github" / "workflows").mkdir(parents=True)
            for name in (
                "build.yml",
                *(p.name for p in WORKFLOWS.glob("_ci-build-*.yml")),
            ):
                shutil.copy(WORKFLOWS / name, tree / ".github" / "workflows" / name)
            (tree / ".github" / "project-matrix.json").write_text(
                MATRIX.read_text() if matrix is None else json.dumps(matrix)
            )

            def git(*args: str) -> str:
                return subprocess.run(
                    ["git", "-C", str(tree), *args],
                    check=True,
                    capture_output=True,
                    text=True,
                ).stdout.strip()

            git("init", "-q")
            git("config", "user.email", "t@example.invalid")
            git("config", "user.name", "t")
            git("config", "commit.gpgsign", "false")
            git("add", "-A")
            git("commit", "-q", "-m", "base")
            base = git("rev-parse", "HEAD")
            for rel in [changed] if isinstance(changed, str) else changed:
                path = tree / rel
                path.parent.mkdir(parents=True, exist_ok=True)
                with path.open("a") as f:
                    f.write("changed\n")
            git("add", "-A")
            git("commit", "-q", "-m", "change")
            output = Path(scratch) / "github-output"
            output.touch()
            env = {
                **os.environ,
                "EVENT": "pull_request",
                "PR_BASE": base,
                "PUSH_BEFORE": "",
                "GITHUB_OUTPUT": str(output),
            }
            proc = subprocess.run(
                ["bash", "-c", step["run"]],
                cwd=tree,
                env=env,
                capture_output=True,
                text=True,
            )
            self.assertEqual(proc.returncode, 0, proc.stderr)
            values = dict(
                line.split("=", 1) for line in output.read_text().splitlines() if line
            )
        return [
            entry
            for system in ("esp32", "esphome", "pico")
            for entry in json.loads(values[system])["include"]
        ]

    def selected(
        self, changed: str | list[str], matrix: list | None = None
    ) -> list[str]:
        return sorted(entry["project"] for entry in self.discover(changed, matrix))

    def test_a_project_nested_in_robocar_does_not_rebuild_the_firmware(self):
        for nested, expected in (
            ("bringup", ["robocar-bringup"]),
            ("integrated", []),
            ("simulation", []),
            ("twin", []),
        ):
            with self.subTest(nested=nested):
                self.assertEqual(
                    self.selected(f"{ROBOCAR}/{nested}/main/x.c"), expected
                )

    def test_a_firmware_change_selects_the_firmware_and_bringup(self):
        # robocar-bringup compiles the firmware's main/ (its extra_paths).
        self.assertEqual(
            self.selected(f"{ROBOCAR}/main/x.c"), ["robocar", "robocar-bringup"]
        )
        self.assertEqual(self.selected(f"{ROBOCAR}/sdkconfig.defaults"), ["robocar"])
        self.assertEqual(
            self.selected(f"{ROBOCAR}/hardware/carrier/x.toml"), ["robocar"]
        )

    def test_a_mixed_change_still_selects_the_firmware(self):
        # exclude_paths drops a changed file, not the project: a pull request
        # that touches the twin and the firmware builds the firmware.
        self.assertEqual(
            self.selected([f"{ROBOCAR}/twin/main/x.c", f"{ROBOCAR}/main/x.c"]),
            ["robocar", "robocar-bringup"],
        )

    def test_the_nested_check_fails_without_the_exclude(self):
        # Control: with exclude_paths dropped, the same change selects the
        # firmware, so the test above reads the field and not the layout.
        matrix = json.loads(MATRIX.read_text())
        self.assertTrue(any("exclude_paths" in entry for entry in matrix))
        for entry in matrix:
            entry.pop("exclude_paths", None)
        self.assertEqual(self.selected(f"{ROBOCAR}/twin/main/x.c", matrix), ["robocar"])

    def test_discovery_inputs_stay_out_of_the_job_matrix(self):
        # Selected by path, and all at once on a change to a shared build file.
        for changed in (f"{ROBOCAR}/main/x.c", "Dockerfile"):
            with self.subTest(changed=changed):
                entries = self.discover(changed)
                leaked = [
                    entry["project"]
                    for entry in entries
                    if {"extra_paths", "exclude_paths"} & set(entry)
                ]
                self.assertEqual(leaked, [])
        self.assertEqual(len(entries), len(json.loads(MATRIX.read_text())))


class ClaudeCallersTest(unittest.TestCase):
    def test_auto_fix_pushes_with_the_release_please_app(self):
        jobs = load("auto-fix.yml")["jobs"]
        callers = [
            j
            for j in jobs.values()
            if "reusable-auto-fix.yml" in str(j.get("uses", ""))
        ]
        self.assertEqual(len(callers), 1)
        self.assertEqual(
            callers[0]["with"].get("app-id"), "${{ vars.RELEASE_PLEASE_CLIENT_ID }}"
        )
        self.assertEqual(
            callers[0]["secrets"].get("APP_PRIVATE_KEY"),
            "${{ secrets.RELEASE_PLEASE_PRIVATE_KEY }}",
        )

    def test_secret_scanning_runs_gitleaks_only(self):
        jobs = load("security-secrets.yml")["jobs"]
        self.assertIn("gitleaks", jobs)
        reusable = [
            job_id
            for job_id, job in jobs.items()
            if "reusable-security-secrets.yml" in str(job.get("uses", ""))
        ]
        self.assertEqual(reusable, [])


class PermissionsTest(unittest.TestCase):
    def test_every_workflow_declares_workflow_level_permissions(self):
        missing = sorted(
            p.name
            for p in workflow_files()
            if "permissions" not in load(p.name) and p.name not in INHERITS_REPO_DEFAULT
        )
        self.assertEqual(missing, [], "add a workflow-level permissions: block")

    def test_the_exemptions_are_still_bare_reusable_callers(self):
        # Tripwire: once a listed caller gains a permissions block, or is
        # deleted, its entry is stale.
        for name in sorted(INHERITS_REPO_DEFAULT):
            with self.subTest(workflow=name):
                self.assertTrue((WORKFLOWS / name).exists())
                workflow = load(name)
                self.assertNotIn("permissions", workflow)
                jobs = workflow["jobs"].values()
                self.assertTrue(all("uses" in job for job in jobs))

    def test_reusable_callers_grant_the_reusables_scopes(self):
        # Each caller, the laurigates/.github reusable it runs, and the scopes
        # that reusable declares at its workflow level (read at @main on
        # 2026-11-13). A called workflow gets no more than its caller job
        # grants, so with a read-only workflow level the caller job grants
        # exactly these. When a reusable adds a scope, its caller fails at
        # startup until the scope is added here and in the caller.
        callers = {
            "auto-fix.yml": (
                "reusable-auto-fix.yml",
                {
                    "contents": "write",
                    "pull-requests": "write",
                    "issues": "write",
                    "actions": "read",
                    "id-token": "write",
                },
            ),
            "auto-resolve-conflicts.yml": (
                "reusable-auto-resolve-conflicts.yml",
                {"contents": "write", "pull-requests": "write", "issues": "write"},
            ),
            "claude.yml": (
                "reusable-claude.yml",
                {
                    "contents": "write",
                    "pull-requests": "write",
                    "issues": "write",
                    "id-token": "write",
                    "actions": "read",
                },
            ),
            "fix-release-conflicts.yml": (
                "reusable-fix-release-conflicts.yml",
                {"contents": "write", "pull-requests": "write", "actions": "write"},
            ),
            "sync-ai-rules.yml": (
                "reusable-sync-ai-rules.yml",
                {"contents": "write", "pull-requests": "write"},
            ),
        }
        for name, (reusable, scopes) in sorted(callers.items()):
            with self.subTest(workflow=name):
                workflow = load(name)
                self.assertEqual(workflow.get("permissions"), {"contents": "read"})
                (caller,) = [
                    job
                    for job in workflow["jobs"].values()
                    if str(job.get("uses", "")).startswith(
                        f"laurigates/.github/.github/workflows/{reusable}@"
                    )
                ]
                self.assertEqual(caller.get("permissions"), scopes)


def glob_regex(glob: str) -> re.Pattern:
    """A dorny/paths-filter (picomatch) glob as a regex: ``**`` spans
    directories, ``*`` stays within one."""
    out, i = "", 0
    while i < len(glob):
        if glob.startswith("**/", i):
            out, i = out + "(?:.*/)?", i + 3
        elif glob.startswith("**", i):
            out, i = out + ".*", i + 2
        elif glob[i] == "*":
            out, i = out + "[^/]*", i + 1
        else:
            out, i = out + re.escape(glob[i]), i + 1
    return re.compile(out + r"\Z")


def tracked_files() -> list[str]:
    return subprocess.run(
        ["git", "-C", str(ROOT), "ls-files"],
        capture_output=True,
        text=True,
        check=True,
    ).stdout.split()


def local_hooks() -> list[dict]:
    config = yaml.safe_load(PRE_COMMIT.read_text())
    return [
        hook
        for repo in config["repos"]
        if repo["repo"] == "local"
        for hook in repo["hooks"]
    ]


class TestSuiteTest(unittest.TestCase):
    def setUp(self):
        self.jobs = load("test.yml")["jobs"]
        (filter_step,) = [
            s for s in self.jobs["detect-changes"]["steps"] if s.get("id") == "filter"
        ]
        filters = yaml.safe_load(filter_step["with"]["filters"])
        self.any_code = [glob_regex(g) for g in filters["any-code"]]
        always_on = [
            job for job in self.jobs.values() if "needs" not in job and "if" not in job
        ]
        self.always_on_runs = [
            str(step.get("run", "")).replace(" -v ", " ")
            for job in always_on
            for step in job.get("steps") or []
        ]
        self.config_hooks = set(
            str(self.jobs[CONFIG_HOOKS_JOB].get("env", {}).get("HOOKS", "")).split()
        )
        # Other workflows that run on pull requests: their path filter (None
        # when there is none) and the scripts they run.
        self.pr_workflows = []
        for path in workflow_files():
            if path.name == "test.yml":
                continue
            workflow = load(path.name)
            triggers = workflow.get(True) or workflow.get("on") or {}
            if not isinstance(triggers, dict) or "pull_request" not in triggers:
                continue
            paths = (triggers["pull_request"] or {}).get("paths")
            self.pr_workflows.append(
                (
                    None if paths is None else [glob_regex(g) for g in paths],
                    [str(s.get("run", "")) for _, _, s in steps_of(workflow)],
                )
            )

    def run_by_its_own_workflow(self, entry: str, file: str) -> bool:
        """A workflow other than test.yml runs ``entry`` on a PR touching
        ``file``, as build-guide-check.yml runs check-typst-docs (#693)."""
        return any(
            (paths is None or any(g.match(file) for g in paths))
            and any(entry in run for run in runs)
            for paths, runs in self.pr_workflows
        )

    def uncovered(self, hook_ids: set[str], runs: list[str]) -> list[str]:
        """``hook: file`` for each CI-config file a local hook watches that
        neither any-code, an always-on job nor the file's own workflow
        covers."""
        top_exclude = re.compile(
            yaml.safe_load(PRE_COMMIT.read_text()).get("exclude") or r"(?!)"
        )
        files = [
            f
            for f in tracked_files()
            if CI_CONFIG.search(f) and not top_exclude.search(f)
        ]
        gaps = []
        for hook in local_hooks():
            if not hook.get("files"):
                continue
            if hook["id"] in hook_ids or any(hook["entry"] in r for r in runs):
                continue
            watched = re.compile(hook["files"])
            gaps += [
                f"{hook['id']}: {f}"
                for f in files
                if watched.search(f)
                and not any(g.match(f) for g in self.any_code)
                and not self.run_by_its_own_workflow(hook["entry"], f)
            ]
        return gaps

    def test_hooks_guarding_ci_config_run_on_a_config_only_pr(self):
        self.assertEqual(self.uncovered(self.config_hooks, self.always_on_runs), [])

    def test_coverage_check_has_teeth(self):
        # Control: without the always-on jobs, the hooks the issue names are
        # uncovered on exactly the files a workflow-only PR changes.
        gaps = self.uncovered(set(), [])
        for expected in (
            "workflow-structure: .github/workflows/test.yml",
            "kconfig-symbols: Dockerfile",
            "kconfig-symbols: .github/workflows/build-firmware.yml",
            "check-flash-recipes: .github/actions/setup-tools/action.yml",
            "check-flash-recipes-tests: Dockerfile",
            "python-suites: .github/python-suites.json",
            # A root config file and a Claude hook, outside .github/ (#1148,
            # #1130).
            "check-ota-consumer-agreement: release-please-config.json",
            "check-ota-consumer-agreement-tests: release-please-config.json",
            "post-edit-lint-hook: .claude/hooks/post-edit-lint.sh",
        ):
            self.assertIn(expected, gaps)
        typst_docs = "python3 tools/check-typst-docs.py"
        build_guide = ".github/workflows/build-guide-check.yml"
        self.assertTrue(self.run_by_its_own_workflow(typst_docs, build_guide))
        self.assertFalse(self.run_by_its_own_workflow(typst_docs, "Dockerfile"))

    def test_glob_regex_follows_picomatch(self):
        for glob, path, hit in (
            ("packages/**", "packages/a/b.c", True),
            ("**/docs/*.typ", "docs/x.typ", True),
            ("**/docs/*.typ", "pkg/a/docs/x.typ", True),
            ("**/docs/*.typ", "pkg/docs/auto/x.typ", False),
            ("tools/*.py", "tools/a/b.py", False),
            ("Dockerfile", "a/Dockerfile", False),
        ):
            with self.subTest(glob=glob, path=path):
                self.assertEqual(bool(glob_regex(glob).match(path)), hit)

    def test_config_hooks_job_runs_on_every_event_and_names_real_hooks(self):
        job = self.jobs[CONFIG_HOOKS_JOB]
        self.assertNotIn("needs", job)
        self.assertNotIn("if", job)
        self.assertTrue(self.config_hooks)
        unknown = self.config_hooks - {h["id"] for h in local_hooks()}
        self.assertEqual(unknown, set())

    def test_config_hooks_job_fetches_the_release_tags(self):
        # workflow-structure checks every release tag in the clone, and
        # skips when there are none; the checkout fetches no tags.
        steps = self.jobs[CONFIG_HOOKS_JOB]["steps"]
        runs = [str(s.get("run", "")) for s in steps]
        fetch = [i for i, r in enumerate(runs) if "refs/tags/*:refs/tags/*" in r]
        hooks = [i for i, r in enumerate(runs) if "pre-commit run" in r]
        self.assertEqual(len(fetch), 1)
        self.assertLess(fetch[0], hooks[0])

    def test_a_push_runs_the_jobs_any_code_selects(self):
        # On push, the workflow's own path filter decides whether
        # detect-changes runs at all, so every any-code glob must be in it.
        paths = load("test.yml")[True]["push"]["paths"]
        (filter_step,) = [
            s for s in self.jobs["detect-changes"]["steps"] if s.get("id") == "filter"
        ]
        globs = yaml.safe_load(filter_step["with"]["filters"])["any-code"]
        self.assertIn("ruff.toml", globs)
        self.assertEqual([g for g in globs if g not in paths], [])

    def test_every_gating_job_reaches_the_summary_check(self):
        summary = self.jobs["test-summary"]
        script = summary["steps"][-1]["run"]
        loop = script[script.index("for result in") :]
        for job_id in self.jobs:
            if job_id == "test-summary":
                continue
            with self.subTest(job=job_id):
                self.assertIn(job_id, summary["needs"])
                gated = f"needs.{job_id}.result" in loop
                self.assertEqual(gated, job_id not in ADVISORY_JOBS)


class ClangFormatPinTest(unittest.TestCase):
    """test.yml's Format Check installs the clang-format the pre-commit hook
    pins (#1082). Its install step runs here with stub ``yq`` and ``pipx``:
    the runner's yq is mikefarah's, which this test does not assume."""

    def setUp(self):
        config = yaml.safe_load(PRE_COMMIT.read_text())
        (self.repo,) = [
            r
            for r in config["repos"]
            if any(h["id"] == "clang-format" for h in r.get("hooks", []))
        ]
        job = load("test.yml")["jobs"]["format-check"]
        (step,) = [s for s in job["steps"] if s.get("name") == "Install clang-format"]
        self.script = step["run"]

    def run_step(self, rev: str) -> tuple[int, str]:
        """Run the step with a yq that prints ``rev`` when asked for the
        hook's repo, and ``null`` otherwise. Returns its exit code and the
        arguments pipx got."""
        with tempfile.TemporaryDirectory() as scratch:
            bin_dir = Path(scratch)
            for name, body in (
                (
                    "yq",
                    'if [[ "$1" == *"\\"$STUB_URL\\""* && "$2" == .pre-commit-config.yaml ]]; '
                    'then echo "$STUB_REV"; else echo null; fi\n',
                ),
                ("pipx", 'echo "$*" >> "$PIPX_LOG"\n'),
            ):
                (bin_dir / name).write_text("#!/usr/bin/env bash\n" + body)
                (bin_dir / name).chmod(0o755)
            log = bin_dir / "pipx.log"
            log.touch()
            env = {
                **os.environ,
                "PATH": f"{bin_dir}{os.pathsep}{os.environ['PATH']}",
                "STUB_URL": self.repo["repo"],
                "STUB_REV": rev,
                "PIPX_LOG": str(log),
            }
            proc = subprocess.run(
                ["bash", "-e", "-c", self.script],
                cwd=ROOT,
                env=env,
                capture_output=True,
                text=True,
            )
            return proc.returncode, log.read_text().strip()

    def test_format_check_installs_the_hooks_rev(self):
        rev = self.repo["rev"]
        self.assertRegex(rev, r"^v\d")
        self.assertEqual(self.run_step(rev), (0, f"install clang-format=={rev[1:]}"))
        self.assertNotRegex(
            self.script, r"clang-format==\d", "read the rev, not a copy"
        )

    def test_format_check_fails_without_a_rev(self):
        for rev in ("null", "", "18.1.8"):
            with self.subTest(rev=rev):
                self.assertEqual(self.run_step(rev), (1, ""))

    def test_a_rev_bump_runs_format_check(self):
        (filter_step,) = [
            s
            for s in load("test.yml")["jobs"]["detect-changes"]["steps"]
            if s.get("id") == "filter"
        ]
        c_code = yaml.safe_load(filter_step["with"]["filters"])["c-code"]
        self.assertIn(".pre-commit-config.yaml", c_code)


def drift_step() -> str:
    (step,) = [
        s
        for _, _, s in steps_of(load("build-guide-check.yml"))
        if str(s.get("name", "")).endswith(DRIFT_STEP_SUFFIX)
    ]
    return step["run"]


class GitGateTest(unittest.TestCase):
    def test_pattern_catches_the_shapes_it_targets(self):
        for text in (
            'if [ -z "$(git status --porcelain -- "${drift_paths[@]}")" ]; then',
            '[[ -n "$(git status --porcelain)" ]] && exit 1',
            'test -z "$(git diff --name-only)"',
            "done < <(git ls-files ':(glob)**/docs/*.typ')",
            "if ! git diff --quiet -- 'docs/schematics/images/*.png'; then",
            "git diff --exit-code -- out/",
        ):
            self.assertIsNotNone(HIDDEN_GIT_FAILURE.search(text), text)
        for text in (
            'drift=$(git status --porcelain -- "${drift_paths[@]}")',
            'if [ -z "$drift" ]; then',
            "git --no-pager diff --stat -- 'docs/schematics/images/*.svg'",
            'mapfile -t projects < <(jq -r ".include[].project" <<< "$MATRIX")',
        ):
            self.assertIsNone(HIDDEN_GIT_FAILURE.search(text), text)

    def test_no_gate_hides_a_git_failure(self):
        found = []
        for path in sorted(WORKFLOWS.glob("*.yml")):
            for job_id, index, step in steps_of(load(path.name)):
                for line in str(step.get("run", "")).splitlines():
                    if line.lstrip().startswith("#"):
                        continue
                    if HIDDEN_GIT_FAILURE.search(line):
                        label = step.get("name", f"step {index}")
                        found.append(f"{path.name}: {job_id} / {label}: {line.strip()}")
        self.assertEqual(
            found,
            [],
            "assign git's output to a variable first (set -e then catches a "
            "failure), and read drift with git status --porcelain",
        )


class BuildGuideDriftStepTest(unittest.TestCase):
    """build-guide-check.yml's drift step, run against a scratch repository
    with stub ``typst`` and ``git``, so a failing git can be staged."""

    REAL_GIT = shutil.which("git")

    def setUp(self):
        scratch = tempfile.TemporaryDirectory()
        self.addCleanup(scratch.cleanup)
        self.root = Path(scratch.name)
        self.repo = self.root / "repo"
        docs = self.repo / "pkg" / "docs"
        docs.mkdir(parents=True)
        (docs / "guide.typ").write_text("= Guide\n")
        (docs / "guide.pdf").write_text("pdf\n")
        self.env = {
            **os.environ,
            "GIT_CONFIG_GLOBAL": os.devnull,
            "GIT_CONFIG_NOSYSTEM": "1",
            "GIT_AUTHOR_NAME": "t",
            "GIT_AUTHOR_EMAIL": "t@example.invalid",
            "GIT_COMMITTER_NAME": "t",
            "GIT_COMMITTER_EMAIL": "t@example.invalid",
            "TYPST_VERSION": "0.0.0-stub",
        }
        for args in (("init", "-q"), ("add", "."), ("commit", "-q", "-m", "init")):
            subprocess.run(
                [self.REAL_GIT, "-C", str(self.repo), *args],
                check=True,
                capture_output=True,
                env=self.env,
            )
        bin_dir = self.root / "bin"
        bin_dir.mkdir()
        # typst: --version prints; compile writes $STUB_PDF to the output if
        # set, else leaves the committed PDF as it is.
        self._stub(
            bin_dir / "typst",
            '[ "$1" = --version ] && { echo "typst stub"; exit 0; }\n'
            'out="${!#}"\n'
            '[ -z "${STUB_PDF-}" ] || printf "%s\\n" "$STUB_PDF" > "$out"\n',
        )
        # git: fails (exit 128) when any argument equals $STUB_GIT_FAIL.
        self._stub(
            bin_dir / "git",
            'for a in "$@"; do\n'
            '  [ "$a" = "${STUB_GIT_FAIL-}" ] && { echo "stub: git $a fails" >&2; exit 128; }\n'
            "done\n"
            f'exec "{self.REAL_GIT}" "$@"\n',
        )
        self.env["PATH"] = f"{bin_dir}{os.pathsep}{os.environ['PATH']}"

    @staticmethod
    def _stub(path: Path, body: str) -> None:
        path.write_text("#!/usr/bin/env bash\n" + body)
        path.chmod(0o755)

    def run_step(self, **env: str) -> subprocess.CompletedProcess:
        script = self.root / "step.sh"
        script.write_text(drift_step())
        return subprocess.run(
            ["bash", str(script)],
            cwd=self.repo,
            env={**self.env, **env},
            capture_output=True,
            text=True,
        )

    def test_committed_state_is_up_to_date(self):
        result = self.run_step()
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        self.assertIn("up to date: pkg/docs/guide.pdf", result.stdout)

    def test_a_recompiled_pdf_that_differs_fails(self):
        result = self.run_step(STUB_PDF="changed")
        self.assertEqual(result.returncode, 1, result.stdout + result.stderr)
        self.assertIn("Committed Typst output is stale", result.stdout)

    def test_a_failing_git_status_fails_the_step(self):
        result = self.run_step(STUB_GIT_FAIL="status")
        self.assertNotEqual(result.returncode, 0, result.stdout + result.stderr)
        self.assertNotIn("up to date", result.stdout)

    def test_a_failing_document_discovery_fails_the_step(self):
        result = self.run_step(STUB_GIT_FAIL="ls-files")
        self.assertNotEqual(result.returncode, 0, result.stdout + result.stderr)


# A `uses:` line and its ref. Third-party actions are pinned to a full commit
# SHA with the tag in a trailing comment, which Renovate's github-actions
# manager keeps in step (issue #1103). GitHub's own actions/*, the
# laurigates/.github reusables (@main by design) and local ./ refs are exempt.
USES_LINE = re.compile(r"^\s*(?:-\s+)?uses:\s*(\S+)(.*)$")
PINNED_SUFFIX = re.compile(r"@[0-9a-f]{40}\Z")
VERSION_COMMENT = re.compile(r"\s+#\s+v\d+\.\d+\.\d+\Z")
UNPINNED_BY_DESIGN = ("./", "actions/", "laurigates/.github/")


def third_party_refs() -> list[tuple[str, int, str, str]]:
    """(file, line, ref, rest of line) for each third-party ``uses:``."""
    files = workflow_files() + sorted(
        (ROOT / ".github" / "actions").glob("*/action.yml")
    )
    refs = []
    for path in files:
        for n, line in enumerate(path.read_text().splitlines(), 1):
            m = USES_LINE.match(line)
            if m and not m.group(1).startswith(UNPINNED_BY_DESIGN):
                refs.append((str(path.relative_to(ROOT)), n, m.group(1), m.group(2)))
    return refs


class ActionPinTest(unittest.TestCase):
    def test_third_party_actions_are_pinned_to_a_commit(self):
        refs = third_party_refs()
        self.assertTrue(refs)
        unpinned = [
            f"{f}:{n}: {ref}{rest}"
            for f, n, ref, rest in refs
            if not (PINNED_SUFFIX.search(ref) and VERSION_COMMENT.fullmatch(rest))
        ]
        self.assertEqual(unpinned, [], "pin to the tag's commit SHA with # vX.Y.Z")

    def test_pin_check_refuses_a_tag_and_a_bare_sha(self):
        sha = "0123456789abcdef0123456789abcdef01234567"
        for ref, rest, pinned in (
            (f"dorny/paths-filter@{sha}", " # v4.0.3", True),
            ("dorny/paths-filter@v4.0.3", "", False),
            (f"dorny/paths-filter@{sha}", "", False),
            (f"dorny/paths-filter@{sha[:12]}", " # v4.0.3", False),
        ):
            with self.subTest(ref=ref, rest=rest):
                ok = bool(PINNED_SUFFIX.search(ref) and VERSION_COMMENT.fullmatch(rest))
                self.assertEqual(ok, pinned)


if __name__ == "__main__":
    unittest.main()
