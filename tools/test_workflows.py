"""Structural tests for .github/workflows/*.yml.

Needs PyYAML, so the pre-commit hook runs it through uv:

    uv run --no-project --with pyyaml==6.0.3 python -m unittest tools/test_workflows.py

What the tests hold:

- No ``run:`` script, and no ``with.command`` (espressif/esp-idf-ci-action
  pastes it into its own script), expands ``inputs.*``, ``github.event.*`` or
  ``github.head_ref`` with ``${{ }}``. Actions substitutes the text into the
  script before the shell parses it, so a value carrying ``$(...)`` or a quote
  runs as code in a job that may hold ``contents: write`` (issue #855). Pass
  such a value through ``env:`` and read it as ``"$VAR"``; for
  esp-idf-ci-action, also forward it into its container with
  ``extra_docker_args: -e NAME``, which the suite checks. A ``boolean`` or
  ``number`` input is exempt, since GitHub rejects any other value. Other contexts
  (``matrix``, ``needs``, ``steps``, ``env``, ``github.repository``) are not
  covered: their values come from this repo's files or from GitHub itself.
- build-firmware.yml checks the release tag before any job uses it: every
  other job needs ``resolve-tag``, directly or through another job.
- build-firmware.yml builds Pico SDK projects by calling
  ``_ci-build-pico.yml``, the reusable workflow build.yml calls, at the
  release tag, and copies none of its steps (issue #1065). Its select step,
  run here against the repo's own files, picks balancebot, skips an ESP-IDF
  component, and selects nothing for a tag older than project-matrix.json.
- auto-fix.yml passes the release-please App to the reusable auto-fix, so the
  fix commits it pushes start CI (issue #866).
- security-secrets.yml runs one secret scan, the gitleaks job, and no longer
  calls the reusable Claude scan (issue #860).

Whether the workflows run as intended needs GitHub; actionlint and these tests
only read them.
"""

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

UNTRUSTED_IN_EXPRESSION = re.compile(
    r"\$\{\{[^}]*?\b(inputs\.[\w-]+|github\.event\.[\w.-]+|github\.head_ref)"
)
PICO_REUSABLE = "./.github/workflows/_ci-build-pico.yml"
ESP_IDF_ACTION = "espressif/esp-idf-ci-action@"
PICO_SELECT_STEP = "Match the tag's component to a Pico SDK project"


def load(name: str) -> dict:
    return yaml.safe_load((WORKFLOWS / name).read_text())


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
        ):
            self.assertIsNotNone(UNTRUSTED_IN_EXPRESSION.search(text), text)
        for text in (
            'if [ "${{ github.event_name }}" = "push" ]',
            'repo="${{ github.repository }}"',
            'cd "${{ matrix.project_path }}"',
            'TAG="$RELEASE_TAG"',
        ):
            self.assertIsNone(UNTRUSTED_IN_EXPRESSION.search(text), text)

    def test_no_run_script_expands_untrusted_context(self):
        found = []
        for path in workflow_files():
            workflow = load(path.name)
            safe = typed_inputs(workflow)
            for job_id, index, step in steps_of(workflow):
                for key, script in scripts_of(step):
                    for match in UNTRUSTED_IN_EXPRESSION.finditer(script):
                        if match.group(1) in safe:
                            continue
                        label = step.get("name", f"step {index}")
                        found.append(
                            f"{path.name}: {job_id} / {label} ({key}): {match.group(1)}"
                        )
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
        code, values = self.run_pico_select("robocar-unified", with_files=True)
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


if __name__ == "__main__":
    unittest.main()
