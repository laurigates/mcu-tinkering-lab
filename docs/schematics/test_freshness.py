"""Tests for the rendered-image freshness check (issue #721).

Two halves keep ``images/`` in step with ``circuits/``:

* ``render.py`` deletes an image no circuit produces any more, so removing
  a circuit removes its SVG and PNG instead of leaving them behind.
* ``schematics-check.yml``'s "Verify SVGs are up to date" step reads
  ``git status --porcelain``, which reports an untracked SVG, where
  ``git diff`` sees only changes to tracked files.

The workflow step is tested by running its shipped ``run:`` block, extracted
from the YAML, against a throwaway repository; a retyped copy of the logic
could pass while the shipped step does not.
"""

from __future__ import annotations

import os
import re
import subprocess
import sys
import types
from pathlib import Path

import pytest
import render

REPO = Path(__file__).resolve().parents[2]
WORKFLOW = REPO / ".github" / "workflows" / "schematics-check.yml"
GATE_STEP = "Verify SVGs are up to date"
IMAGES = "docs/schematics/images"


# --- render.py removes images no circuit produces --------------------------


def _touch(directory: Path, *names: str) -> None:
    for name in names:
        (directory / name).write_text(name + "\n")


def test_remove_orphans_deletes_svg_and_png_no_circuit_produces(tmp_path):
    _touch(tmp_path, "kept.svg", "kept.png", "gone.svg", "gone.png", "lone.png")
    _touch(tmp_path, "notes.txt")

    removed = render.remove_orphans({"kept"}, tmp_path)

    assert sorted(p.name for p in removed) == ["gone.png", "gone.svg", "lone.png"]
    assert sorted(p.name for p in tmp_path.iterdir()) == [
        "kept.png",
        "kept.svg",
        "notes.txt",
    ]


def test_remove_orphans_on_a_missing_directory_removes_nothing(tmp_path):
    assert render.remove_orphans({"kept"}, tmp_path / "absent") == []


_CIRCUIT = """\
import schemdraw
import schemdraw.elements as elm


def draw():
    d = schemdraw.Drawing(show=False)
    d += elm.Resistor()
    return d
"""


@pytest.fixture
def sandbox(tmp_path, monkeypatch):
    """A circuits/ + images/ pair, with cairosvg replaced by a file writer.

    The PNG step needs libcairo, which this test does not exercise; the stub
    writes a placeholder so the render loop runs end to end without it.
    """
    circuits = tmp_path / "circuits"
    images = tmp_path / "images"
    circuits.mkdir()
    images.mkdir()
    name = "zz_freshness_kept"
    (circuits / f"{name}.py").write_text(_CIRCUIT)
    _touch(images, "zz_freshness_gone.svg", "zz_freshness_gone.png")

    fake = types.ModuleType("cairosvg")
    fake.svg2png = lambda url, write_to, output_width: Path(write_to).write_text("png")
    monkeypatch.setitem(sys.modules, "cairosvg", fake)
    monkeypatch.setattr(render, "CIRCUITS", circuits)
    monkeypatch.setattr(render, "IMAGES", images)
    yield name, images
    sys.modules.pop(name, None)


def test_full_render_removes_the_image_of_a_deleted_circuit(sandbox):
    name, images = sandbox

    assert render.main([]) == 0

    assert {p.name for p in images.iterdir()} == {f"{name}.png", f"{name}.svg"}


def test_rendering_named_circuits_leaves_other_images_alone(sandbox):
    # render-one renders a subset, so an image it did not render is not an
    # orphan; deleting it would wipe every other circuit's output.
    name, images = sandbox

    assert render.main([name]) == 0

    assert {p.name for p in images.iterdir()} == {
        f"{name}.png",
        f"{name}.svg",
        "zz_freshness_gone.png",
        "zz_freshness_gone.svg",
    }


# --- schematics-check.yml's SVG gate --------------------------------------


def step_script(name: str) -> str:
    """The shipped ``run:`` block of the step called ``name``, dedented.

    A ``run:`` that follows an ``if:`` key is found too."""
    text = WORKFLOW.read_text(encoding="utf-8")
    m = re.search(
        rf"- name: {re.escape(name)}\n(?: +if: [^\n]*\n)?(?P<indent> +)run: \|\n(?P<body>.*?)(?=\n {{0,9}}\S|\Z)",
        text,
        re.DOTALL,
    )
    assert m, f"no '{name}' step with a run: block in {WORKFLOW.name}"
    body_indent = len(m.group("indent")) + 2
    return "\n".join(line[body_indent:] for line in m.group("body").split("\n")) + "\n"


def gate_script() -> str:
    """The shipped ``run:`` block of the SVG gate step, dedented."""
    return step_script(GATE_STEP)


def _git(repo: Path, *args: str) -> None:
    subprocess.run(["git", "-C", str(repo), *args], check=True, capture_output=True)


@pytest.fixture
def repo(tmp_path, monkeypatch):
    """A repository with one committed SVG + PNG, isolated from user git config."""
    monkeypatch.setenv("GIT_CONFIG_GLOBAL", os.devnull)
    monkeypatch.setenv("GIT_CONFIG_NOSYSTEM", "1")
    monkeypatch.setenv("GIT_AUTHOR_NAME", "test")
    monkeypatch.setenv("GIT_AUTHOR_EMAIL", "test@example.invalid")
    monkeypatch.setenv("GIT_COMMITTER_NAME", "test")
    monkeypatch.setenv("GIT_COMMITTER_EMAIL", "test@example.invalid")
    images = tmp_path / IMAGES
    images.mkdir(parents=True)
    _touch(images, "kept.svg", "kept.png")
    _git(tmp_path, "init", "-q")
    _git(tmp_path, "add", ".")
    _git(tmp_path, "commit", "-q", "-m", "init")
    return tmp_path


def run_gate(repo: Path, name: str = GATE_STEP) -> subprocess.CompletedProcess:
    script = repo.parent / "gate.sh"
    script.write_text(step_script(name))
    return subprocess.run(
        ["bash", str(script)], cwd=repo, capture_output=True, text=True
    )


def test_gate_passes_on_the_committed_state(repo):
    result = run_gate(repo)
    assert result.returncode == 0, result.stdout + result.stderr


def test_gate_fails_on_an_untracked_svg(repo):
    # A circuit added without `git add` on its SVG (the #721 reproduction).
    _touch(repo / IMAGES, "new.svg")
    result = run_gate(repo)
    assert result.returncode != 0
    assert "new.svg" in result.stdout


def test_gate_fails_on_a_modified_svg(repo):
    (repo / IMAGES / "kept.svg").write_text("changed\n")
    assert run_gate(repo).returncode != 0


def test_gate_fails_on_an_svg_render_py_removed(repo):
    # render.py deleting an orphan leaves a tracked SVG missing.
    (repo / IMAGES / "kept.svg").unlink()
    assert run_gate(repo).returncode != 0


def test_gate_fails_when_git_status_fails(tmp_path, monkeypatch):
    # The gate reads drift from git's stdout. If git itself errors (a bad
    # option, no repository), stdout is empty, which must not read as clean.
    work = tmp_path / "not-a-repo"
    work.mkdir()
    monkeypatch.setenv("GIT_CEILING_DIRECTORIES", str(tmp_path))
    assert run_gate(work).returncode != 0


def test_gate_ignores_png_drift(repo):
    # PNG bytes vary with the host's cairo encoder; the gate is SVG-only.
    (repo / IMAGES / "kept.png").write_text("re-encoded\n")
    _touch(repo / IMAGES, "extra.png")
    result = run_gate(repo)
    assert result.returncode == 0, result.stdout + result.stderr


# --- the PNG notice beside it (#1091) ---------------------------------------

PNG_STEP = "Surface incidental PNG drift"


def test_png_notice_is_quiet_on_the_committed_state(repo):
    result = run_gate(repo, PNG_STEP)
    assert result.returncode == 0, result.stdout + result.stderr
    assert "::notice::" not in result.stdout


@pytest.mark.parametrize("change", ["modified", "untracked"])
def test_png_notice_reports_drift_without_failing(repo, change):
    if change == "modified":
        (repo / IMAGES / "kept.png").write_text("re-encoded\n")
    else:
        _touch(repo / IMAGES, "new.png")
    result = run_gate(repo, PNG_STEP)
    assert result.returncode == 0, result.stdout + result.stderr
    assert "::notice::" in result.stdout


def test_png_notice_fails_when_git_status_fails(tmp_path, monkeypatch):
    work = tmp_path / "not-a-repo"
    work.mkdir()
    monkeypatch.setenv("GIT_CEILING_DIRECTORIES", str(tmp_path))
    assert run_gate(work, PNG_STEP).returncode != 0
