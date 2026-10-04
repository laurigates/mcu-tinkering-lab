"""Tests for guide_deps.py — which Typst documents embed a rendered schematic.

The repo-level tests build a throwaway git repository rather than reading this
one, so they pin the discovery rules (glob, docs/auto/ exclusion, untracked
documents, change detection) independently of what happens to be committed.
"""

from __future__ import annotations

import subprocess
from pathlib import Path

import pytest

from guide_deps import (
    changed_images,
    embedded_images,
    embedding_documents,
    project_dir,
    stale_projects,
)


def _git(repo: Path, *args: str) -> None:
    subprocess.run(["git", "-C", str(repo), *args], check=True, capture_output=True)


def _write(repo: Path, rel: str, text: str | bytes) -> None:
    path = repo / rel
    path.parent.mkdir(parents=True, exist_ok=True)
    if isinstance(text, bytes):
        path.write_bytes(text)
    else:
        path.write_text(text)


@pytest.fixture
def repo(tmp_path: Path) -> Path:
    _git(tmp_path, "init", "-q")
    _git(tmp_path, "config", "user.email", "test@example.invalid")
    _git(tmp_path, "config", "user.name", "test")
    _git(tmp_path, "config", "commit.gpgsign", "false")
    _write(tmp_path, "docs/schematics/images/robot.png", b"png-v1")
    _write(tmp_path, "docs/schematics/images/robot.svg", "<svg/>")
    _write(tmp_path, "docs/schematics/images/other.png", b"other-v1")
    _write(
        tmp_path,
        "packages/robot/docs/build-guide.typ",
        '#image("../../../docs/schematics/images/robot.png", width: 100%)\n',
    )
    # A card beside the guide that embeds no schematic.
    _write(tmp_path, "packages/robot/docs/card.typ", "= Card\n")
    # The generated include: discovery must never treat it as a document.
    _write(
        tmp_path,
        "packages/robot/docs/auto/pin_defs.typ",
        "// docs/schematics/images/other.png\n",
    )
    _git(tmp_path, "add", "-A")
    _git(tmp_path, "commit", "-q", "-m", "init")
    return tmp_path


def test_embedded_images_reads_relative_and_root_relative_paths():
    text = (
        '#image("../../../../docs/schematics/images/robocar_unified.png")\n'
        '#image("/docs/schematics/images/balancebot.svg", width: 50%)\n'
        '#image("photos/board.jpg")\n'
    )
    assert embedded_images(text) == {"robocar_unified.png", "balancebot.svg"}


def test_embedded_images_ignores_a_document_with_no_schematic():
    assert embedded_images('= Title\n#image("photo.png")\n') == set()


def test_project_dir_strips_the_docs_component():
    assert project_dir("packages/robocar/unified/docs/build-guide.typ") == (
        "packages/robocar/unified"
    )


def test_project_dir_of_a_repo_root_document_is_the_root():
    assert project_dir("docs/build-guide.typ") == "."


def test_a_repo_root_document_is_discovered_and_owned_by_the_root(repo: Path):
    _write(repo, "docs/guide.typ", '#image("schematics/images/robot.png")\n')
    _write(repo, "docs/schematics/images/robot.png", b"png-v2")
    assert "docs/guide.typ" not in embedding_documents(
        repo
    )  # no docs/schematics/ prefix
    _write(repo, "docs/guide.typ", '#image("/docs/schematics/images/robot.png")\n')
    assert embedding_documents(repo)["docs/guide.typ"] == {"robot.png"}
    assert stale_projects(repo) == [".", "packages/robot"]


def test_a_tracked_document_deleted_from_the_work_tree_is_skipped(repo: Path):
    (repo / "packages/robot/docs/build-guide.typ").unlink()
    assert embedding_documents(repo) == {}


def test_embedding_documents_skips_docs_auto_and_non_embedding_documents(repo: Path):
    assert embedding_documents(repo) == {
        "packages/robot/docs/build-guide.typ": {"robot.png"}
    }


def test_embedding_documents_sees_a_document_not_yet_committed(repo: Path):
    _write(
        repo,
        "packages/second/docs/build-guide.typ",
        '#image("../../../docs/schematics/images/other.png")\n',
    )
    assert "packages/second/docs/build-guide.typ" in embedding_documents(repo)


def test_changed_images_is_empty_on_a_clean_tree(repo: Path):
    assert changed_images(repo) == set()


def test_changed_images_reports_unstaged_staged_and_untracked(repo: Path):
    _write(repo, "docs/schematics/images/robot.png", b"png-v2")
    _write(repo, "docs/schematics/images/other.png", b"other-v2")
    _git(repo, "add", "docs/schematics/images/other.png")
    _write(repo, "docs/schematics/images/new.png", b"new")
    assert changed_images(repo) == {"robot.png", "other.png", "new.png"}


def test_stale_projects_names_only_projects_whose_embedded_image_changed(repo: Path):
    _write(repo, "docs/schematics/images/other.png", b"other-v2")
    assert stale_projects(repo) == []
    _write(repo, "docs/schematics/images/robot.png", b"png-v2")
    assert stale_projects(repo) == ["packages/robot"]
