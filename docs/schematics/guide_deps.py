"""Which Typst documents embed a rendered schematic, and which are now stale.

A build guide that embeds ``docs/schematics/images/<name>.png`` compiles that
image into its committed PDF, so re-rendering a circuit invalidates the PDF —
but nothing in ``render.py`` knows the guide exists. Without this module the
only thing connecting the two is ``build-guide-check.yml`` failing in CI after
the push (issue #595).

Documents are discovered the same way the drift guard discovers them —
``git ls-files ':(glob)**/docs/*.typ'``, so ``docs/auto/pin_defs.typ`` is
never mistaken for a document — plus untracked files, so a guide that is not
committed yet is still covered. Nothing here is hardcoded to one project: a
second guide that embeds a schematic is picked up by reading its source.

Usage (from ``docs/schematics``)::

    uv run python guide_deps.py projects   # every project with an embedding document
    uv run python guide_deps.py hint       # name stale documents after a render
"""

from __future__ import annotations

import re
import subprocess
import sys
from pathlib import Path

IMAGES_DIR = "docs/schematics/images"

# Matches both a path relative to the document ("../../../../docs/schematics/
# images/x.png") and one relative to the Typst root ("/docs/schematics/...").
_IMAGE_REF = re.compile(r"docs/schematics/images/([A-Za-z0-9_.-]+)")

_DOCUMENT_GLOB = ":(glob)**/docs/*.typ"


def _git_lines(repo: Path, *args: str) -> list[str]:
    out = subprocess.run(
        ["git", "-C", str(repo), *args],
        check=True,
        capture_output=True,
        text=True,
        encoding="utf-8",
    ).stdout
    return [line for line in out.splitlines() if line]


def embedded_images(typ_text: str) -> set[str]:
    """Basenames of every rendered schematic a Typst source references."""
    return set(_IMAGE_REF.findall(typ_text))


def project_dir(document: str) -> str:
    """``packages/x/y/docs/guide.typ`` -> ``packages/x/y``; ``docs/x.typ`` -> ``.``.

    The discovery glob's leading ``**/`` also matches zero directories, so a
    document directly under the repo root's ``docs/`` belongs to the root.
    """
    if "/docs/" not in document:
        return "."
    return document.rsplit("/docs/", 1)[0]


def embedding_documents(repo: Path) -> dict[str, set[str]]:
    """Map each Typst document that embeds a schematic to the images it embeds."""
    documents = _git_lines(
        repo, "ls-files", "--cached", "--others", "--exclude-standard", _DOCUMENT_GLOB
    )
    found: dict[str, set[str]] = {}
    for document in sorted(set(documents)):
        path = repo / document
        # --cached still lists a tracked document deleted from the work tree.
        if not path.is_file():
            continue
        images = embedded_images(path.read_text(encoding="utf-8"))
        if images:
            found[document] = images
    return found


def changed_images(repo: Path) -> set[str]:
    """Rendered images that differ from HEAD: staged, unstaged or untracked."""
    paths = _git_lines(repo, "diff", "--name-only", "HEAD", "--", IMAGES_DIR)
    paths += _git_lines(
        repo, "ls-files", "--others", "--exclude-standard", "--", IMAGES_DIR
    )
    return {Path(p).name for p in paths}


def stale_projects(repo: Path) -> list[str]:
    """Projects with a document embedding an image that changed since HEAD."""
    changed = changed_images(repo)
    return sorted(
        {
            project_dir(doc)
            for doc, images in embedding_documents(repo).items()
            if images & changed
        }
    )


def _repo_root() -> Path:
    here = Path(__file__).resolve().parent
    return Path(_git_lines(here, "rev-parse", "--show-toplevel")[0])


def main(argv: list[str]) -> int:
    if len(argv) != 1 or argv[0] not in ("projects", "hint"):
        print("usage: guide_deps.py projects|hint", file=sys.stderr)
        return 2
    repo = _repo_root()
    if argv[0] == "projects":
        for project in sorted({project_dir(doc) for doc in embedding_documents(repo)}):
            print(project)
        return 0
    stale = stale_projects(repo)
    if stale:
        print()
        print("These projects' Typst documents embed a re-rendered schematic, so their")
        print("committed PDFs are now stale and build-guide-check.yml will fail:")
        for project in stale:
            print(f"  {project}")
        print("Recompile them with `just schematics::render-all` (or each project's")
        print("own `build-guide` recipe) and commit the PDFs with the images.")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
