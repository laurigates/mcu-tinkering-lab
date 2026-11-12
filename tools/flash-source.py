#!/usr/bin/env python3
"""Prepare the firmware a flash recipe writes, and prove it is the version asked for.

A flash recipe used to write whatever sat in build/, so a robot could run a
build from another branch, or from before the last pull, while you debugged the
checkout in front of you. This script decides what gets flashed and checks the
build SHA embedded in the app image (ROBOCAR_BUILD_SHA, compiled in at CMake
configure time) before anything is written:

  (no ref)        build the current checkout; the image must embed HEAD, with
                  "-dirty" when tracked files have uncommitted changes
  <commit|branch> clone the repo locally into tmp/flash/<sha>, copy the
                  gitignored main/credentials.h across, build there; the image
                  must embed exactly <sha>
  <version tag>   "v0.2.24", "0.2.24" or "<tag-prefix>0.2.24": download the
                  prebuilt parts from that GitHub release when it carries all
                  four, otherwise build the tag's commit as above; either way
                  the image must embed the tag's commit

The project directory, image stem and tag prefix given are used for every ref.
A commit or release from before one of them changed is refused: robocar's
releases up to 0.2.23 are tagged robocar-unified-v* and its commits before the
move have no packages/robotics/robocar. Flash those from a checkout of that
commit, with the recipe it carries.

It prints shell assignments for the recipe to eval, SRC_DIR (a directory with
the usual build/ layout) plus SHA and SOURCE; progress goes to stderr. The
recipe then cds into SRC_DIR and runs its own esptool line, so the files and
offsets stay literal for tools/check-flash-recipes.py.

Run:   python3 tools/flash-source.py --project-dir packages/robotics/robocar \\
           --bin robocar --tag-prefix robocar-v [REF]
Exit:  0 ready to flash; 1 on any mismatch or failure (nothing is flashed).
Tests: python3 -m unittest tools/test_flash_source.py
"""

from __future__ import annotations

import argparse
import os
import re
import shlex
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path

# ROBOCAR_BUILD_SHA as the firmware embeds it: 40 hex, maybe "-dirty", NUL-ended.
_SHA_RE = re.compile(rb"(?<![0-9a-f])([0-9a-f]{40}(?:-dirty)?)\x00")
_HEX40 = re.compile(r"^[0-9a-f]{40}$")


class FlashSourceError(Exception):
    """Anything that must stop the flash."""


# --------------------------------------------------------------- pure parts --


def embedded_sha(image: bytes) -> str:
    """The one build SHA embedded in an app image; raise unless exactly one."""
    found = {m.decode() for m in _SHA_RE.findall(image)}
    if len(found) == 1:
        return found.pop()
    if not found:
        raise FlashSourceError(
            "the image embeds no build SHA, so what it was built from cannot be "
            "checked. The CMakeLists embeds 'unknown' when git cannot read the "
            "checkout inside the build container, as with a `git worktree` (its "
            ".git points at a host path the container does not mount); build "
            "from the main checkout or flash a commit, which builds in a clone"
        )
    raise FlashSourceError(
        f"expected one embedded build SHA, found: {', '.join(sorted(found))}"
    )


def check_sha(embedded: str, expected: str, what: str) -> None:
    """Refuse to flash an image that is not the build asked for."""
    if embedded != expected:
        raise FlashSourceError(
            f"{what} embeds build {embedded}, expected {expected}; not flashing"
        )


def tag_candidates(ref: str, tag_prefix: str) -> list[str]:
    """Tag names a version ref may mean: 'v0.2.17' and '0.2.17' both map to
    '<prefix>0.2.17'; a full tag name maps to itself."""
    if ref.startswith(tag_prefix):
        return [ref]
    version = ref[1:] if ref[:1] == "v" and ref[1:2].isdigit() else ref
    if re.fullmatch(r"\d+(\.\d+)*", version):
        return [tag_prefix + version]
    return []


def release_parts(stem: str) -> dict[str, str]:
    """build/ path -> release asset name. build-firmware.yml attaches the three
    non-app parts under these names for projects with releaseFlashParts."""
    return {
        "build/bootloader/bootloader.bin": f"{stem}-bootloader.bin",
        "build/partition_table/partition-table.bin": f"{stem}-partition-table.bin",
        "build/ota_data_initial.bin": f"{stem}-ota_data_initial.bin",
        f"build/{stem}.bin": f"{stem}.bin",
    }


def assignments(src_dir: Path, sha: str, source: str) -> str:
    """Shell assignments for the recipe to eval."""
    return "\n".join(
        f"{key}={shlex.quote(value)}"
        for key, value in (("SRC_DIR", str(src_dir)), ("SHA", sha), ("SOURCE", source))
    )


# ---------------------------------------------------------------------- I/O --


def log(msg: str) -> None:
    print(f"flash-source: {msg}", file=sys.stderr, flush=True)


def git(root: Path, *args: str, check: bool = True) -> str:
    proc = subprocess.run(
        ["git", "-C", str(root), *args], capture_output=True, text=True, check=False
    )
    if check and proc.returncode != 0:
        raise FlashSourceError(f"git {' '.join(args)}: {proc.stderr.strip()}")
    return proc.stdout.strip() if proc.returncode == 0 else ""


def checkout_sha(root: Path) -> str:
    """What CMake embeds for this checkout: HEAD, "-dirty" if tracked files
    differ from it (the same `git diff --quiet HEAD --` the CMakeLists runs)."""
    head = git(root, "rev-parse", "HEAD")
    dirty = subprocess.run(
        ["git", "-C", str(root), "diff", "--quiet", "HEAD", "--"], check=False
    ).returncode
    return head + ("-dirty" if dirty else "")


def resolve_commit(root: Path, ref: str) -> str:
    sha = git(root, "rev-parse", "-q", "--verify", f"{ref}^{{commit}}", check=False)
    if not _HEX40.match(sha):
        raise FlashSourceError(
            f"{ref!r} is not a commit, branch or tag here (try git fetch)"
        )
    return sha


def run_build(project_dir: Path, env: dict[str, str] | None = None) -> None:
    log(f"building in {project_dir}")
    # stdout belongs to the assignments the recipe evals; the build talks on stderr.
    proc = subprocess.run(
        ["just", "build"], cwd=project_dir, env=env, stdout=sys.stderr, check=False
    )
    if proc.returncode != 0:
        raise FlashSourceError(f"build failed in {project_dir}")


def image_sha(src_dir: Path, stem: str) -> str:
    image = src_dir / "build" / f"{stem}.bin"
    if not image.is_file():
        raise FlashSourceError(f"no {image}")
    return embedded_sha(image.read_bytes())


def build_checkout(root: Path, project: str, stem: str) -> tuple[Path, str, str]:
    expected = checkout_sha(root)
    src = root / project
    run_build(src)
    check_sha(image_sha(src, stem), expected, f"build/{stem}.bin")
    return src, expected, "checkout"


def build_commit(
    root: Path, project: str, stem: str, sha: str
) -> tuple[Path, str, str]:
    clone = root / "tmp" / "flash" / sha
    if not (clone / ".git").is_dir():
        log(f"cloning {sha[:12]} into {clone}")
        clone.parent.mkdir(parents=True, exist_ok=True)
        # --local copies .git into the clone, so git works inside the build
        # container (a worktree's .git points at a host path it cannot see,
        # and the image would embed "unknown").
        git(root, "clone", "-q", "--local", "--no-checkout", str(root), str(clone))
    git(clone, "checkout", "-q", "--detach", sha)
    if not (clone / project).is_dir():
        raise FlashSourceError(
            f"{sha[:12]} has no {project}: the commit predates the project's "
            "current directory, so this recipe cannot build it; flash it from a "
            "checkout of that commit"
        )
    creds = root / project / "main" / "credentials.h"
    if creds.is_file():
        shutil.copy2(creds, clone / project / "main" / "credentials.h")
    # Share the main checkout's compose project, and so its ESP-IDF tools volume.
    env = {
        **os.environ,
        "COMPOSE_PROJECT_NAME": os.environ.get("COMPOSE_PROJECT_NAME", root.name),
    }
    src = clone / project
    run_build(src, env)
    check_sha(image_sha(src, stem), sha, f"the build of {sha[:12]}")
    return src, sha, f"built {sha[:12]}"


def fetch_release(root: Path, tag: str, stem: str, sha: str) -> Path | None:
    """Stage the release's parts in tmp/flash/<tag>/build/, or None when the
    release lacks any of them (releases before releaseFlashParts)."""
    parts = release_parts(stem)
    listed = subprocess.run(
        ["gh", "release", "view", tag, "--json", "assets", "--jq", ".assets[].name"],
        cwd=root,
        capture_output=True,
        text=True,
        check=False,
    )
    if listed.returncode != 0:
        log(f"no release {tag} on GitHub ({listed.stderr.strip()})")
        return None
    names = set(listed.stdout.split())
    missing = sorted(set(parts.values()) - names)
    if missing:
        log(f"release {tag} lacks {', '.join(missing)}; building {sha[:12]} instead")
        return None
    stage = root / "tmp" / "flash" / tag
    with tempfile.TemporaryDirectory() as tmp:
        for name in parts.values():
            subprocess.run(
                ["gh", "release", "download", tag, "-p", name, "-D", tmp, "--clobber"],
                cwd=root,
                check=True,
            )
        for rel, name in parts.items():
            dest = stage / rel
            dest.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(Path(tmp) / name, dest)
    check_sha(image_sha(stage, stem), sha, f"release {tag}")
    return stage


def prepare(
    root: Path, project: str, stem: str, tag_prefix: str, ref: str
) -> tuple[Path, str, str]:
    if not ref:
        return build_checkout(root, project, stem)
    subprocess.run(
        ["git", "-C", str(root), "fetch", "-q", "--tags", "origin"], check=False
    )
    for tag in tag_candidates(ref, tag_prefix):
        sha = git(
            root,
            "rev-parse",
            "-q",
            "--verify",
            f"refs/tags/{tag}^{{commit}}",
            check=False,
        )
        if _HEX40.match(sha):
            stage = fetch_release(root, tag, stem, sha)
            if stage is not None:
                return stage, sha, f"release {tag}"
            return build_commit(root, project, stem, sha)
    return build_commit(root, project, stem, resolve_commit(root, ref))


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=(__doc__ or "").splitlines()[0])
    parser.add_argument(
        "--project-dir",
        required=True,
        help="repo-relative, e.g. packages/robotics/robocar",
    )
    parser.add_argument("--bin", required=True, help="app image stem, e.g. robocar")
    parser.add_argument(
        "--tag-prefix", required=True, help="release tag prefix, e.g. robocar-v"
    )
    parser.add_argument(
        "ref",
        nargs="?",
        default="",
        help="commit, branch or version tag; empty = checkout",
    )
    args = parser.parse_args(argv)
    root = Path(git(Path.cwd(), "rev-parse", "--show-toplevel"))
    try:
        src, sha, source = prepare(
            root, args.project_dir, args.bin, args.tag_prefix, args.ref
        )
    except (FlashSourceError, subprocess.CalledProcessError) as exc:
        log(f"ERROR: {exc}")
        return 1
    log(f"ready: {sha} from {source}")
    print(assignments(src, sha, source))
    return 0


if __name__ == "__main__":
    sys.exit(main())
