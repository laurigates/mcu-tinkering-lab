"""Fetch a released firmware ELF when no local copy matches an event's build.

build-firmware.yml attaches `robocar.elf` to each `robocar-v<version>` release
(`elfAssetName` in flasher.json). A crash from a released build can then be
symbolicated without anyone having run `just glitchtip::keep-elf`. The
downloaded ELF is kept only if the build SHA embedded in it is exactly the
event's `build_sha`: every release rebuilds every project at the triggering
commit, so the version alone does not name the build, and a mismatched ELF
would resolve addresses to the wrong lines.

Releases 0.2.18 to 0.2.23 are tagged `robocar-unified-v<version>` and carry
`robocar-unified.elf` (earlier ones carry no ELF). The default template does
not reach them, so a crash from that firmware arrives unsymbolicated unless its
ELF was kept by hand (`just glitchtip::keep-elf <path>`) or ELF_RELEASE_URL
names the old template.
"""

from __future__ import annotations

import logging
import re
import urllib.error
import urllib.request
from collections.abc import Callable
from pathlib import Path

log = logging.getLogger(__name__)

DEFAULT_URL_TEMPLATE = (
    "https://github.com/laurigates/mcu-tinkering-lab/releases/download/"
    "robocar-v{version}/robocar.elf"
)

# ROBOCAR_BUILD_SHA as the firmware embeds it: 40 hex, maybe "-dirty", NUL-ended.
_SHA_RE = re.compile(rb"(?<![0-9a-f])([0-9a-f]{40}(?:-dirty)?)\x00")

Getter = Callable[[str], bytes | None]


def embedded_shas(data: bytes) -> set[str]:
    """The build SHA strings embedded in an ELF image (the same scan keep-elf uses)."""
    return {m.decode() for m in _SHA_RE.findall(data)}


def release_version(event: dict) -> str | None:
    """The firmware version an event names: tags.version, else release after '@'."""
    version = (event.get("tags") or {}).get("version")
    if isinstance(version, str) and version:
        return version
    release = event.get("release")
    if isinstance(release, str) and "@" in release:
        return release.split("@", 1)[1] or None
    return None


def http_get(url: str, timeout: float = 30.0) -> bytes | None:
    """GET `url`; None on 404, raise on anything else unexpected."""
    try:
        with urllib.request.urlopen(url, timeout=timeout) as resp:
            return resp.read()
    except urllib.error.HTTPError as exc:
        if exc.code == 404:
            return None
        raise


def fetch_release_elf(
    sha: str,
    version: str | None,
    directory: Path,
    template: str,
    get: Getter = http_get,
) -> Path | None:
    """Download the release ELF for `version` into `<directory>/<sha>.elf`.

    Returns the path, or None when fetching is disabled (empty template), the
    build is local (`-dirty`) or unversioned, the release has no ELF, the
    download fails, or the ELF embeds a different build SHA.
    """
    if not template or not version or sha.endswith("-dirty"):
        return None
    url = template.format(version=version)
    try:
        data = get(url)
    except (OSError, urllib.error.URLError) as exc:
        log.warning("fetching %s failed: %s", url, exc)
        return None
    if data is None:
        log.info("no release ELF at %s", url)
        return None
    found = embedded_shas(data)
    if found != {sha}:
        log.warning(
            "release ELF %s embeds %s, not %s: not used",
            url,
            ", ".join(sorted(found)) or "no build SHA",
            sha,
        )
        return None
    directory.mkdir(parents=True, exist_ok=True)
    dest = directory / f"{sha}.elf"
    tmp = dest.with_suffix(".elf.part")
    tmp.write_bytes(data)
    tmp.replace(dest)
    log.info("kept release ELF %s as %s", url, dest)
    return dest
