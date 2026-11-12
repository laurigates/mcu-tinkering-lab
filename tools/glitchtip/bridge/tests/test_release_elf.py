"""Release-ELF fetch: what is kept, what is refused, and how often it asks."""

from __future__ import annotations

import shutil

from robocar_glitchtip_bridge.release_elf import (
    embedded_shas,
    fetch_release_elf,
    release_version,
)
from robocar_glitchtip_bridge.symbolicate import ElfStore

SHA = "0123456789abcdef0123456789abcdef01234567"
OTHER = "fedcba9876543210fedcba9876543210fedcba98"
TEMPLATE = "https://example.invalid/robocar-v{version}/robocar.elf"


def image(*shas: str) -> bytes:
    return b"\x7fELF junk " + b"".join(s.encode() + b"\x00 pad " for s in shas)


class Recorder:
    def __init__(self, data: bytes | None) -> None:
        self.data = data
        self.urls: list[str] = []

    def __call__(self, url: str) -> bytes | None:
        self.urls.append(url)
        return self.data


def test_embedded_shas_finds_clean_and_dirty():
    assert embedded_shas(image(SHA)) == {SHA}
    assert embedded_shas(image(SHA + "-dirty")) == {SHA + "-dirty"}
    assert embedded_shas(b"x" + SHA.encode() + b"y") == set()  # not NUL-ended


def test_release_version_prefers_the_tag():
    assert release_version({"tags": {"version": "0.2.17"}}) == "0.2.17"
    # Firmware up to 0.2.23 reports itself as robocar-unified@<version>.
    assert release_version({"release": "robocar-unified@0.2.16"}) == "0.2.16"
    assert release_version({"release": "robocar@1.2.3"}) == "1.2.3"
    assert release_version({"release": "robocar@"}) is None
    assert release_version({}) is None


def test_matching_elf_is_kept_under_the_sha(tmp_path):
    get = Recorder(image(SHA))
    path = fetch_release_elf(SHA, "0.2.17", tmp_path, TEMPLATE, get)
    assert path == tmp_path / f"{SHA}.elf"
    assert path.read_bytes() == get.data
    assert get.urls == [TEMPLATE.format(version="0.2.17")]
    assert not list(tmp_path.glob("*.part"))


def test_elf_of_another_build_is_refused(tmp_path):
    assert (
        fetch_release_elf(SHA, "0.2.17", tmp_path, TEMPLATE, Recorder(image(OTHER)))
        is None
    )
    assert (
        fetch_release_elf(
            SHA, "0.2.17", tmp_path, TEMPLATE, Recorder(image(SHA, OTHER))
        )
        is None
    )
    assert not list(tmp_path.iterdir())


def test_nothing_is_fetched_when_it_cannot_match(tmp_path):
    get = Recorder(image(SHA))
    assert fetch_release_elf(SHA + "-dirty", "0.2.17", tmp_path, TEMPLATE, get) is None
    assert fetch_release_elf(SHA, None, tmp_path, TEMPLATE, get) is None
    assert fetch_release_elf(SHA, "0.2.17", tmp_path, "", get) is None
    assert get.urls == []


def test_missing_release_and_network_errors_give_none(tmp_path):
    assert fetch_release_elf(SHA, "9.9.9", tmp_path, TEMPLATE, Recorder(None)) is None

    def boom(_url: str) -> bytes:
        raise OSError("unreachable")

    assert fetch_release_elf(SHA, "0.2.17", tmp_path, TEMPLATE, boom) is None


def test_store_fetches_a_missing_sha_once(tiny_elf, tmp_path):
    calls: list[tuple[str, str | None]] = []

    def fetch(sha, version):
        calls.append((sha, version))
        return None

    store = ElfStore(tmp_path, fetch=fetch)
    assert store.table_for(SHA, "0.2.17") is None
    assert store.table_for(SHA, "0.2.17") is None
    assert calls == [(SHA, "0.2.17")]

    def fetch_ok(sha, version):
        dest = tmp_path / f"{sha}.elf"
        shutil.copy(tiny_elf, dest)
        return dest

    store = ElfStore(tmp_path / "other", fetch=fetch_ok)
    (tmp_path / "other").mkdir()
    assert store.table_for(OTHER, "0.2.17") is not None
