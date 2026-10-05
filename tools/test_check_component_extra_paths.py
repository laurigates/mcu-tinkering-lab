"""Tests for check-component-extra-paths.py.

    uv run --no-project --with pytest==9.1.1 python -m pytest -q tools/test_check_component_extra_paths.py

Most tests build a small packages/ tree under tmp_path. The last two run the
check against the real repository, so a matrix edit that drops a consumer's
entry fails here as well as in the pre-commit hook.
"""

from __future__ import annotations

import copy
import importlib.util
import json
import sys
from pathlib import Path

import pytest

_SPEC = importlib.util.spec_from_file_location(
    "check_component_extra_paths",
    Path(__file__).with_name("check-component-extra-paths.py"),
)
cce = importlib.util.module_from_spec(_SPEC)
assert _SPEC.loader is not None
# Registered before exec: @dataclass resolves its module through sys.modules.
sys.modules[_SPEC.name] = cce
_SPEC.loader.exec_module(cce)

REGISTER = 'idf_component_register(SRCS "x.c" INCLUDE_DIRS "." {requires})\n'


def write(path: Path, text: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(text, encoding="utf-8")


def component(root: Path, rel: str, requires: str = "") -> None:
    write(
        root / "packages" / rel / "CMakeLists.txt", REGISTER.format(requires=requires)
    )


def project(root: Path, rel: str, top: str, main_requires: str = "") -> None:
    write(root / "packages" / rel / "CMakeLists.txt", top)
    component(root, f"{rel}/main", main_requires)


@pytest.fixture
def tree(tmp_path: Path) -> Path:
    """packages/components/{a,b,c} (c requires b), plus packages/robo/components/proto."""
    component(tmp_path, "components/a")
    component(tmp_path, "components/b")
    component(tmp_path, "components/c", "REQUIRES b")
    write(tmp_path / "packages/components/README.md", "not a component\n")
    component(tmp_path, "robo/components/proto")
    return tmp_path


SHARED = 'set(EXTRA_COMPONENT_DIRS "${CMAKE_CURRENT_LIST_DIR}/../../components")\n'


def names(root: Path, project_path: str) -> list[str]:
    comps, problems = cce.consumed_components(root, project_path)
    assert problems == []
    return sorted(c.relative_to(root / "packages").as_posix() for c in comps)


def test_a_directory_of_components_contributes_every_component(tree: Path) -> None:
    project(tree, "robo/car", SHARED)
    assert names(tree, "robo/car") == ["components/a", "components/b", "components/c"]


def test_list_append_adds_to_set_and_relative_paths_resolve_to_the_project(
    tree: Path,
) -> None:
    top = SHARED + 'list(APPEND EXTRA_COMPONENT_DIRS "../components")\n'
    project(tree, "robo/car", top)
    assert names(tree, "robo/car") == [
        "components/a",
        "components/b",
        "components/c",
        "robo/components/proto",
    ]


def test_a_later_set_replaces_the_list(tree: Path) -> None:
    top = SHARED + 'set(EXTRA_COMPONENT_DIRS "../components")\n'
    project(tree, "robo/car", top)
    assert names(tree, "robo/car") == ["robo/components/proto"]


def test_an_entry_that_is_itself_a_component_contributes_only_itself(
    tree: Path,
) -> None:
    project(tree, "robo/car", "set(EXTRA_COMPONENT_DIRS ../../components/a)\n")
    assert names(tree, "robo/car") == ["components/a"]


def test_multiline_set_with_comments_and_in_project_dirs(tree: Path) -> None:
    top = (
        "set(EXTRA_COMPONENT_DIRS\n"
        '    "components"            # in-project: under the project path already\n'
        "    # ../../components/a    (commented out)\n"
        '    "${CMAKE_CURRENT_SOURCE_DIR}/../../components/b"\n'
        ")\n"
    )
    project(tree, "robo/car", top)
    component(tree, "robo/car/components/local")
    assert names(tree, "robo/car") == ["components/b"]


def test_no_extra_component_dirs_means_nothing_out_of_tree(tree: Path) -> None:
    project(tree, "robo/car", "project(car)\n")
    assert names(tree, "robo/car") == []


def test_a_trimmed_build_compiles_only_the_requires_closure(tree: Path) -> None:
    top = SHARED + "idf_build_set_property(MINIMAL_BUILD ON)\n"
    project(
        tree, "robo/car", top, main_requires='REQUIRES "c" esp_wifi PRIV_REQUIRES log'
    )
    # c requires b; a is found but never required, so a trimmed build skips it.
    assert names(tree, "robo/car") == ["components/b", "components/c"]


def test_requires_ignores_names_that_are_not_shared_components() -> None:
    text = REGISTER.format(
        requires='REQUIRES "esp_wifi" improv-wifi\n# REQUIRES ghost\nPRIV_REQUIRES log'
    )
    assert cce.required_names(text) == {"esp_wifi", "improv-wifi", "log"}


def test_an_unknown_variable_is_reported_not_guessed(tree: Path) -> None:
    project(
        tree,
        "robo/car",
        'set(EXTRA_COMPONENT_DIRS "$ENV{X}/c" "${SOMEWHERE}/components")\n',
    )
    _, problems = cce.consumed_components(tree, "robo/car")
    assert len(problems) == 2
    assert "$ENV{X}/c" in problems[0] and "${SOMEWHERE}/components" in problems[1]


def test_a_missing_out_of_tree_dir_is_reported(tree: Path) -> None:
    project(tree, "robo/car", 'set(EXTRA_COMPONENT_DIRS "../gone")\n')
    _, problems = cce.consumed_components(tree, "robo/car")
    assert problems and "robo/gone" in problems[0] and "does not exist" in problems[0]


def matrix_entry(
    path: str, extra: list[str] | None = None, system: str = "esp32"
) -> dict:
    entry = {"system": system, "project": path.replace("/", "-"), "path": path}
    if extra is not None:
        entry["extra_paths"] = extra
    return entry


def test_check_names_the_project_and_each_missed_dir(tree: Path) -> None:
    project(tree, "robo/car", SHARED)
    findings = cce.check(tree, [matrix_entry("robo/car", ["components/a"])])
    assert [str(f) for f in findings] == [
        'robo-car: extra_paths misses "components/b", which it compiles',
        'robo-car: extra_paths misses "components/c", which it compiles',
    ]


def test_an_ancestor_entry_covers_its_components(tree: Path) -> None:
    # Same prefix test build.yml's jq applies: packages/<entry>/ prefixes the file.
    project(tree, "robo/car", SHARED)
    assert cce.check(tree, [matrix_entry("robo/car", ["components"])]) == []
    assert cce.check(tree, [matrix_entry("robo/car", ["components/"])]) == []


def test_a_sibling_with_a_shared_prefix_does_not_cover(tree: Path) -> None:
    # "components/a" must not cover a component named "components/ab".
    component(tree, "components/ab")
    project(tree, "robo/car", "set(EXTRA_COMPONENT_DIRS ../../components/ab)\n")
    findings = cce.check(tree, [matrix_entry("robo/car", ["components/a"])])
    assert [f.message for f in findings] == [
        'extra_paths misses "components/ab", which it compiles'
    ]


def test_non_esp_idf_systems_are_skipped(tree: Path) -> None:
    assert cce.check(tree, [matrix_entry("robo/nothing-here", system="esphome")]) == []


def test_main_exit_codes(tree: Path, monkeypatch: pytest.MonkeyPatch, capsys) -> None:
    project(tree, "robo/car", SHARED)
    write(tree / cce.MATRIX, json.dumps([matrix_entry("robo/car", ["components"])]))
    monkeypatch.setattr(cce, "REPO_ROOT", tree)
    assert cce.main([]) == 0
    write(tree / cce.MATRIX, json.dumps([matrix_entry("robo/car", [])]))
    assert cce.main([]) == 1
    assert "robo-car" in capsys.readouterr().err


REAL_MATRIX = json.loads((cce.REPO_ROOT / cce.MATRIX).read_text(encoding="utf-8"))


def test_the_real_matrix_covers_every_consumer() -> None:
    assert [str(f) for f in cce.check(cce.REPO_ROOT, REAL_MATRIX)] == []


def test_the_real_matrix_fails_when_a_consumer_entry_is_dropped() -> None:
    # Control: the check above must be able to fail on the real tree.
    matrix = copy.deepcopy(REAL_MATRIX)
    brainbox = next(e for e in matrix if e["project"] == "thinkpack-brainbox")
    brainbox["extra_paths"].remove("components/improv-wifi")
    assert [str(f) for f in cce.check(cce.REPO_ROOT, matrix)] == [
        'thinkpack-brainbox: extra_paths misses "components/improv-wifi", which it compiles'
    ]
