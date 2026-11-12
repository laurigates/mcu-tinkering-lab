"""Tests for check-component-extra-paths.py.

    uv run --no-project --with pytest==9.1.1 python -m pytest -q tools/test_check_component_extra_paths.py

Most tests build a small packages/ tree under tmp_path. The ones named `real`
run the check against the real repository, so a matrix edit that drops a
consumer's entry fails here as well as in the pre-commit hook.
"""  # noqa: E501

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


IDF_PROJECT = "include($ENV{IDF_PATH}/tools/cmake/project.cmake)\nproject(x)\n"


def test_a_nested_project_keeps_its_components_to_itself(tree: Path) -> None:
    top = SHARED + "idf_build_set_property(MINIMAL_BUILD ON)\n"
    project(tree, "robo/car", top + IDF_PROJECT, main_requires="REQUIRES b")
    # robo/car/sub is a project of its own; its main/ requires a.
    project(tree, "robo/car/sub", IDF_PROJECT, main_requires="REQUIRES a")
    assert cce.nested_project_dirs(tree / "packages/robo/car") == [
        tree / "packages/robo/car/sub"
    ]
    assert names(tree, "robo/car") == ["components/b"]
    # Control: the same directory with no project.cmake include is one of
    # robo/car's own components, and its REQUIRES count.
    write(tree / "packages/robo/car/sub/CMakeLists.txt", "# not a project\n")
    assert cce.nested_project_dirs(tree / "packages/robo/car") == []
    assert names(tree, "robo/car") == ["components/a", "components/b"]


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


def test_an_exclude_path_must_be_a_directory_inside_the_project(tree: Path) -> None:
    project(tree, "robo/car", "project(car)\n")
    project(tree, "robo/car/bringup", "project(bringup)\n")
    entry = matrix_entry("robo/car")
    entry["exclude_paths"] = ["robo/car/bringup", "robo/car/bringup/"]
    assert cce.check_exclude_paths(tree, [entry]) == []
    # A package-relative name, a sibling sharing the prefix, the project
    # itself and a directory that is not there: none can exclude a nested file.
    entry["exclude_paths"] = ["bringup", "robo/carpet", "robo/car", "robo/car/gone"]
    assert [str(f) for f in cce.check_exclude_paths(tree, [entry])] == [
        'robo-car: exclude_paths entry "bringup" is not inside its path "robo/car"',
        'robo-car: exclude_paths entry "robo/carpet" is not inside its path "robo/car"',
        'robo-car: exclude_paths entry "robo/car" is not inside its path "robo/car"',
        'robo-car: exclude_paths entry "robo/car/gone" is not a directory under packages/',
    ]


def test_main_fails_on_an_exclude_path_that_excludes_nothing(
    tree: Path, monkeypatch: pytest.MonkeyPatch, capsys
) -> None:
    project(tree, "robo/car", "project(car)\n")
    entry = matrix_entry("robo/car", [])
    entry["exclude_paths"] = ["robo/car/main"]
    write(tree / cce.MATRIX, json.dumps([entry]))
    write(tree / cce.JUSTFILE, "mod car 'packages/robo/car'\n")
    monkeypatch.setattr(cce, "REPO_ROOT", tree)
    monkeypatch.setattr(cce, "NOT_IN_MATRIX", {})
    monkeypatch.setattr(cce, "NESTED_NOT_EXCLUDED", {})
    assert cce.main([]) == 0
    entry["exclude_paths"] = ["robo/car/twin"]
    write(tree / cce.MATRIX, json.dumps([entry]))
    assert cce.main([]) == 1
    assert 'exclude_paths entry "robo/car/twin"' in capsys.readouterr().err


def test_main_exit_codes(tree: Path, monkeypatch: pytest.MonkeyPatch, capsys) -> None:
    project(tree, "robo/car", SHARED)
    write(tree / cce.MATRIX, json.dumps([matrix_entry("robo/car", ["components"])]))
    write(tree / cce.JUSTFILE, "mod car 'packages/robo/car'\n")
    monkeypatch.setattr(cce, "REPO_ROOT", tree)
    monkeypatch.setattr(cce, "NOT_IN_MATRIX", {})
    monkeypatch.setattr(cce, "NESTED_NOT_EXCLUDED", {})
    assert cce.main([]) == 0
    write(tree / cce.MATRIX, json.dumps([matrix_entry("robo/car", [])]))
    assert cce.main([]) == 1
    assert "robo-car" in capsys.readouterr().err


def nested_findings(
    modules: dict[str, str], matrix: list[dict], allowed: dict[str, str]
) -> list[str]:
    return [str(f) for f in cce.check_nested(modules, matrix, allowed)]


def test_a_nested_project_needs_an_exclude_or_a_reason() -> None:
    car = matrix_entry("robo/car")
    modules = {"car": "packages/robo/car", "sub": "packages/robo/car/sub"}
    missing = (
        'robo-car: the project at "robo/car/sub" is inside its path but not in '
        "its exclude_paths, so a change there rebuilds it"
    )
    # Registered in the root justfile only, and as a matrix entry only.
    assert nested_findings(modules, [car], {}) == [missing]
    assert nested_findings({}, [car, matrix_entry("robo/car/sub")], {}) == [missing]
    # A sibling sharing the prefix is not nested.
    assert nested_findings({"pet": "packages/robo/carpet"}, [car], {}) == []
    assert nested_findings(modules, [car], {"robo/car/sub": "on purpose"}) == []
    car["exclude_paths"] = ["robo/car/sub/"]
    assert nested_findings(modules, [car], {}) == []


def test_a_nested_exemption_needs_a_reason_and_goes_stale() -> None:
    car = matrix_entry("robo/car")
    modules = {"car": "packages/robo/car", "sub": "packages/robo/car/sub"}
    assert nested_findings(modules, [car], {"robo/car/sub": " "}) == [
        "robo/car/sub: NESTED_NOT_EXCLUDED entry has no reason"
    ]
    assert nested_findings({}, [car], {"robo/car/sub": "on purpose"}) == [
        "robo/car/sub: NESTED_NOT_EXCLUDED names it, but it is not a project "
        "inside a matrix entry's path"
    ]
    car["exclude_paths"] = ["robo/car/sub"]
    assert nested_findings(modules, [car], {"robo/car/sub": "on purpose"}) == [
        "robo/car/sub: is in an exclude_paths list, so its NESTED_NOT_EXCLUDED "
        "entry is stale"
    ]


def release_findings(matrix: list[dict], packages: dict) -> list[str]:
    return [str(f) for f in cce.check_release_excludes(matrix, packages)]


def test_release_please_excludes_the_same_directories() -> None:
    car = matrix_entry("robo/car")
    car["exclude_paths"] = ["robo/car/sub", "robo/car/twin/"]
    both = ["packages/robo/car/twin", "packages/robo/car/sub"]
    assert release_findings([car], {"packages/robo/car": {"exclude-paths": both}}) == []
    # A project release-please does not release needs no list.
    assert release_findings([car], {"packages/robo/boat": {}}) == []
    assert release_findings([car], {"packages/robo/car": {}}) == [
        "robo-car: release-please-config.json exclude-paths [] differ from its "
        "exclude_paths, which need ['packages/robo/car/sub', 'packages/robo/car/twin']"
    ]
    # Relative to the package, as project-matrix.json writes them: refused.
    assert (
        release_findings(
            [car], {"packages/robo/car": {"exclude-paths": ["sub", "twin"]}}
        )
        != []
    )
    # A release-please list with no matrix exclude_paths behind it.
    assert release_findings(
        [matrix_entry("robo/car")], {"packages/robo/car": {"exclude-paths": both[:1]}}
    ) == [
        "robo-car: release-please-config.json exclude-paths "
        "['packages/robo/car/twin'] differ from its exclude_paths, which need []"
    ]


def test_main_fails_on_a_nested_project_and_on_a_release_please_gap(
    tree: Path, monkeypatch: pytest.MonkeyPatch, capsys
) -> None:
    project(tree, "robo/car", "project(car)\n")
    project(tree, "robo/car/sub", "project(sub)\n")
    entry = matrix_entry("robo/car", [])
    entry["exclude_paths"] = ["robo/car/sub"]
    justfile = "mod car 'packages/robo/car'\nmod sub 'packages/robo/car/sub'\n"
    release = {
        "packages": {"packages/robo/car": {"exclude-paths": ["packages/robo/car/sub"]}}
    }
    write(tree / cce.MATRIX, json.dumps([entry]))
    write(tree / cce.JUSTFILE, justfile)
    write(tree / cce.RELEASE_CONFIG, json.dumps(release))
    monkeypatch.setattr(cce, "REPO_ROOT", tree)
    monkeypatch.setattr(cce, "NOT_IN_MATRIX", {"robo/car/sub": "host build"})
    monkeypatch.setattr(cce, "NESTED_NOT_EXCLUDED", {})
    assert cce.main([]) == 0
    capsys.readouterr()

    release["packages"]["packages/robo/car"].pop("exclude-paths")
    write(tree / cce.RELEASE_CONFIG, json.dumps(release))
    assert cce.main([]) == 1
    assert "release-please-config.json exclude-paths []" in capsys.readouterr().err

    entry.pop("exclude_paths")
    write(tree / cce.MATRIX, json.dumps([entry]))
    assert cce.main([]) == 1
    assert 'the project at "robo/car/sub" is inside its path' in capsys.readouterr().err


REAL_MATRIX = json.loads((cce.REPO_ROOT / cce.MATRIX).read_text(encoding="utf-8"))


def test_the_real_matrix_covers_every_consumer() -> None:
    assert [str(f) for f in cce.check(cce.REPO_ROOT, REAL_MATRIX)] == []


def test_the_real_exclude_paths_name_directories_inside_their_project() -> None:
    assert [str(f) for f in cce.check_exclude_paths(cce.REPO_ROOT, REAL_MATRIX)] == []
    # The robocar firmware's directory holds four separate projects; each
    # must be excluded, or a change to it alone rebuilds the firmware.
    robocar = next(e for e in REAL_MATRIX if e["project"] == "robocar")
    assert robocar["exclude_paths"] == [
        "robotics/robocar/bringup",
        "robotics/robocar/integrated",
        "robotics/robocar/simulation",
        "robotics/robocar/twin",
    ]


def test_the_real_exclude_check_fails_on_a_package_relative_entry() -> None:
    # Control: the check above must be able to fail on the real tree.
    matrix = copy.deepcopy(REAL_MATRIX)
    robocar = next(e for e in matrix if e["project"] == "robocar")
    robocar["exclude_paths"][0] = "bringup"
    assert [str(f) for f in cce.check_exclude_paths(cce.REPO_ROOT, matrix)] == [
        'robocar: exclude_paths entry "bringup" is not inside its path '
        '"robotics/robocar"'
    ]


JUSTFILE_TEXT = """\
# comment: mod commented 'packages/x/commented'
mod car 'packages/robo/car'
mod? optional "packages/robo/opt"
mod  spaced   'packages/robo/spaced'  # trailing comment
mod tool 'tools/thing'
mod bare
"""


def test_registered_modules_reads_every_mod_line() -> None:
    assert cce.registered_modules(JUSTFILE_TEXT) == {
        "car": "packages/robo/car",
        "optional": "packages/robo/opt",
        "spaced": "packages/robo/spaced",
        "tool": "tools/thing",
        "bare": "bare",
    }


def registration_findings(
    modules: dict[str, str], matrix: list[dict], exclusions: dict[str, str]
) -> list[str]:
    return [str(f) for f in cce.check_registered(modules, matrix, exclusions)]


def test_a_registered_project_with_no_matrix_entry_fails() -> None:
    modules = {"car": "packages/robo/car", "boat": "packages/robo/boat"}
    assert registration_findings(modules, [matrix_entry("robo/car")], {}) == [
        "boat: packages/robo/boat is registered in the root justfile but has no "
        "entry in .github/project-matrix.json and no NOT_IN_MATRIX exclusion"
    ]


def test_every_build_system_counts_as_an_entry() -> None:
    modules = {"a": "packages/robo/a", "b": "packages/robo/b"}
    matrix = [
        matrix_entry("robo/a", system="esphome"),
        matrix_entry("robo/b", system="pico"),
    ]
    assert registration_findings(modules, matrix, {}) == []


def test_an_excluded_project_passes_and_modules_outside_packages_are_ignored() -> None:
    modules = {"twin": "packages/robo/twin", "tool": "tools/thing", "bare": "bare"}
    assert registration_findings(modules, [], {"robo/twin": "host build"}) == []


def test_a_trailing_slash_in_the_mod_path_still_matches() -> None:
    modules = {"car": "packages/robo/car/"}
    assert registration_findings(modules, [matrix_entry("robo/car")], {}) == []


def test_an_exclusion_needs_a_reason() -> None:
    modules = {"twin": "packages/robo/twin"}
    assert registration_findings(modules, [], {"robo/twin": "  "}) == [
        "robo/twin: NOT_IN_MATRIX exclusion has no reason"
    ]


def test_a_stale_exclusion_fails() -> None:
    modules = {"car": "packages/robo/car"}
    exclusions = {"robo/car": "was not buildable", "robo/gone": "removed project"}
    assert registration_findings(modules, [matrix_entry("robo/car")], exclusions) == [
        "robo/car: has a matrix entry, so its NOT_IN_MATRIX exclusion is stale",
        "robo/gone: NOT_IN_MATRIX excludes it, but no root justfile mod registers "
        "packages/robo/gone",
    ]


def test_main_fails_on_an_unbuilt_registration(
    tree: Path, monkeypatch: pytest.MonkeyPatch, capsys
) -> None:
    project(tree, "robo/car", "project(car)\n")
    write(tree / cce.MATRIX, json.dumps([matrix_entry("robo/car", [])]))
    write(tree / cce.JUSTFILE, "mod car 'packages/robo/car'\n")
    monkeypatch.setattr(cce, "REPO_ROOT", tree)
    monkeypatch.setattr(cce, "NOT_IN_MATRIX", {})
    monkeypatch.setattr(cce, "NESTED_NOT_EXCLUDED", {})
    assert cce.main([]) == 0
    write(
        tree / cce.JUSTFILE,
        "mod car 'packages/robo/car'\nmod boat 'packages/robo/boat'\n",
    )
    assert cce.main([]) == 1
    assert "boat: packages/robo/boat is registered" in capsys.readouterr().err


REAL_MODULES = cce.registered_modules(
    (cce.REPO_ROOT / cce.JUSTFILE).read_text(encoding="utf-8")
)


def test_every_real_registered_project_is_built_or_excluded() -> None:
    assert registration_findings(REAL_MODULES, REAL_MATRIX, cce.NOT_IN_MATRIX) == []


def test_the_real_registration_check_fails_when_a_matrix_entry_is_dropped() -> None:
    # Control: the check above must be able to fail on the real tree.
    matrix = [e for e in REAL_MATRIX if e["path"] != "camera-vision/gemini-vision"]
    assert registration_findings(REAL_MODULES, matrix, cce.NOT_IN_MATRIX) == [
        "gemini-vision: packages/camera-vision/gemini-vision is registered in the "
        "root justfile but has no entry in .github/project-matrix.json and no "
        "NOT_IN_MATRIX exclusion"
    ]


def test_the_real_matrix_fails_when_a_consumer_entry_is_dropped() -> None:
    # Control: the check above must be able to fail on the real tree.
    matrix = copy.deepcopy(REAL_MATRIX)
    brainbox = next(e for e in matrix if e["project"] == "thinkpack-brainbox")
    brainbox["extra_paths"].remove("components/improv-wifi")
    assert [str(f) for f in cce.check(cce.REPO_ROOT, matrix)] == [
        'thinkpack-brainbox: extra_paths misses "components/improv-wifi", which it compiles'
    ]


REAL_RELEASE_PACKAGES = json.loads(
    (cce.REPO_ROOT / cce.RELEASE_CONFIG).read_text(encoding="utf-8")
)["packages"]


def test_every_real_nested_project_is_excluded_or_exempt() -> None:
    assert nested_findings(REAL_MODULES, REAL_MATRIX, cce.NESTED_NOT_EXCLUDED) == []


def test_the_real_nested_check_fails_when_an_exclude_is_dropped() -> None:
    # Control: the check above must be able to fail on the real tree.
    matrix = copy.deepcopy(REAL_MATRIX)
    robocar = next(e for e in matrix if e["project"] == "robocar")
    robocar["exclude_paths"].remove("robotics/robocar/twin")
    assert nested_findings(REAL_MODULES, matrix, cce.NESTED_NOT_EXCLUDED) == [
        'robocar: the project at "robotics/robocar/twin" is inside its path but '
        "not in its exclude_paths, so a change there rebuilds it"
    ]


def test_real_release_please_excludes_match_the_matrix() -> None:
    assert release_findings(REAL_MATRIX, REAL_RELEASE_PACKAGES) == []
    # The comparison must have something to compare.
    robocar = REAL_RELEASE_PACKAGES["packages/robotics/robocar"]
    assert len(robocar["exclude-paths"]) == 4


def test_the_real_release_check_fails_when_the_list_is_lost() -> None:
    # Control: without the list, a bringup-only commit is released as robocar.
    packages = copy.deepcopy(REAL_RELEASE_PACKAGES)
    del packages["packages/robotics/robocar"]["exclude-paths"]
    (finding,) = release_findings(REAL_MATRIX, packages)
    assert finding.startswith(
        "robocar: release-please-config.json exclude-paths [] differ"
    )


def test_real_bringup_components_do_not_count_as_the_firmware_own() -> None:
    robocar = cce.REPO_ROOT / "packages/robotics/robocar"
    assert robocar / "bringup" in cce.nested_project_dirs(robocar)
    own = [
        c.relative_to(robocar).as_posix() for c in cce._local_component_cmakes(robocar)
    ]
    assert "main/CMakeLists.txt" in own
    assert [c for c in own if c.startswith("bringup/")] == []
