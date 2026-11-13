"""Every module in the package imports.

``ty`` does not report an import of a module that no longer exists
(``import-not-found`` is ignored in pyproject.toml), so this is the check that
nothing kept imports something removed. The plant and the robot model must
import on the core dependencies. The legacy camera, visualiser and main entry
point need third-party packages from the ``viz`` extra and are skipped where
those are not installed, which is everywhere CI runs. For those, and for
imports inside functions, ``test_imports_within_the_package_resolve`` reads
the source instead of running it.
"""

import ast
import importlib
import subprocess
import sys
from pathlib import Path

import pytest

import robocar_sim

# Module name -> the viz-extra packages it imports at module level.
MODULES = {
    "arena": (),
    "error_handling": (),
    "motor_controller": (),
    "plant": (),
    "plant_demo": (),
    "robot_model": (),
    "camera_simulation": ("cv2", "spatialmath"),
    "genesis_visualizer": ("matplotlib",),
    "main": ("matplotlib",),
}


def test_every_module_is_listed():
    on_disk = {p.stem for p in Path(robocar_sim.__file__).parent.glob("*.py")} - {"__init__"}

    assert on_disk == set(MODULES)


@pytest.mark.parametrize("name", sorted(MODULES))
def test_module_imports(name):
    for package in MODULES[name]:
        pytest.importorskip(package, reason=f"{package} is in the viz extra")

    importlib.import_module(f"robocar_sim.{name}")


def test_the_legacy_demo_without_the_viz_extra_says_so_in_one_line():
    """The console scripts and ``python -m robocar_sim.main`` are installed in
    every environment, the core-only one the twin's plant runs in included.
    There they name the extra instead of ending in a traceback. matplotlib is
    hidden here, so this runs the same with the extra installed."""
    code = (
        "import runpy, sys\n"
        "sys.modules['matplotlib'] = None\n"
        "sys.argv = ['robocar_sim.main', '--headless']\n"
        "runpy.run_module('robocar_sim.main', run_name='__main__')\n"
    )

    done = subprocess.run(
        [sys.executable, "-c", code], capture_output=True, text=True, timeout=60, check=False
    )

    assert done.returncode == 1
    assert "Traceback" not in done.stderr
    (line,) = done.stderr.strip().splitlines()
    assert "matplotlib" in line and "--extra viz" in line


def _module_level_names(tree: ast.Module) -> set[str]:
    """Names a module binds outside any function or class body."""
    names: set[str] = set()
    pending: list[ast.stmt] = list(tree.body)
    while pending:
        node = pending.pop()
        if isinstance(node, ast.FunctionDef | ast.AsyncFunctionDef | ast.ClassDef):
            names.add(node.name)
            continue
        if isinstance(node, ast.Import | ast.ImportFrom):
            names.update((a.asname or a.name).split(".")[0] for a in node.names)
            continue
        if isinstance(node, ast.Assign | ast.AnnAssign | ast.AugAssign):
            targets = node.targets if isinstance(node, ast.Assign) else [node.target]
            names.update(n.id for t in targets for n in ast.walk(t) if isinstance(n, ast.Name))
            continue
        # if / try / with / for / while: what their bodies bind is module level too.
        for field in ("body", "orelse", "finalbody"):
            pending.extend(getattr(node, field, []))
        for handler in getattr(node, "handlers", []):
            pending.extend(handler.body)
    return names


def _package_imports(tree: ast.Module) -> list[tuple[int, str | None, list[str]]]:
    """(line, sibling module or None for the package, names) of every import
    from this package, at any depth: ``from .x import a`` and
    ``from robocar_sim.x import a``."""
    found = []
    for node in ast.walk(tree):
        if not isinstance(node, ast.ImportFrom):
            continue
        module = node.module or ""
        if node.level == 1:
            sibling = module or None
        elif node.level == 0 and module.split(".")[0] == "robocar_sim":
            sibling = module.partition(".")[2] or None
        else:
            continue
        found.append((node.lineno, sibling, [a.name for a in node.names]))
    return found


@pytest.mark.parametrize("name", sorted(MODULES))
def test_imports_within_the_package_resolve(name):
    """Every ``from .module import name`` names a module that is there and a
    name that module binds. It parses the source, so it needs none of a
    module's third-party packages and covers the three the ``viz`` extra
    keeps ``test_module_imports`` from running."""
    package = Path(robocar_sim.__file__).parent
    trees = {p.stem: ast.parse(p.read_text(encoding="utf-8")) for p in package.glob("*.py")}

    for line, sibling, names in _package_imports(trees[name]):
        where = f"{name}.py:{line}"
        if sibling is None:
            missing = [n for n in names if n not in trees]
            assert not missing, f"{where}: no module {missing} in robocar_sim"
            continue
        assert sibling in trees, f"{where}: no module robocar_sim.{sibling}"
        bound = _module_level_names(trees[sibling])
        missing = [n for n in names if n != "*" and n not in bound]
        assert not missing, f"{where}: robocar_sim.{sibling} does not define {missing}"
