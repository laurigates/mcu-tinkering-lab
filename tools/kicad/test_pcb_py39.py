"""Guards for tools/kicad/pcb/: scripts that only ever run under KiCad's Python 3.9.

CI has no KiCad, so nothing runs these files under 3.9, and the repo's ruff
targets py311 with `fixable = ["ALL"]` (pyupgrade's UP045 would rewrite
`Optional[X]` to `X | None`). These checks read each `pcb/*.py` as text:

- it parses with `ast.parse(..., feature_version=(3, 9))` (no `match`);
- it imports only the stdlib modules KiCad's Python has and the scripts use,
  `pcbnew`, and its siblings `dsn` and `fanout` (no `tomllib`, nothing from tools/kicad or
  tools/hardware);
- an annotation using `|` needs `from __future__ import annotations`;
- `pcb/` has no `__init__.py`, so it stays a directory of scripts;
- ruff.toml keeps `tools/kicad/pcb/**` at target py39.

    python3 -m unittest discover -s tools/kicad -t tools
"""

from __future__ import annotations

import ast
import tomllib
import unittest
from pathlib import Path

from kicad.paths import REPO_ROOT

PCB = Path(__file__).resolve().parent / "pcb"
ALLOWED = frozenset(
    {"fnmatch", "json", "math", "os", "re", "shutil", "subprocess", "sys", "tempfile"}
    | {"pcbnew", "dsn", "fanout", "__future__"}
)


def annotations(tree: ast.AST):
    for node in ast.walk(tree):
        if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef)):
            args = node.args
            for a in [*args.posonlyargs, *args.args, *args.kwonlyargs]:
                if a.annotation is not None:
                    yield a.annotation
            for a in (args.vararg, args.kwarg):
                if a is not None and a.annotation is not None:
                    yield a.annotation
            if node.returns is not None:
                yield node.returns
        elif isinstance(node, ast.AnnAssign):
            yield node.annotation


def problems(src: str) -> list[str]:
    """Why `src` would not run under Python 3.9 as a pcb/ script; [] when fine."""
    try:
        tree = ast.parse(src, feature_version=(3, 9))
    except SyntaxError as e:
        return [f"not Python 3.9: {e.msg} (line {e.lineno})"]
    out = []
    future = False
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            names = [a.name.split(".")[0] for a in node.names]
        elif isinstance(node, ast.ImportFrom):
            if node.level:
                out.append(f"relative import (line {node.lineno})")
                continue
            names = [(node.module or "").split(".")[0]]
            if node.module == "__future__" and any(
                a.name == "annotations" for a in node.names
            ):
                future = True
        else:
            continue
        out += [
            f"imports {n!r} (line {node.lineno})" for n in names if n not in ALLOWED
        ]
    union = any(
        isinstance(n, ast.BinOp) and isinstance(n.op, ast.BitOr)
        for ann in annotations(tree)
        for n in ast.walk(ann)
    )
    if union and not future:
        out.append("`X | Y` annotation without `from __future__ import annotations`")
    return out


class PcbScriptTests(unittest.TestCase):
    def test_scripts(self):
        files = sorted(PCB.glob("*.py"))
        self.assertEqual(
            [f.name for f in files],
            ["build_board.py", "dsn.py", "fanout.py", "route_board.py"],
        )
        for f in files:
            with self.subTest(file=f.name):
                self.assertEqual(problems(f.read_text(encoding="utf-8")), [])

    def test_not_a_package(self):
        self.assertFalse((PCB / "__init__.py").exists())

    def test_ruff_targets_py39(self):
        ruff = tomllib.loads((REPO_ROOT / "ruff.toml").read_text(encoding="utf-8"))
        self.assertEqual(
            ruff.get("per-file-target-version", {}).get("tools/kicad/pcb/**"), "py39"
        )

    def test_the_checks_can_fail(self):
        cases = {
            "match x:\n    case 1:\n        pass\n": "not Python 3.9",
            "def f(x: int | None) -> None:\n    pass\n": "without `from __future__",
            "y: str | None = None\n": "without `from __future__",
            "import tomllib\n": "imports 'tomllib'",
            "from hardware import join\n": "imports 'hardware'",
            "from kicad import netlist\n": "imports 'kicad'",
            "from . import dsn\n": "relative import",
        }
        for src, msg in cases.items():
            with self.subTest(src=src):
                found = problems(src)
                self.assertTrue(any(msg in p for p in found), found)
        ok = "from __future__ import annotations\ndef f(x: int | None): pass\n"
        self.assertEqual(problems(ok), [])


if __name__ == "__main__":
    unittest.main()
