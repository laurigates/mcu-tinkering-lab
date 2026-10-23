"""The product renders' look-alike breakouts: catalog coverage and placement."""

import tomllib

import pytest
from enclosure import board, modules
from kicad import paths, sexpr
from kicad.paths import REPO_ROOT

PROJECTS = [
    REPO_ROOT / "packages" / "robotics" / "balancebot",
    REPO_ROOT / "packages" / "robocar" / "unified",
]


def _heights(project):
    path = project / "hardware" / "enclosure" / board.ENCLOSURE_TOML
    return tomllib.loads(path.read_text())["heights"]


def _footprints(project):
    cfg = paths.load_board_config(project)
    pcb = paths.kicad_project_dir(project) / f"{cfg.name}.kicad_pcb"
    return {board._reference(fp): fp for fp in board._footprints(pcb.read_text())}


@pytest.mark.parametrize("project", PROJECTS, ids=lambda p: p.name)
def test_every_estimated_part_has_a_look_alike(project):
    """A part with an enclosure.toml height has no KiCad model: the render
    needs a kind for it and an outline to place it by."""
    catalog = modules.load_catalog()
    fps = _footprints(project)
    for ref, spec in _heights(project).items():
        fp = fps[ref]
        entry = catalog["footprints"].get(sexpr.head(fp)[1], {})
        kind = spec.get("module", entry.get("kind"))
        assert kind in catalog["kinds"], ref
        assert "outline" in spec or modules._fab_bbox(fp), ref


def test_breakout_kinds_have_a_pcb_and_known_features():
    known = {"chip", "connector", "board", "usb-c", "shield", "heatsink",
             "trimpot", "camera", "terminal", "header"}  # fmt: skip
    for name, kind in modules.load_catalog()["kinds"].items():
        assert ("pcb" in kind) != ("body" in kind), name
        for f in kind.get("features", []):
            assert f["what"] in known, (name, f)
            assert ("xy" in f) != ("at" in f), (name, f)


def test_gy521_outline_maps_back_onto_its_silk_rect():
    """J3 sits at 90 degrees: the outline handed to Blender in J3's frame must
    land back on board.toml's rect, with J3's pads along its rear edge."""
    project = PROJECTS[0]
    cfg = paths.load_board_config(project)
    ox, oy = cfg.data["outline"]["origin"]
    fp = _footprints(project)["J3"]
    at = sexpr.atoms(sexpr.child(fp, "at"))
    fx, fy, rot = float(at[0]) - ox, float(at[1]) - oy, float(at[2])
    rect = _heights(project)["J3"]["outline"]
    local = modules._to_local(rect, fx, fy, rot)
    corners = [
        board._rotate(x, y, rot)
        for x in (local[0], local[2])
        for y in (local[1], local[3])
    ]
    xs = sorted({round(fx + dx, 6) for dx, _ in corners})
    ys = sorted({round(fy + dy, 6) for _, dy in corners})
    assert [xs[0], ys[0], xs[-1], ys[-1]] == pytest.approx(rect)
    for px, py in modules._pads(fp):
        _, dy = board._rotate(px, py, rot)
        assert fy + dy == pytest.approx(47.0)  # the rect's rear edge is 48.5
