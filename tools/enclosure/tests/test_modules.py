"""The product renders' look-alike breakouts: catalog coverage and placement."""

import tomllib

import pytest
from enclosure import board, modules
from kicad import paths, sexpr, sockets
from kicad.paths import REPO_ROOT

PROJECTS = [
    REPO_ROOT / "packages" / "robotics" / "balancebot",
    REPO_ROOT / "packages" / "robotics" / "robocar",
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


def test_gy521_outline_comes_from_its_socket_footprint():
    """J3's look-alike outline is GY521_Socket's F.Fab, not a hand copy.

    enclosure.toml names no outline or module for J3, the footprint's F.Fab is
    sockets.toml's outline (the measured 21.00 x 15.88 mm board), and placed on
    the carrier the module lies forward (-y) of the pad row: the 8d18cd8d
    board put it behind, so the module fitted component side down.
    """
    project = PROJECTS[0]
    spec = _heights(project)["J3"]
    assert "outline" not in spec and "module" not in spec
    fp = _footprints(project)["J3"]
    assert sexpr.head(fp)[1] == "Breakout_Sockets:GY521_Socket"
    kind = modules.load_catalog()["footprints"][sexpr.head(fp)[1]]["kind"]
    assert kind == "gy-521"
    config = sockets.load_config(sockets.subtree() / sockets.CONFIG)
    (socket,) = [s for s in config if s.name == "GY521_Socket"]
    local = modules._fab_bbox(fp)
    assert local == pytest.approx(list(socket.outline))
    assert local[2] - local[0] == pytest.approx(21.00)
    assert local[3] - local[1] == pytest.approx(15.88)

    cfg = paths.load_board_config(project)
    oy = cfg.data["outline"]["origin"][1]
    at = sexpr.atoms(sexpr.child(fp, "at"))
    fy = float(at[1]) - oy
    rot = float(at[2]) if len(at) > 2 else 0.0
    corners = [
        board._rotate(x, y, rot)
        for x in (local[0], local[2])
        for y in (local[1], local[3])
    ]
    top = min(fy + dy for _, dy in corners)
    bottom = max(fy + dy for _, dy in corners)
    rows = {
        round(fy + board._rotate(px, py, rot)[1], 6) for px, py in modules._pads(fp)
    }
    assert rows == {47.0}  # the pads where the 1x8 socket's were
    assert 47.0 - top > 14.0  # the module forward of the row
    assert bottom - 47.0 < 2.0  # the row along its rear edge
