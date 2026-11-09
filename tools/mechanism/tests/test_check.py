"""The clash sweep on synthetic designs: no servo, no board."""

from types import SimpleNamespace

from build123d import Box, Pos, Rot
from mechanism import check
from mechanism.check import Body


def two_blocks(gap: float, same_group=False, allow=()):
    a = Body("a", Box(10, 10, 10), "g1", printed=True, allow=set(allow))
    b = Body("b", Pos(10 + gap, 0, 0) * Box(10, 10, 10), "g1" if same_group else "g2")
    return [a, b]


def test_overlap_between_groups_is_a_clash_with_its_volume():
    (c,) = check.clashes(two_blocks(-2))
    assert (c["a"], c["b"]) == ("a", "b")
    assert abs(c["mm3"] - 200) < 0.01


def test_touching_faces_are_not_a_clash():
    assert check.clashes(two_blocks(0)) == []


def test_bodies_in_one_group_may_overlap():
    assert check.clashes(two_blocks(-2, same_group=True)) == []


def test_an_allowed_pair_is_excused_and_listed():
    bodies = two_blocks(-2, allow=["b"])
    assert check.clashes(bodies) == []
    assert check.excused(bodies) == ["a / b"]


def arm_design():
    """A 30 mm arm on a pan joint that swings into a post at +90 only."""

    def assembly(pan, tilt):
        arm = Rot(0, 0, pan) * Pos(15, 0, 0) * Box(30, 4, 4)
        post = Pos(0, 25, 0) * Box(4, 4, 10)
        return [Body("arm", arm, "moving", printed=True), Body("post", post, "base")]

    return SimpleNamespace(assembly=assembly)


def test_the_sweep_names_the_pose_that_clashes():
    result = check.sweep(arm_design())
    assert set(result) == {f"pan 90 tilt {t}" for t in check.TILT_SWEEP}


def test_a_design_can_name_its_own_joints_and_poses():
    d = arm_design()
    d.JOINTS = ("yaw", "unused")
    d.SWEEP = [(0, 0), (90, 0)]
    assert list(check.sweep(d)) == ["yaw 90 unused 0"]


def test_the_wall_gate_is_tools_enclosures():
    thinnest = check._enclosure_thickness().thinnest
    assert abs(thinnest(Box(20, 20, 0.5)).mm - 0.5) < 0.02


def test_a_design_without_grid_holes_has_no_grid_report():
    """Existing designs opt out by not declaring GRID_HOLES; check.json keeps
    its old shape for them."""
    assert check.grid_report(SimpleNamespace()) is None


def test_grid_holes_on_the_10_mm_grid_pass():
    d = SimpleNamespace(GRID_HOLES={"plate": [(0, 0), (10, 0), (10, 20)]})
    assert check.grid_report(d) == {}


def test_an_off_grid_hole_is_named_by_part_and_fails_the_gate():
    d = SimpleNamespace(
        GRID_HOLES={"ok": [(0, 0), (20, 0)], "bad": [(5, 5), (15.4, 5)]}
    )
    assert check.grid_report(d) == {"bad": [[15.4, 5]]}
    assert check.failed({"clashes": {}, "grid_off": {"bad": [[15.4, 5]]}})
    assert not check.failed({"clashes": {}, "grid_off": {}})
    assert not check.failed({"clashes": {}})
