"""The arena schema: what parses, what is refused and why (ADR-036)."""

import pytest

from robocar_sim.arena import Arena, ArenaError, Box, Cylinder, Wall, load_arena, parse_arena

SQUARE = [[-50, -60], [150, -60], [150, 60], [-50, 60]]


def test_parses_the_adr_example():
    arena = parse_arena(
        {
            "floor": SQUARE,
            "walls": [{"from": [150, -60], "to": [150, 60]}],
            "obstacles": [
                {"box": {"center": [80, 0], "size": [20, 30], "heading_deg": 0}},
                {"cylinder": {"center": [40, 30], "radius_cm": 5}},
            ],
        }
    )

    assert arena == Arena(
        floor=((-50.0, -60.0), (150.0, -60.0), (150.0, 60.0), (-50.0, 60.0)),
        walls=(Wall((150.0, -60.0), (150.0, 60.0)),),
        obstacles=(Box((80.0, 0.0), (20.0, 30.0), 0.0), Cylinder((40.0, 30.0), 5.0)),
    )


def test_walls_and_obstacles_are_optional():
    arena = parse_arena({"floor": SQUARE})

    assert arena.walls == ()
    assert arena.obstacles == ()


def test_box_heading_defaults_to_zero():
    arena = parse_arena(
        {"floor": SQUARE, "obstacles": [{"box": {"center": [0, 0], "size": [1, 2]}}]}
    )

    assert arena.obstacles == (Box((0.0, 0.0), (1.0, 2.0), 0.0),)


@pytest.mark.parametrize(
    ("data", "message"),
    [
        ([], "arena: expected a mapping"),
        ({}, "arena: missing key(s) floor"),
        ({"floor": SQUARE, "wall": []}, "arena: unknown key(s) wall"),
        ({"floor": SQUARE, "size": [5, 5]}, "arena: unknown key(s) size"),
        ({"floor": "square"}, "arena.floor: expected a list"),
        ({"floor": [[0, 0], [1, 0]]}, "arena.floor: a polygon needs at least 3 points"),
        ({"floor": [[0, 0], [1, 0], [2, 0]]}, "arena.floor: the polygon has no area"),
        ({"floor": [[0, 0], [1, 0], [1]]}, "arena.floor[2]: expected [x, y]"),
        ({"floor": [[0, 0], [1, 0], [1, "1"]]}, "arena.floor[2][1]: expected a number"),
        ({"floor": [[0, 0], [1, 0], [1, True]]}, "arena.floor[2][1]: expected a number"),
        ({"floor": [[0, 0], [1, 0], [1, float("inf")]]}, "arena.floor[2][1]: expected a finite"),
        ({"floor": [[0, 0], [1, 0], [1, float("nan")]]}, "arena.floor[2][1]: expected a finite"),
        ({"floor": [[0, 0], [1, 0], [1, 1e9]]}, "arena.floor[2][1]: 1000000000.0 is outside"),
        ({"floor": SQUARE, "walls": None}, "arena.walls: expected a list"),
        ({"floor": SQUARE, "walls": [[0, 0, 1, 1]]}, "arena.walls[0]: expected a mapping"),
        ({"floor": SQUARE, "walls": [{"from": [0, 0]}]}, "arena.walls[0]: missing key(s) to"),
        (
            {"floor": SQUARE, "walls": [{"from": [0, 0], "to": [1, 1], "height": 3}]},
            "arena.walls[0]: unknown key(s) height",
        ),
        (
            {"floor": SQUARE, "walls": [{"from": [0, 0], "to": [0, 0]}]},
            "arena.walls[0]: from and to are the same point",
        ),
        ({"floor": SQUARE, "obstacles": [{}]}, "arena.obstacles[0]: expected exactly one of"),
        (
            {"floor": SQUARE, "obstacles": [{"sphere": {"center": [0, 0]}}]},
            "arena.obstacles[0]: unknown key(s) sphere",
        ),
        (
            {
                "floor": SQUARE,
                "obstacles": [
                    {
                        "box": {"center": [0, 0], "size": [1, 1]},
                        "cylinder": {"center": [0, 0], "radius_cm": 1},
                    }
                ],
            },
            "arena.obstacles[0]: expected exactly one of",
        ),
        (
            {"floor": SQUARE, "obstacles": [{"box": {"center": [0, 0], "size": [0, 1]}}]},
            "arena.obstacles[0].box.size[0]: expected a size",
        ),
        (
            {"floor": SQUARE, "obstacles": [{"box": {"center": [0, 0]}}]},
            "arena.obstacles[0].box: missing key(s) size",
        ),
        (
            {
                "floor": SQUARE,
                "obstacles": [{"box": {"center": [0, 0], "size": [1, 1], "angle": 9}}],
            },
            "arena.obstacles[0].box: unknown key(s) angle",
        ),
        (
            {"floor": SQUARE, "obstacles": [{"cylinder": {"center": [0, 0], "radius_cm": -1}}]},
            "arena.obstacles[0].cylinder.radius_cm: expected a size",
        ),
        (
            {"floor": SQUARE, "obstacles": [{"cylinder": {"center": [0, 0], "radius": 1}}]},
            "arena.obstacles[0].cylinder: unknown key(s) radius",
        ),
    ],
)
def test_refuses_with_the_key_named(data, message):
    with pytest.raises(ArenaError) as error:
        parse_arena(data)

    assert str(error.value).startswith(message)


@pytest.mark.parametrize(
    ("point", "inside"),
    [
        ((0, 0), True),
        ((149.9, 59.9), True),
        ((150.1, 0), False),
        ((0, -60.1), False),
        ((-50.1, 0), False),
    ],
)
def test_on_floor_of_a_rectangle(point, inside):
    arena = parse_arena({"floor": SQUARE})

    assert arena.on_floor(*point) is inside


def test_on_floor_of_a_concave_floor():
    # An L: the notch at the top right is not floor.
    arena = parse_arena({"floor": [[0, 0], [100, 0], [100, 50], [50, 50], [50, 100], [0, 100]]})

    assert arena.on_floor(25, 75)
    assert arena.on_floor(75, 25)
    assert not arena.on_floor(75, 75)
    # Left of the L, level with its upper arm: the edges that close the
    # polygon (the fifth and sixth) are what put this point outside.
    assert not arena.on_floor(-10, 75)


def test_load_arena_reads_yaml(tmp_path):
    path = tmp_path / "table.yaml"
    path.write_text("floor:\n  - [0, 0]\n  - [80, 0]\n  - [80, 60]\n  - [0, 60]\n")

    assert load_arena(path) == Arena(floor=((0.0, 0.0), (80.0, 0.0), (80.0, 60.0), (0.0, 60.0)))


def test_load_arena_reports_bad_yaml(tmp_path):
    path = tmp_path / "broken.yaml"
    path.write_text("floor: [unclosed\n")

    with pytest.raises(ArenaError, match="not valid YAML"):
        load_arena(path)
