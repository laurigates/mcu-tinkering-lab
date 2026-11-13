"""The arena schema: the floor the robot stands on, walls and obstacles (ADR-036).

This module is the one owner of the schema. The twin harness forwards an arena
file as a mapping without reading it, and :func:`parse_arena` here decides
whether it is valid.

Units are centimetres and degrees. Heading 0 points along +x and angles grow
counter-clockwise. Unknown keys are an error.

.. code-block:: yaml

    floor:                                   # required: polygon the robot stands on
      - [-50, -60]
      - [150, -60]
      - [150, 60]
      - [-50, 60]
    walls:                                   # optional: rangefinders see them, the chassis hits them
      - {from: [150, -60], to: [150, 60]}
    obstacles:                               # optional
      - {box: {center: [80, 0], size: [20, 30], heading_deg: 0}}
      - {cylinder: {center: [40, 30], radius_cm: 5}}

A box's ``size`` is its extent along and across its own heading.
"""

from __future__ import annotations

import math
from collections.abc import Mapping, Sequence
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import yaml

Point = tuple[float, float]

# A coordinate beyond this is a typing mistake, not an arena (10 km).
MAX_COORD_CM = 1_000_000.0


class ArenaError(ValueError):
    """The arena description is not valid. The message names the offending key."""


@dataclass(frozen=True)
class Wall:
    """A straight wall with no thickness, from ``start`` to ``end`` (cm)."""

    start: Point
    end: Point


@dataclass(frozen=True)
class Box:
    """A rectangular obstacle: ``size`` is (along, across) its heading, in cm."""

    center: Point
    size: Point
    heading_deg: float = 0.0


@dataclass(frozen=True)
class Cylinder:
    """A round obstacle seen from above."""

    center: Point
    radius_cm: float


@dataclass(frozen=True)
class Arena:
    """A validated arena. Build one with :func:`parse_arena` or :func:`load_arena`."""

    floor: tuple[Point, ...]
    walls: tuple[Wall, ...] = ()
    obstacles: tuple[Box | Cylinder, ...] = ()

    def on_floor(self, x_cm: float, y_cm: float) -> bool:
        """Whether a point is inside the floor polygon (even-odd rule)."""
        inside = False
        n = len(self.floor)
        for i in range(n):
            x1, y1 = self.floor[i]
            x2, y2 = self.floor[(i + 1) % n]
            if (y1 > y_cm) != (y2 > y_cm):
                x_cross = x1 + (y_cm - y1) * (x2 - x1) / (y2 - y1)
                if x_cm < x_cross:
                    inside = not inside
        return inside


def _number(value: Any, where: str) -> float:
    # bool is an int in Python; "true" is never a coordinate.
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ArenaError(f"{where}: expected a number, got {value!r}")
    number = float(value)
    if not math.isfinite(number):
        raise ArenaError(f"{where}: expected a finite number, got {value!r}")
    return number


def _coordinate(value: Any, where: str) -> float:
    number = _number(value, where)
    if abs(number) > MAX_COORD_CM:
        raise ArenaError(f"{where}: {value!r} is outside +/-{MAX_COORD_CM:g} cm")
    return number


def _positive(value: Any, where: str) -> float:
    number = _number(value, where)
    if number <= 0 or number > MAX_COORD_CM:
        raise ArenaError(f"{where}: expected a size in (0, {MAX_COORD_CM:g}] cm, got {value!r}")
    return number


def _pair(value: Any, where: str, item=_coordinate) -> Point:
    if isinstance(value, (str, bytes)) or not isinstance(value, Sequence) or len(value) != 2:
        raise ArenaError(f"{where}: expected [x, y], got {value!r}")
    return (item(value[0], f"{where}[0]"), item(value[1], f"{where}[1]"))


def _mapping(value: Any, where: str, required: set[str], optional: set[str]) -> Mapping:
    if not isinstance(value, Mapping):
        raise ArenaError(f"{where}: expected a mapping, got {value!r}")
    unknown = sorted(str(key) for key in value if key not in required | optional)
    if unknown:
        allowed = ", ".join(sorted(required | optional))
        raise ArenaError(f"{where}: unknown key(s) {', '.join(unknown)} (allowed: {allowed})")
    missing = sorted(required - set(value))
    if missing:
        raise ArenaError(f"{where}: missing key(s) {', '.join(missing)}")
    return value


def _list(value: Any, where: str) -> Sequence:
    if isinstance(value, (str, bytes)) or not isinstance(value, Sequence):
        raise ArenaError(f"{where}: expected a list, got {value!r}")
    return value


def _floor(value: Any) -> tuple[Point, ...]:
    points = tuple(_pair(p, f"arena.floor[{i}]") for i, p in enumerate(_list(value, "arena.floor")))
    if len(points) < 3:
        raise ArenaError(f"arena.floor: a polygon needs at least 3 points, got {len(points)}")
    twice_area = sum(
        points[i][0] * points[(i + 1) % len(points)][1]
        - points[(i + 1) % len(points)][0] * points[i][1]
        for i in range(len(points))
    )
    if twice_area == 0:
        raise ArenaError("arena.floor: the polygon has no area")
    return points


def _wall(value: Any, where: str) -> Wall:
    entry = _mapping(value, where, {"from", "to"}, set())
    wall = Wall(_pair(entry["from"], f"{where}.from"), _pair(entry["to"], f"{where}.to"))
    if wall.start == wall.end:
        raise ArenaError(f"{where}: from and to are the same point")
    return wall


def _obstacle(value: Any, where: str) -> Box | Cylinder:
    entry = _mapping(value, where, set(), {"box", "cylinder"})
    if len(entry) != 1:
        raise ArenaError(f"{where}: expected exactly one of box, cylinder")
    if "box" in entry:
        box = _mapping(entry["box"], f"{where}.box", {"center", "size"}, {"heading_deg"})
        return Box(
            center=_pair(box["center"], f"{where}.box.center"),
            size=_pair(box["size"], f"{where}.box.size", _positive),
            heading_deg=_number(box.get("heading_deg", 0.0), f"{where}.box.heading_deg"),
        )
    cylinder = _mapping(entry["cylinder"], f"{where}.cylinder", {"center", "radius_cm"}, set())
    return Cylinder(
        center=_pair(cylinder["center"], f"{where}.cylinder.center"),
        radius_cm=_positive(cylinder["radius_cm"], f"{where}.cylinder.radius_cm"),
    )


def parse_arena(data: Any) -> Arena:
    """Validate an arena mapping (as loaded from YAML or JSON) and return an Arena.

    Raises :class:`ArenaError` naming the key that is wrong.
    """
    top = _mapping(data, "arena", {"floor"}, {"walls", "obstacles"})
    return Arena(
        floor=_floor(top["floor"]),
        walls=tuple(
            _wall(w, f"arena.walls[{i}]")
            for i, w in enumerate(_list(top.get("walls", []), "arena.walls"))
        ),
        obstacles=tuple(
            _obstacle(o, f"arena.obstacles[{i}]")
            for i, o in enumerate(_list(top.get("obstacles", []), "arena.obstacles"))
        ),
    )


def load_arena(path: str | Path) -> Arena:
    """Read and validate an arena YAML file."""
    with open(path, encoding="utf-8") as f:
        try:
            data = yaml.safe_load(f)
        except yaml.YAMLError as e:
            raise ArenaError(f"{path}: not valid YAML: {e}") from e
    return parse_arena(data)
