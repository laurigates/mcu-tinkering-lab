"""The robocar plant: wheels, contacts, rangefinders and floor sensors (ADR-036).

A :class:`Plant` is stepped with one wheel command per side and returns what
the robot's sensors would see. It is deterministic: no wall clock, no threads,
no process-wide state, no file written, and one numpy generator seeded from
``seed``. Two plants built with the same arguments give identical readings.

.. code-block:: python

    plant = Plant(arena, seed=7, start=Pose(0.0, 0.0, 0.0))
    reading = plant.step(33, left=WheelCmd("forward", 2409), right=WheelCmd("forward", 2409))

Run as ``python -m robocar_sim.plant`` it is the JSONL stepper of ADR-036
section 3, which has the behaviour twin's harness start it as a child process:
one ``init`` line, then one ``step`` line per executor tick, each answered with
one line on stdout.

The plant reports geometric truth every step. The read cycle, the smoothing
and the fail-closed rules stay in the firmware's ``ultrasonic_core`` and
``cliff_core``, which the twin runs.
"""

from __future__ import annotations

import io
import json
import math
import sys
from collections.abc import Iterable, Mapping
from dataclasses import dataclass
from pathlib import Path
from typing import Any, TextIO

import numpy as np
import pymunk
import yaml

from .arena import Arena, ArenaError, Box, parse_arena
from .robot_model import DCMotor, PhysicsEngine

DRIVES = ("forward", "reverse", "coast", "brake")
DUTY_MAX = 4095  # the largest PCA9685 count the driver writes
PWM_STEPS = 4096  # PCA9685 counts per PWM period: a count of n is on for n/4096
MAX_DT_MS = 1000  # one step() call; the twin steps 33 ms at a time

# The motors and the contacts advance in fixed sub-steps, so a step of any
# length gives the same motion as the same time in 1 ms steps.
SUBSTEP_MS = 1
SUBSTEP_S = SUBSTEP_MS / 1000.0

# Chipmunk lets shapes overlap by `collision_slop` before it pushes them apart,
# and its default (0.1) is sized for pixels: in metres it is 10 cm.
COLLISION_SLOP_M = 0.0005
# Share of an overlap beyond the slop that is corrected in each sub-step.
OVERLAP_CORRECTION_PER_SUBSTEP = 0.1
# A start pose may touch an obstacle but not overlap it by more than this.
START_OVERLAP_M = 0.001

# Shapes in the same non-zero group ignore each other, in collisions and in
# queries. The robot's own shape is in this group and so are the rangefinder
# rays, which start inside the chassis outline.
ROBOT_GROUP = 1
# A rangefinder ray has this radius. A ray of no width that passes exactly
# through a box's corner misses the box, and an arena written in whole
# centimetres puts corners on the robot's centre line.
RAY_RADIUS_M = 0.001
# A ray starts this far behind its sensor, along the ray, and the distance is
# taken off the result. A chassis held on a wall rests past the wall's line by
# the collision slop, and a ray that started at a sensor on the chassis edge
# would start beyond the wall and read whatever lies behind it.
RAY_BACKOFF_M = 0.01
# Contacts are against static shapes, so the body's mass changes nothing: the
# inertia the motors accelerate is motor.inertia_kg_m2.
BODY_MASS_KG = 1.0

# Below this wheel speed (rad/s) DCMotor applies static friction.
REST_BAND_RAD_S = 0.01

_DEFAULT_CONFIG = Path(__file__).with_name("plant.yaml")


class PlantError(ValueError):
    """A plant argument, robot description or stepper message is not valid."""


@dataclass(frozen=True)
class WheelCmd:
    """What the TB6612FNG does with one motor.

    ``drive`` is ``forward``, ``reverse``, ``coast`` (IN1 = IN2 = low: the
    winding is open) or ``brake`` (IN1 = IN2 = high: the winding is shorted).
    ``duty`` is the PCA9685 count on the PWM pin, 0..4095; it has no effect on
    ``coast`` and ``brake``.
    """

    drive: str
    duty: int = 0

    def __post_init__(self) -> None:
        if self.drive not in DRIVES:
            raise PlantError(f"drive: expected one of {', '.join(DRIVES)}, got {self.drive!r}")
        if isinstance(self.duty, bool) or not isinstance(self.duty, int):
            raise PlantError(f"duty: expected an integer 0..{DUTY_MAX}, got {self.duty!r}")
        if not 0 <= self.duty <= DUTY_MAX:
            raise PlantError(f"duty: expected 0..{DUTY_MAX}, got {self.duty}")


@dataclass(frozen=True)
class Pose:
    """The midpoint between the drive wheels, in cm, and the heading in degrees.

    Heading 0 points along +x and grows counter-clockwise.
    """

    x_cm: float = 0.0
    y_cm: float = 0.0
    heading_deg: float = 0.0


@dataclass(frozen=True)
class Ranges:
    """Rangefinder readings in whole cm; None where nothing is within 2..400 cm."""

    front: int | None
    rear: int | None


@dataclass(frozen=True)
class Floor:
    """Whether each floor sensor has floor under it."""

    front: bool
    rear: bool


@dataclass(frozen=True)
class Reading:
    """The world at the end of a step.

    ``collided`` is true when the chassis touched a wall or an obstacle during
    the step. ``off_floor`` is true while a drive wheel's contact point is
    outside the floor polygon; the plant does not model the fall and keeps
    moving the robot in the plane.
    """

    at_ms: int
    pose: Pose
    range_cm: Ranges
    floor: Floor
    collided: bool
    off_floor: bool

    def to_message(self) -> dict[str, Any]:
        """The stepper's ``reading`` line. Pose figures are rounded to 0.001."""

        def figure(value: float) -> float:
            return round(value, 3) + 0.0  # adding 0.0 turns -0.0 into 0.0

        return {
            "t": "reading",
            "at_ms": self.at_ms,
            "pose": {
                "x_cm": figure(self.pose.x_cm),
                "y_cm": figure(self.pose.y_cm),
                "heading_deg": figure(self.pose.heading_deg),
            },
            "range_cm": {"front": self.range_cm.front, "rear": self.range_cm.rear},
            "floor": {"front": self.floor.front, "rear": self.floor.rear},
            "collided": self.collided,
            "off_floor": self.off_floor,
        }


# --- validation ---------------------------------------------------------------


def _number(value: Any, where: str, *, minimum: float | None = None, positive: bool = False):
    # bool is an int in Python; "true" is never a figure.
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise PlantError(f"{where}: expected a number, got {value!r}")
    number = float(value)
    if not math.isfinite(number):
        raise PlantError(f"{where}: expected a finite number, got {value!r}")
    if positive and number <= 0:
        raise PlantError(f"{where}: expected a number above 0, got {value!r}")
    if minimum is not None and number < minimum:
        raise PlantError(f"{where}: expected at least {minimum:g}, got {value!r}")
    return number


def _integer(value: Any, where: str, low: int, high: int) -> int:
    if isinstance(value, bool) or not isinstance(value, int):
        raise PlantError(f"{where}: expected an integer {low}..{high}, got {value!r}")
    if not low <= value <= high:
        raise PlantError(f"{where}: expected {low}..{high}, got {value}")
    return value


def _mapping(value: Any, where: str, keys: Iterable[str]) -> Mapping:
    """A mapping with exactly ``keys``: nothing missing, nothing unknown."""
    if not isinstance(value, Mapping):
        raise PlantError(f"{where}: expected a mapping, got {value!r}")
    wanted = set(keys)
    unknown = sorted(str(key) for key in value if key not in wanted)
    if unknown:
        allowed = ", ".join(sorted(wanted))
        raise PlantError(f"{where}: unknown key(s) {', '.join(unknown)} (allowed: {allowed})")
    missing = sorted(wanted - set(value))
    if missing:
        raise PlantError(f"{where}: missing key(s) {', '.join(missing)}")
    return value


@dataclass(frozen=True)
class _Mount:
    """A sensor's place in the robot frame (m, rad)."""

    x: float
    y: float
    heading: float = 0.0


@dataclass(frozen=True)
class _Robot:
    """The robot description, validated and in SI units."""

    supply_v: float
    front_overhang: float
    rear_overhang: float
    width: float
    track: float
    wheel_radius: float
    motor: dict[str, float]
    rolling_resistance: float
    range_min_cm: int
    range_max_cm: int
    range_noise_cm: float
    range_front: _Mount
    range_rear: _Mount
    floor_front: _Mount
    floor_rear: _Mount


def _mount(value: Any, where: str, with_heading: bool) -> _Mount:
    keys = ("x_cm", "y_cm", "heading_deg") if with_heading else ("x_cm", "y_cm")
    entry = _mapping(value, where, keys)
    heading = _number(entry["heading_deg"], f"{where}.heading_deg") if with_heading else 0.0
    return _Mount(
        _number(entry["x_cm"], f"{where}.x_cm") / 100.0,
        _number(entry["y_cm"], f"{where}.y_cm") / 100.0,
        math.radians(heading),
    )


def default_config() -> dict[str, Any]:
    """The robot description shipped with the package (``plant.yaml``)."""
    with open(_DEFAULT_CONFIG, encoding="utf-8") as f:
        return yaml.safe_load(f)


def _parse_config(data: Any) -> _Robot:
    top = _mapping(
        data, "config", ("supply_v", "chassis", "wheels", "motor", "rangefinders", "floor_sensors")
    )
    chassis = _mapping(
        top["chassis"], "config.chassis", ("front_overhang_cm", "rear_overhang_cm", "width_cm")
    )
    wheels = _mapping(top["wheels"], "config.wheels", ("track_cm", "diameter_cm"))
    motor = _mapping(
        top["motor"],
        "config.motor",
        (
            "resistance_ohm",
            "inductance_h",
            "back_emf_v_s_per_rad",
            "stall_torque_nm",
            "stall_current_a",
            "speed_limit_rpm",
            "inertia_kg_m2",
            "rolling_resistance_nm",
        ),
    )
    rangers = _mapping(
        top["rangefinders"],
        "config.rangefinders",
        ("min_cm", "max_cm", "noise_cm", "front", "rear"),
    )
    floor = _mapping(top["floor_sensors"], "config.floor_sensors", ("front", "rear"))

    def motor_figure(key: str) -> float:
        return _number(motor[key], f"config.motor.{key}", positive=True)

    resistance = motor_figure("resistance_ohm")
    inductance = motor_figure("inductance_h")
    # DCMotor.update's current step overshoots past 10 * L / R.
    if SUBSTEP_S > 10.0 * inductance / resistance:
        raise PlantError(
            "config.motor.inductance_h: below resistance_ohm * "
            f"{SUBSTEP_S / 10.0:g} H the {SUBSTEP_MS} ms integration step overshoots"
        )
    range_min = _integer(rangers["min_cm"], "config.rangefinders.min_cm", 0, 10_000)
    range_max = _integer(rangers["max_cm"], "config.rangefinders.max_cm", range_min, 10_000)
    return _Robot(
        supply_v=_number(top["supply_v"], "config.supply_v", positive=True),
        front_overhang=_number(
            chassis["front_overhang_cm"], "config.chassis.front_overhang_cm", positive=True
        )
        / 100.0,
        rear_overhang=_number(
            chassis["rear_overhang_cm"], "config.chassis.rear_overhang_cm", positive=True
        )
        / 100.0,
        width=_number(chassis["width_cm"], "config.chassis.width_cm", positive=True) / 100.0,
        track=_number(wheels["track_cm"], "config.wheels.track_cm", positive=True) / 100.0,
        wheel_radius=_number(wheels["diameter_cm"], "config.wheels.diameter_cm", positive=True)
        / 200.0,
        motor={
            "resistance": resistance,
            "inductance": inductance,
            "back_emf_constant": motor_figure("back_emf_v_s_per_rad"),
            "stall_torque": motor_figure("stall_torque_nm"),
            "stall_current": motor_figure("stall_current_a"),
            "max_rpm": motor_figure("speed_limit_rpm"),
            "inertia": motor_figure("inertia_kg_m2"),
            # DCMotor stores this figure and never reads it.
            "no_load_current": 0.0,
        },
        rolling_resistance=_number(
            motor["rolling_resistance_nm"], "config.motor.rolling_resistance_nm", minimum=0.0
        ),
        range_min_cm=range_min,
        range_max_cm=range_max,
        range_noise_cm=_number(rangers["noise_cm"], "config.rangefinders.noise_cm", minimum=0.0),
        range_front=_mount(rangers["front"], "config.rangefinders.front", with_heading=True),
        range_rear=_mount(rangers["rear"], "config.rangefinders.rear", with_heading=True),
        floor_front=_mount(floor["front"], "config.floor_sensors.front", with_heading=False),
        floor_rear=_mount(floor["rear"], "config.floor_sensors.rear", with_heading=False),
    )


def _parse_pose(value: Any, where: str) -> Pose:
    entry = _mapping(value, where, ("x_cm", "y_cm", "heading_deg"))
    return Pose(
        _number(entry["x_cm"], f"{where}.x_cm"),
        _number(entry["y_cm"], f"{where}.y_cm"),
        _number(entry["heading_deg"], f"{where}.heading_deg"),
    )


# --- the plant ----------------------------------------------------------------


class Plant:
    """The robot in an arena, advanced one :meth:`step` at a time.

    ``arena`` is an :class:`~robocar_sim.arena.Arena` or the mapping
    :func:`~robocar_sim.arena.parse_arena` accepts. ``seed`` seeds the one
    random generator (the rangefinder noise). ``config`` is a robot description
    with the keys of ``plant.yaml``; None loads that file from the package.
    """

    def __init__(
        self,
        arena: Arena | Mapping,
        seed: int,
        start: Pose = Pose(),
        config: Mapping | None = None,
    ) -> None:
        self._arena = arena if isinstance(arena, Arena) else parse_arena(arena)
        if isinstance(seed, bool) or not isinstance(seed, int) or seed < 0:
            raise PlantError(f"seed: expected a non-negative integer, got {seed!r}")
        if not isinstance(start, Pose):
            raise PlantError(f"start: expected a Pose, got {start!r}")
        for name in ("x_cm", "y_cm", "heading_deg"):
            _number(getattr(start, name), f"start.{name}")
        self._robot = _parse_config(default_config() if config is None else config)
        self._rng = np.random.default_rng(seed)
        self._at_ms = 0

        robot = self._robot
        self._left = DCMotor(robot.motor)
        self._right = DCMotor(robot.motor)

        # PhysicsEngine reads the legacy environment layout, in metres and
        # radians. Without a `size` it adds no boundary walls.
        obstacles = [
            {
                "type": "box",
                "position": (o.center[0] / 100.0, o.center[1] / 100.0),
                "size": (o.size[0] / 100.0, o.size[1] / 100.0),
                "angle": math.radians(o.heading_deg),
            }
            if isinstance(o, Box)
            else {
                "type": "cylinder",
                "position": (o.center[0] / 100.0, o.center[1] / 100.0),
                "radius": o.radius_cm / 100.0,
            }
            for o in self._arena.obstacles
        ]
        self._engine = PhysicsEngine({"simulation": {"environment": {"obstacles": obstacles}}})
        space = self._engine.space
        space.collision_slop = COLLISION_SLOP_M
        # Chipmunk keeps `collision_bias` of an overlap after one second.
        space.collision_bias = (1.0 - OVERLAP_CORRECTION_PER_SUBSTEP) ** (1.0 / SUBSTEP_S)
        for wall in self._arena.walls:
            self._engine.add_wall(
                (wall.start[0] / 100.0, wall.start[1] / 100.0),
                (wall.end[0] / 100.0, wall.end[1] / 100.0),
            )

        self._body, self._shape = self._engine.add_robot(
            "plant",
            BODY_MASS_KG,
            {"length": robot.front_overhang + robot.rear_overhang, "width": robot.width},
            position=(start.x_cm / 100.0, start.y_cm / 100.0),
            offset=((robot.front_overhang - robot.rear_overhang) / 2.0, 0.0),
        )
        self._body.angle = math.radians(start.heading_deg)
        self._ray_filter = pymunk.ShapeFilter(group=ROBOT_GROUP)
        self._shape.filter = self._ray_filter

        space.reindex_shapes_for_body(self._body)
        for hit in space.shape_query(self._shape):
            # shape_query reports only overlapping shapes; the sign of the
            # distance depends on the order of the pair.
            depth = max((abs(p.distance) for p in hit.contact_point_set.points), default=0.0)
            if depth > START_OVERLAP_M:
                raise PlantError(
                    f"start: the chassis overlaps a wall or an obstacle by {depth * 100.0:.1f} cm"
                )

    @property
    def at_ms(self) -> int:
        """Simulated time: the sum of every ``dt_ms`` stepped so far."""
        return self._at_ms

    def step(self, dt_ms: int, left: WheelCmd, right: WheelCmd) -> Reading:
        """Hold the two wheel commands for ``dt_ms`` and return the world after it."""
        _integer(dt_ms, "dt_ms", 1, MAX_DT_MS)
        if not isinstance(left, WheelCmd) or not isinstance(right, WheelCmd):
            raise PlantError("left and right: expected WheelCmd")

        collided = False
        for _ in range(dt_ms // SUBSTEP_MS):
            self._advance_wheel(self._left, left)
            self._advance_wheel(self._right, right)
            self._drive_body()
            self._engine.step(SUBSTEP_S)
            if self._in_contact():
                collided = True
                self._wheels_follow_body()
        self._at_ms += dt_ms
        return self._read(collided)

    # -- motion --

    def _advance_wheel(self, motor: DCMotor, cmd: WheelCmd) -> None:
        """One sub-step of a motor under a driver state."""
        before = float(motor.angular_velocity)
        resistance = self._robot.rolling_resistance
        # Rolling resistance opposes the motion; at rest it is part of the
        # holding torque checked below.
        load = math.copysign(resistance, before) if abs(before) > REST_BAND_RAD_S else 0.0
        if cmd.drive == "coast":
            motor.update(0.0, load, SUBSTEP_S, open_circuit=True)
        else:
            duty = cmd.duty / PWM_STEPS
            voltage = {
                "forward": self._robot.supply_v * duty,
                "reverse": -self._robot.supply_v * duty,
                # A shorted winding: 0 V across it, and the back EMF brakes.
                "brake": 0.0,
            }[cmd.drive]
            motor.update(voltage, load, SUBSTEP_S)

        # Friction stops a wheel, it does not reverse it. Where the motor
        # cannot overcome the holding torque, a wheel that has slowed into the
        # rest band, or that an explicit step carried through zero, is at rest.
        after = float(motor.angular_velocity)
        held = abs(float(motor.torque)) <= resistance + motor.FRICTION_STATIC
        if held and (abs(after) <= REST_BAND_RAD_S or before * after < 0.0):
            motor.angular_velocity = 0.0

    def _drive_body(self) -> None:
        """Differential-drive kinematics: wheel speeds to the body's velocity."""
        radius = self._robot.wheel_radius
        v_left = float(self._left.angular_velocity) * radius
        v_right = float(self._right.angular_velocity) * radius
        speed = (v_left + v_right) / 2.0
        heading = self._body.angle
        self._body.velocity = (speed * math.cos(heading), speed * math.sin(heading))
        self._body.angular_velocity = (v_right - v_left) / self._robot.track

    def _in_contact(self) -> bool:
        contacts: list[pymunk.Arbiter] = []
        self._body.each_arbiter(contacts.append)
        return bool(contacts)

    def _wheels_follow_body(self) -> None:
        """The wheels do not slip: a blocked chassis stalls them.

        After contact resolution the body moves only as far as the wall or the
        obstacle lets it. Taking the wheel speeds from that velocity is what
        keeps the chassis out of the obstacle: the next sub-step starts from
        the resolved motion plus one sub-step of motor torque, not from the
        free-running wheel speed.
        """
        heading = self._body.angle
        velocity = self._body.velocity
        speed = velocity.x * math.cos(heading) + velocity.y * math.sin(heading)
        half_turn = self._body.angular_velocity * self._robot.track / 2.0
        radius = self._robot.wheel_radius
        self._left.angular_velocity = (speed - half_turn) / radius
        self._right.angular_velocity = (speed + half_turn) / radius

    # -- sensing --

    def _world(self, mount: _Mount) -> tuple[float, float]:
        """A robot-frame point in the arena frame (m)."""
        heading = self._body.angle
        cos_h, sin_h = math.cos(heading), math.sin(heading)
        position = self._body.position
        return (
            position.x + mount.x * cos_h - mount.y * sin_h,
            position.y + mount.x * sin_h + mount.y * cos_h,
        )

    def _range(self, mount: _Mount, noise_cm: float) -> int | None:
        robot = self._robot
        # The ray reaches past the sensor's range by the noise it could lose.
        reach_cm = robot.range_max_cm + 1.0 + 4.0 * robot.range_noise_cm
        direction = self._body.angle + mount.heading
        x, y = self._world(mount)
        origin = (
            x - RAY_BACKOFF_M * math.cos(direction),
            y - RAY_BACKOFF_M * math.sin(direction),
        )
        distance = self._engine.raycast_distance(
            origin,
            direction,
            reach_cm / 100.0 + RAY_BACKOFF_M,
            self._ray_filter,
            RAY_RADIUS_M,
        )
        if distance is None:
            return None
        reading = math.floor((distance - RAY_BACKOFF_M) * 100.0 + noise_cm + 0.5)
        if not robot.range_min_cm <= reading <= robot.range_max_cm:
            return None
        return reading

    def _on_floor(self, mount: _Mount) -> bool:
        x, y = self._world(mount)
        return self._arena.on_floor(x * 100.0, y * 100.0)

    def _read(self, collided: bool) -> Reading:
        robot = self._robot
        heading = self._body.angle
        position = self._body.position
        # Two draws every step, whatever the rays hit, so the noise a step
        # gets depends on the seed and the step count alone.
        noise = self._rng.normal(0.0, robot.range_noise_cm, size=2)
        half_track = robot.track / 2.0
        return Reading(
            at_ms=self._at_ms,
            pose=Pose(
                position.x * 100.0,
                position.y * 100.0,
                math.degrees(math.atan2(math.sin(heading), math.cos(heading))),
            ),
            range_cm=Ranges(
                front=self._range(robot.range_front, float(noise[0])),
                rear=self._range(robot.range_rear, float(noise[1])),
            ),
            floor=Floor(
                front=self._on_floor(robot.floor_front), rear=self._on_floor(robot.floor_rear)
            ),
            collided=collided,
            off_floor=not (
                self._on_floor(_Mount(0.0, half_track)) and self._on_floor(_Mount(0.0, -half_track))
            ),
        )


# --- the JSONL stepper --------------------------------------------------------


def _wheel(value: Any, where: str) -> WheelCmd:
    entry = _mapping(value, where, ("drive", "duty"))
    try:
        return WheelCmd(entry["drive"], entry["duty"])
    except PlantError as e:
        raise PlantError(f"{where}.{e}") from e


def _expect(message: Any, kind: str, keys: tuple[str, ...]) -> Mapping:
    """A protocol message of one kind with exactly its keys."""
    if not isinstance(message, Mapping):
        raise PlantError(f"{kind}: expected a JSON object, got {message!r}")
    if message.get("t") != kind:
        raise PlantError(f"expected t={kind!r}, got t={message.get('t')!r}")
    return _mapping(message, kind, ("t", *keys))


def _init(message: Any) -> Plant:
    entry = _expect(message, "init", ("seed", "arena", "start"))
    return Plant(entry["arena"], entry["seed"], start=_parse_pose(entry["start"], "init.start"))


def _step(plant: Plant, message: Any) -> dict[str, Any]:
    entry = _expect(message, "step", ("dt_ms", "left", "right"))
    reading = plant.step(
        entry["dt_ms"],
        left=_wheel(entry["left"], "step.left"),
        right=_wheel(entry["right"], "step.right"),
    )
    return reading.to_message()


def run_stepper(lines: Iterable[str], out: TextIO) -> int:
    """Answer ``init`` and ``step`` lines until the input ends.

    Returns the process exit status: 0 at end of input, 1 after a bad message,
    which is answered with one ``error`` line. Nothing else is written to
    ``out``.
    """

    def send(message: dict[str, Any]) -> None:
        out.write(json.dumps(message, separators=(",", ":"), allow_nan=False) + "\n")
        out.flush()

    plant: Plant | None = None
    for line in lines:
        if not line.strip():
            continue
        try:
            try:
                message = json.loads(line)
            except (json.JSONDecodeError, RecursionError) as e:
                raise PlantError(f"not a JSON line: {e}") from e
            if plant is None:
                plant = _init(message)
                reply: dict[str, Any] = {"t": "ready"}
            else:
                reply = _step(plant, message)
        except (ArenaError, PlantError) as e:
            send({"t": "error", "message": str(e)})
            return 1
        send(reply)
    return 0


def main() -> int:
    # Bytes that are not UTF-8 reach json.loads as U+FFFD and are answered as
    # a bad line; a strictly decoding stdin would raise while reading instead.
    if isinstance(sys.stdin, io.TextIOWrapper):
        sys.stdin.reconfigure(encoding="utf-8", errors="replace")
    return run_stepper(sys.stdin, sys.stdout)


if __name__ == "__main__":
    raise SystemExit(main())
