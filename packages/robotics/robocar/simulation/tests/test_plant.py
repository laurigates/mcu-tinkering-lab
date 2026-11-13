"""The plant: determinism, rangefinders, floor sensors, contacts, motors (ADR-036).

Distances here are properties of the model's geometry (a wall 150 cm away
reads 150 cm less the sensor's mount). None of them is a measured figure for
the robot: plant.yaml holds estimates.
"""

import copy

import pytest

from robocar_sim.arena import ArenaError
from robocar_sim.plant import (
    DRIVES,
    Floor,
    Plant,
    PlantError,
    Pose,
    Ranges,
    WheelCmd,
    default_config,
)

TICK_MS = 33
FULL = WheelCmd("forward", 4095)
COAST = WheelCmd("coast")
BRAKE = WheelCmd("brake")

# The default robot description's geometry, read once so the tests below state
# distances from the same figures the plant uses.
CONFIG = default_config()
FRONT_EDGE_CM = CONFIG["chassis"]["front_overhang_cm"]
FRONT_SENSOR_CM = CONFIG["rangefinders"]["front"]["x_cm"]
REAR_SENSOR_CM = -CONFIG["rangefinders"]["rear"]["x_cm"]
HALF_TRACK_CM = CONFIG["wheels"]["track_cm"] / 2

# A room 200 cm long with a wall across each end and none along the sides.
ROOM = {
    "floor": [[-50, -60], [150, -60], [150, 60], [-50, 60]],
    "walls": [{"from": [150, -60], "to": [150, 60]}, {"from": [-50, -60], "to": [-50, 60]}],
}
# Open floor: nothing to see and nothing to hit.
OPEN = {"floor": [[-1000, -1000], [1000, -1000], [1000, 1000], [-1000, 1000]]}
# A table top: the floor ends and there is no wall.
TABLE = {"floor": [[-50, -50], [50, -50], [50, 50], [-50, 50]]}


def quiet_config(**rangefinders):
    """The default robot with no rangefinder noise, so a range is exact."""
    config = copy.deepcopy(CONFIG)
    config["rangefinders"]["noise_cm"] = 0.0
    config["rangefinders"].update(rangefinders)
    return config


def hold(plant, command, ms, right=None):
    """Hold one command for ``ms`` in 33 ms steps; return every reading."""
    return [plant.step(TICK_MS, command, right or command) for _ in range(ms // TICK_MS)]


def script(plant):
    """Every driver state, a turn, and a run into the far wall."""
    readings = hold(plant, WheelCmd("forward", 2457), 1000)
    readings += hold(plant, WheelCmd("forward", 3000), 500, right=WheelCmd("forward", 1500))
    readings += hold(plant, COAST, 500)
    readings += hold(plant, WheelCmd("reverse", 2457), 400, right=WheelCmd("forward", 2457))
    readings += hold(plant, BRAKE, 300)
    readings += hold(plant, FULL, 4000)
    return readings


class TestDeterminism:
    def test_two_plants_with_the_same_arguments_give_identical_readings(self):
        first = script(Plant(ROOM, 7, start=Pose(0.0, 0.0, 0.0)))
        second = script(Plant(ROOM, 7, start=Pose(0.0, 0.0, 0.0)))

        assert first == second
        assert any(r.collided for r in first), "the script should reach the wall"

    def test_the_seed_changes_the_range_noise_and_nothing_else(self):
        first = script(Plant(ROOM, 7))
        second = script(Plant(ROOM, 8))

        assert [r.pose for r in first] == [r.pose for r in second]
        assert [r.range_cm for r in first] != [r.range_cm for r in second]

    def test_a_long_step_equals_the_same_time_in_short_steps(self):
        long_step = Plant(OPEN, 1)
        short_steps = Plant(OPEN, 1)

        after_long = long_step.step(TICK_MS, FULL, WheelCmd("forward", 2000))
        for _ in range(TICK_MS):
            after_short = short_steps.step(1, FULL, WheelCmd("forward", 2000))

        assert after_long.pose == after_short.pose
        assert after_long.at_ms == after_short.at_ms == TICK_MS

    def test_nothing_is_written_to_the_working_directory(self, tmp_path, monkeypatch):
        monkeypatch.chdir(tmp_path)

        script(Plant(ROOM, 3))

        assert list(tmp_path.iterdir()) == []


class TestRangefinders:
    def test_front_and_rear_read_the_distance_to_a_wall(self):
        plant = Plant(ROOM, 0, start=Pose(0.0, 0.0, 0.0), config=quiet_config())

        reading = plant.step(1, COAST, COAST)

        # The walls are 150 cm ahead of and 50 cm behind the axle midpoint.
        assert reading.range_cm == Ranges(
            front=round(150 - FRONT_SENSOR_CM), rear=round(50 - REAR_SENSOR_CM)
        )

    def test_the_default_noise_stays_within_a_centimetre_or_two(self):
        plant = Plant(ROOM, 5)

        fronts = {r.range_cm.front for r in hold(plant, COAST, 3300)}

        assert fronts <= {140, 141, 142, 143, 144}
        assert len(fronts) > 1, "noise_cm is above zero by default"

    def test_the_ray_ignores_the_robots_own_chassis(self):
        # Both sensors mounted at the axle midpoint, well inside the outline.
        inside = quiet_config(
            front={"x_cm": 0.0, "y_cm": 0.0, "heading_deg": 0.0},
            rear={"x_cm": 0.0, "y_cm": 0.0, "heading_deg": 180.0},
        )
        plant = Plant(ROOM, 0, config=inside)

        reading = plant.step(1, COAST, COAST)

        assert reading.range_cm == Ranges(front=150, rear=50)

    @pytest.mark.parametrize(
        ("wall_cm", "expected"),
        [
            (1.0, None),  # closer than the sensor's 2 cm
            (2.0, 2),
            (250.0, 250),
            (400.0, 400),
            (401.0, None),  # past the sensor's 400 cm
            (900.0, None),
        ],
    )
    def test_reads_none_outside_2_to_400_cm(self, wall_cm, expected):
        wall_x = FRONT_SENSOR_CM + wall_cm
        arena = {
            "floor": [[-50, -60], [1000, -60], [1000, 60], [-50, 60]],
            "walls": [{"from": [wall_x, -60], "to": [wall_x, 60]}],
        }
        plant = Plant(arena, 0, config=quiet_config())

        reading = plant.step(1, COAST, COAST)

        assert reading.range_cm == Ranges(front=expected, rear=None)

    @pytest.mark.parametrize(
        ("drive", "sensor", "near_x", "far_x"),
        [("forward", "front", 150, 250), ("reverse", "rear", -150, -250)],
    )
    def test_held_against_a_wall_the_sensor_does_not_read_through_it(
        self, drive, sensor, near_x, far_x
    ):
        # The sensors sit on the chassis edges, and a chassis held on a wall
        # rests a fraction of a millimetre past the wall's line.
        arena = {
            "floor": [[-300, -60], [300, -60], [300, 60], [-300, 60]],
            "walls": [
                {"from": [near_x, -60], "to": [near_x, 60]},
                {"from": [far_x, -60], "to": [far_x, 60]},
            ],
        }
        plant = Plant(arena, 0, config=quiet_config())

        readings = hold(plant, WheelCmd(drive, 4095), 6000)

        held = [getattr(r.range_cm, sensor) for r in readings if r.collided]
        assert len(held) > 30, "the run should end held against the near wall"
        assert set(held) == {None}

    def test_nothing_in_sight_reads_none(self):
        reading = Plant(OPEN, 0).step(1, COAST, COAST)

        assert reading.range_cm == Ranges(front=None, rear=None)

    @pytest.mark.parametrize(
        "obstacle",
        [
            {"box": {"center": [60, 0], "size": [20, 30], "heading_deg": 0}},
            {"cylinder": {"center": [60, 0], "radius_cm": 10}},
        ],
    )
    def test_sees_obstacles(self, obstacle):
        arena = {**OPEN, "obstacles": [obstacle]}
        plant = Plant(arena, 0, config=quiet_config())

        reading = plant.step(1, COAST, COAST)

        # Both obstacles' near faces are 50 cm from the axle midpoint.
        assert reading.range_cm == Ranges(front=round(50 - FRONT_SENSOR_CM), rear=None)

    def test_a_turned_box_is_seen_at_its_corner(self):
        square = {"box": {"center": [60, 0], "size": [20, 20], "heading_deg": 0}}
        diamond = {"box": {"center": [60, 0], "size": [20, 20], "heading_deg": 45}}

        flat = Plant({**OPEN, "obstacles": [square]}, 0, config=quiet_config()).step(
            1, COAST, COAST
        )
        turned = Plant({**OPEN, "obstacles": [diamond]}, 0, config=quiet_config()).step(
            1, COAST, COAST
        )

        # The corner of the turned box is 10 * sqrt(2) cm from its centre.
        assert flat.range_cm.front == round(50 - FRONT_SENSOR_CM)
        assert turned.range_cm.front == round(60 - 14.142 - FRONT_SENSOR_CM)

    @pytest.mark.parametrize(
        ("mount", "expected"),
        [
            # 40 cm to the left, level with the box, whose near face is at x = 50.
            ({"x_cm": 8.0, "y_cm": 40.0, "heading_deg": 0.0}, 42),
            ({"x_cm": 8.0, "y_cm": -40.0, "heading_deg": 0.0}, None),
            # At the axle midpoint, turned to the left toward the box's centre
            # (atan2(40, 60) = 33.69 deg): 50 / cos(33.69 deg) = 60.1 cm.
            ({"x_cm": 0.0, "y_cm": 0.0, "heading_deg": 33.69}, 60),
            ({"x_cm": 0.0, "y_cm": 0.0, "heading_deg": -33.69}, None),
        ],
    )
    def test_a_mount_off_the_centre_line_or_turned_looks_from_where_it_is(self, mount, expected):
        # One box, ahead of the robot and to its left: y is left, and a
        # mount's heading grows counter-clockwise like every other angle.
        arena = {**OPEN, "obstacles": [{"box": {"center": [60, 40], "size": [20, 20]}}]}
        plant = Plant(arena, 0, config=quiet_config(front=mount))

        assert plant.step(1, COAST, COAST).range_cm.front == expected

    def test_a_side_mount_turns_with_the_robot(self):
        # Facing +y, a mount 8 cm ahead and 40 cm to the left is at (-40, 8)
        # and looks along +y at the box's near face, y = 50.
        arena = {**OPEN, "obstacles": [{"box": {"center": [-40, 60], "size": [20, 20]}}]}
        mount = {"x_cm": 8.0, "y_cm": 40.0, "heading_deg": 0.0}
        plant = Plant(arena, 0, start=Pose(0.0, 0.0, 90.0), config=quiet_config(front=mount))

        assert plant.step(1, COAST, COAST).range_cm.front == 42

    @pytest.mark.parametrize(("heading_deg", "expected"), [(30, 45), (-30, 79)])
    def test_a_long_box_turns_counter_clockwise_with_its_heading(self, heading_deg, expected):
        # 60 x 10 cm, centred 10 cm to the left of the ray along y = 0. Turned
        # counter-clockwise its near end comes down across the ray, which
        # enters it at x = 80 - 27.3; turned clockwise the far end does, at
        # x = 80 + 7.3. The sensor is 8 cm ahead of the axle.
        box = {"box": {"center": [80, 10], "size": [60, 10], "heading_deg": heading_deg}}
        plant = Plant({**OPEN, "obstacles": [box]}, 0, config=quiet_config())

        assert plant.step(1, COAST, COAST).range_cm.front == expected

    def test_the_rear_sensor_looks_along_the_heading_reversed(self):
        # Facing +y in the room: no wall ahead or behind, only at the x ends.
        reading = Plant(ROOM, 0, start=Pose(0.0, 0.0, 90.0), config=quiet_config()).step(
            1, COAST, COAST
        )
        assert reading.range_cm == Ranges(front=None, rear=None)

        # Facing the near wall: the far one is behind.
        reading = Plant(ROOM, 0, start=Pose(0.0, 0.0, 180.0), config=quiet_config()).step(
            1, COAST, COAST
        )
        assert reading.range_cm == Ranges(
            front=round(50 - FRONT_SENSOR_CM), rear=round(150 - REAR_SENSOR_CM)
        )


class TestFloorSensors:
    def test_on_the_table_both_sensors_read_floor(self):
        reading = Plant(TABLE, 0).step(1, COAST, COAST)

        assert reading.floor == Floor(front=True, rear=True)
        assert not reading.off_floor

    def test_the_front_sensor_passes_the_edge_before_the_wheels_do(self):
        plant = Plant(TABLE, 0, start=Pose(20.0, 0.0, 0.0))

        readings = hold(plant, WheelCmd("forward", 2000), 3000)

        first_no_floor = next(i for i, r in enumerate(readings) if not r.floor.front)
        first_off = next(i for i, r in enumerate(readings) if r.off_floor)
        assert first_no_floor < first_off
        at_edge = readings[first_no_floor]
        assert at_edge.floor.rear, "the rear sensor is still over the table"
        assert not at_edge.off_floor
        assert readings[first_off].pose.x_cm > 50.0
        assert not any(r.collided for r in readings), "a table edge is not a wall"

    def test_the_rear_sensor_passes_the_edge_when_reversing(self):
        plant = Plant(TABLE, 0, start=Pose(-20.0, 0.0, 0.0))

        readings = hold(plant, WheelCmd("reverse", 2000), 3000)

        first_no_floor = next(i for i, r in enumerate(readings) if not r.floor.rear)
        first_off = next(i for i, r in enumerate(readings) if r.off_floor)
        assert first_no_floor < first_off
        assert readings[first_no_floor].floor.front

    @pytest.mark.parametrize(("mount_y_cm", "floor"), [(30.0, False), (-30.0, True)])
    def test_a_sensor_off_the_centre_line_turns_with_the_robot(self, mount_y_cm, floor):
        # At (-30, 0) facing +y, a sensor 7 cm ahead and 30 cm to the left is
        # at (-60, 7), past the table's edge at x = -50; 30 cm to the right it
        # is at (0, 7). Both wheels are on the table.
        config = copy.deepcopy(CONFIG)
        config["floor_sensors"]["front"] = {"x_cm": 7.0, "y_cm": mount_y_cm}
        plant = Plant(TABLE, 0, start=Pose(-30.0, 0.0, 90.0), config=config)

        reading = plant.step(1, COAST, COAST)

        assert reading.floor == Floor(front=floor, rear=True)
        assert not reading.off_floor

    @pytest.mark.parametrize("side", [1, -1])
    def test_one_wheel_past_the_edge_is_off_the_floor(self, side):
        # The axle midpoint and both sensors are on the table; one wheel is not.
        y = side * (50.0 - HALF_TRACK_CM + 1.0)

        reading = Plant(TABLE, 0, start=Pose(0.0, y, 0.0)).step(1, COAST, COAST)

        assert reading.off_floor
        assert reading.floor == Floor(front=True, rear=True)

    def test_off_floor_clears_when_the_wheels_are_back(self):
        plant = Plant(TABLE, 0, start=Pose(49.0, 0.0, 0.0))

        went_off = hold(plant, WheelCmd("forward", 2000), 1000)
        came_back = hold(plant, WheelCmd("reverse", 2000), 3000)

        assert went_off[-1].off_floor
        assert not came_back[-1].off_floor


class TestContacts:
    def test_full_duty_at_a_wall_for_five_seconds_does_not_enter_it(self):
        plant = Plant(ROOM, 0)

        readings = hold(plant, FULL, 5000)

        front_edge = readings[-1].pose.x_cm + FRONT_EDGE_CM
        assert abs(front_edge - 150.0) <= 1.0
        assert readings[-1].collided
        assert not readings[0].collided
        deepest = max(r.pose.x_cm for r in readings) + FRONT_EDGE_CM
        assert deepest - 150.0 <= 1.0, "nor at any step on the way"

    def test_blocked_wheels_stall_at_the_wall_face(self):
        """A blocked chassis stalls its wheels (they do not slip).

        Free-running wheels behind a blocked chassis would push it about
        0.6 cm into the wall, where the overlap correction balances the drive.
        """
        plant = Plant(ROOM, 0)

        readings = hold(plant, FULL, 5000)

        assert abs(readings[-1].pose.x_cm + FRONT_EDGE_CM - 150.0) <= 0.2
        # Held still against the wall, to the 0.001 the stepper reports.
        assert readings[-1].to_message()["pose"] == readings[-30].to_message()["pose"]

    @pytest.mark.parametrize(
        "obstacle",
        [
            {"box": {"center": [70, 0], "size": [20, 60], "heading_deg": 0}},
            {"cylinder": {"center": [70, 0], "radius_cm": 10}},
        ],
    )
    def test_does_not_enter_an_obstacle(self, obstacle):
        plant = Plant({**OPEN, "obstacles": [obstacle]}, 0)

        readings = hold(plant, FULL, 5000)

        # Both near faces are at x = 60 on the robot's centre line.
        assert abs(readings[-1].pose.x_cm + FRONT_EDGE_CM - 60.0) <= 1.0
        assert readings[-1].collided

    def test_collided_clears_after_backing_off(self):
        plant = Plant(ROOM, 0, start=Pose(120.0, 0.0, 0.0))

        at_wall = hold(plant, FULL, 2000)
        backed_off = hold(plant, WheelCmd("reverse", 3000), 1000)

        assert at_wall[-1].collided
        assert not backed_off[-1].collided
        assert backed_off[-1].pose.x_cm < at_wall[-1].pose.x_cm - 5.0

    def test_a_step_that_starts_in_contact_reports_it_though_it_ends_clear(self):
        # The front edge starts 0.1 cm from the wall at x = 150.
        plant = Plant(ROOM, 0, start=Pose(141.9, 0.0, 0.0))
        back = WheelCmd("reverse", 4095)

        into = plant.step(300, FULL, FULL)
        away = plant.step(1000, back, back)
        after = plant.step(TICK_MS, back, back)

        assert into.collided
        # Touching for the first part of this step and far from the wall at its end.
        assert away.collided
        assert away.pose.x_cm < into.pose.x_cm - 20.0
        assert not after.collided

    def test_free_running_never_reports_a_collision(self):
        assert not any(r.collided for r in hold(Plant(OPEN, 0), FULL, 3000))

    def test_a_start_inside_a_wall_is_refused(self):
        with pytest.raises(PlantError, match="start: the chassis overlaps"):
            Plant(ROOM, 0, start=Pose(148.0, 0.0, 0.0))


def speeds_cm_s(readings):
    xs = [0.0] + [r.pose.x_cm for r in readings]
    return [(b - a) / (TICK_MS / 1000) for a, b in zip(xs, xs[1:], strict=False)]


class TestMotors:
    def test_brake_stops_shorter_than_coast_from_the_same_speed(self):
        distances = {}
        for name, stop in (("coast", COAST), ("brake", BRAKE)):
            plant = Plant(OPEN, 0)
            at_speed = hold(plant, FULL, 2000)[-1]
            stopping = hold(plant, stop, 5000)
            assert stopping[-1].pose == stopping[-10].pose, f"{name} comes to rest"
            distances[name] = stopping[-1].pose.x_cm - at_speed.pose.x_cm

        assert 0.0 < distances["brake"] < distances["coast"]
        assert distances["coast"] > 3 * distances["brake"]

    def test_33_ms_steps_are_stable(self):
        config = CONFIG
        no_load_cm_s = (
            config["supply_v"]
            / config["motor"]["back_emf_v_s_per_rad"]
            * config["wheels"]["diameter_cm"]
            / 2
        )

        speeds = speeds_cm_s(hold(Plant(OPEN, 0), FULL, 10000))

        # Bounded by the speed where the back EMF equals the supply.
        assert max(speeds) < no_load_cm_s
        # Rises to a steady speed with no overshoot and no growth.
        assert all(b >= a - 1e-6 for a, b in zip(speeds, speeds[1:], strict=False))
        assert speeds[-1] > 0.8 * no_load_cm_s
        assert max(speeds[-30:]) - min(speeds[-30:]) < 1e-3 * speeds[-1]

    def test_speed_follows_the_duty(self):
        half = speeds_cm_s(hold(Plant(OPEN, 0), WheelCmd("forward", 2048), 4000))[-1]
        full = speeds_cm_s(hold(Plant(OPEN, 0), FULL, 4000))[-1]

        assert 0.4 * full < half < 0.6 * full

    def test_speed_follows_the_supply_voltage(self):
        low = copy.deepcopy(CONFIG)
        low["supply_v"] = 3.0

        at_3v = speeds_cm_s(hold(Plant(OPEN, 0, config=low), FULL, 4000))[-1]
        at_5v = speeds_cm_s(hold(Plant(OPEN, 0), FULL, 4000))[-1]

        assert 0.5 * at_5v < at_3v < 0.7 * at_5v

    def test_a_duty_too_small_to_overcome_friction_does_not_move_the_robot(self):
        readings = hold(Plant(OPEN, 0), WheelCmd("forward", 100), 2000)

        assert readings[-1].pose == Pose(0.0, 0.0, 0.0)

    def test_coast_and_brake_leave_a_resting_robot_at_rest(self):
        for stop in (COAST, BRAKE):
            readings = hold(Plant(OPEN, 0, start=Pose(3.0, 4.0, 30.0)), stop, 1000)
            assert readings[-1].pose == Pose(3.0, 4.0, pytest.approx(30.0))

    def test_reverse_drives_backward(self):
        readings = hold(Plant(OPEN, 0), WheelCmd("reverse", 3000), 2000)

        assert readings[-1].pose.x_cm < -20.0
        assert readings[-1].pose.y_cm == pytest.approx(0.0, abs=1e-9)

    def test_opposite_wheels_turn_in_place_counter_clockwise(self):
        plant = Plant(OPEN, 0)

        readings = hold(plant, WheelCmd("reverse", 3000), 500, right=WheelCmd("forward", 3000))

        assert readings[-1].pose.heading_deg > 30.0
        assert readings[-1].pose.x_cm == pytest.approx(0.0, abs=1e-6)
        assert readings[-1].pose.y_cm == pytest.approx(0.0, abs=1e-6)

    def test_a_slower_right_wheel_curves_to_the_right(self):
        readings = hold(Plant(OPEN, 0), FULL, 500, right=WheelCmd("forward", 2000))

        assert -90.0 < readings[-1].pose.heading_deg < -10.0
        assert readings[-1].pose.y_cm < 0.0
        assert readings[-1].pose.x_cm > 0.0

    def test_heading_90_drives_along_y(self):
        readings = hold(Plant(OPEN, 0, start=Pose(0.0, 0.0, 90.0)), FULL, 1000)

        assert readings[-1].pose.y_cm > 20.0
        assert readings[-1].pose.x_cm == pytest.approx(0.0, abs=1e-6)
        assert readings[-1].pose.heading_deg == pytest.approx(90.0)

    def test_heading_wraps_into_plus_minus_180(self):
        plant = Plant(OPEN, 0, start=Pose(0.0, 0.0, 170.0))

        readings = hold(plant, WheelCmd("reverse", 3000), 1500, right=WheelCmd("forward", 3000))

        assert all(-180.0 <= r.pose.heading_deg <= 180.0 for r in readings)
        assert readings[-1].pose.heading_deg < 0.0, "turned past 180 and wrapped"


class TestStep:
    def test_at_ms_is_the_sum_of_the_steps(self):
        plant = Plant(OPEN, 0)

        first = plant.step(33, COAST, COAST)
        second = plant.step(7, COAST, COAST)

        assert (first.at_ms, second.at_ms, plant.at_ms) == (33, 40, 40)

    @pytest.mark.parametrize("dt_ms", [0, -1, 1001, 33.0, "33", True, None])
    def test_refuses_a_bad_dt(self, dt_ms):
        with pytest.raises(PlantError, match="dt_ms"):
            Plant(OPEN, 0).step(dt_ms, COAST, COAST)

    def test_refuses_a_command_that_is_not_a_wheel_cmd(self):
        with pytest.raises(PlantError, match="WheelCmd"):
            Plant(OPEN, 0).step(33, "forward", COAST)

    def test_the_message_has_the_stepper_fields(self):
        message = Plant(ROOM, 0, config=quiet_config()).step(33, COAST, COAST).to_message()

        assert message == {
            "t": "reading",
            "at_ms": 33,
            "pose": {"x_cm": 0.0, "y_cm": 0.0, "heading_deg": 0.0},
            "range_cm": {"front": round(150 - FRONT_SENSOR_CM), "rear": round(50 - REAR_SENSOR_CM)},
            "floor": {"front": True, "rear": True},
            "collided": False,
            "off_floor": False,
        }


class TestWheelCmd:
    @pytest.mark.parametrize("drive", DRIVES)
    def test_accepts_the_four_driver_states(self, drive):
        assert WheelCmd(drive, 4095).drive == drive

    @pytest.mark.parametrize("drive", ["stop", "FORWARD", "", None, 1])
    def test_refuses_another_drive(self, drive):
        with pytest.raises(PlantError, match="drive"):
            WheelCmd(drive, 0)

    @pytest.mark.parametrize("duty", [-1, 4096, 100.0, "100", True, None])
    def test_refuses_a_duty_outside_the_pca9685_counts(self, duty):
        with pytest.raises(PlantError, match="duty"):
            WheelCmd("forward", duty)


class TestArguments:
    @pytest.mark.parametrize("seed", [-1, 1.5, "7", True, None])
    def test_refuses_a_bad_seed(self, seed):
        with pytest.raises(PlantError, match="seed"):
            Plant(OPEN, seed)

    def test_refuses_a_start_that_is_not_a_pose(self):
        with pytest.raises(PlantError, match="start"):
            Plant(OPEN, 0, start=(0, 0, 0))

    def test_refuses_a_start_that_is_not_finite(self):
        with pytest.raises(PlantError, match="start.x_cm"):
            Plant(OPEN, 0, start=Pose(float("nan"), 0.0, 0.0))

    def test_an_invalid_arena_is_an_arena_error(self):
        with pytest.raises(ArenaError, match="arena: missing key"):
            Plant({}, 0)


class TestConfig:
    def test_the_default_description_states_what_the_repo_holds(self):
        # 5 V from the buck (WIRING.md); 2..400 cm accepted (ultrasonic_core.h).
        assert CONFIG["supply_v"] == 5.0
        assert (CONFIG["rangefinders"]["min_cm"], CONFIG["rangefinders"]["max_cm"]) == (2, 400)
        assert set(CONFIG["rangefinders"]) >= {"front", "rear"}
        assert set(CONFIG["floor_sensors"]) == {"front", "rear"}

    def test_the_default_description_is_read_from_the_package(self, tmp_path, monkeypatch):
        (tmp_path / "plant.yaml").write_text("supply_v: 99\n")
        monkeypatch.chdir(tmp_path)

        assert default_config() == CONFIG

    def test_refuses_an_unknown_key(self):
        config = copy.deepcopy(CONFIG)
        config["chassis"]["mass_kg"] = 0.6

        with pytest.raises(PlantError, match=r"config\.chassis: unknown key\(s\) mass_kg"):
            Plant(OPEN, 0, config=config)

    def test_refuses_a_missing_key(self):
        config = copy.deepcopy(CONFIG)
        del config["wheels"]["track_cm"]

        with pytest.raises(PlantError, match=r"config\.wheels: missing key\(s\) track_cm"):
            Plant(OPEN, 0, config=config)

    @pytest.mark.parametrize(
        ("section", "key", "value"),
        [
            ("wheels", "diameter_cm", 0),
            ("wheels", "track_cm", -13),
            ("motor", "resistance_ohm", 0),
            ("motor", "inertia_kg_m2", float("inf")),
            ("motor", "rolling_resistance_nm", -0.001),
            ("rangefinders", "noise_cm", -1),
            ("rangefinders", "max_cm", 1),
            ("rangefinders", "min_cm", 2.5),
            ("chassis", "width_cm", "15"),
        ],
    )
    def test_refuses_a_figure_out_of_range(self, section, key, value):
        config = copy.deepcopy(CONFIG)
        config[section][key] = value

        with pytest.raises(PlantError, match=rf"config\.{section}\.{key}"):
            Plant(OPEN, 0, config=config)

    def test_refuses_an_inductance_the_integration_step_cannot_follow(self):
        config = copy.deepcopy(CONFIG)
        config["motor"]["inductance_h"] = 0.0001

        with pytest.raises(PlantError, match="integration step overshoots"):
            Plant(OPEN, 0, config=config)
