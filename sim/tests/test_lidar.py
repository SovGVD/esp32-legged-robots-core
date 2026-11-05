"""Lidar (LD06) mapping and walking to a point: the firmware's driver, map,
scan matching and planner against the simulated lidar (legsim/lidar.py).
And the same map and planner fed by ToF room scans (tof_scan.h) instead."""
import math

import numpy as np
import pytest

from legsim import scenario, terrain
from legsim.lidar import ld06_crc8, ld06_packet

from test_terrain import needs_cxx


def test_ld06_packet_format():
    p = ld06_packet(3600, 35500, 400, list(range(1000, 1012)), [200] * 12, 1234)
    assert len(p) == 47 and p[0] == 0x54 and p[1] == 0x2C
    assert ld06_crc8(p[:-1]) == p[-1]
    assert ld06_crc8(b"\x01") == 0x4D


@needs_cxx
@pytest.mark.parametrize("model", ["dog", "hexapod"])
def test_lidar_maps_the_room_and_walks_to_a_point(model):
    fw = scenario.core(model)
    try:
        fw.set_heading_sensors(gyro_bias_dps=1.0, magnetometer=False)
        fw.set_lidar(True)
        robot = fw.place(scenario.make_robot(model, terrain.rooms(scale=1.0 if model == "dog" else 1.3)))
        fw.drive(robot, duration=1.0)
        st = fw.lidar_state()
        assert st["crc_errors"] == 0 and st["packets"] > 300   # ~375 packets/s
        assert st["scans"] >= 8 and st["occupied"] > 80   # 10 cm cells
        # the map has the walls: the back wall is 900 mm behind the dog (1170 the hexapod)
        grid, cell = fw.lidar_map()
        n = grid.shape[0]
        back = -900 * (1.0 if model == "dog" else 1.3)
        row = int(math.floor(back / cell)) + n // 2
        assert (grid[row - 1:row + 2, n // 2 - 10:n // 2 + 10] > 0).sum() >= 10

        goal = (-500, 1500)
        assert fw.nav_goal(*goal)
        for _ in range(int(30 / fw.loop_dt)):
            fw.drive(robot)
            if fw.nav_state()["state"] != "active":
                break
        assert fw.nav_state()["state"] == "arrived"
        p = robot.body_position_mm()
        assert math.hypot(p[0] - goal[0], p[1] - goal[1]) < 250
        mp, op = fw.lidar_pose()
        assert math.hypot(mp[0] - p[0], mp[1] - p[1]) < 80          # lidar pose vs truth
        yaw = robot.body_orientation()["yaw"]
        assert abs((mp[2] - yaw + math.pi) % (2 * math.pi) - math.pi) < math.radians(3)
        assert fw.lidar_state()["rejected"] < 5
    finally:
        fw.set_lidar(False)
        fw.set_heading_sensors(None)
        fw.reset()


@needs_cxx
def test_stick_cancels_the_goal():
    fw = scenario.core("dog")
    try:
        fw.set_lidar(True)
        robot = fw.place(scenario.make_robot("dog", terrain.rooms()))
        fw.drive(robot, duration=0.5)
        fw.nav_goal(0, 1500)
        fw.drive(robot, duration=0.5)
        assert fw.nav_state()["state"] == "active"
        fw.drive(robot, (0, 0.5, 0, 0, 0, 0))
        assert fw.nav_state()["state"] == "idle"
    finally:
        fw.set_lidar(False)
        fw.reset()


@needs_cxx
@pytest.mark.parametrize("model", ["dog", "hexapod"])
def test_tof_room_scans_map_and_walk_to_a_point(model):
    fw = scenario.core(model)
    try:
        fw.tune_tof_scan(reset=True)   # the model's TOF_SENSORS mounts and TOF_SCAN_PARAMS
        fw.set_heading_sensors(gyro_bias_dps=1.0, magnetometer=False)
        fw.set_tof_nav(True, subrays=2, latency=0.04)
        s = 1.0 if model == "dog" else 1.3
        robot = fw.place(scenario.make_robot(model, terrain.rooms(scale=s)))
        fw.drive(robot, duration=1.0)
        goal = (-500 * s, 1500 * s)
        assert fw.nav_goal(*goal)
        phases = set()
        for _ in range(int(120 / fw.loop_dt)):
            fw.drive(robot)
            ns = fw.nav_state()
            phases.add(ns["phase"])
            if ns["state"] != "active" and ns["phase"] == "idle":
                break
        assert {"stop", "tilt", "turn", "level"} <= phases
        assert fw.nav_state()["state"] == "arrived"
        st = fw.lidar_state()
        assert st["scans"] >= 2 and st["rejected"] <= 1
        p = robot.body_position_mm()
        assert math.hypot(p[0] - goal[0], p[1] - goal[1]) < 350
        mp, op = fw.lidar_pose()
        assert math.hypot(mp[0] - p[0], mp[1] - p[1]) < 250
        yaw = robot.body_orientation()["yaw"]
        assert abs((mp[2] - yaw + math.pi) % (2 * math.pi) - math.pi) < math.radians(8)
        # the back wall is in the map
        grid, cell = fw.lidar_map()
        n = grid.shape[0]
        row = int(math.floor(-900 * s / cell)) + n // 2
        assert (grid[row - 2:row + 3, n // 2 - 8:n // 2 + 8] > 0).sum() >= 5
    finally:
        fw.set_tof_nav(False)
        fw.tune_tof_scan(reset=True)
        fw.set_heading_sensors(None)
        fw.reset()


@needs_cxx
@pytest.mark.parametrize("model", ["dog", "hexapod"])
def test_tof_room_scan_with_terrain_sensing_on(model):
    """The same ToF sensors feed the terrain map and the room scan: the scan still
    sees the walls (the terrain map pauses, the IMU doesn't make up a tilt while
    the robot turns with its nose up)."""
    fw = scenario.core(model)
    try:
        fw.tune_tof_scan(reset=True)
        fw.set_heading_sensors(gyro_bias_dps=1.0, magnetometer=False)
        fw.set_leveling(True)
        fw.set_terrain(**scenario.SENSING["tof"])
        fw.select_gait("rough")
        fw.set_tof_nav(True, subrays=2, latency=0.04)
        s = 1.0 if model == "dog" else 1.3
        robot = fw.place(scenario.make_robot(model, terrain.rooms(scale=s)))
        fw.drive(robot, duration=1.0)
        fw.tof_scan()
        t, tilts = 0.0, []
        while t < 60:
            fw.drive(robot)
            t += fw.loop_dt
            ns = fw.nav_state()
            if ns["phase"] == "turn":
                o = robot.body_orientation()
                tilts.append(abs(o["roll"]))   # sideways
            if t > 2 and ns["phase"] == "idle":
                break
        assert fw.nav_state()["phase"] == "idle" and fw.lidar_state()["scans"] == 1
        pts = fw.lidar_points()
        assert len(pts) > 200
        assert np.hypot(pts[:, 0], pts[:, 1]).min() > 500          # no floor seen as walls
        assert np.mean(tilts) < math.radians(3)                      # no made-up sideways lean
    finally:
        fw.set_tof_nav(False)
        fw.set_terrain()
        fw.set_leveling(False)
        fw.select_gait("default")
        fw.set_heading_sensors(None)
        fw.reset()


@needs_cxx
def test_dog_walks_to_a_goal_with_terrain_sensing_on():
    """Room scans, the nav and the ToF terrain map together: the dog walks from the
    lower room through the door (over the threshold, past the uneven tiles) to a
    goal in the upper room without tipping over."""
    fw = scenario.core("dog")
    try:
        fw.tune_tof_scan(reset=True)
        fw.set_heading_sensors(gyro_bias_dps=1.0, magnetometer=False)
        fw.set_leveling(True)
        fw.set_terrain(**scenario.SENSING["tof"])
        fw.select_gait("rough")
        fw.set_tof_nav(True, subrays=2, latency=0.04)
        robot = fw.place(scenario.make_robot("dog", terrain.rooms(floor=20)))
        fw.drive(robot, duration=1.0)
        goal = (1100, 2900)
        fw.nav_goal(*goal)
        t, worst = 0.0, 0.0
        while t < 150:
            fw.drive(robot)
            t += fw.loop_dt
            o = robot.body_orientation()
            worst = max(worst, abs(o["pitch"]), abs(o["roll"]))
            ns = fw.nav_state()
            if ns["state"] != "active" and ns["phase"] == "idle":
                break
        p = robot.body_position_mm()
        assert ns["state"] == "arrived"
        assert math.hypot(p[0] - goal[0], p[1] - goal[1]) < 500
        assert worst < math.radians(30)
    finally:
        fw.set_tof_nav(False)
        fw.set_terrain()
        fw.set_leveling(False)
        fw.select_gait("default")
        fw.set_heading_sensors(None)
        fw.reset()


@needs_cxx
def test_tof_room_scan_with_a_sensor_tilt_servo():
    """A servo tilts the hexapod's ToF sensors up for a scan (TofScanParams sensorServo)
    instead of the body: both sensors 35 deg down for the ground while walking, 17 deg
    for the walls. The body stays level and the scan sees the walls, not the floor."""
    fw = scenario.core("hexapod")
    try:
        for i in range(len(fw.tof_sensors())):
            fw.set_tof_mount(i, 35)
        fw.tune_tof_scan(reset=True, sensor_servo=1, tilt=math.radians(18), tilt_rate=1.0)
        fw.set_heading_sensors(gyro_bias_dps=1.0, magnetometer=False)
        fw.set_tof_nav(True, subrays=2, latency=0.04)
        robot = fw.place(scenario.make_robot("hexapod", terrain.rooms(scale=1.3)))
        fw.drive(robot, duration=1.0)
        fw.tof_scan()
        t, lean, up = 0.0, [], 0.0
        while t < 60:
            fw.drive(robot)
            t += fw.loop_dt
            ns = fw.nav_state()
            up = max(up, ns["sensor_tilt"])
            if ns["phase"] == "turn":
                o = robot.body_orientation()
                lean.append(max(abs(o["roll"]), abs(o["pitch"])))
            if t > 2 and ns["phase"] == "idle":
                break
        assert fw.nav_state()["phase"] == "idle" and fw.lidar_state()["scans"] == 1
        assert up == pytest.approx(math.radians(18), abs=1e-3) and fw.nav_state()["sensor_tilt"] == 0
        assert max(lean) < math.radians(3)                           # no nose up
        pts = fw.lidar_points()
        assert len(pts) > 200
        assert np.hypot(pts[:, 0], pts[:, 1]).min() > 500          # no floor seen as walls
    finally:
        fw.set_tof_nav(False)
        for i in range(len(fw.tof_sensors())):
            fw.set_tof_mount(i, None)
        fw.tune_tof_scan(reset=True)
        fw.set_heading_sensors(None)
        fw.reset()
