"""Lidar (LD06) mapping and walking to a point: the firmware's driver, map,
scan matching and planner against the simulated lidar (legsim/lidar.py)."""
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
        assert st["scans"] >= 8 and st["occupied"] > 200
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
