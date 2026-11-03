"""
Rough ground: terrain scenes, the simulated foot switches and ToF sensors,
and the firmware's terrain code (lib/robot_core GaitEngine terrain mode +
TerrainMap) walking the dog and the hexapod over obstacles.
"""

import math
import shutil

import numpy as np
import pytest

from legsim import SimDog, terrain
from legsim.hexapod import SimHexapod

needs_cxx = pytest.mark.skipif(shutil.which("c++") is None and shutil.which("g++") is None,
                               reason="needs a C++ compiler for the firmware bridge")


def test_terrain_boxes_are_in_the_model_and_collide():
    t = terrain.step(height=20, y0=50, length=200)
    assert t.height_at(0, 100) == 20 and t.height_at(0, 0) == 0
    dog = SimDog(terrain=t)
    assert dog.model.geom("terrain_0").contype == 1
    # a box under the start: the robot is put on top of it
    d = terrain.drop(height=15, y0=200)
    dog = SimDog(terrain=d)
    dog.step(0.3)
    assert abs(dog.body_position_mm()[2] - (15 + 86)) < 4


@pytest.mark.parametrize("robot_cls", [SimDog, SimHexapod])
def test_foot_switches_carry_the_weight(robot_cls):
    robot = robot_cls()
    robot.step(0.5)
    forces = robot.foot_vertical_forces()
    weight = sum(robot.model.body_mass) * 9.81
    assert abs(forces.sum() - weight) < 0.1 * weight
    assert all(robot.foot_switches())
    # lifted: nothing pressed
    robot.data.qpos[2] += 0.05
    robot.data.qvel[:] = 0
    import mujoco
    mujoco.mj_forward(robot.model, robot.data)
    assert not any(robot.foot_switches())


@needs_cxx
def test_tof_sees_the_floor_and_the_map_puts_it_under_the_feet():
    from legsim import FirmwareCore
    fw = FirmwareCore()
    try:
        fw.set_terrain(tof=True, noise_mm=0, noise_rel=0)
        fw.reset()
        dog = fw.place(SimDog())
        dog.step(0.3)
        sensors = fw.tof_sensors()
        assert len(sensors) >= 1 and len(sensors[0]["rays"]) == 64
        frame = fw.tof.measure(dog)[0]
        assert (frame > 0).sum() > 40          # most zones see the floor
        fw.drive(dog, duration=0.2)            # standing still, a few frames
        assert fw.terrain_cells() > 30
        # flat floor under the foot balls' centres: the map is the floor surface
        found = [fw.terrain_height(x, y) for x in range(-60, 61, 6) for y in range(120, 340, 6)]
        found = np.array([z for z in found if z is not None])
        assert len(found) > 50
        assert np.all(np.abs(found - (-80 - 6)) < 3)
    finally:
        fw.set_terrain()


@needs_cxx
def test_dog_climbs_a_step_with_tof_that_stops_it_open_loop():
    from legsim.scenario import run, scene
    t = scene("dog", "step", 20)
    blind = run("dog", t, "none", gait="rough", speed=1.0)
    seeing = run("dog", t, "tof", gait="rough", speed=1.0)
    assert blind.climbed < 1 and blind.distance < 300          # front feet stuck at the edge
    assert seeing.climbed == pytest.approx(20) and seeing.distance > 550
    assert seeing.mean_tilt < 4 and not seeing.fell


@needs_cxx
def test_hexapod_climbs_a_30mm_step_level_with_tof_and_switches():
    from legsim.scenario import run, scene
    t = scene("hexapod", "step", 30)
    blind = run("hexapod", t, "none", gait="rough", speed=1.0)
    both = run("hexapod", t, "both", gait="rough", speed=1.0)
    assert both.climbed == pytest.approx(30) and both.distance > 600
    # level and in step: less tilt and drift than stumbling up open loop
    assert both.mean_tilt < 2 and both.mean_tilt < blind.mean_tilt
    assert both.distance > blind.distance
    assert both.rejects["reach"] == 0


@needs_cxx
def test_terrain_mode_on_flat_ground_walks_like_open_loop():
    from legsim.scenario import run
    for model in ("dog", "hexapod"):
        a = run(model, terrain.flat(), "none", gait="rough", speed=1.0, duration=4)
        b = run(model, terrain.flat(), "both", gait="rough", speed=1.0, duration=4)
        assert b.distance > 0.9 * a.distance
        assert b.mean_tilt < 2.5, model


def test_rubble_has_mixed_heights_everywhere():
    t = terrain.rubble(max_height=20)
    heights = [b[2] for b in t.boxes]
    assert len(heights) > 100
    assert min(heights) < 5 and max(heights) > 15
    # about half the path is covered
    import numpy as np
    pts = [(x, y) for x in np.linspace(-100, 100, 9) for y in np.linspace(300, 1100, 41)]
    assert 0.3 < sum(t.height_at(x, y) > 0 for x, y in pts) / len(pts) < 0.7
