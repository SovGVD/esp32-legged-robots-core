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
    # Single runs are noisy: one stumble on the step's edge costs ~130 mm and whether it happens
    # is down to chance, and the clamped footholds swing as much (seeds 0-5 at 1122 g: 461-627 mm,
    # 437-704 rejects; at 1104 g: 479-638 mm, 457-587). So 6 seeds are judged together.
    runs = [run("hexapod", t, "both", gait="rough", speed=1.0, seed=s) for s in range(6)]
    median = lambda xs: sorted(xs)[len(xs) // 2]
    dist = median([r.distance for r in runs])
    tilt = sum(r.mean_tilt for r in runs) / len(runs)
    assert all(r.climbed == pytest.approx(30) for r in runs) and dist > 550
    # level and in step: less tilt and drift than stumbling up open loop
    assert tilt < 2.7 and tilt < blind.mean_tilt
    # and about as far: the blind run (no seed dependence) makes 567 mm; the 6 seeing runs spread 466-628
    # around it (median 558 at 1139 g, just over 567 at 1138 g), so the distance is no better than a coin toss
    assert dist > 0.95 * blind.distance
    # the short direct-drive legs (60/50/60) can't reach many of the footholds the terrain code
    # picks on the step: clamped to the reach, it still climbs level. Fewer is the open job
    # (gait / foothold search)
    assert median([r.rejects["reach"] for r in runs]) < 650


@needs_cxx
def test_terrain_mode_on_flat_ground_walks_like_open_loop():
    from legsim.scenario import run
    for model in ("dog", "hexapod"):
        a = run(model, terrain.flat(), "none", gait="rough", speed=1.0, duration=4)
        b = run(model, terrain.flat(), "both", gait="rough", speed=1.0, duration=4)
        assert b.distance > 0.9 * a.distance
        assert b.mean_tilt < 2.5, model


# Known failure: at the CAD's 1120 g the hexapod's map floor drifts 9.4 mm in 40 s and
# levels off at 13.1 mm by 90 s. It depends on the mass: 3.6 mm at 815 g, 4.5 mm at 900 g.
_HEXAPOD_DRIFT = pytest.mark.xfail(reason="hexapod map floor drifts at 1120 g, not solved yet", strict=False)


@needs_cxx
@pytest.mark.parametrize("model", ["dog", pytest.param("hexapod", marks=_HEXAPOD_DRIFT)])
def test_terrain_map_stays_on_the_floor_on_a_long_walk(model):
    """40 s straight on a flat floor with the ToF terrain map (no foot switches):
    the map's floor stays under the feet. The IMU's tilt is 1-2 deg off while
    the robot trots; placed with it, the dog's floor crept up ~2 mm/s, and the
    feet stepped onto ground that isn't there (FrameTilt fixes the tilt from the
    floor the ToF sees, TerrainMap::frameOffset the height)."""
    from legsim import scenario
    fw = scenario.core(model)
    try:
        fw.set_leveling(True)
        fw.set_terrain(**scenario.SENSING["tof"])
        fw.select_gait("rough")
        robot = fw.place(scenario.make_robot(model, terrain.flat()))
        t, worst = 0.0, 0.0
        while t < 40:
            fw.drive(robot, (0, 0.8, 0, 0, 0, 0))
            t += fw.loop_dt
            worst = max(worst, abs(fw.terrain_state()["body"]))
        assert robot.body_position_mm()[1] > 2000
        # hexapod at 900 g: 4.5 mm in 40 s, levels off at 8.3 mm by 90 s. Sensitive to
        # the masses: 3.6 mm at 815 g, 8.8 mm at 924 g
        assert worst < 6
    finally:
        fw.set_terrain()
        fw.set_leveling(False)
        fw.select_gait("default")
        fw.reset()


def test_rubble_has_mixed_heights_everywhere():
    t = terrain.rubble(max_height=20)
    heights = [b[2] for b in t.boxes]
    assert len(heights) > 100
    assert min(heights) < 5 and max(heights) > 15
    # about half the path is covered
    import numpy as np
    pts = [(x, y) for x in np.linspace(-100, 100, 9) for y in np.linspace(300, 1100, 41)]
    assert 0.3 < sum(t.height_at(x, y) > 0 for x, y in pts) / len(pts) < 0.7


def test_gaps_and_bridge_scenes():
    g = terrain.gaps(height=40, widths=(25, 50), segment=250, y0=150)
    assert g.start_height == 40
    assert g.height_at(0, 100) == 40                 # on the raised floor
    assert g.height_at(0, 160) == 0                  # in the first trench
    assert g.height_at(0, 190) == 40
    assert g.height_at(0, 150 + 25 + 250 + 10) == 0  # the second, wider one
    b = terrain.bridge(height=50, ramp=300, deck=900, gap_widths=(25, 40), width=300, y0=200)
    assert len(b.ramps) == 2
    assert b.height_at(0, 100) == 0
    assert b.height_at(0, 350) == pytest.approx(25, abs=1)   # half way up the ramp
    assert b.height_at(0, 600) == 50                          # on the deck
    assert b.height_at(400, 600) == 0                         # off the side
    assert b.max_height() == 50
    dog = SimDog(terrain=b)
    assert dog.model.geom("ramp_0") is not None


@needs_cxx
def test_hexapod_reaches_a_40mm_gap_without_falling():
    """The direct-drive hexapod's short legs (60/50/60) don't reliably step over
    a 40 mm gap yet: the ToFs see the far edge too late for the foothold, and
    a foot dips in (tilt ~20-30 deg), with any mount angle tried (22/35/40/45 deg,
    low or high). What's checked: walking with the ToF it gets to the gap and
    doesn't fall. Crossing it is open (the scans matter more for now)."""
    from legsim.scenario import DEFAULTS, run
    d = DEFAULTS["hexapod"]
    t = terrain.gaps(height=70, widths=(40,), segment=600, y0=400)
    blind = run("hexapod", t, "none", gait=d["gait"], speed=d["speed"], duration=12)
    seeing = run("hexapod", t, "tof", gait=d["gait"], speed=d["speed"], duration=12)
    assert blind.max_tilt > 15                       # a foot goes into the gap
    assert seeing.distance > 300 and not seeing.fell


@needs_cxx
def test_hexapod_stops_at_a_gap_too_wide_to_step_over():
    from legsim.scenario import DEFAULTS, run
    d = DEFAULTS["hexapod"]
    t = terrain.gaps(height=70, widths=(250,), segment=600, y0=400)
    blocked = []
    r = run("hexapod", t, "tof", gait=d["gait"], speed=d["speed"], duration=10,
            frame_cb=lambda robot, fw, _: blocked.append(fw.terrain_state()["blocked"]), frame_dt=0.5)
    assert blocked[-1] and r.distance < 450 and not r.fell
