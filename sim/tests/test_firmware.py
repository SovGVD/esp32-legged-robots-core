"""
The real firmware motion code (lib/robot_core via sim/bridge) driving the
MuJoCo dog. Skipped when no C++ compiler is available.
"""

import math
import shutil

import numpy as np
import pytest

from legsim import SimDog
from legsim import hal
from legsim.kinematics import firmware_ik, default_foot

pytestmark = pytest.mark.skipif(shutil.which("c++") is None and shutil.which("g++") is None,
                                reason="needs a C++ compiler for the firmware bridge")

STOP = (0, 0, 0, 0, 0, 0)


@pytest.fixture(scope="module")
def fw():
    from legsim import FirmwareCore
    return FirmwareCore()


@pytest.fixture
def dog(fw):
    fw.select_gait("default")   # also resets
    d = SimDog()
    d.step(0.3)
    return d


def tilt_deg(dog):
    o = dog.body_orientation()
    return math.degrees(max(abs(o["pitch"]), abs(o["roll"])))


def test_neutral_pulses_match_python_port(fw):
    """C++ IK + HAL and the Python port in legsim agree on the neutral stance."""
    fw.reset()
    pulses = fw.update(STOP, dt=0.0).reshape(4, 3)
    d = SimDog()
    geo = d.p.geometry
    for i, leg in enumerate(d.p.legs):
        angles = firmware_ik(geo, leg, default_foot(geo, leg))
        expected = hal.leg_angles_to_pulses(d.p.hal, leg, angles)
        assert np.all(np.abs(pulses[i] - expected) <= 1), (leg.name, pulses[i], expected)
    assert fw.loop_dt == pytest.approx(0.0044)


def test_stands_still_when_idle(fw, dog):
    p0 = dog.body_position_mm().copy()
    fw.drive(dog, STOP, 2.0)
    assert not fw.gait_active()
    assert np.linalg.norm(dog.body_position_mm()[:2] - p0[:2]) < 3.0
    assert tilt_deg(dog) < 1.0
    assert dog.body_position_mm()[2] > 70.0     # still standing, not collapsed


def test_trots_forward(fw, dog):
    p0 = dog.body_position_mm().copy()
    fw.drive(dog, (0, 1, 0, 0, 0, 0), 3.0)
    d = dog.body_position_mm() - p0
    # 150 mm/s max after a 0.25 s ramp, minus slip
    assert d[1] > 300.0, d
    assert abs(d[0]) < 0.15 * d[1], d
    assert tilt_deg(dog) < 8.0
    assert dog.body_position_mm()[2] > 70.0
    assert fw.unsolved == 0


def test_strafes_right(fw, dog):
    p0 = dog.body_position_mm().copy()
    fw.drive(dog, (1, 0, 0, 0, 0, 0), 3.0)
    d = dog.body_position_mm() - p0
    assert d[0] > 250.0, d
    # Known: the trot drifts backward, worst when strafing (~40 mm/s here,
    # see "Known limits" in README.md). Loose bound so a regression still shows.
    assert abs(d[1]) < 0.5 * d[0], d
    assert tilt_deg(dog) < 8.0


def test_turn_follows_gait_heading(fw, dog):
    """Positive yaw command turns right (clockwise from above), like the web client."""
    fw.drive(dog, (0, 0, 0, 0, 0, 1), 1.5)
    gait_yaw = fw.body()[5]
    sim_yaw = dog.body_orientation()["yaw"]
    assert gait_yaw < -math.radians(90)
    err = math.remainder(sim_yaw - gait_yaw, 2 * math.pi)
    assert abs(err) < math.radians(15), (math.degrees(sim_yaw), math.degrees(gait_yaw))


def test_stops_and_settles(fw, dog):
    fw.drive(dog, (0, 1, 0, 0, 0, 0), 1.5)
    fw.drive(dog, STOP, 1.5)
    assert not fw.gait_active()
    p0 = dog.body_position_mm().copy()
    fw.drive(dog, STOP, 1.0)
    assert np.linalg.norm(dog.body_position_mm()[:2] - p0[:2]) < 3.0
    assert tilt_deg(dog) < 2.0


def _walk(fw, dog, command, seconds):
    p0 = dog.body_position_mm().copy()
    tilt = 0.0
    margin = 1e9
    for _ in range(int(seconds / fw.loop_dt)):
        fw.drive(dog, command)
        tilt = max(tilt, tilt_deg(dog))
        if any(fw.feet()[:, 2] > -79.99):    # a leg in the air
            margin = min(margin, fw.support_margin())
    return dog.body_position_mm() - p0, tilt, margin


@pytest.mark.parametrize("command", [(0, 1, 0, 0, 0, 0), (1, 0, 0, 0, 0, 0), (0, 0, 0, 0, 0, 1)],
                         ids=["forward", "strafe", "turn"])
def test_static_walk_is_steadier_than_without_shift(fw, dog, command):
    """Slow walk with the static stability shift vs the same walk without it."""
    fw.select_gait("walk")
    d = SimDog()
    d.step(0.3)
    moved, tilt, margin = _walk(fw, d, command, 6.0)
    assert fw.unsolved == 0
    assert margin > 3.0            # centre of mass inside the three-foot triangle
    assert tilt < 4.5, tilt
    if command[1]:
        assert moved[1] > 80.0, moved  # 20 mm/s minus slip
    if command[0]:
        assert moved[0] > 80.0, moved

    fw.select_gait("walk")
    fw.set_stability_gain(-1)      # same gait, no shift
    d = SimDog()
    d.step(0.3)
    _, tilt_off, margin_off = _walk(fw, d, command, 6.0)
    assert margin_off < 0.0
    assert tilt < 0.7 * tilt_off, (tilt, tilt_off)
