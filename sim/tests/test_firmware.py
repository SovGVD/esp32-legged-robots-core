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
    fw.set_leveling(False)
    fw.select_gait("default")   # also resets
    d = fw.place(SimDog())
    d.step(0.3)
    return d


def tilt_deg(dog):
    o = dog.body_orientation()
    return math.degrees(max(abs(o["pitch"]), abs(o["roll"])))


def test_neutral_pulses_match_python_port(fw):
    """C++ IK + HAL and the Python port in legsim agree on the neutral stance
    (feet at their defaults, body offset over the centre of mass)."""
    fw.reset()
    pulses = fw.update(STOP, dt=0.0).reshape(4, 3)
    shift = np.append(fw.shift(), 0.0)
    d = SimDog()
    geo = d.p.geometry
    for i, leg in enumerate(d.p.legs):
        angles = firmware_ik(geo, leg, default_foot(geo, leg) - shift)
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
    # the hip roll servos limit strafing to 50 mm/s (maxSideSpeed)
    assert d[0] > 120.0, d
    # no backward drift since the body sits over the centre of mass
    assert abs(d[1]) < 0.2 * d[0], d
    assert tilt_deg(dog) < 5.0


def test_turn_follows_gait_heading(fw, dog):
    """Positive yaw command turns right (clockwise from above), like the web client."""
    fw.drive(dog, (0, 0, 0, 0, 0, 1), 1.5)
    gait_yaw = fw.body()[5]
    sim_yaw = dog.body_orientation()["yaw"]
    assert gait_yaw < -math.radians(45)   # ~0.7 rad/s, limited by maxSideSpeed
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
    """Returns (moved mm, tilt deg, margin mm). Tilt is the 95th percentile: with
    the STL model a foot slipping while turning gives single-tick spikes that say
    little about how steady the walk is."""
    p0 = dog.body_position_mm().copy()
    tilts = []
    margin = 1e9
    for _ in range(int(seconds / fw.loop_dt)):
        fw.drive(dog, command)
        tilts.append(tilt_deg(dog))
        if any(fw.feet()[:, 2] > -79.99):    # a leg in the air
            margin = min(margin, fw.support_margin())
    return dog.body_position_mm() - p0, float(np.percentile(tilts, 95)), margin


@pytest.mark.parametrize("command", [(0, 1, 0, 0, 0, 0), (1, 0, 0, 0, 0, 0), (0, 0, 0, 0, 0, 1)],
                         ids=["forward", "strafe", "turn"])
def test_static_walk_is_steadier_than_without_shift(fw, dog, command):
    """Slow walk with the static stability shift vs the same walk without it."""
    fw.select_gait("walk")
    d = fw.place(SimDog())
    d.step(0.3)
    moved, tilt, margin = _walk(fw, d, command, 6.0)
    assert fw.unsolved == 0
    assert margin > 3.0            # centre of mass inside the three-foot triangle
    assert tilt < 2.5, tilt
    if command[1]:
        assert moved[1] > 80.0, moved  # 20 mm/s minus slip
    if command[0]:
        assert moved[0] > 80.0, moved

    fw.select_gait("walk")
    fw.set_stability_gain(-1)      # same gait, no shift
    d = fw.place(SimDog())
    d.step(0.3)
    _, tilt_off, margin_off = _walk(fw, d, command, 6.0)
    assert margin_off < 0.0
    assert tilt < 0.6 * tilt_off, (tilt, tilt_off)


def _slope(dog, deg, sideways):
    """Stand the dog on a slope by tilting gravity (sideways: about the forward axis)."""
    a = math.radians(deg)
    g = 9.81
    dog.model.opt.gravity[:] = [g * math.sin(a), 0, -g * math.cos(a)] if sideways \
        else [0, g * math.sin(a), -g * math.cos(a)]


def _tilt_from_gravity_deg(dog):
    """True angle between the body Z axis and 'up' (against gravity)."""
    R = dog.data.xmat[dog._torso].reshape(3, 3)
    up = -np.asarray(dog.model.opt.gravity) / np.linalg.norm(dog.model.opt.gravity)
    return math.degrees(math.acos(max(-1.0, min(1.0, float(R[:, 2] @ up)))))


def _mean_tilt_on_slope(fw, deg, sideways, gait, command, seconds):
    fw.select_gait(gait)
    d = fw.place(SimDog())
    _slope(d, deg, sideways)
    fw.drive(d, STOP, 1.0)
    tilts = []
    for i in range(int(seconds / fw.loop_dt)):
        fw.drive(d, command)
        if i * fw.loop_dt > seconds / 2:
            tilts.append(_tilt_from_gravity_deg(d))
    return float(np.mean(tilts))


@pytest.mark.parametrize("sideways", [True, False], ids=["side-slope", "up-slope"])
def test_leveling_on_slope(fw, dog, sideways):
    """IMU leveling (TiltEstimator from the simulated accel + gyro, BodyLeveler) on an 8 deg slope."""
    without = _mean_tilt_on_slope(fw, 8, sideways, "default", STOP, 6.0)
    fw.set_leveling(True)
    try:
        level = _mean_tilt_on_slope(fw, 8, sideways, "default", STOP, 6.0)
    finally:
        fw.set_leveling(False)
    assert without > 7.0, without
    assert level < 3.0, level   # the deadband leaves up to ~1.5 deg per axis
    assert fw.unsolved == 0


def test_leveling_static_walk_on_side_slope(fw, dog):
    without = _mean_tilt_on_slope(fw, 6, True, "walk", (0, 1, 0, 0, 0, 0), 8.0)
    fw.set_leveling(True)
    try:
        level = _mean_tilt_on_slope(fw, 6, True, "walk", (0, 1, 0, 0, 0, 0), 8.0)
    finally:
        fw.set_leveling(False)
    assert level < 0.5 * without, (level, without)


def test_leveling_on_flat_ground_stays_small(fw, dog):
    """On flat ground the leveler only sees the estimate's error; it must not rock the robot."""
    fw.set_leveling(True)
    try:
        moved, tilt, _ = _walk(fw, dog, (0, 1, 0, 0, 0, 0), 6.0)
    finally:
        fw.set_leveling(False)
    assert tilt < 8.0, tilt
    assert moved[1] > 300, moved


def test_battery_sit_down_is_calm(fw, dog):
    """Battery critical: the body is lowered by SIT_HEIGHT while standing; it stays level and in place."""
    fw.drive(dog, STOP, 0.5)
    z0 = dog.body_position_mm()[2]
    p0 = dog.body_position_mm()[:2].copy()
    fw.set_height_offset(fw.sit_height())
    tilt = 0.0
    for _ in range(int(3.0 / fw.loop_dt)):
        fw.drive(dog, STOP)
        tilt = max(tilt, tilt_deg(dog))
    dz = dog.body_position_mm()[2] - z0
    assert fw.unsolved == 0
    assert abs(dz - fw.sit_height()) < 5.0, (dz, fw.sit_height())
    assert tilt < 2.0, tilt
    assert np.linalg.norm(dog.body_position_mm()[:2] - p0) < 5.0


def test_heading_hold_turns_back_after_a_push(fw):
    """The firmware heading code (gyro with 1 deg/s drift + magnetometer) and the hold:
    a push turns the walking dog ~60 deg; with the hold it turns back to its heading."""
    ends = {}
    for hold in (False, True):
        fw.reset()
        fw.select_gait("trot")
        fw.set_heading_sensors(gyro_bias_dps=1.0)
        dog = fw.place(SimDog())
        fw.drive(dog, duration=0.5)
        fw.heading_hold(hold)
        t = 0.0
        while t < 9.0:
            dog.data.xfrc_applied[dog._torso, 5] = -0.12 if 2.5 <= t < 4.0 else 0.0
            fw.drive(dog, (0, 1, 0, 0, 0, 0))
            t += fw.loop_dt
        h = (-math.degrees(dog.body_orientation()["yaw"])) % 360
        ends[hold] = h - 360 if h > 180 else h
        # the estimate follows the true heading (within a few degrees)
        est = fw.heading_state()["heading"]
        assert abs((est - h + 180) % 360 - 180) < 8
    fw.set_heading_sensors(None)
    assert ends[False] > 40
    assert abs(ends[True]) < 10


def test_turn_to_a_compass_heading(fw):
    fw.reset()
    fw.select_gait("trot")
    fw.set_heading_sensors(gyro_bias_dps=1.0)
    dog = fw.place(SimDog())
    fw.drive(dog, duration=0.5)
    fw.turn_to(90)
    fw.drive(dog, duration=4.0)
    h = (-math.degrees(dog.body_orientation()["yaw"])) % 360
    fw.set_heading_sensors(None)
    assert abs(h - 90) < 8
