"""
Hexapod model: joint convention vs the firmware IK, the HAL, the MJCF, and the
real firmware gait (lib/robot_core built for MODEL_HEXAPOD) walking it.
"""

import math
import shutil

import numpy as np
import pytest

from legsim import hal
from legsim.hexapod import HexapodParams, SimHexapod
from legsim.hexapod.kinematics import (firmware_ik, leg_angles_to_q, q_to_leg_angles,
                                       forward_kinematics, coxa_position, default_foot)
from legsim.hexapod.mjcf import servo_joint_range

P = HexapodParams()
GEO = P.geometry
STOP = (0, 0, 0, 0, 0, 0)
needs_cxx = pytest.mark.skipif(shutil.which("c++") is None and shutil.which("g++") is None,
                               reason="needs a C++ compiler for the firmware bridge")


@pytest.mark.parametrize("leg", P.legs, ids=lambda l: l.name)
def test_fk_inverts_firmware_ik(leg):
    """Sim joint convention == HexapodLegIK convention over the working space."""
    for dr in (-30, 0, 20):
        for side in (-40, 0, 40):
            for dz in (-20, 0, 25):
                yaw = leg.yaw
                foot = default_foot(leg) + dr * np.array([math.cos(yaw), math.sin(yaw), 0]) \
                    + side * np.array([-math.sin(yaw), math.cos(yaw), 0]) + [0, 0, dz]
                v = foot - coxa_position(leg)
                if math.hypot(math.hypot(v[0], v[1]) - GEO.coxa, v[2]) > GEO.femur + GEO.tibia - 1:
                    continue  # out of reach: the IK clamps, nothing to compare
                q = leg_angles_to_q(*firmware_ik(GEO, leg, foot))
                np.testing.assert_allclose(forward_kinematics(GEO, leg, q) + coxa_position(leg), foot, atol=1e-6)


def test_neutral_pose_is_servo_middle_rotated():
    """Neutral stance: coxa at the servo middle, femur up 16.5 deg, knee open 24.6 deg."""
    for leg in P.legs:
        a = firmware_ik(GEO, leg, default_foot(leg))
        q = leg_angles_to_q(*a)
        assert q[0] == pytest.approx(0, abs=1e-9)
        assert math.degrees(q[1]) == pytest.approx(16.5, abs=0.1)
        assert math.degrees(q[2]) == pytest.approx(24.6, abs=0.1)


def test_mjcf_matches_fk():
    """Tibia tip sites in the MuJoCo model sit where the analytic FK says."""
    import mujoco
    h = SimHexapod(fixed_base=True)
    rng = np.random.default_rng(1)
    lo = [servo_joint_range(P, j)[0] for j in range(3)] * 6
    hi = [servo_joint_range(P, j)[1] for j in range(3)] * 6
    for _ in range(10):
        q = rng.uniform(lo, hi)
        h.data.qpos[h._qadr] = q
        mujoco.mj_kinematics(h.model, h.data)
        feet = h.foot_positions_body_mm()
        for i, leg in enumerate(P.legs):
            fk = forward_kinematics(GEO, leg, q[i * 3:i * 3 + 3]) + coxa_position(leg)
            np.testing.assert_allclose(feet[i], fk, atol=1e-3)


@pytest.mark.parametrize("leg", P.legs, ids=lambda l: l.name)
def test_hal_round_trip(leg):
    """leg angle -> pulse (firmware HAL, hexapod servo profile) -> sim joint target."""
    h = SimHexapod(fixed_base=True)
    i = P.legs.index(leg)
    angles = (math.radians(100), math.radians(70), math.radians(120))
    pulses = hal.leg_angles_to_pulses(P.hal, leg, angles, profile=P.profile)
    for j in range(3):
        h.write_pulse(i * 3 + j, pulses[j])
    q = h.servos.pending[i * 3:i * 3 + 3]
    # 1 us of pulse truncation is ~0.1 deg at the servo, less at the joint
    np.testing.assert_allclose(q_to_leg_angles(*q), angles, atol=math.radians(0.3))


def test_mass_and_standing_height():
    import mujoco
    h = SimHexapod()
    assert sum(h.model.body_mass) * 1000 == pytest.approx(P.masses.total(), abs=0.5)
    h.step(0.5)
    # IK height 67.3 mm, minus the servos sagging under the weight
    assert 63.0 < h.body_position_mm()[2] < 67.5
    o = h.body_orientation()
    assert abs(o["pitch"]) < 0.01 and abs(o["roll"]) < 0.01


# ---------------------------------------------------------------- firmware

@pytest.fixture(scope="module")
def fw():
    from legsim import FirmwareCore
    return FirmwareCore(model="hexapod")


@pytest.fixture
def hexa(fw):
    fw.reset()
    h = fw.place(SimHexapod())
    h.step(0.3)
    return h


def tilt_deg(h):
    o = h.body_orientation()
    return math.degrees(max(abs(o["pitch"]), abs(o["roll"])))


@needs_cxx
def test_firmware_neutral_pulses_match_python_port(fw):
    fw.reset()
    pulses = fw.update(STOP, dt=0.0).reshape(6, 3)
    assert fw.legs == 6 and fw.loop_dt == pytest.approx(0.007)
    shift = np.append(fw.shift(), 0.0)
    for i, leg in enumerate(P.legs):
        expected = hal.leg_angles_to_pulses(P.hal, leg, firmware_ik(GEO, leg, default_foot(leg) - shift), profile=P.profile)
        assert np.all(np.abs(pulses[i] - expected) <= 1), (leg.name, pulses[i], expected)


@needs_cxx
def test_firmware_stands(fw, hexa):
    p0 = hexa.body_position_mm().copy()
    fw.drive(hexa, STOP, 1.5)
    assert np.linalg.norm(hexa.body_position_mm()[:2] - p0[:2]) < 2.0
    assert tilt_deg(hexa) < 1.0
    assert hexa.body_position_mm()[2] > 63.0


@needs_cxx
@pytest.mark.parametrize("command, axis, sign", [
    ((0, 1, 0, 0, 0, 0), 1, 1),
    ((0, -1, 0, 0, 0, 0), 1, -1),
    ((1, 0, 0, 0, 0, 0), 0, 1),
], ids=["forward", "backward", "strafe right"])
def test_firmware_walks(fw, hexa, command, axis, sign):
    p0 = hexa.body_position_mm().copy()
    fw.drive(hexa, command, 3.0)
    d = hexa.body_position_mm() - p0
    # 75 mm/s max after a short ramp, minus slip
    assert sign * d[axis] > 180.0, d
    assert abs(d[1 - axis]) < 0.1 * abs(d[axis]), d
    assert tilt_deg(hexa) < 5.0
    assert hexa.body_position_mm()[2] > 55.0
    assert fw.unsolved == 0


@needs_cxx
def test_firmware_turns_right(fw, hexa):
    y0 = hexa.body_orientation()["yaw"]
    fw.drive(hexa, (0, 0, 0, 0, 0, 1), 2.0)
    turned = math.degrees(hexa.body_orientation()["yaw"] - y0)
    # yaw + turns right (clockwise from above): 0.75 rad/s max
    assert -90 < turned < -50, turned
    assert np.linalg.norm(hexa.body_position_mm()[:2]) < 30
    assert tilt_deg(hexa) < 5.0
