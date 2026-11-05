"""
The new hexapod (legsim/hexapod/new_body.py, firmware model/hexapod2: MG90S
direct drive, not built yet): the sim model matches the firmware description,
and the firmware's own gait walks it.
"""
import math
import shutil

import numpy as np
import pytest

from legsim import hal
from legsim.hexapod import SimHexapod, new_body
from legsim.hexapod.kinematics import firmware_ik, leg_angles_to_q, forward_kinematics, coxa_position, default_foot

P = new_body.params()
GEO = P.geometry
STOP = (0, 0, 0, 0, 0, 0)
needs_cxx = pytest.mark.skipif(shutil.which("c++") is None and shutil.which("g++") is None,
                               reason="needs a C++ compiler for the firmware bridge")


def test_design_numbers():
    assert (GEO.coxa, GEO.femur, GEO.tibia) == (60, 50, 60)
    assert P.hal.ratio == (1.0, 1.0, 1.0)
    for leg in P.legs:
        v = np.array(leg.foot) - np.array(leg.body)
        assert math.hypot(v[0], v[1]) == pytest.approx(69.0, abs=0.05), leg.name   # coxa 60 + 9
        assert v[2] == -100.0
    # front legs open 20 deg wider than the line from the body centre
    lf = P.legs[0]
    assert math.degrees(lf.yaw - math.atan2(lf.body[1], lf.body[0])) == pytest.approx(20.0, abs=0.05)


def test_mass_and_standing_pose():
    import mujoco
    h = SimHexapod(params=P)
    assert sum(h.model.body_mass) * 1000 == pytest.approx(815.0, abs=0.5)
    for leg in P.legs:
        q = leg_angles_to_q(*firmware_ik(GEO, leg, default_foot(leg)))
        assert q[0] == pytest.approx(0, abs=1e-9)
        assert math.degrees(q[1]) == pytest.approx(-58.0, abs=1.0)    # femur well down: little femur torque
    mujoco.mj_forward(h.model, h.data)
    assert h.body_position_mm()[2] == pytest.approx(100.0, abs=1.0)


def test_mjcf_matches_fk():
    import mujoco
    from legsim.hexapod.mjcf import servo_joint_range
    h = SimHexapod(params=P, fixed_base=True)
    rng = np.random.default_rng(2)
    lo = [servo_joint_range(P, j)[0] for j in range(3)] * 6
    hi = [servo_joint_range(P, j)[1] for j in range(3)] * 6
    for _ in range(10):
        q = rng.uniform(lo, hi)
        h.data.qpos[h._qadr] = q
        mujoco.mj_kinematics(h.model, h.data)
        feet = h.foot_positions_body_mm()
        for i, leg in enumerate(P.legs):
            np.testing.assert_allclose(feet[i], forward_kinematics(GEO, leg, q[i * 3:i * 3 + 3]) + coxa_position(leg),
                                       atol=1e-3)


@pytest.fixture(scope="module")
def fw():
    from legsim import FirmwareCore
    return FirmwareCore(model="hexapod2")


@pytest.fixture
def hexa(fw):
    fw.reset()
    h = fw.place(SimHexapod(params=P))
    h.step(0.3)
    return h


def tilt_deg(h):
    o = h.body_orientation()
    return math.degrees(max(abs(o["pitch"]), abs(o["roll"])))


@needs_cxx
def test_params_match_firmware(fw):
    fw.reset()
    pulses = fw.update(STOP, dt=0.0).reshape(6, 3)
    shift = np.append(fw.shift(), 0.0)
    for i, leg in enumerate(P.legs):
        assert fw.leg_inverse(i) == tuple(leg.inverse_abg), leg.name
        expected = hal.leg_angles_to_pulses(P.hal, leg, firmware_ik(GEO, leg, default_foot(leg) - shift), profile=P.profile)
        assert np.all(np.abs(pulses[i] - expected) <= 1), (leg.name, pulses[i], expected)


@needs_cxx
def test_firmware_stands(fw, hexa):
    p0 = hexa.body_position_mm().copy()
    fw.drive(hexa, STOP, 1.5)
    assert np.linalg.norm(hexa.body_position_mm()[:2] - p0[:2]) < 2.0
    assert tilt_deg(hexa) < 1.0
    assert hexa.body_position_mm()[2] > 97.0
    # standing: the femur servos carry ~5 % of their stall torque, the tibia ~12 %
    assert hexa.joint_torque_ratio().max() < 0.25


@needs_cxx
@pytest.mark.parametrize("command, axis, sign, distance", [
    ((0, 1, 0, 0, 0, 0), 1, 1, 550.0),
    ((0, -1, 0, 0, 0, 0), 1, -1, 550.0),
    ((1, 0, 0, 0, 0, 0), 0, 1, 280.0),
], ids=["forward", "backward", "strafe right"])
def test_firmware_walks(fw, hexa, command, axis, sign, distance):
    p0 = hexa.body_position_mm().copy()
    fw.drive(hexa, command, 3.0)
    d = hexa.body_position_mm() - p0
    # ~250 mm/s forward, ~130 sideways after a ~0.5 s ramp (MG90S at 5 V)
    assert sign * d[axis] > distance, d
    assert abs(d[1 - axis]) < 0.25 * abs(d[axis]), d
    assert tilt_deg(hexa) < 8.0
    assert hexa.body_position_mm()[2] > 90.0
    assert fw.unsolved == 0


@needs_cxx
def test_firmware_turns_right(fw, hexa):
    y0 = hexa.body_orientation()["yaw"]
    fw.drive(hexa, (0, 0, 0, 0, 0, 1), 2.0)
    turned = math.degrees(hexa.body_orientation()["yaw"] - y0)
    assert -100 < turned < -50, turned
    assert np.linalg.norm(hexa.body_position_mm()[:2]) < 30
    assert tilt_deg(hexa) < 5.0


@needs_cxx
def test_leg_test_passes_with_model_flags(fw):
    from legsim.hexapod import legtest
    results = legtest.run(fw, SimHexapod(params=P, fixed_base=True), move=15.0)
    assert all(r["ok"] for r in results), legtest.summary(results)
