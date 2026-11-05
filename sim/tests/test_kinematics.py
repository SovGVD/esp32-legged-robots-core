import math
import numpy as np
import pytest

from legsim import DogParams, SimDog
from legsim.kinematics import firmware_ik, leg_angles_to_q, forward_kinematics, hip_position, default_foot
from legsim import hal

P = DogParams()
GEO = P.geometry
IDEAL = DogParams().geometry          # the firmware's geometry: pitch axis on the roll axis
IDEAL.hip_pitch_drop = 0.0
DROPPED = DogParams().geometry        # a build with the pitch axis below the roll axis
DROPPED.hip_pitch_drop = 9.1


@pytest.mark.parametrize("leg", P.legs, ids=lambda l: l.name)
def test_fk_inverts_firmware_ik(leg):
    """Sim joint convention == firmware IK convention, over the working space."""
    GEO = IDEAL
    hip = hip_position(GEO, leg)
    for dx in (-10, 0, 15, 25):
        for dy in (-35, -10, 0, 20, 35):
            for z in (-60, -80, -92):
                foot = default_foot(GEO, leg) + np.array([leg.side * dx, dy, 0])
                foot[2] = z
                if np.linalg.norm(foot - hip) > GEO.l2 + GEO.l3 - 1:
                    continue  # out of reach: the IK clamps, nothing to compare
                q = leg_angles_to_q(*firmware_ik(GEO, leg, foot))
                np.testing.assert_allclose(forward_kinematics(GEO, leg, q) + hip, foot, atol=1e-6)


def test_mjcf_matches_fk():
    """Foot geoms in the MuJoCo model sit where the analytic FK says."""
    dog = SimDog(fixed_base=True)
    rng = np.random.default_rng(1)
    for _ in range(10):
        q = rng.uniform([-0.5, -1.0, 0.4] * 4, [0.5, 1.0, 2.6] * 4)
        dog.data.qpos[dog._qadr] = q
        import mujoco
        mujoco.mj_kinematics(dog.model, dog.data)
        feet = dog.foot_positions_body_mm()
        for i, leg in enumerate(P.legs):
            expect = forward_kinematics(GEO, leg, q[i * 3:i * 3 + 3]) + hip_position(GEO, leg)
            np.testing.assert_allclose(feet[i], expect, atol=1e-6)


@pytest.mark.parametrize("leg", P.legs, ids=lambda l: l.name)
def test_hip_pitch_drop_offsets_feet(leg):
    """With a hip pitch drop the foot lands hip_pitch_drop further along the leg's down direction."""
    GEO = DROPPED
    foot = default_foot(GEO, leg)
    a = firmware_ik(GEO, leg, foot)
    got = forward_kinematics(GEO, leg, leg_angles_to_q(*a)) + hip_position(GEO, leg)
    roll = math.pi / 2 - a[0]
    expect = foot + GEO.hip_pitch_drop * np.array([leg.side * math.sin(roll), 0, -math.cos(roll)])
    np.testing.assert_allclose(got, expect, atol=1e-6)


def test_default_pose_angles():
    a = firmware_ik(GEO, P.legs[0], default_foot(GEO, P.legs[0]))
    # feet 64 mm out (config.h LEG_POINT_X; 56 gave 78.35, 54.08, 106.56)
    np.testing.assert_allclose(np.degrees(a), (72.97, 55.93, 110.37), atol=0.01)


@pytest.mark.parametrize("leg", P.legs, ids=lambda l: l.name)
def test_hal_round_trip(leg):
    """leg angle -> pulse (firmware) -> servo angle -> leg angle (sim) within pulse quantisation."""
    for a in np.linspace(math.radians(60), math.radians(120), 13):
        angles = (a, a - 0.3, a + 0.2)
        pulses = hal.leg_angles_to_pulses(P.hal, leg, angles)
        for j in range(3):
            back = hal.servo_angle_to_leg_angle(P.hal, leg, j, hal.pulse_to_angle(pulses[j]))
            assert abs(back - angles[j]) < math.radians(0.15)


def test_servo_middle_is_90deg():
    assert hal.angle_to_pulse(math.pi / 2) == 1500
    assert hal.pulse_to_angle(1500) == pytest.approx(math.pi / 2)
