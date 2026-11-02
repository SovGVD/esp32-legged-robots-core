import math
import numpy as np
import mujoco
import pytest

from legsim import DogParams, SimDog
from legsim.demo_gait import run_trot


def test_total_mass():
    dog = SimDog()
    assert 0.40 < mujoco.mj_getTotalmass(dog.model) < 0.47


def test_stands_in_default_pose():
    dog = SimDog()
    z0 = dog.body_position_mm()[2]
    dog.step(3.0)
    pos, ori = dog.body_position_mm(), dog.body_orientation()
    assert abs(pos[2] - z0) < 4.0                       # servo sag, mm
    assert abs(ori["pitch"]) < math.radians(2) and abs(ori["roll"]) < math.radians(2)
    assert all(dog.foot_contacts())
    imu = dog.read_imu()
    np.testing.assert_allclose(imu["accel"], (0, 0, 9.81), atol=0.15)
    np.testing.assert_allclose(imu["gyro"], 0, atol=0.01)


def test_falls_when_servos_unpowered():
    dog = SimDog()
    dog.reset(powered=False)
    dog.step(1.5)
    assert dog.body_position_mm()[2] < 50


def test_pulses_reach_leg_angles():
    """Firmware chain: leg angles -> pulses -> servo bus -> joints (robot in the air)."""
    dog = SimDog(fixed_base=True)
    target = dog.default_leg_angles() + np.radians([5, -10, 15])
    dog.set_leg_angles(target)
    dog.step(0.5)
    np.testing.assert_allclose(dog.leg_angles(), target, atol=math.radians(1.0))


@pytest.mark.parametrize("ch,deg", [(2, 60), (1, -60), (0, -30)])
def test_servo_speed_matches_datasheet(ch, deg):
    """MG90D: 0.10 s/60 deg at the shaft; hip roll is 2:1 so 30 deg joint = 60 deg shaft."""
    dog = SimDog(fixed_base=True)
    dog.step(0.3)
    q0 = dog.joint_q()
    tgt = q0.copy()
    tgt[ch] += math.radians(deg)
    dog.set_joint_targets(tgt)
    t, t90 = 0.0, None
    while t < 0.4:
        dog.step(0.0005)
        t += 0.0005
        if t90 is None and abs(dog.joint_q()[ch] - q0[ch]) >= 0.9 * abs(math.radians(deg)):
            t90 = t
    assert t90 is not None and 0.07 < t90 < 0.16


def test_open_loop_trot_smoke():
    """The model can trot forward without falling over with the estimated servos."""
    dog = SimDog()
    dog.step(0.5)
    log = run_trot(dog, 3.0, speed_mm_s=120)
    assert log[-1, 2] > 40                 # moved forward, mm
    assert log[:, 3].min() > 60            # body never dropped
    assert np.abs(log[:, 4:6]).max() < math.radians(20)
