"""
SimDog: the simulated robot, seen through the same kind of interfaces the
firmware uses on hardware.

  ServoBus-like:  write_pulse(channel, us) / write_pulses(us[12])   (channel = leg*3 + joint, firmware order)
  Imu-like:       read_imu() -> accel (m/s^2, body frame, +9.81 on Z at rest), gyro (rad/s)
  optional:       foot_contacts()  (the real robot has no contact sensors yet)

Convenience layers on top of that:
  set_leg_angles(angles[4][3])  firmware leg angles -> HAL -> pulses (full firmware chain)
  set_joint_targets(q[12])      sim joint space, bypassing the HAL

The servo model (servo.py) latches pulses at 50 Hz, so calling write_* more
often than the real control loop does not give the sim an unfair advantage.
"""

import math
import numpy as np
import mujoco

from .params import DogParams
from .mjcf import build_xml, servo_joint_range
from . import hal
from .kinematics import leg_angles_to_q, q_to_leg_angles, firmware_ik, default_foot, hip_position
from .servo import ServoArray


class SimDog:
    def __init__(self, params: DogParams = None, trim=None, fixed_base=False):
        self.p = params or DogParams()
        self.fixed_base = fixed_base
        self.xml = build_xml(self.p, fixed_base=fixed_base)
        self.model = mujoco.MjModel.from_xml_string(self.xml)
        self.data = mujoco.MjData(self.model)
        # sim servo trim (rad) per leg/joint; 0 = horns mounted perfectly
        self.trim = np.zeros((4, 3)) if trim is None else np.asarray(trim, dtype=float)
        ratio = [self.p.hal.ratio[j] for _ in self.p.legs for j in range(3)]
        self.servos = ServoArray(self.p.servo, ratio)
        names = [f"{leg.name}_{j}" for leg in self.p.legs for j in ("roll", "pitch", "knee")]
        self._qadr = np.array([self.model.jnt_qposadr[self.model.joint(n).id] for n in names])
        self._dadr = np.array([self.model.jnt_dofadr[self.model.joint(n).id] for n in names])
        self._act = np.array([self.model.actuator(n).id for n in names])
        self._torso = self.model.body("torso").id
        self._s_acc = self.model.sensor("imu_accel").adr[0]
        self._s_gyro = self.model.sensor("imu_gyro").adr[0]
        self._s_touch = [self.model.sensor(f"{leg.name}_contact").adr[0] for leg in self.p.legs]
        self.imu_noise = {"accel": 0.0, "gyro": 0.0}   # std dev, m/s^2 and rad/s
        self._rng = np.random.default_rng(0)
        self.reset()

    # ------------------------------------------------------------------ setup
    def default_leg_angles(self):
        geo = self.p.geometry
        return np.array([firmware_ik(geo, leg, default_foot(geo, leg)) for leg in self.p.legs])

    def reset(self, leg_angles=None, height=None, powered=True):
        """Place the robot on the floor in a pose (default: firmware neutral stance)."""
        mujoco.mj_resetData(self.model, self.data)
        if leg_angles is None:
            leg_angles = self.default_leg_angles()
        q = np.array([leg_angles_to_q(*a) for a in leg_angles]).ravel()
        self.data.qpos[self._qadr] = q
        mujoco.mj_kinematics(self.model, self.data)
        if self.fixed_base:
            pass
        elif height is None:
            # lowest foot sphere touching the floor
            r = self.p.geometry.foot_radius * 0.001
            zs = [self.data.geom_xpos[self.model.geom(f"{leg.name}_foot").id][2] for leg in self.p.legs]
            height = self.data.qpos[2] - (min(zs) - r) + 1e-4
        if not self.fixed_base:
            self.data.qpos[2] = height
        mujoco.mj_forward(self.model, self.data)
        self.servos.reset(q if powered else None)

    # ------------------------------------------------------- servo bus layer
    def write_pulse(self, channel, us):
        """One servo, channel = leg*3 + joint (firmware order), us = 0 for no pulse."""
        leg_i, j = divmod(channel, 3)
        leg = self.p.legs[leg_i]
        if us <= 0:
            self.servos.pending[channel] = np.nan
            return
        servo_angle = hal.pulse_to_angle(us)
        a = hal.servo_angle_to_leg_angle(self.p.hal, leg, j, servo_angle, self.trim[leg_i, j])
        angles = [math.pi / 2, math.pi / 2, math.pi / 2]
        angles[j] = a
        self.servos.pending[channel] = leg_angles_to_q(*angles)[j]

    def write_pulses(self, pulses):
        for ch, us in enumerate(np.asarray(pulses).ravel()):
            self.write_pulse(ch, us)

    def set_leg_angles(self, angles, firmware_trim=None):
        """Firmware leg angles [4][3] -> getHALAngle -> angleToPulse -> servo bus."""
        trim = np.zeros((4, 3)) if firmware_trim is None else firmware_trim
        pulses = [hal.leg_angles_to_pulses(self.p.hal, leg, angles[i], trim[i]) for i, leg in enumerate(self.p.legs)]
        self.write_pulses(pulses)
        return pulses

    def set_joint_targets(self, q):
        """Sim joint targets (rad), clamped to what the servos can reach."""
        q = np.asarray(q, dtype=float).ravel()
        lo = np.array([servo_joint_range(self.p, j)[0] for _ in self.p.legs for j in range(3)])
        hi = np.array([servo_joint_range(self.p, j)[1] for _ in self.p.legs for j in range(3)])
        self.servos.command(np.clip(q, lo, hi))

    def set_feet(self, feet_body_mm):
        """Foot targets in the body frame (mm) -> firmware IK -> HAL -> servos."""
        geo = self.p.geometry
        angles = [firmware_ik(geo, leg, feet_body_mm[i]) for i, leg in enumerate(self.p.legs)]
        return self.set_leg_angles(angles)

    # --------------------------------------------------------------- stepping
    def step(self, duration):
        """Advance the physics by `duration` seconds with the current commands."""
        n = max(1, int(round(duration / self.model.opt.timestep)))
        d = self.data
        for _ in range(n):
            tau = self.servos.torque(d.time, d.qpos[self._qadr], d.qvel[self._dadr])
            d.ctrl[self._act] = tau
            mujoco.mj_step(self.model, d)

    # ----------------------------------------------------------------- sensors
    def read_imu(self):
        s = self.data.sensordata
        acc = s[self._s_acc:self._s_acc + 3].copy()
        gyro = s[self._s_gyro:self._s_gyro + 3].copy()
        if self.imu_noise["accel"]:
            acc += self._rng.normal(0, self.imu_noise["accel"], 3)
        if self.imu_noise["gyro"]:
            gyro += self._rng.normal(0, self.imu_noise["gyro"], 3)
        return {"accel": acc, "gyro": gyro}

    def foot_contacts(self):
        return [self.data.sensordata[a] > 0 for a in self._s_touch]

    # ------------------------------------------------------------ ground truth
    def body_position_mm(self):
        return self.data.xpos[self._torso] * 1000.0

    def body_orientation(self):
        """Firmware LR_angle convention: pitch about X (nose up +), roll about Y, yaw about Z."""
        R = self.data.xmat[self._torso].reshape(3, 3)
        return {
            "pitch": math.atan2(R[2, 1], R[2, 2]),
            "roll": math.atan2(-R[2, 0], R[2, 2]),
            "yaw": math.atan2(R[1, 0], R[0, 0]),
        }

    def joint_q(self):
        return self.data.qpos[self._qadr].copy()

    def leg_angles(self):
        return np.array([q_to_leg_angles(*self.joint_q()[i * 3:i * 3 + 3]) for i in range(4)])

    def foot_positions_body_mm(self):
        """Foot sphere centres in the body frame, mm."""
        R = self.data.xmat[self._torso].reshape(3, 3)
        p0 = self.data.xpos[self._torso]
        return np.array([R.T @ (self.data.geom_xpos[self.model.geom(f"{leg.name}_foot").id] - p0) * 1000.0
                         for leg in self.p.legs])
