"""
SimHexapod: the simulated hexapod, seen through the same interfaces as SimDog.

  write_pulse(channel, us) / write_pulses(us[18])   channel = leg*3 + joint, firmware leg order
                                                    (LF, LM, LR, RF, RM, RR), not the PCA9685 channel
  read_imu() -> accel (m/s^2, body frame), gyro (rad/s)
  foot_contacts()  (optional, the real robot has no contact sensors)

plus set_leg_angles(), set_feet(), set_joint_targets() and ground truth, like SimDog.

Pulses turn each joint the way `direction` says (+1: a longer pulse turns the
joint its + q way). The servos aren't mounted yet, so it comes from the inverse
flags in the params (kinematics.hal_direction); flip one entry to see what a
servo mounted the other way does (the leg test catches it).
"""

import math

import numpy as np
import mujoco

from .params import HexapodParams
from .mjcf import build_xml, servo_joint_range
from .kinematics import leg_angles_to_q, q_to_leg_angles, firmware_ik, default_foot, hal_direction
from .. import hal
from ..servo import ServoArray
from ..robot import SimDog

JOINT_NAMES = ("coxa", "femur", "tibia")


class SimHexapod(SimDog):
    def __init__(self, params: HexapodParams = None, trim=None, fixed_base=False, terrain=None):
        self.p = params or HexapodParams()
        self.terrain = terrain
        self.direction = np.array([[hal_direction(leg, j) for j in range(3)] for leg in self.p.legs])
        self.fixed_base = fixed_base
        self.xml = build_xml(self.p, fixed_base=fixed_base)
        if terrain is not None:
            self.xml = terrain.inject(self.xml, self.p.env.floor_friction)
        self.model = mujoco.MjModel.from_xml_string(self.xml)
        self.data = mujoco.MjData(self.model)
        n = len(self.p.legs)
        self.trim = np.zeros((n, 3)) if trim is None else np.asarray(trim, dtype=float)
        ratio = [self.p.hal.ratio[j] for _ in self.p.legs for j in range(3)]
        self.servos = ServoArray(self.p.servo, ratio)
        names = [f"{leg.name}_{j}" for leg in self.p.legs for j in JOINT_NAMES]
        self._qadr = np.array([self.model.jnt_qposadr[self.model.joint(nm).id] for nm in names])
        self._dadr = np.array([self.model.jnt_dofadr[self.model.joint(nm).id] for nm in names])
        self._act = np.array([self.model.actuator(nm).id for nm in names])
        self._torso = self.model.body("torso").id
        self._s_acc = self.model.sensor("imu_accel").adr[0]
        self._s_gyro = self.model.sensor("imu_gyro").adr[0]
        self._s_touch = [self.model.sensor(f"{leg.name}_contact").adr[0] for leg in self.p.legs]
        self._tips = [self.model.site(f"{leg.name}_tip").id for leg in self.p.legs]
        self.imu_noise = {"accel": 0.0, "gyro": 0.0}
        self._rng = np.random.default_rng(0)
        self.reset()

    # ------------------------------------------------------------------ setup
    def default_leg_angles(self):
        geo = self.p.geometry
        return np.array([firmware_ik(geo, leg, default_foot(leg)) for leg in self.p.legs])

    def reset(self, leg_angles=None, height=None, powered=True):
        """Place the robot on the floor in a pose (default: firmware neutral stance)."""
        mujoco.mj_resetData(self.model, self.data)
        if leg_angles is None:
            leg_angles = self.default_leg_angles()
        q = np.array([leg_angles_to_q(*a) for a in leg_angles]).ravel()
        self.data.qpos[self._qadr] = q
        mujoco.mj_kinematics(self.model, self.data)
        if not self.fixed_base:
            if height is None:
                r = self.p.geometry.foot_radius * 0.001
                zs = [self.data.geom_xpos[self.model.geom(f"{leg.name}_foot").id][2] for leg in self.p.legs]
                height = self.data.qpos[2] - (min(zs) - r) + 1e-4
            if getattr(self, "terrain", None) is not None:
                height += self.terrain.start_height * 0.001
            self.data.qpos[2] = height
        mujoco.mj_forward(self.model, self.data)
        self.servos.reset(q if powered else None)

    # ------------------------------------------------------- servo bus layer
    def write_pulse(self, channel, us):
        leg_i, j = divmod(channel, 3)
        if us <= 0:
            self.servos.pending[channel] = np.nan
            return
        servo_angle = hal.pulse_to_angle(us, self.p.profile)
        # servo middle (90 deg) = the zero pose (q = 0)
        turn = servo_angle - math.pi / 2 - self.trim[leg_i, j]
        self.servos.pending[channel] = self.direction[leg_i, j] * turn / self.p.hal.ratio[j]

    def set_leg_angles(self, angles, firmware_trim=None):
        """Firmware leg angles [6][3] -> getHALAngle -> angleToPulse -> servo bus."""
        n = len(self.p.legs)
        trim = np.zeros((n, 3)) if firmware_trim is None else firmware_trim
        pulses = [hal.leg_angles_to_pulses(self.p.hal, leg, angles[i], trim[i], self.p.profile)
                  for i, leg in enumerate(self.p.legs)]
        self.write_pulses(pulses)
        return pulses

    def set_leg_angles_ideal(self, angles):
        """Firmware leg angles [6][3] straight to the joint servos, skipping the
        HAL and pulses: for what-if servos the firmware HAL doesn't describe.
        Out-of-travel angles are clamped."""
        self.set_joint_targets(np.array([leg_angles_to_q(*a) for a in angles]).ravel())

    def set_joint_targets(self, q):
        q = np.asarray(q, dtype=float).ravel()
        lo = np.array([servo_joint_range(self.p, j)[0] for _ in self.p.legs for j in range(3)])
        hi = np.array([servo_joint_range(self.p, j)[1] for _ in self.p.legs for j in range(3)])
        self.servos.command(np.clip(q, lo, hi))

    def set_feet(self, feet_body_mm):
        geo = self.p.geometry
        angles = [firmware_ik(geo, leg, feet_body_mm[i]) for i, leg in enumerate(self.p.legs)]
        return self.set_leg_angles(angles)

    # ------------------------------------------------------------ ground truth
    def leg_angles(self):
        q = self.joint_q()
        return np.array([q_to_leg_angles(*q[i * 3:i * 3 + 3]) for i in range(len(self.p.legs))])

    def foot_positions_body_mm(self):
        """Tibia tips (the firmware's foot points) in the body frame, mm."""
        R = self.data.xmat[self._torso].reshape(3, 3)
        p0 = self.data.xpos[self._torso]
        return np.array([R.T @ (self.data.site_xpos[s] - p0) * 1000.0 for s in self._tips])

    LOWER_LEG = "_tibia_geom"

    def _foot_points(self):
        return [self.data.site_xpos[s] for s in self._tips]

    def joint_torque_ratio(self):
        """|servo torque| / stall torque per joint (1 = the servo is at its limit)."""
        lim = self.p.servo.stall_torque * np.array([self.p.hal.ratio[j] for _ in self.p.legs for j in range(3)])
        return np.abs(self.data.ctrl[self._act]) / lim
