"""
Builds the MuJoCo MJCF model of the robot dog from DogParams.

Frames match the firmware: body X right, Y forward, Z up (MPU9250 aligned).
Lengths in the params are mm and masses g; the MJCF uses m and kg.

Per leg (shoulder at leg.body):
  <leg>_roll   hip roll  (alpha), axis along Y, + moves the foot outward
  <leg>_pitch  hip pitch (beta),  axis along -X, + swings the leg backward
  <leg>_knee   knee      (gamma), axis along +X, 0 = straight leg
See kinematics.py for the mapping to firmware leg angles.

Visuals and mass come from Gleb's printed parts (STL meshes, placed by
assembly.py) plus boxes for the servos, battery and electronics. Collisions use
simple shapes (torso box, battery box, leg capsules, foot spheres), and only
the floor collides with the robot (no self collisions).
"""

import math
import os

import numpy as np
import mujoco

from .params import DogParams
from .kinematics import hip_position
from . import assembly

MM = 0.001
G = 0.001


def _f(*v):
    return " ".join(f"{x:.6g}" for x in v)


def servo_joint_range(p: DogParams, j):
    """Joint range (rad, sim q) the servo can reach, symmetric about the middle."""
    s = p.servo
    half = math.radians((s.max_angle - s.min_angle) / 2) / p.hal.ratio[j]
    if j == 2:  # knee: q = pi - gamma, servo middle at gamma = pi/2
        return (math.pi / 2 - half, math.pi / 2 + half)
    return (-half, half)


MESH_DIR = os.path.normpath(os.path.join(os.path.dirname(__file__), "..", "models", "meshes"))


def _quat(R):
    q = np.zeros(4)
    mujoco.mju_mat2Quat(q, np.asarray(R, dtype=float).ravel())
    return q


class _Geoms:
    """Collects placed parts of one link as MJCF geoms (input in mm, RF description)."""

    def __init__(self, T, prefix):
        self.T = np.asarray(T, dtype=float)
        self.mirror = np.linalg.det(self.T) < 0
        self.prefix = prefix
        self.xml = []
        self.used = 0.0     # g

    def add(self, item, mass=None, rgba=None):
        kind, name, R, t = item[:4]
        R = self.T @ np.asarray(R, dtype=float)
        t = self.T @ np.asarray(t, dtype=float) * MM
        if kind == "mesh":
            mass = assembly.PART_MASS[name] if mass is None else mass
            mesh = name + ("_m" if self.mirror else "")
            if self.mirror:
                R = R @ np.diag([-1.0, 1, 1])     # mirrored asset has x negated
            self.xml.append(f'<geom name="{self.prefix}{name}" type="mesh" mesh="{mesh}" class="part" '
                            f'pos="{_f(*t)}" quat="{_f(*_quat(R))}" mass="{mass * G:.6g}"/>')
        else:
            half = np.abs(R @ np.asarray(item[4], dtype=float)) * MM
            mass = assembly.SERVO_MASS if mass is None else mass
            self.xml.append(f'<geom name="{self.prefix}{name}" type="box" class="part" size="{_f(*half)}" '
                            f'pos="{_f(*t)}" mass="{mass * G:.6g}" rgba="{rgba or "0.12 0.12 0.12 1"}"/>')
        self.used += mass

    def lump(self, total, pos_mm, name="misc"):
        """Rest of the link mass (screws, horns, bearings, wires) as a small sphere."""
        rest = total - self.used
        if rest > 0.05:
            pos = self.T @ np.asarray(pos_mm, dtype=float) * MM
            self.xml.append(f'<geom name="{self.prefix}{name}" type="sphere" class="part" size="0.003" '
                            f'pos="{_f(*pos)}" mass="{rest * G:.6g}" rgba="0.5 0.5 0.5 0"/>')
            self.used = total

    def __str__(self):
        return "\n".join(self.xml)


def build_xml(p: DogParams = None, fixed_base: bool = False, meshdir: str = None) -> str:
    """fixed_base: torso welded in the air 0.3 m up (bench tests of the servos).
    meshdir: where the STL meshes are; default is the absolute path of models/meshes."""
    p = p or DogParams()
    geo, m, sv, env = p.geometry, p.masses, p.servo, p.env
    meshdir = meshdir or MESH_DIR

    tx, ty, tz = (v * MM for v in geo.torso_size)
    bx, by, bz = (v * MM for v in geo.battery_size)
    l2, l3 = geo.l2 * MM, geo.l3 * MM
    drop = geo.hip_pitch_drop * MM
    foot_r = geo.foot_radius * MM
    hip_y = geo.hip_y_front

    # ---- torso: printed parts, servos, battery, electronics
    torso = _Geoms(np.eye(3), "")
    per_end, per_leg, once = assembly.torso_parts(hip_y)
    for end, tag in ((1, "front_"), (-1, "hind_")):
        g = _Geoms(np.diag([end, end, 1.0]), tag)
        for it in per_end:
            g.add(it)
        torso.xml += g.xml
        torso.used += g.used
    for leg in p.legs:
        g = _Geoms(assembly.leg_transform(leg.side, leg.end, True), leg.name + "_")
        for it in per_leg:
            g.add(it)
        torso.xml += g.xml
        torso.used += g.used
    for it in once:
        torso.add(it)
    torso.add(("box", "battery", np.eye(3), (0, 0, geo.battery_z), tuple(v / 2 for v in geo.battery_size)),
              mass=m.battery, rgba="0.1 0.1 0.1 1")
    torso.add(("box", "electronics", np.eye(3), (0, 0, 4.0), (25, 35, 4)), mass=m.electronics, rgba="0.1 0.4 0.2 1")
    torso.lump(m.torso, (0, 0, -2), "torso_misc")

    visual_drop = assembly.HIP_PITCH_DROP - geo.hip_pitch_drop

    legs_xml = []
    actuators = []
    sensors = []
    for leg in p.legs:
        n = leg.name
        hip = hip_position(geo, leg) * MM
        side = leg.side
        r_roll = servo_joint_range(p, 0)
        r_pitch = servo_joint_range(p, 1)
        r_knee = servo_joint_range(p, 2)
        margin = math.radians(10)

        def joint(name, axis, rng, j):
            ratio = p.hal.ratio[j]
            return (f'<joint name="{name}" type="hinge" axis="{_f(*axis)}" '
                    f'range="{_f(rng[0] - margin, rng[1] + margin)}" '
                    f'armature="{sv.armature * ratio * ratio:.6g}" '
                    f'frictionloss="{sv.friction * ratio:.6g}" damping="0.0005"/>')

        sh = _Geoms(assembly.leg_transform(side, leg.end, True), n + "_")
        for it in assembly.shoulder_parts(visual_drop):
            sh.add(it)
        sh.lump(m.shoulder, (0, assembly.SHAFT_FROM_GEAR - 15, visual_drop), "shoulder_misc")
        up = _Geoms(assembly.leg_transform(side, leg.end, False), n + "_")
        for it in assembly.upper_parts():
            up.add(it)
        up.lump(m.upper_leg, (0, 0, -geo.l2 / 2), "upper_misc")
        lo = _Geoms(assembly.leg_transform(side, leg.end, False), n + "_")
        for it in assembly.lower_parts():
            lo.add(it)
        lo.lump(m.lower_leg, (0, 0, -geo.l3 / 2), "lower_misc")

        legs_xml.append(f"""
    <body name="{n}_shoulder" pos="{_f(*hip)}">
      {joint(n + "_roll", (0, -side, 0), r_roll, 0)}
      {sh}
      <body name="{n}_upper" pos="{_f(0, 0, -drop)}">
        {joint(n + "_pitch", (-1, 0, 0), r_pitch, 1)}
        <geom name="{n}_upper_geom" type="capsule" fromto="{_f(0, 0, 0, 0, 0, -l2)}" size="{geo.upper_leg_width * MM / 2:.6g}" class="link"/>
        {up}
        <body name="{n}_lower" pos="{_f(0, 0, -l2)}">
          {joint(n + "_knee", (1, 0, 0), r_knee, 2)}
          <geom name="{n}_lower_geom" type="capsule" fromto="{_f(0, 0, 0, 0, 0, -l3)}" size="{geo.lower_leg_width * MM / 2:.6g}" class="link"/>
          <geom name="{n}_foot" type="sphere" pos="{_f(0, 0, -l3)}" size="{foot_r:.6g}" class="foot"/>
          {lo}
          <site name="{n}_foot_site" pos="{_f(0, 0, -l3)}" size="{foot_r * 1.15:.6g}" rgba="1 0.5 0 0.3"/>
        </body>
      </body>
    </body>""")
        for jn, j in (("roll", 0), ("pitch", 1), ("knee", 2)):
            lim = sv.stall_torque * p.hal.ratio[j]
            actuators.append(f'<motor name="{n}_{jn}" joint="{n}_{jn}" gear="1" ctrllimited="true" ctrlrange="{_f(-lim, lim)}"/>')
        sensors.append(f'<touch name="{n}_contact" site="{n}_foot_site"/>')

    meshes = []
    for name in assembly.PART_MASS:
        for suffix, sx in (("", 1), ("_m", -1)):
            meshes.append(f'<mesh name="{name}{suffix}" file="{name}.stl" scale="{_f(sx * MM, MM, MM)}" inertia="shell"/>')

    imu = tuple(v * MM for v in p.imu_pos)
    start_z = 0.3 if fixed_base else (geo.foot_z + geo.hip_pitch_drop + geo.foot_radius + 2) * MM
    root = "" if fixed_base else '<freejoint name="root"/>'

    return f"""<mujoco model="esp32_robot_dog">
  <!-- Generated by sim/legsim/mjcf.py from sim/legsim/params.py and assembly.py. Do not edit by hand. -->
  <compiler angle="radian" autolimits="true" meshdir="{meshdir}"/>
  <option timestep="{env.timestep:.6g}" integrator="implicitfast" gravity="0 0 -9.81"/>

  <default>
    <geom contype="2" conaffinity="1" condim="3" friction="{env.foot_friction:.6g} 0.005 0.0001"/>
    <default class="part">
      <geom contype="0" conaffinity="0" group="1" rgba="0.95 0.95 0.93 1"/>
    </default>
    <default class="link">
      <geom group="3" rgba="0.92 0.92 0.92 0.4" mass="0"/>
    </default>
    <default class="foot">
      <geom group="3" rgba="0.2 0.2 0.2 0.4" mass="0" condim="4" priority="1"/>
    </default>
  </default>

  <visual>
    <global offwidth="1280" offheight="720"/>
  </visual>

  <asset>
    <texture name="grid" type="2d" builtin="checker" rgb1="0.75 0.75 0.75" rgb2="0.6 0.6 0.6" width="256" height="256"/>
    <material name="grid" texture="grid" texrepeat="20 20" reflectance="0.05"/>
    {chr(10).join("    " + x for x in meshes).strip()}
  </asset>

  <worldbody>
    <light pos="0 0 1.5" dir="0 0 -1" directional="true" castshadow="false"/>
    <light pos="0.5 -0.5 0.8" dir="-1 1 -1.2" directional="true" castshadow="false" diffuse="0.3 0.3 0.3"/>
    <geom name="floor" type="plane" size="5 5 0.05" material="grid" contype="1" conaffinity="1" friction="{env.floor_friction:.6g} 0.005 0.0001"/>
    <body name="torso" pos="{_f(0, 0, start_z)}">
      {root}
      <geom name="torso_geom" type="box" size="{_f(tx / 2, ty / 2, tz / 2)}" pos="{_f(0, 0, geo.torso_z_offset * MM)}" group="3" rgba="0.95 0.95 0.95 0.3" mass="0"/>
      <geom name="battery_geom" type="box" size="{_f(bx / 2, by / 2, bz / 2)}" pos="{_f(0, 0, geo.battery_z * MM)}" group="3" rgba="0.1 0.1 0.1 0.3" mass="0"/>
      {torso}
      <site name="imu" pos="{_f(*imu)}" size="0.004" rgba="0 0.6 1 1"/>
{"".join(legs_xml)}
    </body>
  </worldbody>

  <actuator>
    {chr(10).join("    " + a for a in actuators).strip()}
  </actuator>

  <sensor>
    <accelerometer name="imu_accel" site="imu"/>
    <gyro name="imu_gyro" site="imu"/>
    <framequat name="imu_quat" objtype="site" objname="imu"/>
    {chr(10).join("    " + s for s in sensors).strip()}
  </sensor>
</mujoco>
"""
