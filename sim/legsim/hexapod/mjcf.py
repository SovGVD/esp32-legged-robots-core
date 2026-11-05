"""
Builds the MuJoCo MJCF model of the hexapod from HexapodParams.

Frames match the firmware: body X right, Y forward, Z up, origin at the
femur axis height. Lengths in the params are mm and masses g; the MJCF uses
m and kg.

Per leg (coxa link at leg.body, its X along the leg's neutral direction):
  <leg>_coxa   alpha, axis +Z,          + turns the leg CCW (seen from above)
  <leg>_femur  beta,  axis -Y (link),   + lifts the femur
  <leg>_tibia  gamma, axis -Y (link),   + opens the knee
See kinematics.py for the mapping to firmware leg angles.

Visuals and mass come from Gleb's printed parts (STL meshes, placed by
assembly.py) plus boxes for the servos, battery and electronics; for a robot
without CAD parts yet (params.parts == "simple", new_body.py) from
simple_parts.py. Collisions use
simple shapes (body box, leg capsules, foot spheres); only the floor collides
with the robot.
"""

import math
import os

import numpy as np
import mujoco

from .params import HexapodParams
from . import assembly

MM = 0.001
G = 0.001

MESH_DIR = os.path.normpath(os.path.join(os.path.dirname(__file__), "..", "..", "models", "meshes", "hexapod"))
MESHES = ("body_bottom", "body_top", "coxa", "coxa_gear", "femur", "tibia_side1", "tibia_side2", "tibia_tip", "servo_gear")


def _f(*v):
    return " ".join(f"{x:.6g}" for x in v)


def _quat(R):
    q = np.zeros(4)
    mujoco.mju_mat2Quat(q, np.asarray(R, dtype=float).ravel())
    return q


def servo_joint_range(p: HexapodParams, j):
    """Joint range (rad, sim q) the servo can reach: servo middle is q = 0."""
    s = p.servo
    lo = math.radians(s.min_angle - 90.0) / p.hal.ratio[j]
    hi = math.radians(s.max_angle - 90.0) / p.hal.ratio[j]
    return (min(lo, hi), max(lo, hi))


def _mesh_asset(name):
    return "servo_gear" if name.endswith("servo_gear") else name


class _Geoms:
    """Placed parts of one link as MJCF geoms (input in mm). T maps the description frame to the link frame."""

    def __init__(self, T, prefix):
        self.T = np.asarray(T, dtype=float)
        self.mirror = np.linalg.det(self.T) < 0
        self.prefix = prefix
        self.xml = []
        self.used = 0.0

    def add(self, item, mass=None, rgba=None):
        kind, name, R, t, half = item
        R = self.T @ np.asarray(R, dtype=float)
        t = self.T @ np.asarray(t, dtype=float) * MM
        if kind == "mesh":
            asset = _mesh_asset(name)
            mass = assembly.PART_MASS[asset] if mass is None else mass
            mesh = "hx_" + asset + ("_m" if self.mirror else "")
            if self.mirror:
                R = R @ np.diag([-1.0, 1, 1])     # mirrored asset has x negated
            color = ' rgba="0.98 0.78 0.15 1"' if name.startswith(("tibia_side", "tibia_tip")) else ""
            self.xml.append(f'<geom name="{self.prefix}{name}" type="mesh" mesh="{mesh}" class="part"{color} '
                            f'pos="{_f(*t)}" quat="{_f(*_quat(R))}" mass="{mass * G:.6g}"/>')
        else:
            h = np.abs(R) @ np.asarray(half, dtype=float) * MM
            mass = assembly.SERVO_MASS if mass is None else mass
            self.xml.append(f'<geom name="{self.prefix}{name}" type="box" class="part" size="{_f(*h)}" '
                            f'pos="{_f(*t)}" mass="{mass * G:.6g}" rgba="{rgba or "0.12 0.12 0.12 1"}"/>')
        self.used += mass

    def lump(self, total, pos_mm, name="misc"):
        """Rest of the link mass (screws, pins, bearings, wires) as a small sphere."""
        rest = total - self.used
        if rest > 0.05:
            pos = self.T @ np.asarray(pos_mm, dtype=float) * MM
            self.xml.append(f'<geom name="{self.prefix}{name}" type="sphere" class="part" size="0.003" '
                            f'pos="{_f(*pos)}" mass="{rest * G:.6g}" rgba="0.5 0.5 0.5 0"/>')
            self.used = total

    def __str__(self):
        return "\n".join(self.xml)


def _stl_torso(p):
    """Body from Gleb's printed parts: halves, coxa servos, battery, electronics."""
    m = p.masses
    torso = _Geoms(np.eye(3), "")
    for it in assembly.torso_parts():
        torso.add(it)
    for leg in p.legs:
        g = _Geoms(np.eye(3), leg.name + "_")
        for it in assembly.coxa_servo(leg):
            g.add(it)
        torso.xml += g.xml
        torso.used += g.used
    # 2x18650 holder in the middle, under the window in the top cover [est]
    torso.add(("box", "battery", np.eye(3), (0, 0, 12.2), (20, 38.5, 10.5)), mass=m.battery, rgba="0.1 0.1 0.1 1")
    # PCA9685 boards front and back, ESP32-CAM on the front face [est]
    torso.add(("box", "pca9685_front", np.eye(3), (0, 55, 8), (31, 12.5, 4)), mass=9.0, rgba="0.15 0.3 0.6 1")
    torso.add(("box", "pca9685_rear", np.eye(3), (0, -55, 8), (31, 12.5, 4)), mass=9.0, rgba="0.15 0.3 0.6 1")
    torso.add(("box", "esp32cam", np.eye(3), (0, 91, -12), (13.5, 2.5, 20)), mass=10.0, rgba="0.05 0.05 0.05 1")
    torso.add(("box", "electronics", np.eye(3), (0, 0, 2), (25, 15, 3)), mass=m.electronics - 28.0, rgba="0.1 0.4 0.2 1")
    torso.lump(m.torso, (0, 0, 0), "torso_misc")
    return torso


def _stl_leg(p, leg):
    """Coxa, femur and tibia link parts from the printed parts."""
    geo, m, n = p.geometry, p.masses, leg.name
    mir = assembly.leg_mirror(leg)
    cx = _Geoms(mir, n + "_")
    for it in assembly.coxa_parts():
        cx.add(it)
    cx.lump(m.coxa, (20, 0, 0), "coxa_misc")
    fe = _Geoms(mir, n + "_")
    for it in assembly.femur_parts():
        fe.add(it)
    fe.lump(m.femur, (geo.femur / 2, 0, 0), "femur_misc")
    ti = _Geoms(mir, n + "_")
    for it in assembly.tibia_parts(geo.femur):
        ti.add(it)
    ti.lump(m.tibia, (0, 0, -geo.tibia / 2), "tibia_misc")
    return cx, fe, ti


def build_xml(p: HexapodParams = None, fixed_base: bool = False, meshdir: str = None) -> str:
    """fixed_base: body welded in the air 0.3 m up (bench tests of the servos).
    meshdir: where the STL meshes are; default is the absolute path of models/meshes/hexapod."""
    p = p or HexapodParams()
    geo, m, sv, env = p.geometry, p.masses, p.servo, p.env
    meshdir = meshdir or MESH_DIR

    simple = p.parts == "simple"
    if simple:
        from . import simple_parts
        torso = simple_parts.torso(p)
    else:
        torso = _stl_torso(p)

    legs_xml, actuators, sensors = [], [], []
    margin = math.radians(10)
    for leg in p.legs:
        n = leg.name
        if simple:
            cx, fe, ti = simple_parts.leg(p, leg)
        else:
            cx, fe, ti = _stl_leg(p, leg)

        def joint(name, axis, j):
            rng = servo_joint_range(p, j)
            ratio = p.hal.ratio[j]
            return (f'<joint name="{name}" type="hinge" axis="{_f(*axis)}" '
                    f'range="{_f(rng[0] - margin, rng[1] + margin)}" '
                    f'armature="{sv.armature * ratio * ratio:.6g}" '
                    f'frictionloss="{sv.friction * ratio:.6g}" damping="0.0005"/>')

        L = geo.tibia
        foot_z = -(L - geo.foot_radius)
        legs_xml.append(f"""
    <body name="{n}_coxa" pos="{_f(*(np.array(leg.body) * MM))}" euler="0 0 {leg.yaw:.6g}">
      {joint(n + "_coxa", (0, 0, 1), 0)}
      {cx}
      <body name="{n}_femur" pos="{_f(geo.coxa * MM, 0, 0)}">
        {joint(n + "_femur", (0, -1, 0), 1)}
        <geom name="{n}_femur_geom" type="capsule" fromto="{_f(0, 0, 0, geo.femur * MM, 0, 0)}" size="{geo.femur_width * MM:.6g}" class="link"/>
        {fe}
        <body name="{n}_tibia" pos="{_f(geo.femur * MM, 0, 0)}">
          {joint(n + "_tibia", (0, -1, 0), 2)}
          <geom name="{n}_tibia_geom" type="capsule" fromto="{_f(0, 0, 0, 0, 0, foot_z * MM)}" size="{geo.tibia_width * MM:.6g}" class="link"/>
          <geom name="{n}_foot" type="sphere" pos="{_f(0, 0, foot_z * MM)}" size="{geo.foot_radius * MM:.6g}" class="foot"/>
          {ti}
          <site name="{n}_tip" pos="{_f(0, 0, -L * MM)}" size="0.002" rgba="1 0.3 0 0.6"/>
          <site name="{n}_foot_site" pos="{_f(0, 0, foot_z * MM)}" size="{geo.foot_radius * MM * 1.2:.6g}" rgba="1 0.5 0 0.3"/>
        </body>
      </body>
    </body>""")
        for jn, j in (("coxa", 0), ("femur", 1), ("tibia", 2)):
            lim = sv.stall_torque * p.hal.ratio[j]
            actuators.append(f'<motor name="{n}_{jn}" joint="{n}_{jn}" gear="1" ctrllimited="true" ctrlrange="{_f(-lim, lim)}"/>')
        sensors.append(f'<touch name="{n}_contact" site="{n}_foot_site"/>')

    meshes = simple_parts.meshes(p) if simple else []
    for name in () if simple else MESHES:
        for suffix, sx in (("", 1), ("_m", -1)):
            meshes.append(f'<mesh name="hx_{name}{suffix}" file="{name}.stl" scale="{_f(sx * MM, MM, MM)}" inertia="shell"/>')

    tx, ty, tz = (v * MM for v in geo.torso_size)
    imu = tuple(v * MM for v in p.imu_pos)
    start_z = 0.3 if fixed_base else (-p.legs[0].foot[2] + 2) * MM
    root = "" if fixed_base else '<freejoint name="root"/>'

    return f"""<mujoco model="esp32_hexapod">
  <!-- Generated by sim/legsim/hexapod/mjcf.py from params.py and assembly.py. Do not edit by hand. -->
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
      <geom name="torso_geom" type="box" size="{_f(tx / 2, ty / 2, tz / 2)}" pos="{_f(0, 0, geo.torso_z * MM)}" group="3" rgba="0.95 0.95 0.95 0.3" mass="0"/>
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
