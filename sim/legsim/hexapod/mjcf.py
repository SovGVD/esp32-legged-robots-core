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

Visuals are the CAD parts (cad/hexapod/export_sim_meshes.py -> models/meshes/hexapod/, no mass).
Mass comes from simple_parts.py: boxes and cylinders for the printed parts, servos, cells and boards,
hidden in geom group 4 when the meshes are there (shown as before without them). Collisions use
simple shapes (body box, leg capsules, foot spheres); only the floor collides with the robot.
"""

import math
import os

import numpy as np

from .params import HexapodParams
from . import simple_parts

MM = 0.001
G = 0.001
MESH_DIR = os.path.normpath(os.path.join(os.path.dirname(__file__), "..", "..", "models", "meshes", "hexapod"))

# CAD meshes (file, colour) per link; the left legs use the mirrored _L files
TORSO_MESHES = (("torso_bottom", "0.55 0.33 0.22 1"), ("torso_top", "0.82 0.45 0.25 1"), ("torso_parts", "0.15 0.3 0.25 1"))
LEG_MESHES = {"coxa": (("coxa", "0.88 0.63 0.11 1"), ("coxa_servo", "0.16 0.2 0.5 1")),
              "femur": (("femur", "0.2 0.55 0.75 1"), ("femur_servos", "0.16 0.2 0.5 1")),
              "tibia": (("tibia", "0.35 0.65 0.35 1"), ("presser", "0.25 0.25 0.25 1"))}


def _cad_meshes(meshdir):
    """Asset lines for the CAD meshes, or None when they haven't been exported."""
    names = sorted({f for f, _ in TORSO_MESHES} | {f + side for link in LEG_MESHES.values() for f, _ in link
                                                    for side in (("",) if f == "presser" else ("_R", "_L"))})
    if not all(os.path.exists(os.path.join(meshdir, n + ".stl")) for n in names):
        return None
    return [f'<mesh name="cad_{n}" file="{n}.stl" scale="0.001 0.001 0.001" inertia="shell"/>' for n in names]


def _look(prefix, files, side):
    out = []
    for f, rgba in files:
        mesh = f if f == "presser" else f + side
        out.append(f'<geom name="{prefix}{f}_look" type="mesh" mesh="cad_{mesh}" class="look" rgba="{rgba}"/>')
    return "\n".join(out)



def _f(*v):
    return " ".join(f"{x:.6g}" for x in v)


def servo_joint_range(p: HexapodParams, j):
    """Joint range (rad, sim q) the servo can reach: servo middle is q = 0."""
    s = p.servo
    lo = math.radians(s.min_angle - 90.0) / p.hal.ratio[j]
    hi = math.radians(s.max_angle - 90.0) / p.hal.ratio[j]
    return (min(lo, hi), max(lo, hi))


def build_xml(p: HexapodParams = None, fixed_base: bool = False, meshdir: str = None) -> str:
    """fixed_base: body welded in the air 0.3 m up (bench tests of the servos).
    meshdir: where the CAD meshes are; default is the absolute path of models/meshes/hexapod."""
    p = p or HexapodParams()
    geo, sv, env = p.geometry, p.servo, p.env
    meshdir = meshdir or MESH_DIR
    cad = _cad_meshes(meshdir if os.path.isabs(meshdir) else os.path.join(MESH_DIR, "..", "..", meshdir))
    # with the CAD meshes the mass boxes go to a hidden group
    mass_cls = (lambda x: x.replace('class="part"', 'class="mass"')) if cad else (lambda x: x)

    torso = mass_cls(str(simple_parts.torso(p)))
    if cad:
        torso += "\n      " + _look("", TORSO_MESHES, "").replace("\n", "\n      ")

    legs_xml, actuators, sensors = [], [], []
    margin = math.radians(10)
    for leg in p.legs:
        n = leg.name
        cx, fe, ti = (mass_cls(x) for x in simple_parts.leg(p, leg))
        if cad:
            side = "_L" if leg.side < 0 else "_R"
            cx, fe, ti = (x + "\n" + _look(n + "_", LEG_MESHES[k], side) for x, k in ((cx, "coxa"), (fe, "femur"), (ti, "tibia")))

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

    meshes = simple_parts.meshes(p) + (cad or [])

    tx, ty, tz = (v * MM for v in geo.torso_size)
    imu = tuple(v * MM for v in p.imu_pos)
    start_z = 0.3 if fixed_base else (-p.legs[0].foot[2] + 2) * MM
    root = "" if fixed_base else '<freejoint name="root"/>'

    return f"""<mujoco model="esp32_hexapod">
  <!-- Generated by sim/legsim/hexapod/mjcf.py from params.py and simple_parts.py. Do not edit by hand. -->
  <compiler angle="radian" autolimits="true" meshdir="{meshdir}"/>
  <option timestep="{env.timestep:.6g}" integrator="implicitfast" gravity="0 0 -9.81"/>

  <default>
    <geom contype="2" conaffinity="1" condim="3" friction="{env.foot_friction:.6g} 0.005 0.0001"/>
    <default class="part">
      <geom contype="0" conaffinity="0" group="1" rgba="0.95 0.95 0.93 1"/>
    </default>
    <default class="mass">
      <geom contype="0" conaffinity="0" group="4"/>
    </default>
    <default class="look">
      <geom contype="0" conaffinity="0" group="1" mass="0"/>
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
