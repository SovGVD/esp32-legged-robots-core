"""
Mass parts of the hexapod: boxes and cylinders sized like the design concept
(params.py), the masses from the CAD. The look is the CAD's own meshes (mjcf.py);
these boxes show only when those meshes are missing. Everything in mm and g
here, converted to the MJCF's m and kg.

Link frames as in mjcf.py: X along the leg (neutral direction), Z up, joint
on the link origin, at the zero pose (femur level, tibia straight down).
"""

import math

import numpy as np

from .params import plate_outline, BOTTOM_Z, TOP_Z

MM = 0.001
G = 0.001

SERVO_MASS = 13.4                       # MG90S, g [ds]
SERVO_LEN, SERVO_WID, SERVO_HGT = 22.8, 12.2, 22.5   # body without tabs; shaft along the height [ds]
SHAFT_FROM_END = 6.0                    # shaft centre from the near end of the body [ds]

PLA = "0.88 0.64 0.12 1"
PLA_LIGHT = "0.85 0.82 0.77 1"
SERVO = "0.16 0.19 0.22 1"
PCB = "0.18 0.44 0.7 1"
BUCK = "0.12 0.37 0.23 1"
CELL = "0.23 0.56 0.35 1"


def _f(*v):
    return " ".join(f"{x:.6g}" for x in v)


class _Parts:
    """Geoms of one link; `used` counts the mass placed so far."""

    def __init__(self, prefix):
        self.prefix = prefix
        self.xml = []
        self.used = 0.0

    def box(self, name, pos, half, mass, rgba, yaw=0.0):
        self.xml.append(f'<geom name="{self.prefix}{name}" type="box" class="part" size="{_f(*(np.array(half) * MM))}" '
                        f'pos="{_f(*(np.array(pos) * MM))}" euler="0 0 {yaw:.6g}" mass="{mass * G:.6g}" rgba="{rgba}"/>')
        self.used += mass

    def raw(self, xml, mass):
        self.xml.append(xml)
        self.used += mass

    def lump(self, total, pos, name="misc"):
        """The rest of the link mass (screws, wires, printed walls) as a small sphere."""
        rest = total - self.used
        if rest > 0.05:
            self.xml.append(f'<geom name="{self.prefix}{name}" type="sphere" class="part" size="0.003" '
                            f'pos="{_f(*(np.array(pos) * MM))}" mass="{rest * G:.6g}" rgba="0.5 0.5 0.5 0"/>')
            self.used = total

    def __str__(self):
        return "\n".join(self.xml)


def meshes(p):
    """The hexagon body plate (3 mm, convex) as an inline mesh asset."""
    pts = plate_outline()
    v = [c for z in (0.0, 3.0) for (x, y) in pts for c in (x * MM, y * MM, z * MM)]
    return [f'<mesh name="hx2_plate" vertex="{_f(*v)}"/>']


def torso(p):
    m = p.masses
    t = _Parts("")
    # bottom plate and the (see-through) top cover
    t.raw(f'<geom name="body_bottom" type="mesh" mesh="hx2_plate" class="part" pos="{_f(0, 0, BOTTOM_Z * MM)}" '
          f'mass="{48.3 * G:.6g}" rgba="{PLA_LIGHT}"/>', 48.3)
    t.raw(f'<geom name="body_top" type="mesh" mesh="hx2_plate" class="part" pos="{_f(0, 0, TOP_Z * MM)}" '
          f'mass="{58.7 * G:.6g}" rgba="0.85 0.82 0.77 0.35"/>', 58.7)
    # 2x 18650 at the bottom centre, along the body
    for x in (-9.9, 9.9):
        t.raw(f'<geom name="cell_{"l" if x < 0 else "r"}" type="cylinder" class="part" size="{_f(9 * MM, 32.5 * MM)}" '
              f'pos="{_f(x * MM, -8 * MM, -14.8 * MM)}" euler="1.5708 0 0" mass="{m.battery / 2 * G:.6g}" rgba="{CELL}"/>',
              m.battery / 2)
    e0 = t.used
    # a PCA9685 + 3 Mini360 board standing along each side
    for sx in (-1, 1):
        side = "l" if sx < 0 else "r"
        t.box(f"pca9685_{side}", (sx * 23, 0, -12), (0.8, 31, 12.5), 9.0, PCB)
        for k, dy in enumerate((-18, 0, 18)):
            t.box(f"mini360_{side}{k}", (sx * 26.3, dy, -12), (1.5, 8.5, 5.5), 2.0, BUCK)
    t.box("esp32", (0, 45, -12), (26, 0.8, 14), 10.0, PCB)           # standing across the front
    t.box("logic_buck", (0, -44, -12), (8.5, 1.5, 5.5), 2.0, BUCK)
    t.box("gy91", tuple(p.imu_pos), (7, 10, 1), 3.0, "0.6 0.1 0.6 1")  # under the top cover, centre
    for sx in (-1, 1):
        t.box(f"tof_{'l' if sx < 0 else 'r'}", (sx * 14, 74, 22), (6.5, 1.5, 4.5), 3.0, "0.48 0.25 0.69 1", -sx * 0.35)
    rest = m.electronics - (t.used - e0)
    if rest > 0:                     # fuse, switch, connectors
        t.box("electronics_misc", (0, -30, -24), (6, 4, 2), rest, "0.2 0.2 0.2 1")
    t.lump(m.torso, (0, 0, -5), "torso_misc")
    return t


def leg(p, leg):
    """Coxa, femur and tibia link parts: the servos are in the links (cad/hexapod): the
    coxa carries its own servo, the femur both the femur and the tibia servo."""
    geo, m, n = p.geometry, p.masses, leg.name
    servo_half = (SERVO_LEN / 2, SERVO_HGT / 2, SERVO_WID / 2)     # shaft along the link Y (the joint axis)
    off = SERVO_LEN / 2 - SHAFT_FROM_END

    cx = _Parts(n + "_")
    cx.box("coxa_bracket", (geo.coxa / 2, 0, 0), (geo.coxa / 2 - 4, 7, 2), 36.1, PLA)
    cx.box("coxa_servo", (off, 0, -8), (SERVO_LEN / 2, SERVO_WID / 2, SERVO_HGT / 2), SERVO_MASS, SERVO)
    cx.lump(m.coxa, (geo.coxa / 2, 0, -5), "coxa_misc")

    fe = _Parts(n + "_")
    fe.box("femur_arm", (geo.femur / 2, 0, 0), (geo.femur / 2, 2, 6), 21.8, PLA)
    fe.box("femur_servo", (off, 0, 0), servo_half, SERVO_MASS, SERVO)
    fe.box("tibia_servo", (geo.femur - off, 0, 0), servo_half, SERVO_MASS, SERVO)
    fe.lump(m.femur, (geo.femur / 2, 0, 0), "femur_misc")

    ti = _Parts(n + "_")
    L = geo.tibia
    ti.box("tibia_arm", (0, 0, -L / 2), (4, 4, L / 2 - 3), 15.3, PLA)
    ti.box("foot_switch", (5, 0, -L + 12), (3, 3, 6), 2.0, "0.76 0.23 0.23 1")   # switch in the tibia tip
    ti.lump(m.tibia, (0, 0, -L / 2), "tibia_misc")
    return str(cx), str(fe), str(ti)
