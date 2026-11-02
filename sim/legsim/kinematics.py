"""
Joint conventions shared by the firmware and the simulator.

The firmware talks in "leg angles" (alpha, beta, gamma) produced by
QuadrupedLegIK (lib/robot_core/src/ik_quadruped.cpp). The simulator uses
MuJoCo joint positions with a zero-at-straight-down convention:

  q_roll  = pi/2 - alpha   hip roll,  + moves the foot outward
  q_pitch = pi/2 - beta    hip pitch, + swings the upper leg backward
  q_knee  = pi - gamma     knee,      0 = straight leg, + bends the lower leg forward

`firmware_ik` is a line-by-line Python port of QuadrupedLegIK (yaw = 0) and
is used as a test oracle: forward kinematics of the MJCF model must put the
foot where the firmware IK was asked to put it.
"""

import math
import numpy as np

from .params import Geometry, LegConfig


def leg_angles_to_q(alpha, beta, gamma):
    return (math.pi / 2 - alpha, math.pi / 2 - beta, math.pi - gamma)


def q_to_leg_angles(q_roll, q_pitch, q_knee):
    return (math.pi / 2 - q_roll, math.pi / 2 - q_pitch, math.pi - q_knee)


def hip_position(geo: Geometry, leg: LegConfig):
    """Shoulder position in the body frame, mm (leg.body in the firmware)."""
    y = geo.hip_y_front if leg.end > 0 else geo.hip_y_hind
    return np.array([leg.side * geo.hip_x, leg.end * y, geo.hip_z])


def default_foot(geo: Geometry, leg: LegConfig):
    """Neutral foot point in the body frame, mm (leg.defaultFoot)."""
    y = geo.foot_y_front if leg.end > 0 else geo.foot_y_hind
    return np.array([leg.side * geo.foot_x, leg.end * y, -geo.foot_z])


def _clamp_unit(v):
    return max(-1.0, min(1.0, v))


def firmware_ik(geo: Geometry, leg: LegConfig, foot, body=None):
    """Port of QuadrupedLegIK::solve for zero body yaw.

    foot/body are leg.foot and leg.body in mm. Returns (alpha, beta, gamma).
    """
    if body is None:
        body = hip_position(geo, leg)
    lx = foot[0] - body[0]
    ly = foot[1] - body[1]
    lz = foot[2] - body[2]
    inv = leg.inverse_xyz
    if inv[0]:
        lx = -lx
    if inv[1]:
        ly = -ly
    if inv[2]:
        lz = -lz
    a = lx * lx + lz * lz
    sqrta = math.sqrt(a)
    b = ly * ly + a
    sqrtb = math.sqrt(b)
    l2, l3 = geo.l2, geo.l3
    alpha = math.pi / 2 - math.asin(_clamp_unit(lx / sqrta))
    beta = (math.pi / 2 - math.asin(_clamp_unit(ly / sqrtb))
            - math.acos(_clamp_unit((b + l2 * l2 - l3 * l3) / (2 * sqrtb * l2))))
    gamma = math.acos(_clamp_unit((l2 * l2 + l3 * l3 - b) / (2 * l2 * l3)))
    return alpha, beta, gamma


def forward_kinematics(geo: Geometry, leg: LegConfig, q):
    """Foot point relative to the shoulder in the body frame, mm, from sim joint q."""
    q_roll, q_pitch, q_knee = q
    back = geo.l2 * math.sin(q_pitch) + geo.l3 * math.sin(q_pitch - q_knee)
    down = geo.hip_pitch_drop + geo.l2 * math.cos(q_pitch) + geo.l3 * math.cos(q_pitch - q_knee)
    return np.array([leg.side * down * math.sin(q_roll), -back, -down * math.cos(q_roll)])
