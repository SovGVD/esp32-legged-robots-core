"""
Hexapod joint conventions shared by the firmware and the simulator.

The firmware's HexapodLegIK (lib/robot_core/src/ik_hexapod.cpp) produces leg
angles (alpha, beta, gamma), each pi/2 at the servo middle:

  alpha  coxa,  pi/2 + yaw of the leg away from its neutral direction (+ = CCW seen from above)
  beta   femur, pi/2 - femur elevation (the model sets inverse.z, so + elevation = femur up)
  gamma  tibia, inner knee angle between femur and tibia (pi/2 = tibia square to the femur)

The simulator joints are zero at the servo middle, which is the CAD pose:
coxa along its neutral direction, femur level, tibia straight down.

  q_coxa  = alpha - pi/2   + turns the leg CCW (seen from above)
  q_femur = pi/2 - beta    + lifts the femur
  q_tibia = gamma - pi/2   + opens the knee (foot moves outward)

`firmware_ik` is a line-by-line Python port of HexapodLegIK (body yaw = 0)
used as a test oracle: forward kinematics of the MJCF model must put the
foot tip where the firmware IK was asked to put it.
"""

import math
import numpy as np

from .params import HexGeometry, HexLegConfig


def leg_angles_to_q(alpha, beta, gamma):
    return (alpha - math.pi / 2, math.pi / 2 - beta, gamma - math.pi / 2)


def q_to_leg_angles(q_coxa, q_femur, q_tibia):
    return (q_coxa + math.pi / 2, math.pi / 2 - q_femur, q_tibia + math.pi / 2)


def coxa_position(leg: HexLegConfig):
    return np.array(leg.body, dtype=float)


def default_foot(leg: HexLegConfig):
    return np.array(leg.foot, dtype=float)


def _acos(v):
    return math.acos(max(-1.0, min(1.0, v)))


def firmware_ik(geo: HexGeometry, leg: HexLegConfig, foot, body=None):
    """Port of HexapodLegIK::solve for zero body yaw. Returns (alpha, beta, gamma)."""
    if body is None:
        body = coxa_position(leg)
    v = np.asarray(foot, dtype=float) - np.asarray(body, dtype=float)
    inv = leg.inverse_xyz
    lx, ly, lz = (-v[0] if inv[0] else v[0]), (-v[1] if inv[1] else v[1]), (-v[2] if inv[2] else v[2])
    leg_len = math.hypot(lx, ly)
    l_tib_sq = (leg_len - geo.coxa) ** 2 + lz ** 2
    l_tib = math.sqrt(l_tib_sq)
    a1 = _acos(lz / l_tib)
    a2 = _acos((geo.femur ** 2 + l_tib_sq - geo.tibia ** 2) / (2 * geo.femur * l_tib))
    nx, ny = leg.foot[0] - leg.body[0], leg.foot[1] - leg.body[1]
    if inv[0]:
        nx = -nx
    if inv[1]:
        ny = -ny
    d = math.atan2(ly, lx) - math.atan2(ny, nx)
    d = (d + math.pi) % (2 * math.pi) - math.pi
    alpha = math.pi / 2 + d
    beta = math.pi - (a1 + a2)
    gamma = _acos((geo.tibia ** 2 + geo.femur ** 2 - l_tib_sq) / (2 * geo.femur * geo.tibia))
    return alpha, beta, gamma


def forward_kinematics(geo: HexGeometry, leg: HexLegConfig, q):
    """Foot tip relative to the coxa axis in the body frame, mm, from sim joint q."""
    q_coxa, q_femur, q_tibia = q
    reach = geo.coxa + geo.femur * math.cos(q_femur) + geo.tibia * math.sin(q_femur + q_tibia)
    z = geo.femur * math.sin(q_femur) - geo.tibia * math.cos(q_femur + q_tibia)
    yaw = leg.yaw + q_coxa
    return np.array([reach * math.cos(yaw), reach * math.sin(yaw), z])


# + q per + leg angle: coxa q = alpha - pi/2, femur q = pi/2 - beta, tibia q = gamma - pi/2
_Q_PER_ANGLE = (1, -1, 1)


def hal_direction(leg: HexLegConfig, j):
    """Which way the firmware expects joint j to turn for a longer pulse: +1 = + q.
    Servo angle = (leg angle - mid) * ratio + 90 deg, mirrored by the inverse flag."""
    return _Q_PER_ANGLE[j] * (-1 if leg.inverse_abg[j] else 1)
