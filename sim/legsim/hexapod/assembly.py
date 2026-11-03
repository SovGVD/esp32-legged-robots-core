"""
Where each printed part of the hexapod sits on the sim links.

The STL exports keep their FreeCAD frames, and those turned out to be one
assembly drawn for the right middle leg (RM) with the leg straight out along +X:

  Body-bottom/top  body frame, Y forward; coxa bearing holes at (+-46, +-80)
                   and (+-57.5, 0) = LEG_BODY_X/Y and LEG_MIDDLE_BODY_X
  Coxa             yaw axis at x = 57.5, femur axis at x = 92.5 (coxa 35),
                   z = 28.7 above the body bottom, along Y
  Femur, Tibia_*   femur frame: femur axis on Y through the origin, knee axis
                   at x = 55, tibia tip at z = -110 (femur level, tibia down:
                   all servos at their middle)
  CoxaGear         36-tooth sector, centre at (-36, 12), z 13.6..18.6
  servo screw holes  coxa servos in the body bottom (pockets with 27.8 mm
                   hole pairs), femur servo in the coxa, tibia servo in the femur

Gear centre distances fix where the servo shafts are: 27 mm (18 + 36 teeth,
module 1) for the coxa and femur, 22.5 mm (18 + 27) for the tibia.

Photos (Gleb, 2026-11-03): all femur and tibia gears face forward, so the
right legs are the CAD parts mirrored and the left legs the CAD parts turned
by 180 degrees about Z (see `leg_transform`).

Estimated, not in the parts: how the servos sit sideways in their pockets
(+-2 mm), the electronics and battery positions.

Everything here is in mm, in the frame of the sim link it belongs to, with
the link's X along the leg (neutral direction), Z up, at the zero pose.
"""

import math

import numpy as np

AXIS_Z = 28.7                # femur axis above the body bottom (body origin) [cad]
COXA_X = 57.5                # RM coxa axis in the CAD body frame [cad]
FEMUR_Y = -13.8              # femur file -> femur centred on the coxa fork [cad]

# Part masses (g), PLA, ~1 mm shell, 20% infill, from the STL volumes [stl]
PART_MASS = {
    "body_bottom": 62.2,
    "body_top": 61.6,
    "coxa": 11.6,
    "coxa_gear": 3.3,
    "femur": 12.8,
    "tibia_side1": 8.8,
    "tibia_side2": 9.5,
    "tibia_tip": 0.4,
    "servo_gear": 1.0,
}

SERVO_MASS = 13.4                   # MG90S, g [ds]
SERVO_LEN, SERVO_WID, SERVO_HGT = 22.8, 12.2, 22.5   # body without tabs, mm [ds]
SERVO_GEAR_FROM_TOP = 0.0           # gear sits right on the servo top face [est]

# Servo gear mesh: z is the shaft axis (0..5); turn it so the shaft points along -Y
_GEAR_TO_MINUS_Y = np.array([[1, 0, 0], [0, 0, -1], [0, 1, 0]], dtype=float)
_GEAR_TO_PLUS_Y = np.array([[1, 0, 0], [0, 0, 1], [0, -1, 0]], dtype=float)


def leg_transform(leg):
    """Rotation from the RM-leg description to the leg's link frame axes in the body frame.
    Right legs: CAD mirrored (y -> -y) so the gears face forward; left legs: CAD turned."""
    c, s = math.cos(leg.yaw), math.sin(leg.yaw)
    R = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
    return R @ np.diag([1.0, -1.0, 1.0]) if leg.side > 0 else R


def leg_mirror(leg):
    """The part transform inside the leg frame (mirror for right legs)."""
    return np.diag([1.0, -1.0, 1.0]) if leg.side > 0 else np.eye(3)


def torso_parts():
    """Body halves, electronics. (kind, name, R, t, half_size) in the body frame."""
    I = np.eye(3)
    return [
        ("mesh", "body_bottom", I, (0, 0, -AXIS_Z), None),
        ("mesh", "body_top", I, (0, 0, -AXIS_Z), None),
    ]


def coxa_servo(leg):
    """Coxa servo (upside down, shaft down) and its gear in the body bottom, body frame.

    Hole pairs 27.8 mm apart at z = 25 [cad]: RF servo centre (16.65, 67.3) along X,
    RM centre (30.5, -5.5) along Y; shafts 27 mm from the coxa axis. The servo top
    face (with the gear under it) is at z = 18.6, the CoxaGear plane [cad]."""
    sx = 1 if leg.body[0] > 0 else -1
    middle = abs(leg.body[1]) < 1
    sy = 1 if leg.body[1] >= 0 else -1
    if middle:
        centre = (sx * 30.5, -5.5, 0)
        shaft = (sx * 30.5, 0.0)
        half = (SERVO_WID / 2, SERVO_LEN / 2, SERVO_HGT / 2)
    else:
        centre = (sx * 16.65, sy * 67.3, 0)
        shaft = (sx * 22.15, sy * 67.3)
        half = (SERVO_LEN / 2, SERVO_WID / 2, SERVO_HGT / 2)
    top = 18.6 - AXIS_Z
    centre = (centre[0], centre[1], top + SERVO_HGT / 2)
    return [
        ("box", "coxa_servo", np.eye(3), centre, half),
        ("mesh", "servo_gear", np.eye(3), (shaft[0], shaft[1], top - 5.0), None),
    ]


def coxa_parts():
    """Coxa link (origin on the coxa axis at body origin height), RM description."""
    I = np.eye(3)
    return [
        ("mesh", "coxa", I, (-COXA_X, 0, -AXIS_Z), None),
        # sector keyed to the coxa pin, teeth inward to the servo in the body
        ("mesh", "coxa_gear", I, (35.98, -11.99, -AXIS_Z), None),
        # femur servo: tab holes at x = 67.4, z = 10.3 / 38.2, shaft 27 mm from the femur axis
        ("box", "femur_servo", I, (67.4 - COXA_X, 0.0, 24.25 - AXIS_Z), (SERVO_WID / 2, SERVO_HGT / 2, SERVO_LEN / 2)),
        ("mesh", "femur_servo_gear", _GEAR_TO_MINUS_Y, (67.4 - COXA_X, FEMUR_Y, 18.75 - AXIS_Z), None),
    ]


def femur_parts():
    """Femur link (origin on the femur axis), RM description."""
    I = np.eye(3)
    return [
        ("mesh", "femur", I, (0, FEMUR_Y, 0), None),
        # tibia servo: tab holes at x = 13.1 / 40.9, shaft 22.5 mm from the knee
        ("box", "tibia_servo", I, (27.0, 0.0, 0.0), (SERVO_LEN / 2, SERVO_HGT / 2, SERVO_WID / 2)),
        ("mesh", "tibia_servo_gear", _GEAR_TO_MINUS_Y, (32.5, FEMUR_Y, 0.0), None),
    ]


def tibia_parts(femur_len=55.0):
    """Tibia link (origin on the knee axis), RM description."""
    I = np.eye(3)
    t = (-femur_len, FEMUR_Y, 0)
    return [
        ("mesh", "tibia_side1", I, t, None),
        ("mesh", "tibia_side2", I, t, None),
        ("mesh", "tibia_tip", I, (0, FEMUR_Y + 13.8, 0), None),
    ]


# ---------------------------------------------------------------- servo directions
#
# Which way each joint really turns for a longer pulse, from how the servo is
# mounted, independent of the firmware's inverse flags. The sim moves the
# joints this way, so a wrong flag in model_data.h shows up as a leg moving
# the wrong way in the sim, like on the robot.

# A longer pulse turns the servo output counter-clockwise seen from the shaft
# end (right-handed about the shaft's outward direction). From the dog's
# working config: its hip-pitch servos drive the leg directly, shaft pointing
# outward, and RF (no inverse) / LF (inverse) both come out this way.
SERVO_TURNS_CCW = True

# Servo shaft directions (outward from the servo body), RM description, link frames:
# coxa servo upside down in the body bottom (gear under it), femur servo in the
# coxa and tibia servo in the femur with their gears on the -Y side of the leg
# (the femur and tibia gears are on that side of the parts) [cad]
SHAFT = {
    0: (0.0, 0.0, -1.0),
    1: (0.0, -1.0, 0.0),
    2: (0.0, -1.0, 0.0),
}
GEAR_STAGES = {0: 1, 1: 1, 2: 1}       # external gear pairs between servo and joint (each reverses)
JOINT_AXIS = {0: (0, 0, 1), 1: (0, -1, 0), 2: (0, -1, 0)}   # sim joint axes, + q direction (mjcf.py)


def servo_direction(leg, j):
    """+1 if a longer pulse turns joint j of `leg` in its + q direction, -1 if the other way."""
    s = np.asarray(SHAFT[j], dtype=float)
    if j != 0:                           # coxa servos sit in the body, the same for every leg
        s = leg_mirror(leg) @ s          # the mirrored legs have the servo the other way round
    turn = s if SERVO_TURNS_CCW else -s
    if GEAR_STAGES[j] % 2:
        turn = -turn
    return 1 if float(turn @ np.asarray(JOINT_AXIS[j], dtype=float)) > 0 else -1
