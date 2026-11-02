"""
Where each printed part (STL from Thingiverse 4822059) sits on the sim links.

The FreeCAD file has the parts but no assembly, so the placements below were
worked out from the part geometry (hole positions) and the photos:

  leg_top     hub holes 51.94 mm apart                 -> exactly L2 (51.9)
  leg_bottom  foot ring to knee servo shaft ~50 mm     -> L3 (50.0)
  holders     bearing holes at x = +-39.5, z = 0       -> hip X, roll axis height
  holders     alpha servo gear at x = +-11             -> 28.5 mm centre distance, 2:1 gear
  body        86.6 mm long, its ends meet the inner holder plates
              -> hip pitch axis at y = +-70.4, matches LEG_BODY_Y (70.5, "not sure")

Everything is described for the right front leg (RF) in mm, in the frame of
the sim link it belongs to, with the link axes parallel to the body axes
(X right, Y forward, Z up) at the zero pose. Other legs are mirrored (left)
and/or turned around (hind) from it; see `leg_transform`.

Estimates (not visible in the parts): the hip-pitch servo shaft position in
the shoulder bracket (HIP_PITCH_DROP below the roll axis, SHAFT_FROM_GEAR
along it), and the stacking of leg_top, the knee servo and leg_bottom.
"""

import numpy as np

# Part masses (g) from sim/stl-mass-estimate.txt (PLA, ~1 mm shell, 20% infill)
PART_MASS = {
    "legs_holder_part1": 17.7,
    "legs_holder_part2": 7.5,
    "body": 9.9,
    "servo_gear": 1.2,
    "shoulder_part1": 2.2,
    "shoulder_part2": 3.1,
    "leg_top": 2.8,
    "leg_bottom": 2.2,
    "leg_shoes_print_with_flexible": 0.5,
}

SERVO_MASS = 13.4                       # MG90D, g
SERVO_HALF = (11.4, 6.1, 11.35)         # MG90 body half sizes: length, width, height (mm)
SERVO_SHAFT_FROM_END = 5.9              # shaft centre from one end of the body, mm

# Shoulder bracket (shoulder_part2) frame: z along the roll axis, y radial.
SHAFT_FROM_GEAR = 10.9                  # hip pitch shaft, mm along the roll axis from the gear end [est]
HIP_PITCH_DROP = 9.1                    # hip pitch shaft below the roll axis, mm [est]


def _rot(cols):
    """Rotation from part axes to link axes, given the link-frame images of part x, y, z."""
    return np.array(cols, dtype=float).T


def _rot_from_pairs(a1, b1, a2, b2):
    """Rotation mapping unit vector a1 -> b1 and a2 -> b2 (a1 is perpendicular to a2)."""
    a1, b1, a2, b2 = (np.asarray(v, dtype=float) for v in (a1, b1, a2, b2))
    a1 /= np.linalg.norm(a1)
    b1 /= np.linalg.norm(b1)
    A = np.column_stack([a1, a2, np.cross(a1, a2)])
    B = np.column_stack([b1, b2, np.cross(b1, b2)])
    return B @ A.T


def torso_parts(hip_y):
    """Printed parts and boxes on the torso, front holder assembly, right side where it matters.

    Returns list of (kind, name, R, t, extra) in mm; kind is "mesh" or "box".
    `per_leg` entries are repeated for every leg with the leg transform,
    the others once per end (front/hind).
    """
    ys0 = hip_y + SHAFT_FROM_GEAR                 # shoulder gear plane (shoulder_part z = 0)
    per_end = [
        ("mesh", "legs_holder_part1", np.eye(3), (0, ys0 + 4, 0)),
        ("mesh", "legs_holder_part2", np.eye(3), (0, ys0 - 34, 0)),
    ]
    per_leg = [
        # alpha servo gear: axis along Y, in the shoulder gear plane, meshes at x = 11
        ("mesh", "servo_gear", _rot([(1, 0, 0), (0, 0, -1), (0, 1, 0)]), (11, ys0 - 1, 0)),
        # alpha servo body in the holder arms, shaft along Y at (11, z = 0)
        ("box", "alpha_servo", np.eye(3), (11, ys0 - 5 - SERVO_HALF[2], -SERVO_HALF[0] + SERVO_SHAFT_FROM_END),
         (SERVO_HALF[1], SERVO_HALF[2], SERVO_HALF[0])),
    ]
    once = [
        # middle frame: its long axis (x) along the robot's Y
        ("mesh", "body", _rot([(0, 1, 0), (-1, 0, 0), (0, 0, 1)]), (0, 0, -2.6)),
    ]
    return per_end, per_leg, once


def shoulder_parts(visual_drop):
    """Shoulder link (origin on the roll axis at the hip, rotates with hip roll).

    visual_drop: how far the drawn bracket is raised so the drawn hip-pitch
    shaft lands on the sim pitch joint (HIP_PITCH_DROP when the sim uses the
    firmware geometry with the pitch axis on the roll axis, 0 when it models the drop).
    """
    t = np.array([0, SHAFT_FROM_GEAR, visual_drop])
    r2 = _rot([(1, 0, 0), (0, 0, -1), (0, 1, 0)])     # bracket: radial (y) down, z along the roll axis
    r1 = _rot([(0, 0, -1), (-1, 0, 0), (0, 1, 0)])    # gear sector: teeth (y) toward the alpha servo
    servo_c = np.array([0, SHAFT_FROM_GEAR - 5 - SERVO_HALF[0], -HIP_PITCH_DROP + visual_drop])
    return [
        ("mesh", "shoulder_part2", r2, t),
        ("mesh", "shoulder_part1", r1, t),
        ("box", "pitch_servo", np.eye(3), servo_c, (SERVO_HALF[2], SERVO_HALF[0], SERVO_HALF[1])),
    ]


# leg_top: hip hub at its origin, knee hub at (50.02, -13.99); horns sit on the -y face, outward.
LEG_TOP_KNEE = np.array([50.02, 0.0, -13.99])
LEG_TOP_INNER_FACE = 11.1      # mm outward of the leg plane (knee servo top + spline) [est]

# leg_bottom: foot ring at (0, -14), knee servo pocket 44.8..67.8 along x, shaft ~50 from the ring.
LEG_BOTTOM_KNEE = np.array([50.0, 0.0, -14.0])
SHOE_ARC_CENTRE = np.array([-0.45, 0.0, -13.97])
KNEE_SERVO_LATERAL = -4.65     # knee servo body centre, inward of the leg plane [est]


def upper_parts():
    R = _rot_from_pairs(LEG_TOP_KNEE, (0, 0, -1), (0, 1, 0), (-1, 0, 0))
    return [("mesh", "leg_top", R, (LEG_TOP_INNER_FACE, 0, 0))]


def lower_parts():
    R = _rot([(0, 0, 1), (1, 0, 0), (0, 1, 0)])       # x: foot -> knee is up, y: outward
    t = -R @ LEG_BOTTOM_KNEE
    shoe_t = R @ (np.array([0, 0, -14.0]) - SHOE_ARC_CENTRE) + t
    servo_c = R @ np.array([LEG_BOTTOM_KNEE[0] - 5.2 + SERVO_HALF[0], KNEE_SERVO_LATERAL, -14.0]) + t
    return [
        ("mesh", "leg_bottom", R, t),
        ("mesh", "leg_shoes_print_with_flexible", R, shoe_t),
        ("box", "knee_servo", np.eye(3), servo_c, (SERVO_HALF[2], SERVO_HALF[1], SERVO_HALF[0])),
    ]


def leg_transform(side, end, turn_hind):
    """Linear map from the RF description to a leg. Mirror for the left side;
    the holder and shoulder of hind legs are additionally turned around (180 deg about Z).
    Leg links are only mirrored: all knees bend the same way in the firmware IK."""
    return np.diag([side * (end if turn_hind else 1), end if turn_hind else 1, 1.0])
