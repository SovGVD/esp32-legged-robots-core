"""
Servo HAL mapping, ported from the firmware (hal.cpp, servo.cpp).

  leg angle --getHALAngle--> servo angle --angleToPulse--> pulse (us)

The simulator runs it backwards: pulse --> servo shaft angle (the real servo
is assumed to follow the calibrated servoMainProfile exactly) --> joint angle
through the gear ratio. With the same mid/ratio/inverse/trim as the firmware,
a pulse produced by the firmware lands on the joint angle the IK asked for.
"""

import math
import numpy as np

from .params import SERVO_PROFILE_DEG, SERVO_PROFILE_US, HalParams, LegConfig


def get_hal_angle(angle, mid, trim, ratio, inverse):
    """getHALAngle: leg angle -> servo angle, rad."""
    a = (angle - mid) * ratio + math.pi / 2
    if inverse:
        a = math.pi - a
    return a + trim


def hal_angle_to_leg_angle(servo_angle, mid, trim, ratio, inverse):
    """Inverse of getHALAngle."""
    a = servo_angle - trim
    if inverse:
        a = math.pi - a
    return (a - math.pi / 2) / ratio + mid


def limit_servo_angle(a):
    return min(max(a, 0.0), math.pi)


DOG_PROFILE = (SERVO_PROFILE_DEG, SERVO_PROFILE_US)


def angle_to_pulse(angle_rad, profile=DOG_PROFILE):
    """angleToPulse with servoMainProfile (piecewise linear, truncated to uint16).
    profile: (degrees, pulses) of the model's servoMainProfile, default the dog's."""
    deg_tab, us_tab = profile
    deg = math.degrees(angle_rad)
    deg = min(max(deg, deg_tab[0]), deg_tab[-1])
    return int(np.interp(deg, deg_tab, us_tab))


def pulse_to_angle(us, profile=DOG_PROFILE):
    """Servo shaft angle (rad) the calibrated servo moves to for a pulse width."""
    deg_tab, us_tab = profile
    return math.radians(float(np.interp(us, us_tab, deg_tab)))


def leg_angles_to_pulses(hal: HalParams, leg: LegConfig, angles, trim=(0.0, 0.0, 0.0), profile=DOG_PROFILE):
    """Three leg angles (alpha, beta, gamma) -> three pulses, like runServoHAL."""
    return [angle_to_pulse(limit_servo_angle(get_hal_angle(
        angles[j], hal.mid[j], trim[j], hal.ratio[j], leg.inverse_abg[j])), profile) for j in range(3)]


def servo_angle_to_leg_angle(hal: HalParams, leg: LegConfig, j, servo_angle, trim=0.0):
    return hal_angle_to_leg_angle(servo_angle, hal.mid[j], trim, hal.ratio[j], leg.inverse_abg[j])
