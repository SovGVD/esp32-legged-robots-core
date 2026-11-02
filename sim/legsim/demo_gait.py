"""
Minimal open-loop trot used for smoke tests and examples.

This is NOT the firmware gait. It only checks that the model and servo
limits allow walking at all. The real gait (lib/robot_core GaitEngine) will
drive the sim through the bridge described in sim/README.md.
"""

import math
import numpy as np

from .kinematics import default_foot

# diagonal pairs: LF+RH, RF+LH
TROT_OFFSETS = (0.0, 0.5, 0.5, 0.0)


def foot_targets(params, t, speed_mm_s=60.0, cycle=0.4, duty=0.5, step_height=15.0, offsets=TROT_OFFSETS):
    """Foot targets in the body frame (mm) at time t."""
    geo = params.geometry
    stride = speed_mm_s * cycle * duty   # distance the body travels during one stance
    feet = []
    for i, leg in enumerate(params.legs):
        home = default_foot(geo, leg)
        ph = (t / cycle + offsets[i]) % 1.0
        if ph < duty:
            s = ph / duty
            dy = stride / 2 - stride * s
            dz = 0.0
        else:
            s = (ph - duty) / (1 - duty)
            c = (2 * math.pi * s - math.sin(2 * math.pi * s)) / (2 * math.pi)   # cycloid 0..1
            dy = -stride / 2 + stride * c
            dz = step_height * (1 - math.cos(2 * math.pi * s)) / 2
        feet.append(home + np.array([0.0, dy, dz]))
    return feet


def run_trot(dog, duration=4.0, control_dt=0.0044, **kw):
    """Drive SimDog with the open-loop trot. Returns the log of body positions (mm)."""
    log = []
    t = 0.0
    while t < duration:
        dog.set_feet(foot_targets(dog.p, t, **kw))
        dog.step(control_dt)
        t += control_dt
        log.append((t, *dog.body_position_mm(), *dog.body_orientation().values()))
    return np.array(log)
