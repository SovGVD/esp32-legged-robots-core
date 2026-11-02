"""
Hobby servo model (MG90D class), vectorised over all joints.

What a real servo does, and what is modelled:
  * reads the pulse once per 20 ms frame (50 Hz), so commands are latched
  * proportional position loop that saturates after a few degrees of error
  * a deadband: inside it the motor is not driven at all
  * DC motor torque-speed line: available torque drops to 0 at no-load speed
  * reflected rotor inertia and gear friction (set on the MJCF joints)
  * pulse 0 = no signal: the servo goes limp

Everything is computed at the servo shaft and reflected to the joint through
the gear ratio (2:1 on hip roll: twice the torque, half the speed).
"""

import numpy as np

from .params import ServoParams


class ServoArray:
    def __init__(self, params: ServoParams, ratio):
        self.p = params
        self.ratio = np.asarray(ratio, dtype=float)   # per joint
        n = len(self.ratio)
        self.target = np.full(n, np.nan)       # latched joint target (rad, sim q), nan = limp
        self.pending = np.full(n, np.nan)      # last commanded target, latched on the next frame
        self.next_frame = 0.0
        # deadband in shaft radians: profile slope is ~10 us/deg around the middle
        self.deadband = np.radians(self.p.deadband_us / 10.0)

    def reset(self, q=None):
        self.next_frame = 0.0
        if q is None:
            self.target[:] = np.nan
            self.pending[:] = np.nan
        else:
            self.target[:] = q
            self.pending[:] = q

    def command(self, q_target):
        """Joint targets (sim q, rad); nan entries make that servo limp."""
        self.pending[:] = q_target

    def torque(self, t, q, qd):
        """Joint torques for the current state. Call every physics step."""
        if t >= self.next_frame:
            self.target[:] = self.pending
            self.next_frame += self.p.pwm_period
            if self.next_frame <= t:
                self.next_frame = t + self.p.pwm_period
        p, r = self.p, self.ratio
        err_s = (self.target - q) * r            # shaft error
        w_s = qd * r                             # shaft speed
        active = ~np.isnan(err_s)
        err_s = np.where(active, err_s, 0.0)
        # deadband: no drive close to the target
        driven = active & (np.abs(err_s) >= self.deadband)
        err_s = err_s - np.sign(err_s) * self.deadband
        tau_s = np.where(driven, p.kp * err_s - p.kd * w_s, 0.0)
        # motor torque-speed line (with current limit at stall torque)
        hi = np.minimum(p.stall_torque, p.stall_torque * (1.0 - w_s / p.no_load_speed))
        lo = np.maximum(-p.stall_torque, -p.stall_torque * (1.0 + w_s / p.no_load_speed))
        tau_s = np.clip(tau_s, lo, hi)
        return tau_s * r
