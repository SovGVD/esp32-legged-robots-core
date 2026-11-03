"""
Simulated multizone ToF distance sensors (VL53L5CX 8x8 / 4x4 zones, or a
VL53L1X with ROI zones).

The sensor mounts and zone rays come from the firmware model
(TOF_SENSORS in model_data.h, through FirmwareCore.tof_sensors()), so the
sim and the firmware agree on where each zone looks. Each zone is cast as
one ray (or `subrays` x `subrays` rays across the zone, reporting the
closest, like the sensor's "closest target" mode) against the floor and the
terrain (geom group 0); the robot itself is not seen.

Defaults are VL53L5CX-like: 15 Hz frames at 8x8, ~2 mm + 1 % noise,
readings beyond the sensor's max range report nothing (0).
"""

import math

import mujoco
import numpy as np


class ToFArray:
    def __init__(self, sensors, rate_hz=15.0, noise_mm=2.0, noise_rel=0.01, subrays=1, zone_fov=None, seed=0):
        """sensors: list of dicts from FirmwareCore.tof_sensors() (position mm, rays body frame, max_range mm)."""
        self.sensors = sensors
        self.period = 1.0 / rate_hz
        self.noise_mm = noise_mm
        self.noise_rel = noise_rel
        self.subrays = subrays
        self.zone_fov = zone_fov
        self._rng = np.random.default_rng(seed)
        self._clock = 0.0
        self.last_hits = []      # world points (m) of the last frames, for drawing
        self._geomgroup = np.array([1, 0, 0, 0, 0, 0], dtype=np.uint8)
        self._geomid = np.zeros(1, dtype=np.int32)

    def reset(self):
        self._clock = 0.0
        self.last_hits = []

    def _cast(self, robot, pnt, vec):
        d = mujoco.mj_ray(robot.model, robot.data, pnt, vec, self._geomgroup, 1, -1, self._geomid)
        return d if d >= 0 else math.inf

    def _subray_dirs(self, ray, n_zones):
        if self.subrays <= 1:
            return [ray]
        # spread across the zone: two vectors perpendicular to the ray
        zone = (self.zone_fov or math.radians(45)) / math.sqrt(n_zones)
        a = np.cross(ray, [0, 0, 1.0])
        if np.linalg.norm(a) < 1e-6:
            a = np.array([1.0, 0, 0])
        a /= np.linalg.norm(a)
        b = np.cross(a, ray)
        out = []
        for i in range(self.subrays):
            for j in range(self.subrays):
                u = ((i + 0.5) / self.subrays - 0.5) * zone
                v = ((j + 0.5) / self.subrays - 0.5) * zone
                d = ray + math.tan(u) * a + math.tan(v) * b
                out.append(d / np.linalg.norm(d))
        return out

    def measure(self, robot):
        """One frame of every sensor now: list of arrays of distances (mm, 0 = no target)."""
        R = robot.data.xmat[robot._torso].reshape(3, 3)
        p0 = robot.data.xpos[robot._torso]
        frames = []
        hits = []
        for s in self.sensors:
            origin = p0 + R @ (np.asarray(s["position"]) * 0.001)
            out = np.zeros(len(s["rays"]))
            for z, ray in enumerate(s["rays"]):
                dists = [self._cast(robot, origin, R @ d) for d in self._subray_dirs(np.asarray(ray), len(s["rays"]))]
                d = min(dists) * 1000.0
                if not math.isfinite(d) or d > s["max_range"]:
                    continue
                hits.append(origin + R @ np.asarray(ray) * d * 0.001)
                d += self._rng.normal(0, self.noise_mm + self.noise_rel * d)
                out[z] = max(d, 1.0)
            frames.append(out)
        self.last_hits = hits
        return frames

    def poll(self, robot, dt):
        """Call every control tick: a new frame list when the sensors have one, else None."""
        self._clock += dt
        if self._clock + 1e-9 < self.period:
            return None
        self._clock -= self.period
        return self.measure(robot)
