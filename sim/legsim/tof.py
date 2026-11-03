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

    def _corners(self, rays):
        """Edge directions of the field of view (body frame), from the corner zone rays."""
        rays = np.asarray(rays)
        n = int(round(math.sqrt(len(rays))))
        centre = rays.mean(axis=0)
        centre /= np.linalg.norm(centre)
        if n < 2:
            return centre, []
        out = []
        for c, d in ((0, n + 1), (n - 1, 2 * n - 2), (n * n - 1, n * n - n - 2), (n * n - n, n * n - 2 * n + 1)):
            # zones are evenly spaced in tangent: the edge is half a zone past the corner zone
            a = rays[c] / np.dot(rays[c], centre)
            b = rays[d] / np.dot(rays[d], centre)
            e = a + 0.5 * (a - b)
            out.append(e / np.linalg.norm(e))
        return centre, out

    def draw(self, robot, scene, hits=True, cone=True, box=True):
        """Add the sensors (small boxes), their view cones (out to the ground or max
        range) and the last frame's zone hits (dots) to an mjvScene, after
        renderer.update_scene()."""
        R = robot.data.xmat[robot._torso].reshape(3, 3)
        p0 = robot.data.xpos[robot._torso]

        def add():
            if scene.ngeom >= scene.maxgeom:
                return None
            g = scene.geoms[scene.ngeom]
            scene.ngeom += 1
            return g

        for s in self.sensors:
            origin = p0 + R @ (np.asarray(s["position"]) * 0.001)
            centre, edges = self._corners(s["rays"])
            fwd = R @ centre
            if box and (g := add()) is not None:
                # sensor board: 13 x 13 mm, 3 mm thick, facing along the view direction
                y = fwd
                x = np.cross(y, R[:, 2])
                x = x / np.linalg.norm(x) if np.linalg.norm(x) > 1e-6 else R[:, 0]
                z = np.cross(x, y)
                mat = np.column_stack([x, y, z])
                mujoco.mjv_initGeom(g, mujoco.mjtGeom.mjGEOM_BOX, np.array([0.0065, 0.0015, 0.0065]),
                                    origin - y * 0.0015, mat.ravel(), np.array([0.6, 0.15, 0.85, 1], dtype=np.float32))
            if cone and edges:
                ends = []
                for e in edges:
                    d = R @ e
                    dist = min(self._cast(robot, origin, d), s["max_range"] * 0.001)
                    ends.append(origin + d * dist)
                rgba = np.array([1.0, 0.55, 0.1, 0.9], dtype=np.float32)
                for k, end in enumerate(ends):
                    for a, b in ((origin, end), (end, ends[(k + 1) % len(ends)])):
                        if (g := add()) is None:
                            break
                        mujoco.mjv_initGeom(g, mujoco.mjtGeom.mjGEOM_LINE, np.zeros(3), np.zeros(3), np.eye(3).ravel(), rgba)
                        mujoco.mjv_connector(g, mujoco.mjtGeom.mjGEOM_LINE, 2.0, a, b)
        if hits:
            for p in self.last_hits:
                if (g := add()) is None:
                    break
                mujoco.mjv_initGeom(g, mujoco.mjtGeom.mjGEOM_SPHERE, np.array([0.0025, 0, 0]),
                                    np.asarray(p, dtype=float), np.eye(3).ravel(), np.array([0.1, 0.6, 1.0, 0.9], dtype=np.float32))

    def poll(self, robot, dt):
        """Call every control tick: a new frame list when the sensors have one, else None."""
        self._clock += dt
        if self._clock + 1e-9 < self.period:
            return None
        self._clock -= self.period
        return self.measure(robot)
