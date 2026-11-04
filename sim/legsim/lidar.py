"""
Simulated 2D lidar (LDROBOT LD06 / LD19) that talks the real serial protocol.

The sensor spins at 10 rev/s and measures 4500 points a second (0.8 deg
apart). Every control tick the points measured during that tick are cast
from where the lidar is *now* (so a walking, turning, rocking robot skews
and tilts the scan like on the real thing), packed 12 at a time into LD06
packets (47 bytes, CRC-8) and handed to the firmware as UART bytes
(FirmwareCore.lidar_bytes), which parses them with its own driver code.

The mount (position, yaw of the sensor's 0 deg, clockwise) comes from the
firmware model (LIDAR_PARAMS in model_data.h). Rays hit geom group 0: the
floor, walls and furniture of the scene; the robot itself is not seen.
Noise: ~10 mm + 1 % of the distance (LD06 datasheet: +-10 mm up to 300 mm,
a few % further), max range 12 m.
"""

import math

import mujoco
import numpy as np

LD06_HEADER = 0x54
LD06_VER_LEN = 0x2C


def ld06_crc8(data):
    crc = 0
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = ((crc << 1) ^ 0x4D) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
    return crc


def ld06_packet(speed, start, end, mm, intensity, timestamp):
    """One 47 byte packet: angles in 0.01 deg, 12 distances (mm) and intensities."""
    out = bytearray([LD06_HEADER, LD06_VER_LEN])
    out += int(speed).to_bytes(2, "little") + int(start % 36000).to_bytes(2, "little")
    for d, i in zip(mm, intensity):
        out += int(d).to_bytes(2, "little") + bytes([int(i)])
    out += int(end % 36000).to_bytes(2, "little") + int(timestamp % 30000).to_bytes(2, "little")
    out.append(ld06_crc8(out))
    return bytes(out)


class LidarLD06:
    def __init__(self, mount, rate_hz=10.0, points_per_s=4500, noise_mm=10.0, noise_rel=0.01,
                 max_range=12000.0, seed=0, start_angle=0.0):
        """mount: dict from FirmwareCore.lidar_params() (position mm body frame, yaw rad, clockwise)."""
        self.mount = mount
        self.rate = rate_hz
        self.step = 360.0 * rate_hz / points_per_s      # deg between points
        self.noise_mm = noise_mm
        self.noise_rel = noise_rel
        self.max_range = max_range
        self._rng = np.random.default_rng(seed)
        self._angle = start_angle                       # deg, sensor angle of the next point
        self._clock = 0.0
        self._pending = []                              # (angle 0.01 deg, mm, intensity)
        self._geomgroup = np.array([1, 0, 0, 0, 0, 0], dtype=np.uint8)
        self.last_hits = []                             # world points (m) of the last revolution, for drawing
        self._hits = []

    def reset(self):
        self._clock = 0.0
        self._pending = []
        self.last_hits = []
        self._hits = []

    def _cast(self, robot, angles):
        R = robot.data.xmat[robot._torso].reshape(3, 3)
        p0 = robot.data.xpos[robot._torso]
        origin = p0 + R @ (np.asarray(self.mount["position"]) * 0.001)
        a = np.radians(angles)
        body = self.mount["yaw"] + (-a if self.mount["clockwise"] else a)
        dirs = np.stack([-np.sin(body), np.cos(body), np.zeros_like(body)], axis=1) @ R.T
        n = len(angles)
        dist = np.full(n, -1.0)
        geomid = np.full(n, -1, dtype=np.int32)
        vec = dirs.ravel().copy()
        try:   # MuJoCo >= 3.3 has a `normal` output argument
            mujoco.mj_multiRay(robot.model, robot.data, origin, vec, self._geomgroup, 1, -1,
                               geomid, dist, None, n, self.max_range * 0.001)
        except TypeError:
            mujoco.mj_multiRay(robot.model, robot.data, origin, vec, self._geomgroup, 1, -1,
                               geomid, dist, n, self.max_range * 0.001)
        return origin, dirs, dist

    def poll(self, robot, dt):
        """Call every control tick: the UART bytes the lidar sent during it."""
        self._clock += dt
        n = int(self._clock * self.rate * 360.0 / self.step)
        if n <= 0:
            return b""
        self._clock -= n * self.step / (self.rate * 360.0)
        angles = (self._angle + self.step * np.arange(n)) % 360.0
        self._angle = (self._angle + self.step * n) % 360.0
        origin, dirs, dist = self._cast(robot, angles)
        for k in range(n):
            if angles[k] < self.step and self._hits:
                self.last_hits = self._hits
                self._hits = []
            d = dist[k] * 1000.0
            if dist[k] < 0 or d > self.max_range:
                self._pending.append((int(round(angles[k] * 100)), 0, 0))
                continue
            self._hits.append(origin + dirs[k] * dist[k])
            d += self._rng.normal(0, self.noise_mm + self.noise_rel * d)
            self._pending.append((int(round(angles[k] * 100)), int(max(1, min(d, 65000))), 200))
        out = bytearray()
        while len(self._pending) >= 12:
            pts, self._pending = self._pending[:12], self._pending[12:]
            out += ld06_packet(self.rate * 360, pts[0][0], pts[-1][0], [p[1] for p in pts], [p[2] for p in pts],
                               int(self._clock * 1000))
        return bytes(out)

    def draw(self, robot, scene, every=2):
        """Add the last revolution's hits (red dots) and the sensor (dark cylinder) to an mjvScene."""
        def add():
            if scene.ngeom >= scene.maxgeom:
                return None
            g = scene.geoms[scene.ngeom]
            scene.ngeom += 1
            return g
        R = robot.data.xmat[robot._torso].reshape(3, 3)
        p0 = robot.data.xpos[robot._torso]
        origin = p0 + R @ (np.asarray(self.mount["position"]) * 0.001)
        if (g := add()) is not None:
            mujoco.mjv_initGeom(g, mujoco.mjtGeom.mjGEOM_CYLINDER, np.array([0.019, 0.017, 0]),
                                origin - R[:, 2] * 0.012, R.ravel(), np.array([0.15, 0.15, 0.18, 1], dtype=np.float32))
        for p in self.last_hits[::every]:
            if (g := add()) is None:
                break
            mujoco.mjv_initGeom(g, mujoco.mjtGeom.mjGEOM_SPHERE, np.array([0.012, 0, 0]),
                                np.asarray(p, dtype=float), np.eye(3).ravel(), np.array([1.0, 0.15, 0.1, 1], dtype=np.float32))
