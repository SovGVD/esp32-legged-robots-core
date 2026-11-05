"""
The real firmware motion code, running against the simulator.

`FirmwareCore` builds lib/robot_core plus sim/bridge/robot_core_bridge.cpp
into a shared library with the host C++ compiler (rebuilt automatically when
any source changes) and calls it through ctypes. Each `update()` is one
firmware control tick: move command -> GaitEngine -> BodyKinematics ->
QuadrupedLegIK -> servo HAL -> pulse widths, all from the C++ sources the
ESP32 runs. `drive()` feeds those pulses to a SimDog.

`FirmwareCore(model="hexapod")` builds the same bridge for the hexapod
(HexapodLegIK and model/hexapod/model_data.h) and drives a SimHexapod.
"""

import ctypes
import math
import glob
import os
import subprocess

import numpy as np

SIM_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
REPO_DIR = os.path.dirname(SIM_DIR)
CORE_SRC = os.path.join(REPO_DIR, "lib", "robot_core", "src")
FW_SRC = os.path.join(REPO_DIR, "src", "esp32-legged-robot")
BRIDGE = os.path.join(SIM_DIR, "bridge", "robot_core_bridge.cpp")
BUILD_DIR = os.path.join(SIM_DIR, "build")
# ROBOT_MODEL values from def/def.h
MODELS = {"dog": 100, "hexapod": 103}
LIB_PATH = os.path.join(BUILD_DIR, "librobot_core_bridge.so")


def lib_path(model="dog"):
    return LIB_PATH if model == "dog" else os.path.join(BUILD_DIR, f"librobot_core_bridge_{model}.so")


def _sources():
    return sorted(glob.glob(os.path.join(CORE_SRC, "*.cpp"))) + [BRIDGE]


def _inputs(model="dog"):
    headers = glob.glob(os.path.join(CORE_SRC, "*.h")) + [
        os.path.join(FW_SRC, "def", "def.h"),
        os.path.join(FW_SRC, "model", model, "config.h"),
        os.path.join(FW_SRC, "model", model, "model_data.h"),
    ]
    return _sources() + headers


def build(force=False, cxx=None, model="dog"):
    """Compile the bridge for `model` ("dog" or "hexapod") if it is missing or
    older than any source. Returns the library path."""
    path = lib_path(model)
    if not force and os.path.exists(path):
        built = os.path.getmtime(path)
        if all(os.path.getmtime(f) <= built for f in _inputs(model)):
            return path
    os.makedirs(BUILD_DIR, exist_ok=True)
    cxx = cxx or os.environ.get("CXX", "c++")
    tmp = f"{path}.{os.getpid()}.tmp"   # parallel builds (several processes) must not share it
    cmd = [cxx, "-std=gnu++17", "-O2", "-shared", "-fPIC", "-Wall",
           # the ESP32 build treats float literals as float; keep the numbers identical
           "-fsingle-precision-constant",
           f"-DROBOT_MODEL={MODELS[model]}",
           "-I", CORE_SRC, "-I", FW_SRC,
           *_sources(), "-o", tmp]
    res = subprocess.run(cmd, capture_output=True, text=True)
    if res.returncode != 0:
        raise RuntimeError("robot_core bridge build failed:\n" + " ".join(cmd) + "\n" + res.stderr)
    os.replace(tmp, path)
    return path


class FirmwareCore:
    """Firmware motion pipeline (one instance per model and process: the C++ side is global state).

    model: "dog" (default) or "hexapod"."""

    def __init__(self, lib_path=None, model="dog"):
        self.model = model
        self.lib = ctypes.CDLL(lib_path or build(model=model))
        L = self.lib
        f32p = ctypes.POINTER(ctypes.c_float)
        L.rc_leg_count.restype = ctypes.c_int
        L.rc_leg_dof.restype = ctypes.c_int
        L.rc_loop_dt.restype = ctypes.c_float
        L.rc_init.restype = None
        L.rc_update.restype = ctypes.c_int
        L.rc_update.argtypes = [ctypes.c_float, f32p, f32p, ctypes.POINTER(ctypes.c_uint16)]
        for name in ("rc_leg_angles", "rc_feet", "rc_body"):
            getattr(L, name).restype = None
            getattr(L, name).argtypes = [f32p]
        L.rc_gait_active.restype = ctypes.c_int
        L.rc_select_gait.restype = ctypes.c_int
        L.rc_select_gait.argtypes = [ctypes.c_int]
        L.rc_support_margin.restype = ctypes.c_float
        L.rc_set_stability_gain.restype = None
        L.rc_set_stability_gain.argtypes = [ctypes.c_float, ctypes.c_float]
        L.rc_tune_gait.restype = None
        L.rc_tune_gait.argtypes = [ctypes.c_float] * 5
        L.rc_tune_terrain.restype = None
        L.rc_tune_terrain.argtypes = [ctypes.c_int, ctypes.c_float]
        L.rc_set_side_limits.restype = None
        L.rc_set_side_limits.argtypes = [ctypes.c_float, ctypes.c_float]
        L.rc_set_center_of_mass.restype = None
        L.rc_set_center_of_mass.argtypes = [ctypes.c_float, ctypes.c_float]
        L.rc_reject_counts.restype = None
        L.rc_reject_counts.argtypes = [ctypes.POINTER(ctypes.c_uint32)]
        L.rc_shift.restype = None
        L.rc_shift.argtypes = [f32p]
        L.rc_tilt_estimate.restype = None
        L.rc_tilt_estimate.argtypes = [ctypes.c_float, f32p, f32p, f32p]
        L.rc_tilt_reset.restype = None
        L.rc_heading_feed.restype = None
        L.rc_heading_feed.argtypes = [ctypes.c_float, f32p, f32p]
        L.rc_heading_hold.restype = None
        L.rc_heading_hold.argtypes = [ctypes.c_int]
        L.rc_turn_to.restype = None
        L.rc_turn_to.argtypes = [ctypes.c_float]
        L.rc_heading_state.restype = None
        L.rc_heading_state.argtypes = [f32p]
        L.rc_set_height_offset.restype = None
        L.rc_set_height_offset.argtypes = [ctypes.c_float, ctypes.c_float]
        L.rc_sit_height.restype = ctypes.c_float
        L.rc_set_leveling.restype = None
        L.rc_set_leveling.argtypes = [ctypes.c_int]
        L.rc_level_correction.restype = None
        L.rc_level_correction.argtypes = [f32p]
        L.rc_leg_test_pulses.restype = ctypes.c_int
        L.rc_leg_test_pulses.argtypes = [ctypes.c_int, f32p, ctypes.POINTER(ctypes.c_uint16)]
        L.rc_leg_inverse.restype = None
        L.rc_leg_inverse.argtypes = [ctypes.c_int, ctypes.POINTER(ctypes.c_int)]
        i32p = ctypes.POINTER(ctypes.c_int)
        L.rc_set_contact_sensing.restype = None
        L.rc_set_contact_sensing.argtypes = [ctypes.c_int]
        L.rc_set_contacts.restype = None
        L.rc_set_contacts.argtypes = [i32p]
        L.rc_set_terrain.restype = None
        L.rc_set_terrain.argtypes = [ctypes.c_int, ctypes.c_int]
        L.rc_tof_count.restype = ctypes.c_int
        L.rc_tof_sensor.restype = ctypes.c_int
        L.rc_tof_sensor.argtypes = [ctypes.c_int, f32p]
        L.rc_tof_rays.restype = None
        L.rc_tof_rays.argtypes = [ctypes.c_int, f32p]
        L.rc_tof_frame.restype = None
        L.rc_tof_frame.argtypes = [ctypes.c_int, f32p]
        L.rc_terrain_height.restype = ctypes.c_int
        L.rc_terrain_height.argtypes = [ctypes.c_float, ctypes.c_float, f32p]
        L.rc_terrain_cells.restype = ctypes.c_int
        L.rc_terrain_state.restype = None
        L.rc_terrain_state.argtypes = [f32p]
        u8p = ctypes.POINTER(ctypes.c_uint8)
        L.rc_lidar_enable.restype = ctypes.c_int
        L.rc_lidar_enable.argtypes = [ctypes.c_int]
        L.rc_lidar_bytes.restype = ctypes.c_int
        L.rc_lidar_bytes.argtypes = [u8p, ctypes.c_int]
        for name in ("rc_lidar_params", "rc_lidar_pose", "rc_lidar_state", "rc_lidar_grid", "rc_nav_state"):
            getattr(L, name).restype = None
            getattr(L, name).argtypes = [f32p]
        L.rc_lidar_map.restype = None
        L.rc_lidar_map.argtypes = [ctypes.POINTER(ctypes.c_int8)]
        L.rc_lidar_points.restype = ctypes.c_int
        L.rc_lidar_points.argtypes = [f32p]
        L.rc_nav_path.restype = ctypes.c_int
        L.rc_nav_path.argtypes = [f32p]
        L.rc_lidar_freeze.restype = None
        L.rc_lidar_freeze.argtypes = [ctypes.c_int]
        L.rc_nav_goal.restype = ctypes.c_int
        L.rc_nav_goal.argtypes = [ctypes.c_float, ctypes.c_float]
        L.rc_nav_stop.restype = None
        L.rc_tofnav_enable.restype = ctypes.c_int
        L.rc_tofnav_enable.argtypes = [ctypes.c_int]
        L.rc_tofnav_scan.restype = None
        L.rc_tofnav_params.restype = None
        L.rc_tofnav_params.argtypes = [f32p]
        L.rc_tofnav_tune.restype = None
        L.rc_tofnav_tune.argtypes = [f32p, ctypes.c_int]
        L.rc_nav_tune.restype = None
        L.rc_nav_tune.argtypes = [f32p, ctypes.c_int]
        L.rc_tof_mount.restype = None
        L.rc_tof_mount.argtypes = [ctypes.c_int, ctypes.c_float]
        self.legs = L.rc_leg_count()
        self.dof = L.rc_leg_dof()
        self.loop_dt = float(L.rc_loop_dt())
        self.unsolved = 0
        self.last_imu = (0.0, 0.0, 0.0)
        self.contacts = None      # foot switch threshold (N) while contact sensing is on
        self.tof = None           # legsim.tof.ToFArray while the terrain map is fed
        self._sensor_tilt = 0.0   # sensor servo angle the ToF rays were last set for
        self.heading_sim = None   # set_heading_sensors()
        self.lidar = None         # legsim.lidar.LidarLD06 while the lidar is on (set_lidar)
        self.tof_nav = False      # ToF room scans feed the map (set_tof_nav)
        self._terrain_tof = None  # ToFArray args while set_terrain(tof=True)
        self._nav_tof = None      # (sensors, ToFArray args) while set_tof_nav(True)
        self.reset()

    # ----------------------------------------------------------------- heading
    def set_heading_sensors(self, gyro_bias_dps=0.0, magnetometer=True, mag_noise=0.5,
                            field=(0.0, 20.0, -40.0), seed=0):
        """Feed the firmware heading code (HeadingEstimator) every tick from the sim:
        the gyro's yaw rate with a constant bias (deg/s, a real MEMS gyro drifts 0.5-2),
        and a magnetometer every 4th tick (100 Hz) with noise (uT). field: earth field
        in the world frame, uT, world x = east, y = north (the robots start facing north).
        None turns the heading feed off."""
        if gyro_bias_dps is None:
            self.heading_sim = None
            return
        self.heading_sim = {"bias": math.radians(gyro_bias_dps), "mag": magnetometer, "noise": mag_noise,
                            "field": np.array(field, dtype=float), "rng": np.random.default_rng(seed), "tick": 0}

    def _feed_heading(self, robot, dt):
        h = self.heading_sim
        R = robot.data.xmat[robot._torso].reshape(3, 3).copy()
        # the body's mean turn rate over the last tick, like a gyro behind its low-pass
        # filter: the instantaneous rate at the tick sampled every tick aliases with
        # the gait's foot impacts (2-3 % short over a turn in place)
        prev = h.get("R")
        h["R"] = R
        if prev is None or dt <= 0:
            rate = np.array(robot.read_imu()["gyro"], dtype=float)
        else:
            Rel = prev.T @ R
            angle = math.acos(max(-1.0, min(1.0, (np.trace(Rel) - 1) / 2)))
            axis = np.array([Rel[2, 1] - Rel[1, 2], Rel[0, 2] - Rel[2, 0], Rel[1, 0] - Rel[0, 1]])
            n = np.linalg.norm(axis)
            rate = axis / n * angle / dt if n > 1e-12 else np.zeros(3)
        gyro = (ctypes.c_float * 3)(float(rate[0]), float(rate[1]), float(rate[2]) + h["bias"])
        mag = None
        h["tick"] += 1
        if h["mag"] and h["tick"] % 4 == 0:
            m = R.T @ h["field"] + h["rng"].normal(0, h["noise"], 3)
            mag = (ctypes.c_float * 3)(*map(float, m))
        self.lib.rc_heading_feed(dt, gyro, mag)

    def heading_hold(self, on=True):
        """Heading hold on/off (`heading on|off`): keeps the heading the robot has when the yaw command is let go."""
        self.lib.rc_heading_hold(1 if on else 0)

    def turn_to(self, degrees):
        """`turnto <deg>`: turn to a compass heading and hold it."""
        self.lib.rc_turn_to(math.radians(degrees % 360))

    def heading_state(self):
        v = self._floats(self.lib.rc_heading_state, 6)
        return {"heading": math.degrees(v[0]), "target": math.degrees(v[1]), "hold": bool(v[2]),
                "has_target": bool(v[3]), "absolute": bool(v[4]), "field": float(v[5])}

    def reset(self):
        if getattr(self, "heading_sim", None) is not None:
            self.heading_sim.pop("R", None)
        self.lib.rc_init()
        self.lib.rc_tilt_reset()
        self.unsolved = 0
        if self.tof is not None:
            self.tof.reset()
        if self.lidar is not None:
            self.lidar.reset()

    # ------------------------------------------------------------------ lidar
    NAV_STATES = ("idle", "active", "arrived", "no path", "no memory")
    MATCHES = ("none", "ok", "new area", "rejected", "few points")

    def lidar_params(self):
        """The model's lidar mount (LIDAR_PARAMS): position (mm, body frame), yaw (rad), clockwise, ranges (mm)."""
        v = self._floats(self.lib.rc_lidar_params, 7)
        return {"position": v[:3], "yaw": float(v[3]), "clockwise": bool(v[4]), "min_range": float(v[5]),
                "max_range": float(v[6])}

    def set_lidar(self, on=True, **lidar_args):
        """Simulated LD06 lidar on the robot feeding the firmware's lidar driver, map and
        localization (kept across reset(), which clears the map). lidar_args go to
        legsim.lidar.LidarLD06 (noise_mm, noise_rel, seed, ...). on=False turns it off."""
        from .lidar import LidarLD06
        if not on:
            self.lidar = None
            self.lib.rc_lidar_enable(0)
            return
        if not self.lib.rc_lidar_enable(1):
            raise MemoryError("lidar map allocation failed")
        self.lidar = LidarLD06(self.lidar_params(), **lidar_args)
        self.reset()

    TOF_PHASES = ("idle", "stop", "tilt", "turn", "level")

    def tof_scan_params(self):
        """The model's TOF_SCAN_PARAMS (tof_scan.h): tilt (rad), rates, turn angle, ranges (mm), ..."""
        v = self._floats(self.lib.rc_tofnav_params, 14)
        keys = ("tilt", "tilt_rate", "turn_rate", "turn_angle", "band", "min_range", "max_range", "min_height",
                "scan_every", "settle_time", "obstacle_height", "crouch", "rescan_angle", "sensor_servo")
        return dict(zip(keys, map(float, v)))

    def set_tof_nav(self, on=True, sensor_range=None, **tof_args):
        """Map, localization and walking to a point with the ToF sensors instead of a lidar
        (tof_scan.h: stop, tilt the nose up, turn once around, scan). The simulated ToF
        sensors (legsim.tof.ToFArray, tof_args: noise, latency, subrays, seed) report out to
        sensor_range mm (default: the scan's max range plus a little), not the terrain
        map's short TOF_SENSORS range. Kept across reset(), which clears the map."""
        from .tof import ToFArray
        if not on:
            self.tof_nav = False
            self.lib.rc_tofnav_enable(0)
            # back to the terrain map's own sensors, if those are on
            self.tof = ToFArray(self.tof_sensors(), **self._terrain_tof) if self._terrain_tof is not None else None
            return
        if not self.lib.rc_tofnav_enable(1):
            raise MemoryError("map allocation failed")
        self.tof_nav = True
        reach = sensor_range or self.tof_scan_params()["max_range"] + 500
        sensors = [dict(s, max_range=reach) for s in self.tof_sensors()]
        # one set of sensors for both: the terrain map ignores readings beyond its TOF_SENSORS range
        self._nav_tof = (sensors, tof_args)
        self.tof = ToFArray(sensors, **tof_args)
        self._sensor_tilt = 0.0
        self.reset()

    def tune_tof_scan(self, reset=False, **fields):
        """Override TOF_SCAN_PARAMS fields (names as tof_scan_params()) for experiments."""
        keys = ("tilt", "tilt_rate", "turn_rate", "turn_angle", "band", "min_range", "max_range", "min_height",
                "scan_every", "settle_time", "obstacle_height", "crouch", "rescan_angle", "sensor_servo")
        v = [float(fields.pop(k, -1)) for k in keys]
        if fields:
            raise TypeError(f"unknown fields {list(fields)}")
        self.lib.rc_tofnav_tune((ctypes.c_float * 14)(*v), 1 if reset else 0)

    def set_tof_mount(self, i, tilt_deg=None):
        """Experiment: ToF sensor i looks tilt_deg down instead of its TOF_SENSORS tilt
        (None: back to the model's). Call before set_terrain() / set_tof_nav()."""
        self.lib.rc_tof_mount(int(i), -1.0 if tilt_deg is None else math.radians(tilt_deg))

    def tof_scan(self):
        """One ToF room scan now (`map scan`)."""
        self.lib.rc_tofnav_scan()

    def lidar_bytes(self, data):
        """LD06 UART bytes into the firmware driver; returns the number of finished scans."""
        if not data:
            return 0
        buf = (ctypes.c_uint8 * len(data)).from_buffer_copy(data)
        return int(self.lib.rc_lidar_bytes(buf, len(data)))

    def lidar_pose(self):
        """(map pose, odometry pose): each (x mm, y mm, theta rad CCW, 0 = facing +y)."""
        v = self._floats(self.lib.rc_lidar_pose, 6)
        return v[:3], v[3:]

    def lidar_state(self):
        v = self._floats(self.lib.rc_lidar_state, 9)
        return {"match": self.MATCHES[int(v[0])], "score": float(v[1]), "scans": int(v[2]), "rejected": int(v[3]),
                "occupied": int(v[4]), "jump": float(v[5]), "turn": float(v[6]), "packets": int(v[7]),
                "crc_errors": int(v[8])}

    def lidar_map(self):
        """(log-odds grid [y, x] int8, cell size mm): cell (ix, iy) covers x from (ix - n/2) * cell."""
        g = self._floats(self.lib.rc_lidar_grid, 2)
        n = int(g[0])
        buf = (ctypes.c_int8 * (n * n))()
        self.lib.rc_lidar_map(buf)
        return np.frombuffer(buf, dtype=np.int8).reshape(n, n).copy(), float(g[1])

    def lidar_points(self):
        buf = (ctypes.c_float * 720)()
        n = self.lib.rc_lidar_points(buf)
        return np.array(buf[:2 * n]).reshape(n, 2)

    def lidar_freeze(self, on=True):
        """Keep the map as it is (localize only)."""
        self.lib.rc_lidar_freeze(1 if on else 0)

    def nav_goal(self, x, y):
        """`goto`: walk to map point (x, y), mm. The move command passed to update()/drive() must be
        zero meanwhile (a stick command cancels it, like on the robot)."""
        return bool(self.lib.rc_nav_goal(float(x), float(y)))

    def nav_stop(self):
        self.lib.rc_nav_stop()

    def nav_state(self):
        v = self._floats(self.lib.rc_nav_state, 12)
        return {"state": self.NAV_STATES[int(v[0])], "blocked": bool(v[1]), "distance": float(v[2]),
                "phase": self.TOF_PHASES[int(v[7])], "tilt": float(v[8]), "scan_points": int(v[9]),
                "partial": bool(v[10]), "sensor_tilt": float(v[11]),
                "goal": v[3:5], "forward": float(v[5]), "yaw": float(v[6])}

    def nav_path(self):
        buf = (ctypes.c_float * 128)()
        n = self.lib.rc_nav_path(buf)
        return np.array(buf[:2 * n]).reshape(n, 2)

    # ------------------------------------------------------------ rough ground
    def tof_sensors(self):
        """The model's ToF sensors (TOF_SENSORS): position (mm, body frame), zone rays
        (body frame unit vectors) and max range (mm)."""
        out = []
        for i in range(self.lib.rc_tof_count()):
            buf = (ctypes.c_float * 4)()
            zones = self.lib.rc_tof_sensor(i, buf)
            rays = (ctypes.c_float * (zones * 3))()
            self.lib.rc_tof_rays(i, rays)
            out.append({"position": np.array(buf[:3]), "max_range": float(buf[3]),
                        "rays": np.array(rays[:]).reshape(zones, 3)})
        return out

    def set_terrain(self, contacts=False, tof=False, adapt=None, switch_threshold=0.3, **tof_args):
        """Rough ground mode (kept across reset()).
        contacts: foot switches on (GaitParams.useContactSensors), fed from the sim each tick.
        tof: simulated ToF sensors feed the firmware terrain map (tof_args go to ToFArray:
             rate_hz, noise_mm, noise_rel, subrays).
        adapt: TERRAIN_PARAMS on (default: on when contacts or tof).
        All False: the plain open-loop gait."""
        from .tof import ToFArray
        adapt = (contacts or tof) if adapt is None else adapt
        self.contacts = switch_threshold if contacts else None
        self._terrain_tof = dict(tof_args) if tof else None
        if self.tof_nav:
            # the room scans' sensors (longer reach) feed the terrain map too
            self.tof = ToFArray(self._nav_tof[0], **self._nav_tof[1])
            self._sensor_tilt = 0.0
        else:
            self.tof = ToFArray(self.tof_sensors(), **tof_args) if tof else None
        self.lib.rc_set_contact_sensing(1 if contacts else 0)
        self.lib.rc_set_terrain(1 if adapt else 0, 1 if tof else 0)

    TERRAIN_FIELDS = ("search_depth", "search_speed", "clearance", "max_step", "body_rate", "edge_limit",
                      "foothold_shift", "foot_radius", "tolerance", "latency", "gap_margin")

    def tune_nav(self, reset=False, **fields):
        """Override NAV_PARAMS fields (navigator.h) for experiments: robot_radius, margin, lookahead,
        goal_tolerance (mm), speed (0..1 of the gait's max), turn_gain, turn_in_place, stop_distance,
        replan_period, doubtful (log-odds). Applies to the lidar and the ToF navigator from their next start."""
        keys = ("robot_radius", "margin", "lookahead", "goal_tolerance", "speed", "turn_gain", "turn_in_place",
                "stop_distance", "replan_period", "doubtful")
        v = [float(fields.pop(k, -1)) for k in keys]
        if fields:
            raise TypeError(f"unknown fields {list(fields)}")
        self.lib.rc_nav_tune((ctypes.c_float * 10)(*v), 1 if reset else 0)

    def tune_terrain(self, reset=False, **fields):
        """Change TERRAIN_PARAMS fields for experiments (kept until tune_terrain(reset=True)),
        e.g. tune_terrain(clearance=15, body_rate=0)."""
        if reset:
            self.lib.rc_tune_terrain(-1, 0.0)
        for k, v in fields.items():
            self.lib.rc_tune_terrain(self.TERRAIN_FIELDS.index(k), float(v))

    def terrain_height(self, x, y):
        """Mapped ground (mm, gait frame) at gait-frame (x, y), or None."""
        z = (ctypes.c_float * 1)()
        return float(z[0]) if self.lib.rc_terrain_height(float(x), float(y), z) else None

    def terrain_cells(self):
        return int(self.lib.rc_terrain_cells())

    def terrain_state(self):
        """Per leg: ground height (mm, gait frame), reaching for contact, swinging; the body's
        terrain height, and whether a gap too wide to step
        over stops the robot (blocked)."""
        v = self._floats(self.lib.rc_terrain_state, self.legs * 3 + 2)
        legs = v[:-2].reshape(self.legs, 3)
        return {"ground": legs[:, 0], "searching": legs[:, 1] > 0, "swing": legs[:, 2] > 0,
                "body": float(v[-2]), "blocked": bool(v[-1])}

    def sense(self, robot, dt=None):
        """Feed the sensors of the current rough ground mode for one tick (drive() calls this)."""
        if self.contacts is not None:
            sw = robot.foot_switches(self.contacts)
            self.lib.rc_set_contacts((ctypes.c_int * self.legs)(*[1 if c else 0 for c in sw]))
        if self.lidar is not None:
            self.lidar_bytes(self.lidar.poll(robot, self.loop_dt if dt is None else dt))
        if self.tof is not None:
            if self.tof_nav:
                # a sensor servo (TOF_SCAN_PARAMS sensorServo) tilts the sensors up for a room scan
                if self.tof.servo is None and self.tof_scan_params()["sensor_servo"] > 0:
                    self.tof.servo = {"pivot": np.mean([s["position"] for s in self.tof.sensors], axis=0)}
                up = self.nav_state()["sensor_tilt"]
                if up != self._sensor_tilt:
                    self._sensor_tilt = up
                    for s, now in zip(self.tof.sensors, self.tof_sensors()):
                        s["rays"] = now["rays"]
            frames = self.tof.poll(robot, self.loop_dt if dt is None else dt)
            if frames is not None:
                for i, f in enumerate(frames):
                    self.lib.rc_tof_frame(i, (ctypes.c_float * len(f))(*map(float, f)))

    def update(self, command=(0, 0, 0, 0, 0, 0), dt=None, imu=None):
        """One control tick. command: move x, y, z, rotate pitch, roll, yaw, each in [-1, 1]
        (y = forward, yaw = turn). imu: (pitch, roll, yaw) rad, firmware convention (see imu_angles()), used by the leveler.
        Returns the servo pulses (us), leg*dof + joint (not the PCA9685 channel)."""
        cmd = (ctypes.c_float * 6)(*[float(c) for c in command])
        imu_arr = None if imu is None else (ctypes.c_float * 3)(*[float(v) for v in imu])
        out = (ctypes.c_uint16 * (self.legs * self.dof))()
        self.unsolved += self.lib.rc_update(self.loop_dt if dt is None else dt, cmd, imu_arr, out)
        return np.frombuffer(out, dtype=np.uint16).copy()

    def _floats(self, fn, n):
        buf = (ctypes.c_float * n)()
        fn(buf)
        return np.array(buf[:], dtype=float)

    def leg_angles(self):
        return self._floats(self.lib.rc_leg_angles, self.legs * 3).reshape(self.legs, 3)

    def feet(self):
        """Foot targets the firmware wants, body frame, mm."""
        return self._floats(self.lib.rc_feet, self.legs * 3).reshape(self.legs, 3)

    def body(self):
        """Gait body pose: x, y, z (mm), pitch, roll, yaw (rad)."""
        return self._floats(self.lib.rc_body, 6)

    GAITS = {"default": 0, "trot": 0, "walk": 1, "rough": 2}

    def select_gait(self, name):
        """'default'/'trot' (model GAIT_PARAMS), 'walk' (GAIT_PARAMS_WALK, static stability,
        dog only) or 'rough' (GAIT_PARAMS_ROUGH, slower and higher for terrain). Resets the robot."""
        if not self.lib.rc_select_gait(self.GAITS[name]):
            raise ValueError(name)
        self.reset()

    def support_margin(self):
        """mm from the centre of mass to the support polygon edge, negative = outside."""
        return float(self.lib.rc_support_margin())

    def set_stability_gain(self, sideways, fore_aft=0.0):
        """Override the current gait's stabilityGain (0..1 per axis); a negative sideways gain
        turns static stability off. Undone by reset()/select_gait()."""
        self.lib.rc_set_stability_gain(float(sideways), float(fore_aft))

    def tune_gait(self, cycle_time=-1, duty_factor=-1, max_speed=-1, step_height=-1, stability_blend=-1):
        """Change the current gait's timing for experiments (undone by reset()/select_gait()).
        A new duty factor keeps each leg's swing start."""
        self.lib.rc_tune_gait(cycle_time, duty_factor, max_speed, step_height, stability_blend)

    def set_side_limits(self, max_side_speed, max_yaw_rate=-1):
        """Override the current gait's sideways foot speed limit (mm/s, 0 = off) and,
        if >= 0, its max yaw rate (rad/s). Undone by reset()/select_gait()."""
        self.lib.rc_set_side_limits(max_side_speed, max_yaw_rate)

    def set_center_of_mass(self, x, y):
        """Override the current gait's centre of mass, mm in the body frame (undone by reset()/select_gait())."""
        self.lib.rc_set_center_of_mass(x, y)

    def shift(self):
        """Static stability body shift, world x, y, mm."""
        return self._floats(self.lib.rc_shift, 2)

    def reject_counts(self):
        """Leg poses the firmware rejected since reset(): out of reach, joint limits, servo range."""
        buf = (ctypes.c_uint32 * 3)()
        self.lib.rc_reject_counts(buf)
        return {"reach": buf[0], "joint_limits": buf[1], "servo_range": buf[2]}

    def imu_angles(self, dog, dt=None):
        """What the firmware IMU driver reports: the simulated accelerometer and
        gyro through the firmware's TiltEstimator (so walking accelerations
        are in it, like on the robot). (pitch, roll, yaw), firmware convention:
        pitch is the sideways lean, roll the nose up/down tilt. Call once per tick."""
        imu = dog.read_imu()
        f3 = ctypes.c_float * 3
        out = (ctypes.c_float * 2)()
        self.lib.rc_tilt_estimate(self.loop_dt if dt is None else dt,
                                  f3(*(float(a) / 9.81 for a in imu["accel"])), f3(*map(float, imu["gyro"])), out)
        return (out[0], out[1], dog.body_orientation()["yaw"])

    def set_height_offset(self, mm, rate=20.0):
        """Lower (negative) or raise the body, mm, like the battery sit-down. Undone by reset()."""
        self.lib.rc_set_height_offset(float(mm), float(rate))

    def sit_height(self):
        """The model's SIT_HEIGHT, mm."""
        return float(self.lib.rc_sit_height())

    def set_leveling(self, on):
        """IMU leveling (BodyLeveler) on/off; stays across reset()."""
        self.lib.rc_set_leveling(1 if on else 0)

    def level_correction(self):
        """Leveler output: pitch, roll in rad, firmware convention."""
        return self._floats(self.lib.rc_level_correction, 2)

    def place(self, dog):
        """Put `dog` on the floor in the firmware's current pose (call after reset()
        or select_gait()): the standing pose is not the plain neutral stance when
        the gait offsets the body over the centre of mass."""
        dog.reset(leg_angles=self.leg_angles())
        return dog

    def leg_test_pulses(self, leg, offset):
        """Pulses for one leg with its foot at the neutral point + offset (mm, body frame),
        through the model's own IK and HAL. Returns (pulses[dof], solved)."""
        out = (ctypes.c_uint16 * self.dof)()
        ok = self.lib.rc_leg_test_pulses(int(leg), (ctypes.c_float * 3)(*map(float, offset)), out)
        return np.frombuffer(out, dtype=np.uint16).copy(), bool(ok)

    def leg_inverse(self, leg):
        """The model's angle inverse flags (alpha, beta, gamma) for one leg."""
        out = (ctypes.c_int * 3)()
        self.lib.rc_leg_inverse(int(leg), out)
        return tuple(bool(v) for v in out)

    def gait_active(self):
        return bool(self.lib.rc_gait_active())

    def drive(self, dog, command=(0, 0, 0, 0, 0, 0), duration=None, ideal_servos=False):
        """Run the firmware on a SimDog (or SimHexapod) for `duration` seconds (default: one tick)
        at the firmware loop rate, holding `command`. Returns the last pulses.
        ideal_servos: send the firmware's leg angles to the joints instead of its
        pulses (SimHexapod.set_leg_angles_ideal), for servos/gears the HAL doesn't model."""
        ticks = 1 if duration is None else max(1, int(round(duration / self.loop_dt)))
        pulses = None
        for _ in range(ticks):
            self.sense(dog)
            if getattr(self, "heading_sim", None) is not None:
                self._feed_heading(dog, self.loop_dt)
            self.last_imu = self.imu_angles(dog)
            pulses = self.update(command, imu=self.last_imu)
            if ideal_servos:
                dog.set_leg_angles_ideal(self.leg_angles())
            else:
                dog.write_pulses(pulses)
            dog.step(self.loop_dt)
        return pulses
