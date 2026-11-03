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
    cmd = [cxx, "-std=gnu++17", "-O2", "-shared", "-fPIC", "-Wall",
           # the ESP32 build treats float literals as float; keep the numbers identical
           "-fsingle-precision-constant",
           f"-DROBOT_MODEL={MODELS[model]}",
           "-I", CORE_SRC, "-I", FW_SRC,
           *_sources(), "-o", path + ".tmp"]
    res = subprocess.run(cmd, capture_output=True, text=True)
    if res.returncode != 0:
        raise RuntimeError("robot_core bridge build failed:\n" + " ".join(cmd) + "\n" + res.stderr)
    os.replace(path + ".tmp", path)
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
        L.rc_set_height_offset.restype = None
        L.rc_set_height_offset.argtypes = [ctypes.c_float, ctypes.c_float]
        L.rc_sit_height.restype = ctypes.c_float
        L.rc_set_leveling.restype = None
        L.rc_set_leveling.argtypes = [ctypes.c_int]
        L.rc_level_correction.restype = None
        L.rc_level_correction.argtypes = [f32p]
        self.legs = L.rc_leg_count()
        self.dof = L.rc_leg_dof()
        self.loop_dt = float(L.rc_loop_dt())
        self.unsolved = 0
        self.last_imu = (0.0, 0.0, 0.0)
        self.reset()

    def reset(self):
        self.lib.rc_init()
        self.lib.rc_tilt_reset()
        self.unsolved = 0

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

    GAITS = {"default": 0, "trot": 0, "walk": 1}

    def select_gait(self, name):
        """'default'/'trot' (model GAIT_PARAMS) or 'walk' (GAIT_PARAMS_WALK, static stability,
        dog only). Resets the robot."""
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

    def gait_active(self):
        return bool(self.lib.rc_gait_active())

    def drive(self, dog, command=(0, 0, 0, 0, 0, 0), duration=None):
        """Run the firmware on a SimDog (or SimHexapod) for `duration` seconds (default: one tick)
        at the firmware loop rate, holding `command`. Returns the last pulses."""
        ticks = 1 if duration is None else max(1, int(round(duration / self.loop_dt)))
        pulses = None
        for _ in range(ticks):
            self.last_imu = self.imu_angles(dog)
            pulses = self.update(command, imu=self.last_imu)
            dog.write_pulses(pulses)
            dog.step(self.loop_dt)
        return pulses
