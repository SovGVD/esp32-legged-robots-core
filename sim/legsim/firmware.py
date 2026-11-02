"""
The real firmware motion code, running against the simulator.

`FirmwareCore` builds lib/robot_core plus sim/bridge/robot_core_bridge.cpp
into a shared library with the host C++ compiler (rebuilt automatically when
any source changes) and calls it through ctypes. Each `update()` is one
firmware control tick: move command -> GaitEngine -> BodyKinematics ->
QuadrupedLegIK -> servo HAL -> pulse widths, all from the C++ sources the
ESP32 runs. `drive()` feeds those pulses to a SimDog.
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
LIB_PATH = os.path.join(BUILD_DIR, "librobot_core_bridge.so")


def _sources():
    return sorted(glob.glob(os.path.join(CORE_SRC, "*.cpp"))) + [BRIDGE]


def _inputs():
    headers = glob.glob(os.path.join(CORE_SRC, "*.h")) + [
        os.path.join(FW_SRC, "def", "def.h"),
        os.path.join(FW_SRC, "model", "dog", "config.h"),
        os.path.join(FW_SRC, "model", "dog", "model_data.h"),
    ]
    return _sources() + headers


def build(force=False, cxx=None):
    """Compile the bridge if it is missing or older than any source. Returns the library path."""
    if not force and os.path.exists(LIB_PATH):
        built = os.path.getmtime(LIB_PATH)
        if all(os.path.getmtime(f) <= built for f in _inputs()):
            return LIB_PATH
    os.makedirs(BUILD_DIR, exist_ok=True)
    cxx = cxx or os.environ.get("CXX", "c++")
    cmd = [cxx, "-std=gnu++17", "-O2", "-shared", "-fPIC", "-Wall",
           # the ESP32 build treats float literals as float; keep the numbers identical
           "-fsingle-precision-constant",
           "-DROBOT_MODEL=100",   # MODEL_DOG
           "-I", CORE_SRC, "-I", FW_SRC,
           *_sources(), "-o", LIB_PATH + ".tmp"]
    res = subprocess.run(cmd, capture_output=True, text=True)
    if res.returncode != 0:
        raise RuntimeError("robot_core bridge build failed:\n" + " ".join(cmd) + "\n" + res.stderr)
    os.replace(LIB_PATH + ".tmp", LIB_PATH)
    return LIB_PATH


class FirmwareCore:
    """Firmware motion pipeline (one instance per process: the C++ side is global state)."""

    def __init__(self, lib_path=None):
        self.lib = ctypes.CDLL(lib_path or build())
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
        self.legs = L.rc_leg_count()
        self.dof = L.rc_leg_dof()
        self.loop_dt = float(L.rc_loop_dt())
        self.unsolved = 0
        self.reset()

    def reset(self):
        self.lib.rc_init()
        self.unsolved = 0

    def update(self, command=(0, 0, 0, 0, 0, 0), dt=None, imu=None):
        """One control tick. command: move x, y, z, rotate pitch, roll, yaw, each in [-1, 1]
        (y = forward, yaw = turn). imu: (pitch, roll, yaw) rad, used only with HORIZON_LEVEL.
        Returns the servo pulses (us), leg*dof + joint."""
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

    def gait_active(self):
        return bool(self.lib.rc_gait_active())

    def drive(self, dog, command=(0, 0, 0, 0, 0, 0), duration=None):
        """Run the firmware on a SimDog for `duration` seconds (default: one tick)
        at the firmware loop rate, holding `command`. Returns the last pulses."""
        ticks = 1 if duration is None else max(1, int(round(duration / self.loop_dt)))
        pulses = None
        for _ in range(ticks):
            o = dog.body_orientation()
            pulses = self.update(command, imu=(o["pitch"], o["roll"], o["yaw"]))
            dog.write_pulses(pulses)
            dog.step(self.loop_dt)
        return pulses
