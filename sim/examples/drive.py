"""
Drive the simulated dog by hand with the real firmware motion code.

  python examples/drive.py

Opens the MuJoCo viewer and a small "robot control" window. Keyboard and
Xbox 360 gamepad controls are listed in that window and in legsim/control.py.
Needs a display, a C++ compiler (the firmware is built on first run) and
`pip install pygame-ce`. On macOS the MuJoCo viewer needs `mjpython`, which runs
the script off the main thread; pygame windows need the main thread there, so
this example is for Linux and Windows.
"""

import math
import os
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

import mujoco
import mujoco.viewer
from legsim import SimDog, FirmwareCore
from legsim.control import ManualControl

fw = FirmwareCore()
dog = fw.place(SimDog())
ui = ManualControl()

dt = fw.loop_dt
paused = False
gait = "trot"

with mujoco.viewer.launch_passive(dog.model, dog.data) as viewer:
    viewer.cam.distance, viewer.cam.elevation, viewer.cam.azimuth = 0.7, -20, 135
    wall_start = time.perf_counter()
    sim_start = dog.data.time
    next_draw = 0.0
    while viewer.is_running():
        cmd, events = ui.poll()
        if "quit" in events:
            break
        if "reset" in events:
            fw.reset()
            fw.place(dog)
            viewer.sync()
            wall_start, sim_start = time.perf_counter(), dog.data.time
        if "gait" in events and not fw.gait_active():
            # only while standing: no foot is in the air
            gait = "walk" if gait == "trot" else "trot"
            fw.select_gait(gait)
        if "pause" in events:
            paused = not paused
            wall_start, sim_start = time.perf_counter(), dog.data.time

        if not paused:
            # keep the sim in step with the wall clock, one firmware tick at a time
            while dog.data.time - sim_start < time.perf_counter() - wall_start:
                fw.drive(dog, cmd.as_tuple())

        now = time.perf_counter()
        if now >= next_draw:
            next_draw = now + 1 / 30
            viewer.cam.lookat[:] = dog.data.xpos[dog._torso]
            viewer.sync()
            o = dog.body_orientation()
            p = dog.body_position_mm()
            ui.draw([
                ("PAUSED" if paused else f"sim time {dog.data.time:6.1f} s") + f"   gait: {gait}",
                f"body x {p[0]:7.0f}  y {p[1]:7.0f}  z {p[2]:5.0f} mm",
                f"pitch {math.degrees(o['pitch']):+5.1f}  roll {math.degrees(o['roll']):+5.1f}  "
                f"yaw {math.degrees(o['yaw']):+6.1f} deg",
            ])
        time.sleep(0.001)

# SDL (pygame) and GLFW (MuJoCo viewer) share the X11 connection, and their
# atexit teardown crashes or hangs on Linux. Nothing is left to save, so leave
# without running it.
sys.stdout.flush()
os._exit(0)
