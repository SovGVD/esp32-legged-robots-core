"""
Drive the simulated dog by hand with the real firmware motion code.

  python examples/drive.py                                   # the dog, flat floor
  python examples/drive.py --hexapod                         # the hexapod
  python examples/drive.py --terrain bridge                  # ramp, raised deck with gaps; ToF + foot switches on
  python examples/drive.py --hexapod --terrain rubble --sensing tof
  python examples/drive.py --terrain gaps --sensing none     # open loop, like the real robots today

--terrain: flat, bumps, rocks, rubble, bars, step, drop, gaps, bridge, course
(legsim/terrain.py, sized for the robot). With a terrain the gait is the slower
"rough" one and the sensing defaults to both (foot switches + ToF map); the
ToF sensors are drawn as purple boxes with orange view cones and their zone
hits as blue dots. The robot stops at a gap it can't step over until you let
go of the stick (or steer away).

Opens the MuJoCo viewer and a small "robot control" window. Keyboard and
Xbox 360 gamepad controls are listed in that window and in legsim/control.py.
Needs a display, a C++ compiler (the firmware is built on first run) and
`pip install pygame-ce`. On macOS the MuJoCo viewer needs `mjpython`, which runs
the script off the main thread; pygame windows need the main thread there, so
this example is for Linux and Windows.
"""

import argparse
import math
import os
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

import mujoco
import mujoco.viewer
from legsim import SimDog, FirmwareCore
from legsim.control import ManualControl

from legsim import scenario, terrain as terrains  # noqa: E402

ap = argparse.ArgumentParser()
ap.add_argument("--hexapod", action="store_true")
ap.add_argument("--terrain", default="flat", choices=sorted(terrains.SCENES))
ap.add_argument("--height", type=float, help="obstacle / deck height, mm (default: per robot)")
ap.add_argument("--sensing", choices=sorted(scenario.SENSING),
                help="none (open loop), contacts (foot switches), tof, both (default with a terrain)")
ap.add_argument("--gait", help="default, rough (default with a terrain), walk (dog)")
args = ap.parse_args()

model = "hexapod" if args.hexapod else "dog"
rough = args.terrain != "flat"
sensing = args.sensing or ("both" if rough else "none")
fw = FirmwareCore(model=model)
fw.set_leveling(sensing != "none")
fw.set_terrain(**scenario.SENSING[sensing])
gait = args.gait or ("rough" if rough else ("trot" if model == "dog" else "tripod"))
if gait in fw.GAITS:
    fw.select_gait(gait)
ground = scenario.scene(model, args.terrain, args.height)
if model == "hexapod":
    from legsim.hexapod import SimHexapod
    dog = fw.place(SimHexapod(terrain=ground))
    view_distance = 1.1
else:
    dog = fw.place(SimDog(terrain=ground))
    view_distance = 0.7
ui = ManualControl()

dt = fw.loop_dt
paused = False

with mujoco.viewer.launch_passive(dog.model, dog.data) as viewer:
    viewer.cam.distance, viewer.cam.elevation, viewer.cam.azimuth = view_distance, -20, 135
    wall_start = time.perf_counter()
    sim_start = dog.data.time
    next_draw = 0.0
    while viewer.is_running():
        cmd, events = ui.poll()
        if "quit" in events:
            break
        if "reset" in events:
            fw.reset()
            if gait in fw.GAITS:
                fw.select_gait(gait)
            fw.place(dog)
            viewer.sync()
            wall_start, sim_start = time.perf_counter(), dog.data.time
        if "gait" in events and not fw.gait_active():
            # only while standing: no foot is in the air
            cycle = ["trot", "walk", "rough"] if fw.model == "dog" else ["tripod", "rough"]
            gait = cycle[(cycle.index(gait) + 1) % len(cycle)] if gait in cycle else cycle[0]
            fw.select_gait(gait if gait in fw.GAITS else "default")
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
            if fw.tof is not None:
                viewer.user_scn.ngeom = 0
                fw.tof.draw(dog, viewer.user_scn)
            viewer.sync()
            o = dog.body_orientation()
            p = dog.body_position_mm()
            state = fw.terrain_state() if sensing != "none" else None
            ui.draw([
                ("PAUSED" if paused else f"sim time {dog.data.time:6.1f} s") + f"   gait: {gait}",
                f"terrain: {ground.name}   sensing: {sensing}"
                + ("   GAP AHEAD: stopped" if state and state["blocked"] else ""),
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
