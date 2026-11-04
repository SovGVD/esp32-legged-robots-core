"""
Drive the simulated dog by hand with the real firmware motion code.

  python examples/drive.py                                   # the dog, flat floor
  python examples/drive.py --hexapod                         # the hexapod
  python examples/drive.py --terrain bridge                  # ramp, raised deck with gaps; ToF + foot switches on
  python examples/drive.py --hexapod --terrain rubble --sensing tof
  python examples/drive.py --terrain gaps --sensing none     # open loop, like the real robots today
  python examples/drive.py --nav tof                         # two rooms: ToF room scans, click the map to walk there
  python examples/drive.py --hexapod --nav lidar             # the same with the LD06 lidar

--terrain: flat, bumps, rocks, rubble, bars, step, drop, gaps, bridge, course
(legsim/terrain.py, sized for the robot). With a terrain the gait is the slower
"rough" one and the sensing defaults to both (foot switches + ToF map); the
ToF sensors are drawn as purple boxes with orange view cones and their zone
hits as blue dots. The robot stops at a gap it can't step over until you let
go of the stick (or steer away).

--nav tof|lidar: the robot starts in two rooms (legsim.terrain.rooms) with the
firmware's map, localization and walk-to-point. The control window shows the
firmware's map (as the web UI would): click it to walk there, N scans the room
(ToF), X stops; driving with the keys or a stick also cancels the walk.
Green = true path, blue = the map's pose, red = odometry only, orange = plan.

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
ap.add_argument("--nav", choices=("tof", "lidar"),
                help="map + walk to point in two rooms: ToF room scans or the LD06 lidar (click the map)")
ap.add_argument("--goal", nargs=2, type=float, metavar=("X", "Y"), help="with --nav: walk here first, mm")
ap.add_argument("--mount", type=float, metavar="DEG", help="with --nav tof: ToF sensors DEG down (TOF_SENSORS)")
ap.add_argument("--scan-tilt", type=float, metavar="DEG", help="with --nav tof: nose up while scanning")
ap.add_argument("--gyro-bias", type=float, default=1.0, help="with --nav: gyro drift, deg/s (default 1)")
args = ap.parse_args()

model = "hexapod" if args.hexapod else "dog"
rough = args.terrain != "flat" and not args.nav
sensing = args.sensing or ("both" if rough else "none")
fw = FirmwareCore(model=model)
fw.set_leveling(sensing != "none")
fw.set_terrain(**scenario.SENSING[sensing])
gait = args.gait or ("rough" if rough else ("trot" if model == "dog" else "tripod"))
if gait in fw.GAITS:
    fw.select_gait(gait)
nav = None
if args.nav:
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    from lidar_nav import SCALE, draw_map
    k = SCALE[model]
    ground = terrains.rooms(scale=k)
    fw.set_heading_sensors(gyro_bias_dps=args.gyro_bias, magnetometer=False)
    if args.nav == "tof":
        if args.mount is not None:
            for i in range(len(fw.tof_sensors())):
                fw.set_tof_mount(i, args.mount)
        if args.scan_tilt is not None:
            fw.tune_tof_scan(tilt=math.radians(args.scan_tilt))
        fw.set_tof_nav(True, subrays=2, latency=0.04)
    else:
        fw.set_lidar(True)
    PANEL = (440, 570)
    nav = {"view": (-2400 * k, 2600 * k, -1300 * k, 5200 * k), "goal": tuple(args.goal) if args.goal else None,
           "true": [], "map": [], "odom": [], "img": None, "next": 0.0, "ticks": 0}
    if nav["goal"]:
        fw.nav_goal(*nav["goal"])
    elif args.nav == "tof":
        fw.tof_scan()   # map the room first


    def panel_to_map(x, y):
        """Control window map pixel -> map mm (as draw_map places it)."""
        xmin, xmax, ymin, ymax = nav["view"]
        s = min(PANEL[0] / (xmax - xmin), PANEL[1] / (ymax - ymin))
        return xmin + x / s, ymin + (PANEL[1] - y) / s
else:
    ground = scenario.scene(model, args.terrain, args.height)
if model == "hexapod":
    from legsim.hexapod import SimHexapod
    dog = fw.place(SimHexapod(terrain=ground))
    view_distance = 1.1
else:
    dog = fw.place(SimDog(terrain=ground))
    view_distance = 0.7
ui = ManualControl(panel=PANEL if nav else None)
NAV_HELP = ["Map: click = walk there, N scan the room, X stop"] if nav else []

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
        for e in events:
            if nav and isinstance(e, tuple) and e[0] == "click":
                nav["goal"] = panel_to_map(e[1], e[2])
                fw.nav_goal(*nav["goal"])
        if nav and "scan" in events and args.nav == "tof":
            fw.tof_scan()
        if nav and "stop" in events:
            fw.nav_stop()
            nav["goal"] = None
        if "reset" in events:
            fw.reset()
            if nav:
                for key in ("true", "map", "odom"):
                    nav[key].clear()
                nav["goal"] = None
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
                if nav:
                    nav["ticks"] += 1
                    if nav["ticks"] % 10 == 0:
                        p = dog.body_position_mm()
                        mp, op = fw.lidar_pose()
                        for key, q in (("true", p), ("map", mp), ("odom", op)):
                            nav[key].append((q[0], q[1]))
                            del nav[key][:-3000]

        now = time.perf_counter()
        if now >= next_draw:
            next_draw = now + 1 / 30
            viewer.cam.lookat[:] = dog.data.xpos[dog._torso]
            if fw.tof is not None or (nav and fw.lidar is not None):
                viewer.user_scn.ngeom = 0
                (fw.tof if fw.tof is not None else fw.lidar).draw(dog, viewer.user_scn)
            viewer.sync()
            o = dog.body_orientation()
            p = dog.body_position_mm()
            state = fw.terrain_state() if sensing != "none" else None
            nav_lines = []
            if nav:
                ns = fw.nav_state()
                mp, op = fw.lidar_pose()
                ls = fw.lidar_state()
                yaw = o["yaw"]

                def err(q):
                    return (math.hypot(q[0] - p[0], q[1] - p[1]),
                            math.degrees(abs((q[2] - yaw + math.pi) % (2 * math.pi) - math.pi)))
                phase = f"   scan: {ns['phase']}" if ns["phase"] != "idle" else ""
                nav_lines = [
                    f"nav ({args.nav}): {ns['state']}{phase}" + ("   blocked ahead" if ns["blocked"] else ""),
                    f"map pose off {err(mp)[0]:4.0f} mm {err(mp)[1]:4.1f} deg   scans {ls['scans']} "
                    f"(rejected {ls['rejected']})",
                    f"odometry only off {err(op)[0]:5.0f} mm {err(op)[1]:5.1f} deg",
                ]
                if now >= nav["next"]:   # the map picture a few times a second
                    nav["next"] = now + 0.25
                    grid, cell = fw.lidar_map()
                    nav["img"] = draw_map(grid, cell, PANEL, nav["view"], nav["true"], nav["map"], nav["odom"],
                                          fw.nav_path() if ns["state"] == "active" else None,
                                          nav["goal"] if ns["state"] == "active" else None, mp, fw.lidar_points())
            ui.draw(help_lines=NAV_HELP, panel=nav["img"] if nav else None, status_lines=[
                ("PAUSED" if paused else f"sim time {dog.data.time:6.1f} s") + f"   gait: {gait}",
                f"terrain: {ground.name}   sensing: {sensing}"
                + ("   GAP AHEAD: stopped" if state and state["blocked"] else ""),
                f"body x {p[0]:7.0f}  y {p[1]:7.0f}  z {p[2]:5.0f} mm",
                f"pitch {math.degrees(o['pitch']):+5.1f}  roll {math.degrees(o['roll']):+5.1f}  "
                f"yaw {math.degrees(o['yaw']):+6.1f} deg",
                *nav_lines,
            ])
        time.sleep(0.001)

# SDL (pygame) and GLFW (MuJoCo viewer) share the X11 connection, and their
# atexit teardown crashes or hangs on Linux. Nothing is left to save, so leave
# without running it.
sys.stdout.flush()
os._exit(0)
