"""
Walk the dog or the hexapod over uneven ground and obstacles with the
firmware's terrain code, with or without foot switches and ToF sensors.

  python examples/terrain_walk.py                              # dog, obstacle course, foot switches + ToF
  python examples/terrain_walk.py --hexapod --terrain bumps --sensing tof
  python examples/terrain_walk.py --compare                    # every sensing mode on every scene, a table
  python examples/terrain_walk.py --hexapod --video walk.mp4   # render: ToF sensors (boxes), view cones, zone hits (dots)
  MUJOCO_GL=glfw xvfb-run -a python examples/terrain_walk.py --video walk.mp4   # headless

Scenes: flat, bumps, rocks, rubble, bars, step, drop, course (legsim/terrain.py), sized
for the robot (the hexapod gets 1.5x higher obstacles). Sensing: none (open loop,
like the real robots today), contacts (foot switches), tof (ToF terrain map),
both.
"""

import argparse
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))

from legsim import scenario, terrain  # noqa: E402

ap = argparse.ArgumentParser()
ap.add_argument("--hexapod", action="store_true")
ap.add_argument("--terrain", default="course", choices=sorted(terrain.SCENES))
ap.add_argument("--height", type=float, help="obstacle / bump height, mm (default: per robot)")
ap.add_argument("--sensing", default="both", choices=sorted(scenario.SENSING))
ap.add_argument("--gait", default=None, help="default, rough (slower, higher steps; the default here), walk (dog)")
ap.add_argument("--speed", type=float, default=None, help="move command 0..1")
ap.add_argument("--duration", type=float, default=None, help="s")
ap.add_argument("--no-level", action="store_true", help="IMU leveling off (on by default with sensing)")
ap.add_argument("--compare", action="store_true", help="table of every scene x sensing mode")
ap.add_argument("--video", help="render to mp4: ToF sensors as boxes with their view cone (orange) and zone hits (blue dots)")
args = ap.parse_args()

model = "hexapod" if args.hexapod else "dog"
defaults = scenario.DEFAULTS[model]
gait = args.gait or defaults["gait"]
speed = args.speed if args.speed is not None else defaults["speed"]


def scene(name):
    return scenario.scene(model, name, args.height)


if args.compare:
    for name in ("flat", "bumps", "rocks", "bars", "step", "drop"):
        for sensing in ("none", "contacts", "tof", "both"):
            r = scenario.run(model, scene(name), sensing, speed=speed, gait=gait,
                             duration=args.duration, leveling=False if args.no_level else None)
            print(r.summary(), flush=True)
    sys.exit(0)

frames = []
cb = None
if args.video:
    import mujoco
    state = {}

    def cb(robot, fw, t):
        if "renderer" not in state:
            state["renderer"] = mujoco.Renderer(robot.model, 540, 960)
            cam = mujoco.MjvCamera()
            cam.distance, cam.elevation, cam.azimuth = (0.55, -22, 120) if model == "dog" else (0.8, -25, 125)
            state["cam"] = cam
        r, cam = state["renderer"], state["cam"]
        cam.lookat[:] = robot.data.xpos[robot._torso]
        r.update_scene(robot.data, cam)
        if fw.tof is not None:
            fw.tof.draw(robot, r.scene)   # sensor boxes, view cones (orange), zone hits (blue dots)
        frames.append(r.render())

res = scenario.run(model, scene(args.terrain), args.sensing, speed=speed, gait=gait,
                   duration=args.duration, leveling=False if args.no_level else None, frame_cb=cb)
print(res.summary())

if frames:
    try:
        import mediapy
        mediapy.write_video(args.video, frames, fps=30)
    except ImportError:
        import imageio
        imageio.mimsave(args.video, frames, fps=30)
    print("wrote", args.video)
