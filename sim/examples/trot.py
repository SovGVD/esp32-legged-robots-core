"""
Open-loop trot demo.

  python examples/trot.py                 # print a summary
  python examples/trot.py --view          # interactive MuJoCo viewer (needs a display)
  python examples/trot.py --video out.mp4 # render offscreen (needs mediapy or imageio)
"""

import argparse
import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

import numpy as np
from legsim import SimDog
from legsim.demo_gait import foot_targets

ap = argparse.ArgumentParser()
ap.add_argument("--speed", type=float, default=100.0, help="mm/s")
ap.add_argument("--time", type=float, default=5.0)
ap.add_argument("--view", action="store_true")
ap.add_argument("--video")
args = ap.parse_args()

dog = SimDog()
dog.step(0.5)
CONTROL_DT = 0.0044   # firmware LOOP_TIME


def control(t):
    dog.set_feet(foot_targets(dog.p, t, speed_mm_s=args.speed))


if args.view:
    import mujoco.viewer
    with mujoco.viewer.launch_passive(dog.model, dog.data) as v:
        import time
        t0 = time.time()
        t = 0.0
        while v.is_running():
            control(t)
            dog.step(CONTROL_DT)
            t += CONTROL_DT
            v.sync()
            time.sleep(max(0.0, t - (time.time() - t0)))
    sys.exit()

frames = []
renderer = None
if args.video:
    import mujoco
    renderer = mujoco.Renderer(dog.model, 480, 640)
    cam = mujoco.MjvCamera()
    cam.distance, cam.elevation, cam.azimuth = 0.6, -20, 135

t = 0.0
next_frame = 0.0
while t < args.time:
    control(t)
    dog.step(CONTROL_DT)
    t += CONTROL_DT
    if renderer and t >= next_frame:
        cam.lookat[:] = dog.data.xpos[dog._torso]
        renderer.update_scene(dog.data, cam)
        frames.append(renderer.render())
        next_frame += 1 / 30

pos = dog.body_position_mm()
ori = {k: round(math.degrees(v), 1) for k, v in dog.body_orientation().items()}
print(f"after {args.time:.1f}s at {args.speed:.0f} mm/s: body at {np.round(pos, 1)} mm, orientation {ori} deg")

if frames:
    try:
        import mediapy
        mediapy.write_video(args.video, frames, fps=30)
    except ImportError:
        import imageio
        imageio.mimsave(args.video, frames, fps=30)
    print("wrote", args.video)
