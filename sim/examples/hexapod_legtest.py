"""
Check that every hexapod servo turns the way the firmware expects.

  python examples/hexapod_legtest.py                 # all legs, prints a table
  python examples/hexapod_legtest.py --leg RF        # one leg
  python examples/hexapod_legtest.py --video legtest.mp4
  python examples/hexapod_legtest.py --view          # watch it in the MuJoCo viewer (needs a display)

The robot hangs in the air. One leg at a time moves its foot up, out, forward
and turns around its coxa, through the firmware's own IK and servo HAL (the
inverse flags in model/hexapod/model_data.h), while the sim servos turn the
way they are mounted on the real robot. A leg that moves the wrong way is
reported with the joint and the inverse flag to flip. Edit model_data.h and
run again: the firmware bridge rebuilds by itself.
"""

import argparse
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

import numpy as np
from legsim import FirmwareCore
from legsim.hexapod import SimHexapod, legtest

ap = argparse.ArgumentParser()
ap.add_argument("--leg", action="append", help="LF, LM, LR, RF, RM or RR (repeat for several); default all")
ap.add_argument("--video")
ap.add_argument("--view", action="store_true")
args = ap.parse_args()

fw = FirmwareCore(model="hexapod")
robot = SimHexapod(fixed_base=True)
on_frame = None
frames = []

if args.video:
    import mujoco
    from PIL import Image, ImageDraw, ImageFont
    renderer = mujoco.Renderer(robot.model, 544, 960)
    cam = mujoco.MjvCamera()
    cam.distance, cam.elevation, cam.azimuth = 0.6, -30, 235
    cam.lookat[:] = robot.data.xpos[robot._torso] + np.array([0, 0, -0.05])
    opt = mujoco.MjvOption()
    try:
        font = ImageFont.truetype("DejaVuSans-Bold.ttf", 30)
    except OSError:
        font = ImageFont.load_default()

    def on_frame(r, label):
        renderer.update_scene(r.data, cam, scene_option=opt)
        img = Image.fromarray(renderer.render())
        d = ImageDraw.Draw(img)
        color = (200, 0, 0) if label.endswith("WRONG") else (0, 120, 0) if label.endswith("OK") else (20, 20, 20)
        d.text((20, 16), label, fill=color, font=font)
        d.text((20, 510), "front = +Y (camera board), view from front-right", fill=(60, 60, 60), font=font.font_variant(size=18) if hasattr(font, "font_variant") else font)
        frames.append(np.asarray(img))

if args.view:
    import time
    from legsim import display
    display.prepare()
    import mujoco.viewer
    viewer = mujoco.viewer.launch_passive(robot.model, robot.data)
    viewer.cam.distance, viewer.cam.elevation, viewer.cam.azimuth = 0.6, -30, 235
    wall0, sim0 = time.perf_counter(), robot.data.time

    def on_frame(r, label):
        viewer.sync()
        lag = (r.data.time - sim0) - (time.perf_counter() - wall0)
        if lag > 0:
            time.sleep(lag)

results = legtest.run(fw, robot, legs=args.leg, on_frame=on_frame)
print("inverse flags (alpha, beta, gamma) in model_data.h:")
for i, leg in enumerate(robot.p.legs):
    print(f"  {leg.name}: {fw.leg_inverse(i)}")
print()
print(legtest.summary(results))

if frames:
    try:
        import mediapy
        mediapy.write_video(args.video, frames, fps=30)
    except ImportError:
        import imageio
        imageio.mimsave(args.video, frames, fps=30)
    print("wrote", args.video)
