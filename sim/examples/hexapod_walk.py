"""
The hexapod walking with the real firmware gait (lib/robot_core, MODEL_HEXAPOD).

  python examples/hexapod_walk.py                    # print a summary
  python examples/hexapod_walk.py --video walk.mp4   # render offscreen (needs mediapy or imageio)
  MUJOCO_GL=glfw xvfb-run -a python examples/hexapod_walk.py --video walk.mp4   # headless

Drives a fixed script: stand, forward, strafe right, turn right, back, stop,
and prints distance, tilt, body height and how hard the servos work.
"""

import argparse
import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

import numpy as np
from legsim import FirmwareCore
from legsim.hexapod import SimHexapod

ap = argparse.ArgumentParser()
ap.add_argument("--video")
args = ap.parse_args()

SCRIPT = [
    ("stand", (0, 0, 0, 0, 0, 0), 1.0),
    ("forward", (0, 1, 0, 0, 0, 0), 4.0),
    ("strafe right", (1, 0, 0, 0, 0, 0), 3.0),
    ("turn right", (0, 0, 0, 0, 0, 1), 3.0),
    ("backward", (0, -1, 0, 0, 0, 0), 3.0),
    ("stop", (0, 0, 0, 0, 0, 0), 1.5),
]

fw = FirmwareCore(model="hexapod")
robot = fw.place(SimHexapod())
robot.step(0.3)

frames, renderer = [], None
if args.video:
    import mujoco
    renderer = mujoco.Renderer(robot.model, 544, 960)
    cam = mujoco.MjvCamera()
    cam.distance, cam.elevation, cam.azimuth = 0.6, -25, 135
    opt = mujoco.MjvOption()

next_frame = robot.data.time
for name, cmd, duration in SCRIPT:
    p0 = robot.body_position_mm().copy()
    yaw0 = robot.body_orientation()["yaw"]
    tilt, z_min = 0.0, 1e9
    torque = []
    for _ in range(int(round(duration / fw.loop_dt))):
        fw.drive(robot, cmd)
        o = robot.body_orientation()
        tilt = max(tilt, abs(o["pitch"]), abs(o["roll"]))
        z_min = min(z_min, robot.body_position_mm()[2])
        torque.append(robot.joint_torque_ratio().reshape(6, 3))
        if renderer and robot.data.time >= next_frame:
            cam.lookat[:] = robot.data.xpos[robot._torso]
            renderer.update_scene(robot.data, cam, scene_option=opt)
            frames.append(renderer.render())
            next_frame += 1 / 30
    d = robot.body_position_mm() - p0
    c, s = math.cos(-yaw0), math.sin(-yaw0)
    d = np.array([c * d[0] - s * d[1], s * d[0] + c * d[1], d[2]])   # in the body frame at the start
    turned = math.degrees(robot.body_orientation()["yaw"] - yaw0)
    t = np.array(torque)
    sat = (t > 0.95).mean(axis=(0, 1))
    print(f"{name:13s} moved right {d[0]:+6.0f} forward {d[1]:+6.0f} mm, turned {turned:+6.1f} deg, "
          f"max tilt {math.degrees(tilt):4.1f} deg, lowest body {z_min:5.1f} mm, "
          f"servos at stall (coxa/femur/tibia) {sat[0]:4.0%} {sat[1]:4.0%} {sat[2]:4.0%}")
print("rejected leg poses:", fw.reject_counts(), " unsolved:", fw.unsolved)

if frames:
    try:
        import mediapy
        mediapy.write_video(args.video, frames, fps=30)
    except ImportError:
        import imageio
        imageio.mimsave(args.video, frames, fps=30)
    print("wrote", args.video)
