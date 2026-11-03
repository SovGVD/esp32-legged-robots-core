"""
Render still images of the robot standing in its neutral pose.

  python examples/snapshot.py out.png            # 2x2 views
  python examples/snapshot.py out.png --hexapod  # the hexapod
  MUJOCO_GL=glfw xvfb-run -a python examples/snapshot.py out.png   # on a headless machine
"""
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

import numpy as np
import mujoco
from legsim import SimDog

args = [a for a in sys.argv[1:] if not a.startswith("--")]
out = args[0] if args else "snapshot.png"
scale = 1.0
if "--hexapod" in sys.argv:
    from legsim.hexapod import SimHexapod
    dog = SimHexapod()
    scale = 1.7
else:
    dog = SimDog()
dog.step(0.3)
r = mujoco.Renderer(dog.model, 480, 640)
cam = mujoco.MjvCamera()
opt = mujoco.MjvOption()
opt.geomgroup[:] = [1, 1, 1, 0, 0, 0]   # hide collision shapes (group 3)
tiles = []
for az, el, dist in ((135, -20, 0.42), (90, 0, 0.38), (180, 0, 0.38), (60, -55, 0.45)):
    cam.azimuth, cam.elevation, cam.distance = az, el, dist * scale
    cam.lookat[:] = dog.data.xpos[dog._torso] + np.array([0, 0, -0.02])
    r.update_scene(dog.data, cam, scene_option=opt)
    tiles.append(r.render())
img = np.vstack([np.hstack(tiles[:2]), np.hstack(tiles[2:])])
try:
    import imageio.v3 as iio
    iio.imwrite(out, img)
except ImportError:
    from PIL import Image
    Image.fromarray(img).save(out)
print("wrote", out)
