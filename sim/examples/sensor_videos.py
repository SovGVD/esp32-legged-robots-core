"""
Videos of the firmware's heading hold, `turnto` and the gap stop in the sim
(gyro with 1 deg/s drift + magnetometer through the firmware HeadingEstimator).

  MUJOCO_GL=glfw xvfb-run -a python examples/sensor_videos.py hold-off out/
  ... hold-on | turnto | gap   (one per process: each run opens its own renderer)
"""
import math, os, sys
import numpy as np
import mujoco
from PIL import Image, ImageDraw, ImageFont
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
from legsim import SimDog, scenario
from legsim.firmware import FirmwareCore
from legsim.hexapod import SimHexapod
import mediapy

W, H = 960, 540
try:
    FONT = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf", 26)
    SMALL = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf", 20)
except OSError:
    FONT = SMALL = ImageFont.load_default()


def overlay(img, lines, warn=None):
    im = Image.fromarray(img)
    d = ImageDraw.Draw(im)
    y = 14
    for i, t in enumerate(lines):
        f = FONT if i == 0 else SMALL
        d.text((18, y), t, font=f, fill=(255, 255, 255), stroke_width=3, stroke_fill=(0, 0, 0))
        y += 34 if i == 0 else 28
    if warn:
        d.text((W // 2 - 170, H - 70), warn, font=FONT, fill=(255, 80, 60), stroke_width=3, stroke_fill=(0, 0, 0))
    return np.asarray(im)


def add_geom(scn, gtype, size, pos, mat, rgba):
    if scn.ngeom >= scn.maxgeom:
        return
    g = scn.geoms[scn.ngeom]
    mujoco.mjv_initGeom(g, gtype, np.array(size, float), np.array(pos, float), np.array(mat, float).flatten(), np.array(rgba, np.float32))
    scn.ngeom += 1


def arrow(scn, start, heading_deg, length, rgba, z=0.003):
    # compass heading: 0 = +Y (north), clockwise
    a = math.radians(heading_deg)
    d = np.array([math.sin(a), math.cos(a), 0.0])
    g_from = np.array([start[0], start[1], z])
    g_to = g_from + d * length
    if scn.ngeom >= scn.maxgeom:
        return
    g = scn.geoms[scn.ngeom]
    mujoco.mjv_initGeom(g, mujoco.mjtGeom.mjGEOM_ARROW, np.zeros(3), np.zeros(3), np.eye(3).flatten(), np.array(rgba, np.float32))
    mujoco.mjv_connector(g, mujoco.mjtGeom.mjGEOM_ARROW, 0.006, g_from, g_to)
    scn.ngeom += 1


class Recorder:
    def __init__(self, robot, dist, elev, azim):
        self.r = mujoco.Renderer(robot.model, H, W)
        self.cam = mujoco.MjvCamera()
        self.cam.distance, self.cam.elevation, self.cam.azimuth = dist, elev, azim
        self.frames = []
        self.trail = []

    def frame(self, robot, lines, warn=None, extra=None, follow=True):
        p = robot.data.xpos[robot._torso].copy()
        if follow:
            self.cam.lookat[:] = p
        self.trail.append(p)
        self.r.update_scene(robot.data, self.cam)
        scn = self.r.scene
        for q in self.trail[::3]:
            add_geom(scn, mujoco.mjtGeom.mjGEOM_SPHERE, [0.004, 0, 0], [q[0], q[1], 0.002], np.eye(3), [1, 0.85, 0.1, 1])
        if extra:
            extra(scn, p)
        self.frames.append(overlay(self.r.render(), lines, warn))


def kick_video(path, holds):
    """Walking north; a push turns the robot ~60 deg. Hold off: it walks off that way. Hold on: it turns back."""
    clips = []
    for hold in holds:
        fw = FirmwareCore(model="dog")
        fw.reset()
        fw.select_gait("trot")
        fw.set_heading_sensors(gyro_bias_dps=1.0)
        dog = fw.place(SimDog())
        rec = Recorder(dog, 1.1, -55, 90)
        rec.cam.lookat[:] = [0, 0.5, 0]
        fw.drive(dog, duration=0.5)
        if hold:
            fw.heading_hold(True)
        t = 0.0
        dt = fw.loop_dt
        next_frame = 0.0
        while t < 9.0:
            torso = dog._torso
            dog.data.xfrc_applied[torso, 5] = -0.12 if 2.5 <= t < 4.0 else 0.0   # yaw torque: a push, clockwise
            fw.drive(dog, (0, 1, 0, 0, 0, 0))
            t += dt
            if t >= next_frame:
                next_frame += 1 / 30
                s = fw.heading_state()
                true_h = (-math.degrees(dog.body_orientation()["yaw"])) % 360
                lines = [("Heading hold ON" if hold else "Heading hold OFF") + "  (dog, trot, stick: forward)",
                         f"heading {s['heading']:5.1f} deg (gyro + magnetometer)   true {true_h:5.1f}",
                         ("hold target %.0f deg" % s["target"]) if hold else "no target: keeps going where it points"]
                warn = "push" if 2.5 <= t < 4.0 else None

                def extra(scn, p, s=s, hold=hold):
                    arrow(scn, p, s["heading"], 0.12, [0.2, 0.5, 1, 1], z=0.006)
                    if hold:
                        arrow(scn, p, s["target"], 0.15, [0.1, 0.9, 0.2, 1], z=0.004)
                rec.frame(dog, lines, warn, extra, follow=False)
        clips += rec.frames
        dog.data.xfrc_applied[:] = 0
    mediapy.write_video(path, clips, fps=30)
    print("wrote", path, len(clips))


def turnto_video(path):
    """Standing facing north: turnto 90, walk; turnto 225, walk; turnto 0, walk."""
    fw = FirmwareCore(model="dog")
    fw.reset()
    fw.select_gait("trot")
    fw.set_heading_sensors(gyro_bias_dps=1.0)
    dog = fw.place(SimDog())
    rec = Recorder(dog, 1.3, -60, 90)
    fw.drive(dog, duration=0.5)
    plan = [(0.0, None, (0, 0, 0, 0, 0, 0), "standing, facing north"),
            (1.0, 90, (0, 0, 0, 0, 0, 0), "turnto 90"),
            (4.5, None, (0, 1, 0, 0, 0, 0), "walk forward (stick), hold 90"),
            (8.0, 225, (0, 0, 0, 0, 0, 0), "turnto 225"),
            (11.5, None, (0, 1, 0, 0, 0, 0), "walk forward, hold 225"),
            (15.0, 0, (0, 1, 0, 0, 0, 0), "turnto 0 while walking"),
            (19.0, None, (0, 0, 0, 0, 0, 0), "let go")]
    t, dt, nf = 0.0, fw.loop_dt, 0.0
    step = 0
    cmd, label = plan[0][2], plan[0][3]
    while t < 20.5:
        while step < len(plan) and t >= plan[step][0]:
            _, target, cmd, label = plan[step]
            if target is not None:
                fw.turn_to(target)
            step += 1
        fw.drive(dog, cmd)
        t += dt
        if t >= nf:
            nf += 1 / 30
            s = fw.heading_state()
            lines = [f"> {label}", f"heading {s['heading']:5.1f} deg",
                     (f"target {s['target']:.0f} deg" if s["hold"] else "hold off")]

            def extra(scn, p, s=s):
                arrow(scn, p, s["heading"], 0.12, [0.2, 0.5, 1, 1], z=0.006)
                if s["hold"]:
                    arrow(scn, p, s["target"], 0.15, [0.1, 0.9, 0.2, 1], z=0.004)
                arrow(scn, [p[0] - 0.25, p[1] - 0.2], 0, 0.08, [0.9, 0.2, 0.2, 1])   # north
            rec.frame(dog, lines, None, extra)
    mediapy.write_video(path, rec.frames, fps=30)
    print("wrote", path, len(rec.frames))


def gap_video(path, model="hexapod", seed=1):
    d = scenario.DEFAULTS[model]
    t_scene = scenario.scene(model, "gaps")
    fw = scenario.core(model)
    fw.reset()
    rec = {}
    frames = []

    def cb(robot, fw, tt):
        if "r" not in rec:
            rec["r"] = Recorder(robot, 0.85, -25, 125)
        st = fw.terrain_state()
        y = robot.body_position_mm()[1]
        lines = [f"terrain on: ToF map, rough gait ({model})", f"t {tt:4.1f} s   walked {y:4.0f} mm",
                 "gaps 40, 80, 220 mm in a 70 mm high floor"]
        r = rec["r"]
        r.r.update_scene(robot.data, r.cam)
        r.cam.lookat[:] = robot.data.xpos[robot._torso]
        r.r.update_scene(robot.data, r.cam)
        fw.tof.draw(robot, r.r.scene)
        frames.append(overlay(r.r.render(), lines, "GAP AHEAD: stopped" if st["blocked"] else None))
    scenario.run(model, t_scene, "tof", gait=d["gait"], speed=d["speed"], duration=12, seed=seed, frame_cb=cb)
    mediapy.write_video(path, frames, fps=30)
    print("wrote", path, len(frames))


if __name__ == "__main__":
    out = sys.argv[2] if len(sys.argv) > 2 else "./"
    which = sys.argv[1]
    if which == "hold-off":
        kick_video(out + "heading-hold-off.mp4", (False,))
    elif which == "hold-on":
        kick_video(out + "heading-hold-on.mp4", (True,))
    elif which == "turnto":
        turnto_video(out + "dog-turnto.mp4")
    elif which == "gap":
        gap_video(out + "hexapod-gap-stop.mp4")
