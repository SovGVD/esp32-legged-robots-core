"""
Indoor mapping and walking to a point with the LD06 lidar, in the sim.

The firmware's own code does everything: the LD06 driver parses the
simulated lidar's UART bytes, LidarSlam builds the map and corrects the
odometry (gait velocity + a gyro with a constant drift, no magnetometer:
indoors it is thrown off by steel and wiring), Navigator plans a path and
writes the move command. The robot starts in the lower of two rooms
(legsim.terrain.rooms) and walks to each goal in turn.

  python examples/lidar_nav.py                       # dog, default goals, prints the result
  python examples/lidar_nav.py --model hexapod
  python examples/lidar_nav.py --goal -600 3600 --goal 0 0 --png out/map.png
  MUJOCO_GL=egl python examples/lidar_nav.py --video out/dog-lidar.mp4
  python examples/lidar_nav.py --tof                             # no lidar: ToF room scans
  MUJOCO_GL=egl python examples/lidar_nav.py --tof --video out/dog-tof.mp4 --speedup 6

Map frame = the robot's start pose = the sim's world frame (the robots start
at the origin facing +Y), so the map pose can be compared with the truth.
"""
import argparse
import math
import os
import sys

import numpy as np
from PIL import Image, ImageDraw, ImageFont

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
from legsim import scenario, terrain  # noqa: E402
from legsim.firmware import FirmwareCore  # noqa: E402

GOALS = {
    "dog": [(-700, 3000), (1100, 2900), (300, 0)],
    "hexapod": [(-900, 3900), (1300, 3700), (500, 0)],
}
SCALE = {"dog": 1.0, "hexapod": 1.3}

try:
    FONT = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf", 22)
    SMALL = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf", 17)
except OSError:
    FONT = SMALL = ImageFont.load_default()


def draw_map(grid, cell, size, view, true_path, map_path, odom_path, plan, goal, pose, points, title=None):
    """The firmware map as the web UI would show it, plus the true / lidar / odometry tracks.
    view: (xmin, xmax, ymin, ymax) mm."""
    n = grid.shape[0]
    xmin, xmax, ymin, ymax = view
    W, H = size
    scale = min(W / (xmax - xmin), H / (ymax - ymin))

    def px(x, y):
        return ((x - xmin) * scale, H - (y - ymin) * scale)

    # cells -> image (map y up)
    img = np.full((n, n, 3), 205, np.uint8)
    img[grid < 0] = (250, 250, 250)
    occ = np.clip(grid.astype(int), 0, 40) / 40.0
    mask = grid > 0
    img[mask] = (np.stack([1 - occ[mask]] * 3, axis=1) * 150).astype(np.uint8)
    half = n / 2 * cell
    im = Image.fromarray(img[::-1])
    x0, y0 = px(-half, half)
    x1, y1 = px(half, -half)
    im = im.resize((max(1, int(round(x1 - x0))), max(1, int(round(y1 - y0)))), Image.NEAREST)
    out = Image.new("RGB", size, (205, 205, 205))
    out.paste(im, (int(round(x0)), int(round(y0))))
    d = ImageDraw.Draw(out)

    def line(path, color, width):
        if len(path) > 1:
            d.line([px(p[0], p[1]) for p in path], fill=color, width=width)

    line(odom_path, (220, 60, 50), 2)
    line(true_path, (40, 170, 60), 3)
    line(map_path, (40, 90, 220), 2)
    if plan is not None and len(plan) > 1:
        line(plan, (250, 160, 0), 2)
    if pose is not None:
        s, c = math.sin(pose[2]), math.cos(pose[2])
        for bx, by in points[::2]:
            x, y = pose[0] + bx * c - by * s, pose[1] + bx * s + by * c
            q = px(x, y)
            d.ellipse([q[0] - 1.5, q[1] - 1.5, q[0] + 1.5, q[1] + 1.5], fill=(240, 30, 30))
        q = px(pose[0], pose[1])
        tip = px(pose[0] - 200 * s, pose[1] + 200 * c)
        d.ellipse([q[0] - 6, q[1] - 6, q[0] + 6, q[1] + 6], outline=(20, 60, 200), width=3)
        d.line([q, tip], fill=(20, 60, 200), width=3)
    if goal is not None:
        q = px(*goal)
        d.line([q[0] - 8, q[1] - 8, q[0] + 8, q[1] + 8], fill=(200, 0, 160), width=3)
        d.line([q[0] - 8, q[1] + 8, q[0] + 8, q[1] - 8], fill=(200, 0, 160), width=3)
    y = 6
    if title:
        d.text((8, y), title, font=FONT, fill=(0, 0, 0))
        y += 28
    d.rectangle([4, y - 2, 190, y + 4 * 21 + 2], fill=(255, 255, 255))
    for label, color in (("true path (sim)", (40, 170, 60)), ("lidar map pose", (40, 90, 220)),
                         ("odometry only", (220, 60, 50)), ("planned path", (250, 160, 0))):
        d.line([8, y + 9, 28, y + 9], fill=color, width=3)
        d.text((34, y), label, font=SMALL, fill=(0, 0, 0))
        y += 21
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--model", default="dog", choices=("dog", "hexapod"))
    ap.add_argument("--goal", nargs=2, type=float, action="append", metavar=("X", "Y"),
                    help="map point in mm (repeat for several); default: through the doorway and back")
    ap.add_argument("--gyro-bias", type=float, default=1.0, help="gyro drift, deg/s (default 1)")
    ap.add_argument("--timeout", type=float, help="s per goal (default 90, 300 with --tof)")
    ap.add_argument("--png", help="write the final map with the tracks")
    ap.add_argument("--video", help="write a video (needs MUJOCO_GL=egl or xvfb-run)")
    ap.add_argument("--speedup", type=float, default=1.0, help="video plays this many times faster than real time")
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--tof", action="store_true",
                    help="no lidar: room scans with the terrain ToF sensors (stop, nose up, turn around)")
    ap.add_argument("--mount", type=float, metavar="DEG",
                    help="with --tof: the ToF sensors look DEG down instead of their TOF_SENSORS angle")
    ap.add_argument("--scan-tilt", type=float, metavar="DEG", help="with --tof: nose up while scanning (TOF_SCAN_PARAMS tilt)")
    ap.add_argument("--sensor-servo", type=float, metavar="DEG",
                    help="with --tof: a servo tilts the sensors DEG up for a scan, the body stays level "
                         "(TOF_SCAN_PARAMS sensorServo; with --mount for the walking angle)")
    ap.add_argument("--rescan", type=float, metavar="DEG",
                    help="with --tof: later scans sweep DEG around the heading (TOF_SCAN_PARAMS rescanAngle; 0 = full turns)")
    ap.add_argument("--floor", type=float, default=0, metavar="MM",
                    help="rug, door threshold, platform and tiles up to MM high (drive.py's default: 20 dog / 30 hexapod)")
    ap.add_argument("--sensing", choices=sorted(scenario.SENSING), default="none",
                    help="rough ground sensing while walking (with the rough gait), like drive.py")
    ap.add_argument("--obstacle", type=float, metavar="MM",
                    help="with --tof: something this high ahead stops the walk (TOF_SCAN_PARAMS obstacleHeight)")
    ap.add_argument("--doubtful", type=float, metavar="L",
                    help="map cells under L log-odds are doubtful walls: walk up and rescan (NAV_PARAMS doubtful)")
    ap.add_argument("--log", action="store_true", help="with --tof: print every scan (match, pose error)")
    ap.add_argument("--nav-speed", type=float, metavar="F",
                    help="walking speed while navigating, 0..1 of the gait's max speed (NAV_PARAMS speed)")
    args = ap.parse_args()

    model = args.model
    goals = [tuple(g) for g in args.goal] if args.goal else GOALS[model]
    fw = FirmwareCore(model=model)
    fw.set_heading_sensors(gyro_bias_dps=args.gyro_bias, magnetometer=False, seed=args.seed)
    if args.nav_speed is not None:
        fw.tune_nav(speed=args.nav_speed)
    if args.doubtful is not None:
        fw.tune_nav(doubtful=args.doubtful)
    if args.tof:
        if args.mount is not None:
            for i in range(len(fw.tof_sensors())):
                fw.set_tof_mount(i, args.mount)
        if args.scan_tilt is not None:
            fw.tune_tof_scan(tilt=math.radians(args.scan_tilt))
        if args.sensor_servo is not None:
            fw.tune_tof_scan(sensor_servo=1, tilt=math.radians(args.sensor_servo), tilt_rate=1.0)
        if args.obstacle is not None:
            fw.tune_tof_scan(obstacle_height=args.obstacle)
        if args.rescan is not None:
            fw.tune_tof_scan(rescan_angle=math.radians(args.rescan))
        # VL53L5CX: 8x8 at 15 Hz, closest target in each zone (2x2 rays), frames reach the firmware ~40 ms late
        fw.set_tof_nav(True, seed=args.seed, subrays=2, latency=0.04)
        sensor = "ToF room scans"
    else:
        fw.set_lidar(True, seed=args.seed)
        sensor = "LD06 lidar"
    if args.sensing != "none":
        fw.set_leveling(True)
        fw.set_terrain(**scenario.SENSING[args.sensing])
        fw.select_gait("rough")
    scene = terrain.rooms(scale=SCALE[model], floor=args.floor)
    robot = fw.place(scenario.make_robot(model, scene))
    dt = fw.loop_dt
    view = (-2400 * SCALE[model], 2600 * SCALE[model], -1300 * SCALE[model], 5200 * SCALE[model])

    rec = None
    if args.video:
        import mujoco
        rec = {"r": mujoco.Renderer(robot.model, 540, 540), "cam": mujoco.MjvCamera(), "frames": [], "next": 0.0}
        rec["cam"].distance = 2.6 if model == "dog" else 3.3
        rec["cam"].elevation = -55
        rec["cam"].azimuth = 90

    true_path, map_path, odom_path = [], [], []
    t = 0.0
    results = []

    last_phase = ["idle", False]
    phase_time = {}   # s in each scan phase, this scan (--log)

    def tick(goal):
        nonlocal t
        fw.drive(robot)
        t += dt
        p = robot.body_position_mm()
        mp, op = fw.lidar_pose()
        if args.log and args.tof:
            ns = fw.nav_state()
            if ns["phase"] != "idle":
                phase_time[ns["phase"]] = phase_time.get(ns["phase"], 0.0) + dt
            if ns["phase"] == "idle" and last_phase[0] != "idle" and phase_time:
                print("           phases: " + ", ".join(f"{k} {v:.1f} s" for k, v in phase_time.items()))
                phase_time.clear()
            if ns["phase"] == "turn" and last_phase[0] != "turn":
                last_phase[1] = ns["partial"]
            if ns["phase"] != "turn" and last_phase[0] == "turn":
                ls = fw.lidar_state()
                yaw = robot.body_orientation()["yaw"]
                yerr = math.degrees(abs((mp[2] - yaw + math.pi) % (2 * math.pi) - math.pi))
                print(f"  t {t:6.1f} s  {'partial' if last_phase[1] else 'full   '} scan: {ls['match']:9s} "
                      f"score {ls['score']:.2f}, {ns['scan_points']:4d} points, pose error "
                      f"{math.hypot(mp[0] - p[0], mp[1] - p[1]):4.0f} mm {yerr:4.1f} deg  at ({p[0]:5.0f}, {p[1]:5.0f})")
            last_phase[0] = ns["phase"]
        if int(t / dt) % 10 == 0:
            true_path.append((p[0], p[1]))
            map_path.append((mp[0], mp[1]))
            odom_path.append((op[0], op[1]))
        if rec is not None and t >= rec["next"]:
            rec["next"] += args.speedup / 15
            frame(goal)

    def frame(goal):
        import mujoco
        r, cam = rec["r"], rec["cam"]
        cam.lookat[:] = robot.data.xpos[robot._torso]
        r.update_scene(robot.data, cam)
        (fw.tof if args.tof else fw.lidar).draw(robot, r.scene)
        left = Image.fromarray(r.render())
        grid, cell = fw.lidar_map()
        mp, _ = fw.lidar_pose()
        ns = fw.nav_state()
        right = draw_map(grid, cell, (420, 540), view, true_path, map_path, odom_path, fw.nav_path(), goal, mp,
                         fw.lidar_points())
        img = Image.new("RGB", (960, 540))
        img.paste(left, (0, 0))
        img.paste(right, (540, 0))
        d = ImageDraw.Draw(img)
        d.text((12, 10), f"{model}: {sensor}: map + walk to point", font=FONT, fill=(255, 255, 255),
               stroke_width=3, stroke_fill=(0, 0, 0))
        phase = f"  scan: {ns['phase']}" if args.tof and ns["phase"] != "idle" else ""
        fast = f"  ({args.speedup:g}x)" if args.speedup != 1 else ""
        d.text((12, 40), f"t {t:4.1f} s{fast}   nav: {ns['state']}{phase}{'  (blocked ahead)' if ns['blocked'] else ''}",
               font=SMALL, fill=(255, 255, 255), stroke_width=3, stroke_fill=(0, 0, 0))
        rec["frames"].append(np.asarray(img))

    for _ in range(int(1.0 / dt)):   # stand: first scans
        tick(None)
    if args.tof:
        fw.tof_scan()                 # the first room scan before any goal (goals start with one anyway)
        while fw.nav_state()["phase"] == "idle":
            tick(None)
        while fw.nav_state()["phase"] != "idle":
            tick(None)
    for goal in goals:
        fw.nav_goal(*goal)
        start = t
        while t - start < (args.timeout or (300.0 if args.tof else 90.0)):
            tick(goal)
            if fw.nav_state()["state"] not in ("active",) and fw.nav_state()["phase"] == "idle":
                break
        for _ in range(int(0.5 / dt)):
            tick(goal)
        ns = fw.nav_state()
        p = robot.body_position_mm()
        yaw = robot.body_orientation()["yaw"]
        mp, op = fw.lidar_pose()
        res = {
            "goal": goal, "state": ns["state"], "time": t - start,
            "to_goal": math.hypot(p[0] - goal[0], p[1] - goal[1]),
            "map_err": math.hypot(mp[0] - p[0], mp[1] - p[1]),
            "map_yaw_err": math.degrees(abs((mp[2] - yaw + math.pi) % (2 * math.pi) - math.pi)),
            "odom_err": math.hypot(op[0] - p[0], op[1] - p[1]),
            "odom_yaw_err": math.degrees(abs((op[2] - yaw + math.pi) % (2 * math.pi) - math.pi)),
        }
        results.append(res)
        print(f"goal ({goal[0]:6.0f}, {goal[1]:6.0f}): {res['state']:8s} in {res['time']:5.1f} s, "
              f"true distance to goal {res['to_goal']:4.0f} mm | lidar pose error {res['map_err']:4.0f} mm "
              f"{res['map_yaw_err']:4.1f} deg | odometry only {res['odom_err']:5.0f} mm {res['odom_yaw_err']:5.1f} deg")
    ls = fw.lidar_state()
    if args.tof:
        print(f"scans {ls['scans']}, rejected {ls['rejected']}, total time {t:.0f} s")
    else:
        print(f"scans {ls['scans']}, rejected {ls['rejected']}, packets {ls['packets']}, CRC errors {ls['crc_errors']}")

    if args.png:
        grid, cell = fw.lidar_map()
        mp, _ = fw.lidar_pose()
        img = draw_map(grid, cell, (700, 910), view, true_path, map_path, odom_path, None, None, mp,
                       fw.lidar_points(), title=f"{model}: firmware map after {t:.0f} s")
        os.makedirs(os.path.dirname(os.path.abspath(args.png)), exist_ok=True)
        img.save(args.png)
        print("wrote", args.png)
    if rec is not None:
        import mediapy
        os.makedirs(os.path.dirname(os.path.abspath(args.video)), exist_ok=True)
        mediapy.write_video(args.video, rec["frames"], fps=15)
        print("wrote", args.video)
    return results


if __name__ == "__main__":
    main()
