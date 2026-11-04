"""
What-if for a new hexapod body: MG90S servos driving the joints directly (no
printed gears), 2x 18650 + buck converters. Same legs and stance as now.

  python examples/hexapod_body_check.py                     # table: gears now vs direct drive, 5 and 6 V
  python examples/hexapod_body_check.py --mass 700 850 1000 # total robot mass, g (default ~892, the sim estimate)
  python examples/hexapod_body_check.py --femur-ratio 1.5   # direct coxa + tibia, 1.5:1 gear on the femur only

For each setup: servo load standing and walking (share of stall torque), the
time the femur servos of the legs on the ground spend at stall, the body height lost, the speed reached,
and the servo current from the torque (I = idle + stall current x load). The
MG90S numbers are datasheet values; clones are often weaker.

Live: python examples/drive.py --hexapod --servos mg90s-direct [--mass 850]
"""

import argparse
import dataclasses
import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

import numpy as np
from legsim import FirmwareCore
from legsim.hexapod import SimHexapod, HexapodParams
from legsim.hexapod.params import mg90s, with_mass, HexHalParams, LINEAR_500_2500

ap = argparse.ArgumentParser()
ap.add_argument("--mass", type=float, nargs="*", help="total robot mass, g")
ap.add_argument("--volts", type=float, nargs="*", default=[5.0, 6.0], help="direct-drive servo supply")
ap.add_argument("--femur-ratio", type=float, default=1.0, help="gear on the femur joint only (1 = none)")
ap.add_argument("--speed", type=float, default=200, help="forward walk speed, mm/s")
ap.add_argument("--cycle", type=float, default=0.4, help="gait cycle, s (duty 0.5)")
args = ap.parse_args()

fw = FirmwareCore(model="hexapod")


def direct(volts, femur_ratio):
    p = HexapodParams()
    return dataclasses.replace(p, servo=mg90s(volts), hal=HexHalParams(ratio=(1.0, femur_ratio, 1.0)),
                               profile=LINEAR_500_2500)


def run(params, seconds, command):
    ideal = params.hal.ratio != HexapodParams().hal.ratio
    fw.reset()
    fw.tune_gait(cycle_time=args.cycle, duty_factor=0.5, max_speed=args.speed)
    fw.set_side_limits(0)
    robot = fw.place(SimHexapod(params=params))
    robot.step(0.3)
    fw.drive(robot, (0, 0, 0, 0, 0, 0), 1.0, ideal_servos=ideal)
    z0 = robot.body_position_mm()[2]
    stand = robot.joint_torque_ratio().reshape(6, 3)
    p0 = robot.body_position_mm().copy()
    loads, zs, feet = [], [], []
    n = int(round(seconds / fw.loop_dt))
    for _ in range(n):
        fw.drive(robot, command, ideal_servos=ideal)
        loads.append(robot.joint_torque_ratio().reshape(6, 3))
        zs.append(robot.body_position_mm()[2])
        feet.append(robot.foot_vertical_forces())
    v = (robot.body_position_mm() - p0) / seconds
    return stand, np.array(loads), z0, np.array(zs), v, np.array(feet) > 2.0


def current_a(loads, volts):
    """Per-servo current from the torque share. MG90S: ~0.7 A stall at 4.8 V,
    ~0.9 A at 6 V [ds]; ~10 mA idle electronics."""
    stall = 0.7 + (volts - 4.8) / 1.2 * 0.2
    return 0.01 + stall * np.clip(loads, 0, 1)


masses = args.mass or [HexapodParams().masses.total()]
setups = [("MG90S + gears (now), 5 V", lambda: HexapodParams(), 5.0)]
for v in args.volts:
    lab = "direct" if args.femur_ratio == 1 else f"direct, femur {args.femur_ratio}:1"
    setups.append((f"MG90S {lab}, {v:.0f} V", lambda v=v: direct(v, args.femur_ratio), v))

print(f"walk {args.speed:.0f} mm/s forward, cycle {args.cycle} s, duty 0.5")
for m in masses:
    print(f"\n=== total mass {m:.0f} g ===")
    if m < 840:
        print("(the sim model can't go below ~840 g: printed parts, servos and battery alone weigh that)")
    for name, make, volts in setups:
        params = with_mass(make(), m)
        stand, loads, z0, zs, v, down = run(params, 3.0, (0, 1, 0, 0, 0, 0))
        fem = loads[:, :, 1][down]          # femur servos of the legs carrying the robot
        sat = (fem > 0.98).mean()
        per_leg = current_a(loads, volts).sum(axis=2)            # [tick, leg]: one buck per leg
        k = max(1, int(round(0.05 / fw.loop_dt)))
        leg50 = max(np.convolve(per_leg[:, i], np.ones(k) / k, "valid").max() for i in range(6))
        cur = per_leg.sum(axis=1)
        cur_stand = current_a(stand, volts).sum()
        print(f"{name}")
        print(f"  standing: femur {stand[:, 1].max():.0%} of stall, tibia {stand[:, 2].max():.0%}, "
              f"servos {cur_stand:.1f} A")
        print(f"  walking : femur on the ground mean {fem.mean():.0%}, p90 {np.percentile(fem, 90):.0%}, "
              f"at stall {sat:.0%} of the time; tibia p90 {np.percentile(loads[:, :, 2][down], 90):.0%}")
        print(f"            body sag {z0 - zs.min():.0f} mm, speed {v[1]:.0f} mm/s, "
              f"servo current mean {cur.mean():.1f} A, peak {cur.max():.1f} A")
        print(f"            per leg (3 servos): mean {per_leg.mean(axis=0).max():.2f} A, "
              f"50 ms peak {leg50:.1f} A, peak {per_leg.max():.1f} A")
