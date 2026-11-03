"""
What-if: the hexapod with other servos. Same legs, stance and weight, firmware
tripod gait; compares the current MG90S + gears with PTK 7465MG servos driving
the joints directly (no gears).

  python examples/hexapod_servo_compare.py                       # 6 V and 7.4 V at 50 Hz, 7.4 V at 333 Hz
  python examples/hexapod_servo_compare.py --volts 8.4 --hz 333 --speeds 400 700 900

For each setup: femur/tibia load standing (share of stall torque), then per
commanded speed the speed reached, the worst servo lag behind its target, and
the worst body tilt. Datasheet torque/speed only: real servos are usually weaker.
"""

import argparse
import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

import numpy as np
from legsim import FirmwareCore
from legsim.hexapod import SimHexapod, HexapodParams, ptk_7465mg, direct_drive

ap = argparse.ArgumentParser()
ap.add_argument("--volts", type=float, nargs="*", help="7465MG supply voltages (5.0-8.4)")
ap.add_argument("--hz", type=float, default=None, help="servo frame rate for the 7465MG setups (PCA9685: 50)")
ap.add_argument("--speeds", type=float, nargs="+", default=[400, 700])
ap.add_argument("--cycle", type=float, default=0.3, help="gait cycle time, s (duty 0.5)")
args = ap.parse_args()

if args.volts:
    setups = [(f"7465MG {v:.1f} V {args.hz or 50:.0f} Hz", direct_drive(servo=ptk_7465mg(v, args.hz or 50)))
              for v in args.volts]
else:
    setups = [(f"7465MG {v:.1f} V {hz} Hz", direct_drive(servo=ptk_7465mg(v, hz)))
              for v, hz in ((6.0, 50), (7.4, 50), (7.4, 333))]
setups.insert(0, ("MG90S + gears (now)", HexapodParams()))

fw = FirmwareCore(model="hexapod")


def run(params, command, seconds, speed=None):
    ideal = params.hal.ratio != HexapodParams().hal.ratio
    fw.reset()
    if speed:
        fw.tune_gait(cycle_time=args.cycle, duty_factor=0.5, max_speed=speed)
        fw.set_side_limits(0)
    robot = fw.place(SimHexapod(params=params))
    robot.step(0.3)
    fw.drive(robot, command, 1.5, ideal_servos=ideal)
    p0 = robot.body_position_mm().copy()
    lag, tilt = np.zeros(3), 0.0
    for _ in range(int(round(seconds / fw.loop_dt))):
        fw.drive(robot, command, ideal_servos=ideal)
        err = np.nan_to_num(np.abs(robot.servos.target - robot.joint_q())).reshape(6, 3).max(axis=0)
        lag = np.maximum(lag, np.degrees(err))
        o = robot.body_orientation()
        tilt = max(tilt, math.degrees(max(abs(o["pitch"]), abs(o["roll"]))))
    return robot, (robot.body_position_mm() - p0) / seconds, lag, tilt


for name, params in setups:
    robot, *_ = run(params, (0, 0, 0, 0, 0, 0), 0.5)
    load = robot.joint_torque_ratio().reshape(6, 3).max(axis=0)
    print(f"{name}: standing load femur {load[1]:.0%}, tibia {load[2]:.0%}")
    for label, command, axis in (("forward", (0, 1, 0, 0, 0, 0), 1), ("sideways", (1, 0, 0, 0, 0, 0), 0)):
        for speed in args.speeds:
            _, v, lag, tilt = run(params, command, 3.0, speed)
            print(f"  {label:8s} {speed:5.0f} mm/s -> {v[axis]:5.0f} mm/s, "
                  f"lag coxa/femur/tibia {lag[0]:3.0f}/{lag[1]:3.0f}/{lag[2]:3.0f} deg, tilt {tilt:4.1f} deg")
