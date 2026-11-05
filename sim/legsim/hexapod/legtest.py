"""
Leg-by-leg direction test: does every servo turn the way the firmware expects?

The robot hangs in the air (fixed base). One leg at a time gets foot targets
through the firmware's own IK and servo HAL (FirmwareCore.leg_test_pulses, so
the inverse flags in model_data.h are used), and the sim servos turn the way
SimHexapod.direction says (from the flags until the robot is built; flip an
entry to see a servo mounted the other way). Each move is checked on the sim foot:

  UP       foot 15 mm up            -> tip rises
  OUT      foot 20 mm outward       -> tip moves away from the body
  FORWARD  foot 15 mm forward       -> tip moves forward (+Y; the rear feet reach ~12 mm)
  TURN     leg 15 deg CCW from above around its coxa axis -> leg turns CCW

and every joint that moved against the IK's plan is named, which points at
the inverse flag to change. The same moves can run on the real robot later.
"""

import math

import numpy as np

from .kinematics import firmware_ik, leg_angles_to_q, default_foot, coxa_position
from .params import JOINTS

MOVE = 15.0          # mm
MOVE_OUT = 20.0      # mm
TURN = 15.0          # deg
RAMP = 0.6           # s, smooth move to the target and back
HOLD = 0.3           # s


def step_offsets(leg, move=MOVE):
    """(name, what should happen, foot offset in mm) for one leg."""
    out = np.array([math.cos(leg.yaw), math.sin(leg.yaw), 0.0])
    f = default_foot(leg) - coxa_position(leg)
    a = math.radians(TURN)
    rot = np.array([f[0] * math.cos(a) - f[1] * math.sin(a), f[0] * math.sin(a) + f[1] * math.cos(a), f[2]])
    return [
        ("UP", "foot goes up", np.array([0.0, 0.0, move])),
        ("OUT", "foot goes away from the body", MOVE_OUT * out),
        ("FORWARD", "foot goes forward", np.array([0.0, move, 0.0])),
        ("TURN", f"leg turns {TURN:.0f} deg counter-clockwise (from above)", rot - f),
    ]


def _check(name, leg, before, after, move=MOVE):
    """Did the foot tip (body frame, mm) move as asked? Returns (ok, measured text)."""
    d = after - before
    if name == "UP":
        return d[2] > move / 2, f"dz {d[2]:+.0f} mm"
    if name == "OUT":
        r = np.array([math.cos(leg.yaw), math.sin(leg.yaw), 0.0])
        return d @ r > MOVE_OUT / 2, f"out {d @ r:+.0f} mm"
    if name == "FORWARD":
        return d[1] > move / 2, f"dy {d[1]:+.0f} mm"
    c = coxa_position(leg)
    a0 = math.atan2(before[1] - c[1], before[0] - c[0])
    a1 = math.atan2(after[1] - c[1], after[0] - c[0])
    turned = math.degrees((a1 - a0 + math.pi) % (2 * math.pi) - math.pi)
    return turned > TURN / 2, f"turned {turned:+.0f} deg"


def run(fw, robot, legs=None, on_frame=None, frame_dt=1 / 30, move=MOVE):
    """Run the test. fw: FirmwareCore(model="hexapod"), robot: SimHexapod(fixed_base=True).
    move: mm up and forward.
    on_frame(robot, label) is called every frame_dt of sim time (for videos).
    Returns a list of dicts: leg, move, expect, ok, measured, wrong_joints."""
    names = [l.name for l in robot.p.legs]
    legs = names if legs is None else legs
    geo = robot.p.geometry
    neutral = [fw.leg_test_pulses(i, (0, 0, 0))[0] for i in range(len(names))]
    pulses = np.array(neutral, dtype=float)
    state = {"next": robot.data.time}

    def advance(seconds, label):
        n = max(1, int(round(seconds / 0.02)))
        for _ in range(n):
            robot.write_pulses(pulses.ravel())
            robot.step(0.02)
            if on_frame and robot.data.time >= state["next"]:
                on_frame(robot, label)
                state["next"] += frame_dt

    advance(0.5, "")
    results = []
    for name in legs:
        i = names.index(name)
        leg = robot.p.legs[i]
        for move_name, expect, offset in step_offsets(leg, move):
            label = f"{name}: {expect}"
            before = robot.foot_positions_body_mm()[i].copy()
            q0 = robot.joint_q()[i * 3:i * 3 + 3].copy()
            target, solved = fw.leg_test_pulses(i, offset)
            for k in range(1, int(RAMP / 0.02) + 1):
                s = k * 0.02 / RAMP
                s = s * s * (3 - 2 * s)
                pulses[i] = neutral[i] + s * (target.astype(float) - neutral[i])
                advance(0.02, label)
            advance(HOLD, label)
            after = robot.foot_positions_body_mm()[i].copy()
            dq = robot.joint_q()[i * 3:i * 3 + 3] - q0
            plan = (np.array(leg_angles_to_q(*firmware_ik(geo, leg, default_foot(leg) + offset)))
                    - np.array(leg_angles_to_q(*firmware_ik(geo, leg, default_foot(leg)))))
            wrong = [JOINTS[j] for j in range(3)
                     if abs(plan[j]) > math.radians(3) and np.sign(dq[j]) != np.sign(plan[j])]
            ok, measured = _check(move_name, leg, before, after, move)
            ok = ok and solved and not wrong
            results.append({"leg": name, "move": move_name, "expect": expect, "ok": ok,
                            "measured": measured, "wrong_joints": wrong})
            label = f"{name}: {expect}  {'OK' if ok else 'WRONG'}"
            advance(HOLD, label)
            for k in range(1, int(RAMP / 0.02) + 1):
                s = k * 0.02 / RAMP
                s = s * s * (3 - 2 * s)
                pulses[i] = target + s * (neutral[i] - target.astype(float))
                advance(0.02, f"{name}: back")
            pulses[i] = neutral[i]
            advance(0.2, f"{name}: back")
    return results


def summary(results, fw=None):
    """Text table, plus which inverse flags to flip."""
    lines = []
    for r in results:
        joints = f"  wrong way: {', '.join(r['wrong_joints'])}" if r["wrong_joints"] else ""
        lines.append(f"{r['leg']}  {r['move']:8s} {'OK   ' if r['ok'] else 'WRONG'}  {r['measured']:16s}{joints}")
    bad = {}
    for r in results:
        for j in r["wrong_joints"]:
            bad.setdefault(r["leg"], set()).add(j)
    if bad:
        lines.append("")
        lines.append("Flip these inverse flags in model_data.h:")
        flag = {"coxa": "alpha", "femur": "beta", "tibia": "gamma"}
        for leg, joints in bad.items():
            lines.append(f"  {leg}: " + ", ".join(f"{j} ({flag[j]})" for j in JOINTS if j in joints))
    else:
        lines.append("")
        lines.append("All servos turn the way the firmware expects.")
    return "\n".join(lines)
