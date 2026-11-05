"""
Search leg shapes for a compact direct-drive hexapod (MG90S, no gears).

  python tools/hexapod_leg_sweep.py                # best shapes for the default limits
  python tools/hexapod_leg_sweep.py --max-load 0.5 --min-speed 400 --top 15

For every coxa / femur / tibia / foot position on a grid it checks, statically:
  servo load   femur and tibia, tripod (3 feet), worst point of the stride, share
               of MG90S stall at 5 V (slippery floor; the sim reads ~half)
  stability    centre of mass to the nearest tripod edge, worst point of the stride
  clearance    body underside to the floor
  step height  the foot can still lift --lift mm at both ends of the stride
  speed        longest stride the leg reaches with the coxa inside +-coxa range and
               neighbouring feet >= --foot-gap apart; swing time from the biggest
               joint move at --servo-dps; speed = stride / swing time (duty 0.5)
  size         width across the middle feet
and keeps the smallest robots that pass all limits.
"""

import argparse
import itertools
import math

KGCM = 0.0980665
ap = argparse.ArgumentParser()
ap.add_argument("--body", type=float, nargs=3, default=(35, 58, 45), metavar=("X", "Y", "MIDX"))
ap.add_argument("--body-bottom", type=float, default=28, help="femur axis to body underside, mm")
ap.add_argument("--base-mass", type=float, default=600, help="g without the printed legs and 12 leg servos")
ap.add_argument("--stall", type=float, default=1.87, help="servo stall torque, kg*cm (MG90S datasheet at 5 V)")
ap.add_argument("--max-load", type=float, default=0.6)
ap.add_argument("--min-margin", type=float, default=35)
ap.add_argument("--min-clearance", type=float, default=40)
ap.add_argument("--lift", type=float, default=30)
ap.add_argument("--min-speed", type=float, default=300)
ap.add_argument("--coxa-range", type=float, default=40, help="+- deg the coxa may swing")
ap.add_argument("--foot-gap", type=float, default=30)
ap.add_argument("--servo-dps", type=float, default=300, help="usable joint speed under load, deg/s (MG90S no-load ~600)")
ap.add_argument("--speed-scale", type=float, default=1.6,
                help="calibration: this static model gives 249 mm/s for today's geared robot, the MuJoCo sim 400")
ap.add_argument("--top", type=int, default=12)
args = ap.parse_args()

BX, BY, MX = args.body
BODY = [(-BX, BY), (-MX, 0), (-BX, -BY), (BX, BY), (MX, 0), (BX, -BY)]
YAW = [math.atan2(y, x) if y else (0 if x > 0 else math.pi) for x, y in BODY]
GROUP = [0, 1, 0, 1, 0, 1]   # LF LR RM / LM RF RR


def ik(f, t, r, h):
    d = math.hypot(r, h)
    if d > f + t - 0.5 or d < abs(f - t) + 0.5:
        return None
    a = math.atan2(-h, r) + math.acos((f * f + d * d - t * t) / (2 * f * d))
    kx, kz = f * math.cos(a), f * math.sin(a)
    if kz < -h + 5:              # knee must stay above the foot
        return None
    return kx, kz, a, math.atan2(-h - kz, r - kx)


def foot(i, R, s):
    x, y = BODY[i]
    return x + R * math.cos(YAW[i]), y + R * math.sin(YAW[i]) + s


def leg_local(i, fx, fy, coxa):
    x, y = BODY[i]
    dx, dy = fx - x, fy - y
    ang = math.degrees(math.atan2(dy, dx) - YAW[i])
    ang = (ang + 180) % 360 - 180
    return math.hypot(dx, dy) - coxa, ang


def margin(pts):
    best = 1e9
    for i in range(3):
        (x1, y1), (x2, y2) = pts[i], pts[(i + 1) % 3]
        best = min(best, abs((x2 - x1) * (-y1) - (y2 - y1) * (-x1)) / math.hypot(x2 - x1, y2 - y1))
    return best


def evaluate(coxa, femur, tibia, reach, h):
    mass = args.base_mass + 6 * (0.16 * coxa + 0.2 * femur + 0.17 * tibia + 6)
    F = mass / 1000 * 9.81 / 3
    R = coxa + reach
    # longest stride that stays reachable, inside the coxa range, feet apart, liftable
    best = None
    for stride in range(120, 0, -5):
        ok = True
        worst_r = worst_t = 0.0
        joints = []
        for s in (-stride / 2, stride / 2):
            for i in range(6):
                r, ang = leg_local(i, *foot(i, R, s), coxa)
                k = ik(femur, tibia, r, h)
                if not k or abs(ang) > args.coxa_range or not ik(femur, tibia, r, h - args.lift):
                    ok = False
                    break
                worst_r, worst_t = max(worst_r, r), max(worst_t, abs(r - k[0]))
                joints.append((i, s, ang, k[2], k[3]))
            if not ok:
                break
        if not ok:
            continue
        # neighbouring feet in opposite tripods: one at +s, the other at -s
        for i, j in ((0, 1), (1, 2), (3, 4), (4, 5)):
            for s in (-stride / 2, stride / 2):
                a, b = foot(i, R, s), foot(j, R, -s)
                if math.dist(a, b) < args.foot_gap:
                    ok = False
        if not ok:
            continue
        m = min(margin([foot(i, R, s) for i in tri]) for s in (-stride / 2, stride / 2) for tri in ((0, 2, 4), (1, 3, 5)))
        # swing: coxa sweeps the stride, femur/tibia lift and return
        dcoxa = max(abs(a - b) for (i1, s1, a, *_), (i2, s2, b, *_) in itertools.combinations(joints, 2) if i1 == i2)
        k0 = ik(femur, tibia, reach, h)
        kl = ik(femur, tibia, reach, h - args.lift)
        if not k0 or not kl:
            continue
        dfem = abs(math.degrees(kl[2] - k0[2])) * 2
        dtib = abs(math.degrees((kl[3] - kl[2]) - (k0[3] - k0[2]))) * 2
        t_swing = max(dcoxa, dfem, dtib) / args.servo_dps + 0.04
        speed = stride / t_swing * args.speed_scale
        best = dict(stride=stride, margin=m, speed=speed,
                    femur=F * worst_r / 1000 / (args.stall * KGCM), tibia=F * worst_t / 1000 / (args.stall * KGCM))
        break
    if not best:
        return None
    best.update(shape=(coxa, femur, tibia, reach, h), mass=mass, width=2 * (MX + R), clearance=h - args.body_bottom)
    return best


rows = []
for coxa, femur, tibia, reach, h in itertools.product(range(30, 81, 5), range(30, 61, 5), range(60, 121, 5),
                                                      range(0, 61, 3), range(60, 101, 5)):
    e = evaluate(coxa, femur, tibia, reach, h)
    if not e:
        continue
    if (max(e["femur"], e["tibia"]) <= args.max_load and e["margin"] >= args.min_margin
            and e["clearance"] >= args.min_clearance and e["speed"] >= args.min_speed):
        rows.append(e)

rows.sort(key=lambda e: (e["width"], max(e["femur"], e["tibia"])))
print(f"{len(rows)} shapes pass: load <= {args.max_load:.0%}, margin >= {args.min_margin:.0f} mm, "
      f"clearance >= {args.min_clearance:.0f} mm, speed >= {args.min_speed:.0f} mm/s, lift {args.lift:.0f} mm")
print(f"{'coxa/femur/tibia':>16s} {'foot out/down':>13s} {'width':>6s} {'femur':>6s} {'tibia':>6s} "
      f"{'margin':>6s} {'clear':>6s} {'stride':>6s} {'speed':>6s} {'mass':>5s}")
seen = set()
for e in rows:
    key = (e["shape"][0], e["shape"][1], e["shape"][2])
    if key in seen:
        continue
    seen.add(key)
    c, f, t, r, h = e["shape"]
    print(f"{c:>6.0f}/{f:.0f}/{t:<5.0f} {r:>6.0f}/{h:<6.0f} {e['width']:>6.0f} {e['femur']:>6.0%} {e['tibia']:>6.0%} "
          f"{e['margin']:>6.0f} {e['clearance']:>6.0f} {e['stride']:>6.0f} {e['speed']:>6.0f} {e['mass']:>5.0f}")
    if len(seen) >= args.top:
        break
