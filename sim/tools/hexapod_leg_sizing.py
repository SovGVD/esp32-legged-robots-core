"""
Leg sizing for a new hexapod: static servo load for a given leg shape and mass,
direct drive (no gears) or with a gear ratio per joint.

  python tools/hexapod_leg_sizing.py                      # the design's legs vs a few other shapes, MG90S
  python tools/hexapod_leg_sizing.py --mass 800 --shape 55 40 85 30 80
      shape = coxa femur tibia reach height, mm: reach = foot out from the femur
      axis, height = foot below the femur axis

Static, slippery-floor worst case: 3 feet carry the robot with vertical forces
only, worst foot of the stride. On a grippy floor the feet also push sideways
and share the load, so the MuJoCo sim reads
about half of this (more with 6 feet down at rest). Target: <= ~60% here, so a weak
clone, 1 kg of robot, a slope or a smooth floor still walks. Stability = how far the centre of mass is
inside the tripod triangle at the worst point of the stride.
"""

import argparse
import math

KGCM = 0.0980665
SERVOS = {"MG90S 5V": 1.87, "MG90S 6V": 2.2}        # kg*cm stall, datasheet 1.8 @ 4.8 V, 2.2 @ 6 V (clones often ~70%)

ap = argparse.ArgumentParser()
ap.add_argument("--mass", type=float, nargs="+", default=[850.0], help="total robot mass, g")
ap.add_argument("--shape", type=float, nargs=5, action="append",
                metavar=("COXA", "FEMUR", "TIBIA", "REACH", "HEIGHT"))
ap.add_argument("--stride", type=float, default=60.0, help="step length along the body, mm")
ap.add_argument("--body", type=float, nargs=3, default=(35.0, 58.0, 45.0),
                metavar=("X", "Y", "MIDX"), help="coxa axes: corner legs (+-X, +-Y), middle legs (+-MIDX, 0)")
ap.add_argument("--femur-ratio", type=float, default=1.0)
ap.add_argument("--tibia-ratio", type=float, default=1.0)
args = ap.parse_args()

SHAPES = [("design (60/50/60)", (60, 50, 60, 9, 100), (1.0, 1.0)),
          ("old hexapod legs, direct drive", (35, 55, 110, 83, 95), (1.0, 1.0), (46, 80, 57.5)),
          ("tall, short reach", (45, 45, 95, 40, 85), (1.0, 1.0)),
          ("long coxa, short femur", (55, 40, 85, 30, 80), (1.0, 1.0)),
          ("long coxa, femur 1.5:1", (55, 50, 95, 45, 85), (1.5, 1.0)),
          ("compact knee (60/40/90)", (60, 40, 90, 18, 75), (1.0, 1.0)),
          ("compact knee, smaller body", (60, 40, 90, 18, 75), (1.0, 1.0), (40, 70, 50))]
if args.shape:
    SHAPES = [(f"{'/'.join(f'{v:g}' for v in s[:3])} reach {s[3]:g} h {s[4]:g}", tuple(s),
               (args.femur_ratio, args.tibia_ratio)) for s in args.shape]


def knee(femur, tibia, r, h):
    """Knee point (x, z) in the leg plane, femur axis at the origin, foot at (r, -h),
    knee up (the usual hexapod pose)."""
    d = math.hypot(r, h)
    if d > femur + tibia or d < abs(femur - tibia):
        return None
    a = math.atan2(-h, r) + math.acos((femur ** 2 + d ** 2 - tibia ** 2) / (2 * femur * d))
    return femur * math.cos(a), femur * math.sin(a)


def feet(coxa, reach, s):
    """Foot points (x, y) of the six legs with the stride offset s along y."""
    bx, by, mx = args.body
    out = []
    for x0, y0 in ((-bx, by), (-mx, 0), (-bx, -by), (bx, by), (mx, 0), (bx, -by)):
        ang = math.atan2(y0, x0) if y0 else (0 if x0 > 0 else math.pi)
        R = coxa + reach
        out.append((x0 + R * math.cos(ang), y0 + R * math.sin(ang) + s))
    return out


def margin(tri):
    """Distance from (0, 0) to the nearest edge of the triangle (negative = outside)."""
    best = 1e9
    for i in range(3):
        (x1, y1), (x2, y2) = tri[i], tri[(i + 1) % 3]
        cross = (x2 - x1) * (0 - y1) - (y2 - y1) * (0 - x1)
        best = min(best, cross / math.hypot(x2 - x1, y2 - y1))
    sign = 1 if sum((tri[(i + 1) % 3][0] - tri[i][0]) * (tri[(i + 1) % 3][1] + tri[i][1]) for i in range(3)) < 0 else -1
    return best * sign


SHAPES = [s if len(s) == 4 else (*s, tuple(args.body)) for s in SHAPES]
for mass in args.mass:
    F = mass / 1000 * 9.81 / 3          # N per foot, tripod
    print(f"\n=== {mass:.0f} g, tripod stance {F:.2f} N per foot, stride {args.stride:.0f} mm ===")
    print(f"{'shape':34s} {'femur':>12s} {'tibia':>12s}  {'body h':>6s} {'margin':>6s}  (share of stall: " +
          ", ".join(SERVOS) + ")")
    for name, (coxa, femur, tibia, reach, h), (rf, rt), body in SHAPES:
        args.body = body
        worst_r, worst_t = 0.0, 0.0
        for s in (-args.stride / 2, 0, args.stride / 2):
            for x0, y0 in ((args.body[0], args.body[1]), (args.body[2], 0)):
                # radial reach of the foot from the femur axis when the foot is s along the body
                R = coxa + reach
                ang = math.atan2(y0, x0)
                fx, fy = x0 + R * math.cos(ang), y0 + R * math.sin(ang) + s
                r = math.hypot(fx - x0, fy - y0) - coxa
                k = knee(femur, tibia, r, h)
                if k is None:
                    worst_r = float("nan")
                    continue
                worst_r = max(worst_r, r)
                worst_t = max(worst_t, abs(r - k[0]))
        tf, tt = F * worst_r / 1000 / rf, F * worst_t / 1000 / rt     # N*m at the servo
        cells_f = " ".join(f"{tf / (st * KGCM):4.0%}" for st in SERVOS.values())
        cells_t = " ".join(f"{tt / (st * KGCM):4.0%}" for st in SERVOS.values())
        m = min(margin([f for i, f in enumerate(feet(coxa, reach, s)) if i in tri])
                for s in (-args.stride / 2, args.stride / 2) for tri in ((0, 2, 4), (1, 3, 5)))
        print(f"{name:34s} {cells_f:>12s} {cells_t:>12s}  {h:5.0f}  {m:5.0f}mm")
