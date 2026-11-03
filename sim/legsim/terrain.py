"""
Uneven ground and obstacles for the dog and hexapod sims.

A Terrain is a list of boxes standing on the floor (mm, world frame: the
robot starts at the origin facing +Y). Pass it to SimDog/SimHexapod:

    from legsim.terrain import bumps
    dog = SimDog(terrain=bumps(max_height=15))

The boxes are static MuJoCo geoms in geom group 0 like the floor, so the
robot's feet, legs and body collide with them and the simulated ToF sensors
(legsim/tof.py) see them.

Scenes (each leaves the start area flat):
  bumps()     tiles of random height, like rough ground or a lawn of small rocks
  rocks()     scattered rotated blocks
  rubble()    dense overlapping blocks of mixed size and height
  bars()      obstacles across the path, one after another
  step()      a platform to climb onto (and off again)
  drop()      start on a platform and walk off its edge (a step down)
  gaps()      a raised floor with trenches across the path, narrow to too wide
  bridge()    a ramp up onto a raised deck with gaps in it, and a ramp down
  course()    bars, then bumps, then a step: everything in one run
"""

import math

import numpy as np

MM = 0.001


class Terrain:
    def __init__(self, name="flat"):
        self.name = name
        self.boxes = []   # (cx, cy, height, half_x, half_y, yaw), mm / rad; box from z = 0 to height
        self.ramps = []   # (cx, y0, y1, h0, h1, half_x), mm: surface from h0 at y0 to h1 at y1 (along +Y)
        self.start_height = 0.0   # mm, ground under the robot at the start (drop())

    def add_box(self, cx, cy, height, size_x, size_y, yaw=0.0):
        if height > 0.2:
            self.boxes.append((float(cx), float(cy), float(height), size_x / 2.0, size_y / 2.0, float(yaw)))
        return self

    def add_ramp(self, cx, y0, y1, h0, h1, size_x):
        """A slope along +Y: the surface rises (or falls) from h0 at y0 to h1 at y1."""
        self.ramps.append((float(cx), float(y0), float(y1), float(h0), float(h1), size_x / 2.0))
        return self

    def __add__(self, other):
        t = Terrain(f"{self.name}+{other.name}")
        t.boxes = self.boxes + other.boxes
        t.ramps = self.ramps + other.ramps
        t.start_height = max(self.start_height, other.start_height)
        return t

    def height_at(self, x, y):
        """Ground height (mm) at world (x, y)."""
        h = 0.0
        for cx, cy, top, hx, hy, yaw in self.boxes:
            c, s = math.cos(yaw), math.sin(yaw)
            dx, dy = x - cx, y - cy
            lx, ly = c * dx + s * dy, -s * dx + c * dy
            if abs(lx) <= hx and abs(ly) <= hy and top > h:
                h = top
        for cx, y0, y1, h0, h1, hx in self.ramps:
            if abs(x - cx) <= hx and min(y0, y1) <= y <= max(y0, y1):
                h = max(h, h0 + (h1 - h0) * (y - y0) / (y1 - y0))
        return h

    def max_height(self):
        return max([b[2] for b in self.boxes] + [max(r[3], r[4]) for r in self.ramps], default=0.0)

    def geoms_xml(self, friction=0.8):
        out = []
        for k, (cx, cy, top, hx, hy, yaw) in enumerate(self.boxes):
            shade = 0.45 + 0.35 * min(1.0, top / 40.0)
            out.append(
                f'<geom name="terrain_{k}" type="box" size="{hx * MM:.6g} {hy * MM:.6g} {top * MM / 2:.6g}" '
                f'pos="{cx * MM:.6g} {cy * MM:.6g} {top * MM / 2:.6g}" euler="0 0 {yaw:.6g}" '
                f'group="0" contype="1" conaffinity="1" friction="{friction:.6g} 0.005 0.0001" '
                f'rgba="{shade:.3f} {shade * 0.85:.3f} {shade * 0.7:.3f} 1"/>')
        for k, (cx, y0, y1, h0, h1, hx) in enumerate(self.ramps):
            # a 10 mm slab tilted about X, its top surface on the slope
            length = math.hypot(y1 - y0, h1 - h0)
            angle = math.atan2(h1 - h0, y1 - y0)
            thick = 5.0
            nx, nz = -math.sin(angle), math.cos(angle)   # surface normal in the YZ plane (y, z)
            my, mz = (y0 + y1) / 2 - nx * thick, (h0 + h1) / 2 - nz * thick
            out.append(
                f'<geom name="ramp_{k}" type="box" size="{hx * MM:.6g} {length * MM / 2:.6g} {thick * MM:.6g}" '
                f'pos="{cx * MM:.6g} {my * MM:.6g} {mz * MM:.6g}" euler="{angle:.6g} 0 0" '
                f'group="0" contype="1" conaffinity="1" friction="{friction:.6g} 0.005 0.0001" '
                f'rgba="0.55 0.6 0.45 1"/>')
        return "\n    ".join(out)

    def inject(self, xml, friction=0.8):
        """Put the boxes into an MJCF model next to its floor."""
        if not self.boxes and not self.ramps:
            return xml
        i = xml.index('<geom name="floor"')
        j = xml.index("/>", i) + 2
        return xml[:j] + "\n    " + self.geoms_xml(friction) + xml[j:]


def flat():
    return Terrain("flat")


def bumps(max_height=15.0, tile=40.0, length=1200.0, width=500.0, y0=200.0, seed=0, min_height=0.0):
    """Tiles of random height between min_height and max_height (mm), from y0 forward."""
    rng = np.random.default_rng(seed)
    t = Terrain(f"bumps{max_height:g}")
    nx = int(round(width / tile))
    ny = int(round(length / tile))
    for iy in range(ny):
        for ix in range(nx):
            h = rng.uniform(min_height, max_height)
            t.add_box(-width / 2 + (ix + 0.5) * tile, y0 + (iy + 0.5) * tile, h, tile, tile)
    return t


def rocks(count=40, max_height=20.0, size=(20.0, 60.0), length=1200.0, width=400.0, y0=200.0, seed=1):
    """Scattered blocks, random size, height and yaw."""
    rng = np.random.default_rng(seed)
    t = Terrain(f"rocks{max_height:g}")
    for _ in range(count):
        t.add_box(rng.uniform(-width / 2, width / 2), rng.uniform(y0, y0 + length),
                  rng.uniform(0.3, 1.0) * max_height, rng.uniform(*size), rng.uniform(*size),
                  rng.uniform(0, math.pi))
    return t


def rubble(max_height=20.0, density=1.0, length=1000.0, width=450.0, y0=200.0, seed=7):
    """Dense rubble: many overlapping blocks of mixed size (10-80 mm), height
    (10-100 % of max_height) and yaw, covering about half the ground, so
    every step meets small and large edges."""
    rng = np.random.default_rng(seed)
    t = Terrain(f"rubble{max_height:g}")
    count = int(density * length * width / 2500.0)
    for _ in range(count):
        big = rng.random() < 0.3
        sx, sy = rng.uniform(40, 80, 2) if big else rng.uniform(10, 40, 2)
        h = rng.uniform(0.1, 0.6 if big else 1.0) * max_height
        t.add_box(rng.uniform(-width / 2, width / 2), rng.uniform(y0, y0 + length), h, sx, sy, rng.uniform(0, math.pi))
    return t


def bars(heights=(10.0, 20.0, 30.0), depth=30.0, spacing=350.0, width=600.0, y0=250.0):
    """Obstacles across the path (depth mm along the path), `spacing` mm apart."""
    t = Terrain("bars")
    for k, h in enumerate(heights):
        t.add_box(0, y0 + k * spacing + depth / 2, h, width, depth)
    return t


def step(height=25.0, y0=250.0, length=400.0, width=600.0):
    """A platform `height` mm high from y0 to y0 + length."""
    return Terrain(f"step{height:g}").add_box(0, y0 + length / 2, height, width, length)


def drop(height=20.0, y0=150.0, width=600.0, back=600.0):
    """The robot starts on a platform `height` mm high that ends at y0 and walks off it."""
    t = Terrain(f"drop{height:g}").add_box(0, y0 - back / 2, height, width, back)
    t.start_height = height
    return t


def gaps(height=40.0, widths=(25.0, 50.0, 150.0), segment=250.0, y0=150.0, width=600.0, back=600.0):
    """A raised floor `height` mm high with gaps (trenches down to the floor) across
    the path, `widths` mm wide, `segment` mm of floor between them. The robot
    starts on it. A gap it can step over it should cross, one too wide it
    should stop in front of."""
    t = Terrain(f"gaps{height:g}")
    t.add_box(0, y0 - back / 2, height, width, back)
    y = y0
    for w in widths:
        y += w
        t.add_box(0, y + segment / 2, height, width, segment)
        y += segment
    t.start_height = height
    return t


def bridge(height=50.0, ramp=300.0, deck=900.0, gap_widths=(25.0, 40.0), width=300.0, y0=200.0):
    """A ramp up from the floor onto a raised deck `height` mm high, gaps across
    the deck (`gap_widths`, spread along it), and a ramp back down."""
    t = Terrain(f"bridge{height:g}")
    t.add_ramp(0, y0, y0 + ramp, 0, height, width)
    y = y0 + ramp
    end = y + deck
    n = len(gap_widths)
    segment = (deck - sum(gap_widths)) / (n + 1)
    for w in list(gap_widths) + [None]:
        t.add_box(0, y + segment / 2, height, width, segment)
        y += segment
        if w is not None:
            y += w
    t.add_ramp(0, end, end + ramp, height, 0, width)
    return t


def course(scale=1.0, y0=250.0):
    """Bars, bumps, a step up and down; scale 1 = dog sizes, ~1.6 suits the hexapod."""
    s = scale
    t = bars(heights=(10 * s, 20 * s), depth=30, spacing=300 * s, y0=y0)
    t += bumps(max_height=12 * s, tile=40, length=400 * s, width=600, y0=y0 + 650 * s, seed=3)
    t += step(height=20 * s, y0=y0 + 1150 * s, length=350 * s)
    t.name = f"course{scale:g}"
    return t


SCENES = {
    "flat": flat,
    "bumps": bumps,
    "rocks": rocks,
    "rubble": rubble,
    "bars": bars,
    "step": step,
    "drop": drop,
    "gaps": gaps,
    "bridge": bridge,
    "course": course,
}
