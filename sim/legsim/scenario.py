"""
Walk a robot over a terrain with the firmware and measure how it went.

    from legsim.scenario import run
    r = run("dog", "bars", sensing="both")
    print(r.summary())

sensing: "none" (open loop, the real robots today), "contacts" (foot switches),
"tof" (ToF terrain map only), "both".
"""

import math
from dataclasses import dataclass, field

import numpy as np

from . import terrain as terrains
from .firmware import FirmwareCore
from .robot import SimDog

SENSING = {
    "none": dict(contacts=False, tof=False),
    "contacts": dict(contacts=True, tof=False),
    "tof": dict(contacts=False, tof=True),
    "both": dict(contacts=True, tof=True),
}

# per robot: gait, move command and scene sizes for terrain_walk.py and the tests
DEFAULTS = {
    "dog": {"gait": "rough", "speed": 1.0, "scale": 1.0},
    "hexapod": {"gait": "rough", "speed": 1.0, "scale": 1.5},   # higher than ~30 mm: femur servo range
}


def scene(model, name, height=None):
    """A terrain scene sized for the robot (height: obstacle / bump height in mm)."""
    k = DEFAULTS[model]["scale"]
    y0 = 200.0 if model == "dog" else 300.0
    if name == "bumps":
        return terrains.bumps(max_height=height or 12 * k, y0=y0)
    if name == "rocks":
        return terrains.rocks(max_height=height or 15 * k, y0=y0)
    if name == "bars":
        h = height or 20 * k
        return terrains.bars(heights=(h / 2, h), spacing=300, y0=y0)
    if name == "step":
        return terrains.step(height=height or 20 * k, y0=y0, length=500)
    if name == "drop":
        return terrains.drop(height=height or 15 * k, y0=150.0 if model == "dog" else 200.0)
    if name == "course":
        return terrains.course(scale=k, y0=y0)
    return terrains.SCENES[name]()


_cores = {}


def core(model):
    """One FirmwareCore per model and process (the C++ side is global state)."""
    if model not in _cores:
        _cores[model] = FirmwareCore(model=model)
    return _cores[model]


def make_robot(model, terrain):
    if model == "hexapod":
        from .hexapod import SimHexapod
        return SimHexapod(terrain=terrain)
    return SimDog(terrain=terrain)


@dataclass
class Result:
    model: str
    terrain: str
    sensing: str
    duration: float
    distance: float = 0.0        # mm forward (y)
    sideways: float = 0.0        # mm drift (x)
    max_tilt: float = 0.0        # deg, worst pitch/roll
    mean_tilt: float = 0.0       # deg, mean |tilt|
    fell: bool = False           # tilt over 45 deg or body on the ground
    body_hits: int = 0           # ticks the body (not a foot) touched the terrain
    climbed: float = 0.0         # mm, highest ground the body centre got over (above the start)
    unsolved: int = 0            # IK failures (leg out of reach)
    rejects: dict = field(default_factory=dict)   # FirmwareCore.reject_counts()
    odometry: float = 0.0        # mm forward the gait thinks it walked
    trace: list = field(default_factory=list)

    def summary(self):
        return (f"{self.model:8s} {self.terrain:12s} {self.sensing:9s} "
                f"{self.distance:6.0f} mm fwd  {self.sideways:+5.0f} side  tilt max {self.max_tilt:4.1f} "
                f"mean {self.mean_tilt:3.1f} deg  body hits {self.body_hits:4d}  "
                f"{'FELL' if self.fell else 'ok'}")


def _body_contacts(robot):
    """Number of contacts between the terrain or floor and anything but a foot."""
    feet = {k for _, k in robot.foot_contact_list()}
    return sum(1 for k in range(robot.data.ncon) if k not in feet)


def run(model="dog", terrain="course", sensing="both", speed=0.5, duration=None, leveling=None,
        frame_cb=None, frame_dt=1 / 30, seed=0, gait="default", tune=None, **terrain_args):
    """Walk forward over `terrain` (a name from legsim.terrain.SCENES or a Terrain) for
    `duration` s (default 8 dog / 6 hexapod) holding the move command `speed`.
    leveling: IMU leveling (default: on with any sensing; the terrain code needs it).
    frame_cb(robot, fw, t) is called every frame_dt seconds (videos)."""
    t = terrains.SCENES[terrain](**terrain_args) if isinstance(terrain, str) else terrain
    fw = core(model)
    if leveling is None:
        # the terrain code relies on the IMU leveler to keep the body level
        # (the feet stand at different heights); open loop runs without it
        leveling = sensing != "none"
    if duration is None:
        duration = 8.0 if model == "dog" else 6.0
    res = Result(model, t.name, sensing, duration)
    try:
        fw.set_leveling(leveling)
        fw.set_terrain(**SENSING[sensing], seed=seed)
        fw.select_gait(gait)
        if tune:
            fw.tune_gait(**tune)
        robot = fw.place(make_robot(model, t))
        start = robot.body_position_mm().copy()
        tilts = []
        ticks = int(round(duration / fw.loop_dt))
        next_frame = 0.0
        for k in range(ticks):
            fw.drive(robot, (0, speed, 0, 0, 0, 0))
            o = robot.body_orientation()
            tilt = math.degrees(max(abs(o["pitch"]), abs(o["roll"])))
            tilts.append(tilt)
            res.body_hits += 1 if _body_contacts(robot) else 0
            p = robot.body_position_mm()
            res.climbed = max(res.climbed, t.height_at(p[0], p[1]) - t.start_height)
            if k % 10 == 0:
                res.trace.append((k * fw.loop_dt, *p, o["pitch"], o["roll"]))
            if tilt > 45:
                res.fell = True
                break
            if frame_cb and k * fw.loop_dt >= next_frame:
                frame_cb(robot, fw, k * fw.loop_dt)
                next_frame += frame_dt
        p = robot.body_position_mm()
        res.distance = p[1] - start[1]
        res.sideways = p[0] - start[0]
        res.max_tilt = max(tilts)
        res.mean_tilt = float(np.mean(tilts))
        res.unsolved = fw.unsolved
        res.rejects = fw.reject_counts()
        res.odometry = float(fw.body()[1])
    finally:
        # back to the plain firmware for whoever uses this FirmwareCore next
        fw.set_terrain()
        fw.set_leveling(False)
        fw.select_gait("default")
    return res
