"""
Physical parameters of the hexapod (6 legs, 3 joints each, MG90S straight on
every joint, no gears). The 2026-11 design ("Recommended" in the body study),
not built yet: hexagon body, 2x 18650 at the bottom centre, a PCA9685 + 3
Mini360 board standing along each side, the ESP32 standing across the front,
GY-91 under the top cover in the middle, 2 VL53L5CX at the front, lever switch
feet. There are no CAD parts yet, so the sim model is simple shapes
(simple_parts.py) sized like the design concept (artifact "Direct-drive
hexapod concept").

Units follow the firmware: mm, g, degrees/radians as noted. The MJCF builder
converts to SI. Every number says where it comes from:

  [code]  src/esp32-legged-robot/model/hexapod (config.h, model_data.h)
  [design] the body study (sim/tools/hexapod_leg_sweep.py, the concept artifact)
  [ds]    MG90S datasheet (typical values, not measured on this robot)
  [est]   engineering guess, tune against the real robot
"""

import dataclasses
from dataclasses import dataclass, field
import math

from ..params import Environment


@dataclass(frozen=True)
class HexLegConfig:
    """One leg, mirrors the `leg` struct in model/hexapod/model_data.h [code]."""
    name: str
    body: tuple          # coxa axis on the body (defaultBody), mm, X right, Y forward
    foot: tuple          # neutral foot point (defaultFoot), mm
    inverse_xyz: tuple = (True, True, True)      # LR_legInverse x, y, z [code]
    inverse_abg: tuple = (False, False, False)   # LR_legInverse alpha, beta, gamma [code]

    @property
    def side(self):
        """-1 left, +1 right."""
        return -1 if self.body[0] < 0 else 1

    @property
    def yaw(self):
        """Direction the leg points in its neutral stance (rad, from +X towards +Y).
        HexapodLegIK measures the coxa angle from it: alpha = pi/2 there."""
        return math.atan2(self.foot[1] - self.body[1], self.foot[0] - self.body[0])


# config.h [code]: coxa axes at (+-35, +-58) and (+-45, 0); neutral feet 69 mm out
# from the coxa axis, 100 mm below it; rear and middle legs straight out from the
# body centre, the front legs 20 deg further out to the side (out of the ToF view)
FRONT_BODY, FRONT_FOOT = (35.0, 58.0), (88.71, 101.32)
REAR_BODY, REAR_FOOT = (35.0, -58.0), (70.65, -117.08)
MIDDLE_BODY, MIDDLE_FOOT = (45.0, 0.0), (114.0, 0.0)
FOOT_Z = 100.0

# Angle inverse flags: right legs mirrored, their femur and tibia servos turn the
# other way [code, assumed until the robot is built: check with the leg test]
_INV_L = (False, False, False)
_INV_R = (False, True, True)


def _leg(name, body, foot, right):
    sx = 1 if right else -1
    return HexLegConfig(name, (sx * body[0], body[1], 0.0), (sx * foot[0], foot[1], -FOOT_Z),
                        inverse_abg=_INV_R if right else _INV_L)


# Firmware order: LF, LM, LR, RF, RM, RR [code]
LEGS = (
    _leg("LF", FRONT_BODY, FRONT_FOOT, False),
    _leg("LM", MIDDLE_BODY, MIDDLE_FOOT, False),
    _leg("LR", REAR_BODY, REAR_FOOT, False),
    _leg("RF", FRONT_BODY, FRONT_FOOT, True),
    _leg("RM", MIDDLE_BODY, MIDDLE_FOOT, True),
    _leg("RR", REAR_BODY, REAR_FOOT, True),
)

JOINTS = ("coxa", "femur", "tibia")   # alpha, beta, gamma

PLATE_INFLATE = 14.0    # body plates reach 14 mm past the coxa axes [design]
BOTTOM_Z = -28.0        # bottom plate under the femur axis (body origin) [design]
TOP_Z = 16.0            # top cover [design]


@dataclass
class HexGeometry:
    coxa: float = 60.0              # link lengths [code]
    femur: float = 50.0
    tibia: float = 60.0
    femur_axis_z: float = -BOTTOM_Z  # body origin above the bottom of the body [design]
    torso_size: tuple = (110.0, 150.0, TOP_Z + 3 - BOTTOM_Z)   # collision box [design]
    torso_z: float = (TOP_Z + 3 + BOTTOM_Z) / 2                # box centre relative to the body origin
    foot_radius: float = 5.0        # lever switch pad [est]
    femur_width: float = 7.0        # collision capsule radius, only the floor collides [est]
    tibia_width: float = 5.0


@dataclass
class HexMasses:
    """Grams, ~815 g in all [design]. Legs: coxa bracket 8 + femur servo 13.4;
    femur 7 + tibia servo 13.4; tibia 6 + lever switch 2 [est]. Body: the rest
    (6 coxa servos, cells, boards, plates, wires)."""
    torso: float = 516.2
    battery: float = 100.0      # 2x 18650 (~47 g each) + holder [ds, est]
    electronics: float = 55.0   # ESP32, 2x PCA9685, 7x Mini360, GY-91, 2x VL53L5CX, fuse, switch [est]
    coxa: float = 21.4
    femur: float = 20.4
    tibia: float = 8.0

    def total(self) -> float:
        return self.torso + 6 * (self.coxa + self.femur + self.tibia)


@dataclass
class HexServoParams:
    """Servo model at the shaft (= the joint: direct drive). The defaults are
    the MG90S at 4.8 V [ds]; `mg90s(volts)` gives other supply voltages."""
    stall_torque: float = 0.176         # N*m (1.8 kg*cm at 4.8 V) [ds]
    no_load_speed: float = math.radians(60) / 0.10   # rad/s (0.10 s/60 deg at 4.8 V) [ds]
    kp: float = 3.0                     # N*m/rad, saturates at ~3.4 deg error [est]
    kd: float = 0.004                   # N*m*s/rad [est]
    deadband_us: float = 5.0            # MG90 class [ds]
    pwm_period: float = 0.020           # s, 50 Hz servo frame (PCA9685 at 50 Hz)
    armature: float = 2.0e-5            # kg*m^2 reflected rotor inertia at the shaft [est]
    friction: float = 0.004             # N*m Coulomb friction at the shaft [est]
    min_angle: float = 0.0              # deg, servoMainProfile minAngle/maxAngle [code]
    max_angle: float = 180.0


# servoMainProfile in model/hexapod/model_data.h: 500-2500 us over 0-180 deg [code, ds]
LINEAR_500_2500 = ((0, 180), (500, 2500))
SERVO_PROFILE = LINEAR_500_2500


@dataclass
class HexHalParams:
    """LR_legHal, same for every leg [code]: direct drive, ratio 1."""
    mid: tuple = (math.pi / 2, math.pi / 2, math.pi / 2)
    ratio: tuple = (1.0, 1.0, 1.0)


# MG90S: 1.8 kg*cm, 0.10 s/60 deg at 4.8 V; 2.2 kg*cm, 0.08 s/60 deg at 6 V [ds]. Linear in between.
MG90S = {4.8: (1.8, 0.10), 6.0: (2.2, 0.08)}
_KGCM = 0.0980665   # N*m per kg*cm


def mg90s(volts=5.0) -> HexServoParams:
    """MG90S at `volts` (4.8-6.0). Loop gains scaled with the torque [est]."""
    volts = min(max(volts, 4.8), 6.0)
    f = (volts - 4.8) / 1.2
    stall_kgcm, s60 = (a + f * (b - a) for a, b in zip(MG90S[4.8], MG90S[6.0]))
    base = HexServoParams()
    k = stall_kgcm * _KGCM / base.stall_torque
    return dataclasses.replace(base, stall_torque=stall_kgcm * _KGCM, no_load_speed=math.radians(60) / s60,
                               kp=base.kp * k, kd=base.kd * k, friction=base.friction * k)


@dataclass
class HexapodParams:
    geometry: HexGeometry = field(default_factory=HexGeometry)
    masses: HexMasses = field(default_factory=HexMasses)
    servo: HexServoParams = field(default_factory=lambda: mg90s(5.0))   # Mini360 bucks set to 5 V [design]
    hal: HexHalParams = field(default_factory=HexHalParams)
    env: Environment = field(default_factory=Environment)
    legs: tuple = LEGS
    imu_pos: tuple = (0.0, 0.0, TOP_Z - 3.0)    # GY-91 under the top cover, body centre [design]
    profile: tuple = SERVO_PROFILE


def with_mass(params: HexapodParams, total_g: float) -> HexapodParams:
    """Change the body mass (the torso lump) so the robot weighs `total_g`;
    legs, servos, battery and electronics stay as they are."""
    m = params.masses
    return dataclasses.replace(params, masses=dataclasses.replace(m, torso=m.torso + total_g - m.total()))


def plate_outline():
    """Hexagon of the body plates (x, y) in mm, around the coxa axes."""
    by = {leg.name: leg.body for leg in LEGS}
    pts = []
    for n in ("LF", "LM", "LR", "RR", "RM", "RF"):
        x, y = by[n][0], by[n][1]
        r = math.hypot(x, y)
        pts.append((x + x / r * PLATE_INFLATE, y + y / r * PLATE_INFLATE))
    return pts


# ------------------------------------------------------------ other servos
# PTK 7465MG (Gleb's datasheet): coreless, metal gears, 13 g, 180 deg travel,
# 500-2500 us, digital (up to 333 Hz frames), 3 us deadband. Stall torque and
# speed per supply voltage [ds]:
PTK_7465MG = {
    # volts: (stall kg*cm, s/60 deg)
    5.0: (3.8, 0.110),
    6.0: (4.5, 0.092),
    7.4: (5.5, 0.075),
    8.4: (5.8, 0.070),
}


def ptk_7465mg(volts=7.4, frame_hz=50.0) -> HexServoParams:
    """7465MG servo at `volts` (5.0-8.4, linear in between) reading a pulse
    `frame_hz` times a second (the PCA9685 runs at 50 Hz; the servo takes up
    to 333). The position loop gain saturates at 2 deg of error (digital
    servo, stiffer than the MG90S's 3.4) [est]; damping and friction scale
    with the torque like the MG90S's [est]."""
    vs = sorted(PTK_7465MG)
    volts = min(max(volts, vs[0]), vs[-1])
    hi = next(v for v in vs if v >= volts)
    lo = max(v for v in vs if v <= volts)
    f = 0.0 if hi == lo else (volts - lo) / (hi - lo)
    stall_kgcm, s60 = (a + f * (b - a) for a, b in zip(PTK_7465MG[lo], PTK_7465MG[hi]))
    stall = stall_kgcm * _KGCM
    ref = HexServoParams()
    k = stall / ref.stall_torque
    return HexServoParams(stall_torque=stall, no_load_speed=math.radians(60) / s60,
                          kp=stall / math.radians(2.0), kd=ref.kd * k,
                          deadband_us=3.0, pwm_period=1.0 / frame_hz,
                          armature=ref.armature, friction=ref.friction * k)
