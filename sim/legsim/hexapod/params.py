"""
Physical parameters of Gleb's hexapod (6 legs, 3 joints each, geared).

Units follow the firmware: mm, g, degrees/radians as noted. The MJCF builder
converts to SI. Every number says where it comes from:

  [code]  src/esp32-legged-robot/model/hexapod (config.h, model_data.h)
  [cad]   Gleb's printed parts (STL exports of the FreeCAD design), measured:
          axis holes, gear tooth tips, servo screw holes
  [stl]   mass estimated from the STL volume (PLA, ~1 mm shell, 20% infill)
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


# config.h: LEG_BODY_X/Y, LEG_FOOT_X/Y/Z, LEG_MIDDLE_* [code]. The STLs put the coxa
# axes exactly here: bearing holes at (+-46, +-80) and (+-57.5, 0) [cad].
_BX, _BY, _MX = 46.0, 80.0, 57.5
_FX, _FY, _FZ, _MFX = 150.24, 135.31, 95.00, 175.5

# Angle inverse flags: left tibia, right femur (right legs are mirrored) [code]
_INV_L = (False, False, True)
_INV_R = (False, True, False)

# Firmware order: LF, LM, LR, RF, RM, RR [code]
LEGS = (
    HexLegConfig("LF", (-_BX, _BY, 0.0), (-_FX, _FY, -_FZ), inverse_abg=_INV_L),
    HexLegConfig("LM", (-_MX, 0.0, 0.0), (-_MFX, 0.0, -_FZ), inverse_abg=_INV_L),
    HexLegConfig("LR", (-_BX, -_BY, 0.0), (-_FX, -_FY, -_FZ), inverse_abg=_INV_L),
    HexLegConfig("RF", (_BX, _BY, 0.0), (_FX, _FY, -_FZ), inverse_abg=_INV_R),
    HexLegConfig("RM", (_MX, 0.0, 0.0), (_MFX, 0.0, -_FZ), inverse_abg=_INV_R),
    HexLegConfig("RR", (_BX, -_BY, 0.0), (_FX, -_FY, -_FZ), inverse_abg=_INV_R),
)

JOINTS = ("coxa", "femur", "tibia")   # alpha, beta, gamma


@dataclass
class HexGeometry:
    # Link lengths [code], all three confirmed by the parts [cad]:
    #   Coxa: yaw axis x = 57.5 -> femur axis x = 92.5 (35)
    #   Femur/Tibia: femur axis at the origin -> knee axis x = 55 -> tibia tip z = -110
    coxa: float = 35.0
    femur: float = 55.0
    tibia: float = 110.0
    # Body origin (firmware z = 0) is at the femur axis height: 28.7 mm above the
    # bottom of the body [cad]. The body is 131 x 176 mm, 53.5 mm tall.
    femur_axis_z: float = 28.7
    torso_size: tuple = (100.0, 176.0, 53.5)   # collision box: main body without the coxa lobes [cad]
    torso_z: float = -1.95                     # box centre relative to the body origin [cad]
    foot_radius: float = 4.0                   # TibiaTip is 8 mm across, rounded [cad]
    femur_width: float = 8.0                   # collision capsule radius, only the floor collides [est]
    tibia_width: float = 6.0


@dataclass
class HexMasses:
    """Grams. Total ~892 g, a scale reading would replace most of the [est].

    Printed parts are placed as meshes (assembly.py), servos and electronics as
    boxes, so MuJoCo computes centres of mass and inertias from them; whatever
    is left of a link's total (screws, pins, bearings, wires) is a small lump.
    """
    # body top + bottom (124) [stl], 6 coxa servos + gears (86) [ds], 2x18650 + holder (106) [ds, est],
    # electronics (47) [est], servo wires (30) [est], 12 coxa bearings (24) [est]
    torso: float = 417.0
    battery: float = 106.0      # 2x 18650 (~47 g each) + holder [ds, est]
    electronics: float = 47.0   # ESP32-CAM, 2x PCA9685, INA219, MPU9250, bucks, switch [est]
    # Coxa 11.6 + CoxaGear 3.3 [stl] + femur servo 13.4 + servo gear 1.0 [ds] + pins, screws
    coxa: float = 31.3
    # Femur 12.8 [stl] + tibia servo 13.4 + servo gear 1.0 [ds] + bearing, screws
    femur: float = 28.7
    # Tibia_side1 8.8 + Tibia_side2 9.5 + TibiaTip 0.4 [stl] + screws
    tibia: float = 19.2

    def total(self) -> float:
        return self.torso + 6 * (self.coxa + self.femur + self.tibia)


@dataclass
class HexServoParams:
    """MG90S clone (ServoGear_MG90S_clone.stl) at ~5 V [ds, est]. At the servo shaft;
    the joint gears (2:1 coxa and femur, 1.5:1 tibia) are applied on top."""
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


# servoMainProfile in model/hexapod/model_data.h: pulse widths at 0, 30, 50, ..., 150, 180 deg [code]
SERVO_PROFILE_DEG = (0, 30, 50, 70, 90, 110, 130, 150, 180)
SERVO_PROFILE_US = (590, 870, 1080, 1290, 1500, 1710, 1890, 2100, 2200)
SERVO_PROFILE = (SERVO_PROFILE_DEG, SERVO_PROFILE_US)


@dataclass
class HexHalParams:
    """LR_legHal, same for every leg [code]. Gear ratios checked on the parts [cad]:
    18-tooth servo gears (tip r 10, module 1), 36-tooth coxa sector and femur gear
    (tip r ~19), 27-tooth tibia gear (tip r 14.5)."""
    mid: tuple = (math.pi / 2, math.pi / 2, math.pi / 2)
    ratio: tuple = (2.0, 2.0, 1.5)


@dataclass
class HexapodParams:
    geometry: HexGeometry = field(default_factory=HexGeometry)
    masses: HexMasses = field(default_factory=HexMasses)
    servo: HexServoParams = field(default_factory=HexServoParams)
    hal: HexHalParams = field(default_factory=HexHalParams)
    env: Environment = field(default_factory=Environment)
    legs: tuple = LEGS
    imu_pos: tuple = (0.0, 0.0, 0.0)    # mm, MPU9250 position unknown: body centre [est]
    profile: tuple = SERVO_PROFILE
    parts: str = "stl"                  # "stl": Gleb's printed parts (meshes); "simple": boxes (new_body.py)


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
_KGCM = 0.0980665   # N*m per kg*cm


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
    mg90s = HexServoParams()
    k = stall / mg90s.stall_torque
    return HexServoParams(stall_torque=stall, no_load_speed=math.radians(60) / s60,
                          kp=stall / math.radians(2.0), kd=mg90s.kd * k,
                          deadband_us=3.0, pwm_period=1.0 / frame_hz,
                          armature=mg90s.armature, friction=mg90s.friction * k)


# 500-2500 us over 0-180 deg [ds]
LINEAR_500_2500 = ((0, 180), (500, 2500))


def direct_drive(params: HexapodParams = None, servo: HexServoParams = None) -> HexapodParams:
    """A what-if robot: same legs, stance and weight, but every joint driven
    straight by its servo (no gears, so 180 deg of travel per joint).

    The firmware HAL still has the geared ratios, so drive it by leg angles,
    not pulses: FirmwareCore.drive(robot, cmd, ideal_servos=True)."""
    import dataclasses
    p = params or HexapodParams()
    return dataclasses.replace(p, servo=servo or ptk_7465mg(), hal=HexHalParams(ratio=(1.0, 1.0, 1.0)),
                               profile=LINEAR_500_2500)


# MG90S (the datasheet the current robot's clone follows): 1.8 kg*cm, 0.10 s/60 deg
# at 4.8 V; 2.2 kg*cm, 0.08 s/60 deg at 6 V [ds]. Linear in between.
MG90S = {4.8: (1.8, 0.10), 6.0: (2.2, 0.08)}


def mg90s(volts=5.0) -> HexServoParams:
    """MG90S at `volts` (4.8-6.0). Same loop gains as the default servo,
    scaled with the torque [est]."""
    volts = min(max(volts, 4.8), 6.0)
    f = (volts - 4.8) / 1.2
    stall_kgcm, s60 = (a + f * (b - a) for a, b in zip(MG90S[4.8], MG90S[6.0]))
    base = HexServoParams()
    k = stall_kgcm * _KGCM / base.stall_torque
    return dataclasses.replace(base, stall_torque=stall_kgcm * _KGCM, no_load_speed=math.radians(60) / s60,
                               kp=base.kp * k, kd=base.kd * k, friction=base.friction * k)


def with_mass(params: HexapodParams, total_g: float) -> HexapodParams:
    """Change the body mass (the torso lump: screws, wires, bearings, the
    printed halves) so the robot weighs `total_g`; legs, servos, battery and
    electronics stay as they are."""
    m = params.masses
    return dataclasses.replace(params, masses=dataclasses.replace(m, torso=m.torso + total_g - m.total()))
