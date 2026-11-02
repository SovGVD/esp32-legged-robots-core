"""
Physical parameters of the ESP32 small robot dog.

Units follow the firmware: mm, g, degrees/radians as noted. The MJCF builder
converts to SI. Every number says where it comes from:

  [code]  src/esp32-legged-robot/model/dog/model.h (development branch)
  [stl]   mass/size estimated from the Thingiverse 4822059 STLs (PLA, 20% infill)
  [ds]    TowerPro MG90D datasheet (typical values, not measured on this robot)
  [est]   engineering guess, tune against the real robot
"""

from dataclasses import dataclass, field
import math


@dataclass(frozen=True)
class LegConfig:
    """One leg, mirrors the `leg` struct in model.h."""
    name: str
    side: int            # -1 left, +1 right (sign of body X)
    end: int             # +1 front, -1 hind (sign of body Y)
    pins: tuple          # alpha, beta, gamma servo GPIO [code]
    inverse_xyz: tuple   # LR_legInverse x, y, z [code]
    inverse_abg: tuple   # LR_legInverse alpha, beta, gamma [code]


# Leg order is the firmware order: LF=0, RF=1, LH=2, RH=3 [code]
LEGS = (
    LegConfig("LF", -1, +1, (25, 26, 27), (True, True, False), (False, True, True)),
    LegConfig("RF", +1, +1, (16, 18, 17), (False, True, False), (True, False, False)),
    LegConfig("LH", -1, -1, (13, 12, 14), (True, True, False), (True, True, True)),
    LegConfig("RH", +1, -1, (4, 2, 15), (False, True, False), (False, False, False)),
)

JOINTS = ("alpha", "beta", "gamma")  # hip roll, hip pitch, knee


@dataclass
class Geometry:
    # Hip (shoulder) positions, body frame X right, Y forward, Z up [code]
    hip_x: float = 39.5
    hip_y_front: float = 70.5   # "not sure" in model.h
    hip_y_hind: float = 70.5
    hip_z: float = 0.0
    # Default (neutral) foot point [code]
    foot_x: float = 56.0
    foot_y_front: float = 70.5
    foot_y_hind: float = 70.5
    foot_z: float = 80.0        # body height above the foot point
    # Link lengths [code]
    l1: float = 0.0             # hip offset; 0 in the firmware IK
    l2: float = 51.9            # upper leg (hip pitch axis to knee axis)
    l3: float = 50.0            # lower leg (knee axis to foot point)
    # Collision/visual shapes [stl, est]
    torso_size: tuple = (94.0, 190.0, 32.0)   # full size x, y, z (legs holder width, overall length)
    torso_z_offset: float = 4.0               # torso box centre above the hip axes
    foot_radius: float = 5.5                  # TPU shoe, ~11 mm across [stl]
    upper_leg_width: float = 6.5              # leg_top thickness [stl]
    lower_leg_width: float = 5.8              # leg_bottom thickness [stl]


@dataclass
class Masses:
    """Grams. Total ~435 g, see /mnt/project-files/sim/dog-hardware-sources.md."""
    # Torso: 4 hip-roll servos (54), 2x18650 + holder (107), electronics (30),
    # wiring/screws/bearings (15), legs holders + body + cover + clamps (~74) [stl, ds, est]
    torso: float = 280.0
    torso_com: tuple = (0.0, 0.0, -6.0)       # mm; battery under the body pulls it down [est]
    # Shoulder (rotates with hip roll): shoulder parts 5.3 + servo gear 1.2 + hip-pitch servo 13.4 [stl, ds]
    shoulder: float = 20.0
    # Upper leg: leg_top 2.8 + horns/screws [stl]
    upper_leg: float = 3.5
    # Lower leg: leg_bottom 2.2 + knee servo 13.4 + shoe 0.5 [stl, ds]; the article
    # puts the knee servo housing in the lower leg, so its CoM is close to the knee.
    lower_leg: float = 16.0
    lower_leg_com_from_knee: float = 12.0     # mm along the lower leg [est]

    def total(self) -> float:
        return self.torso + 4 * (self.shoulder + self.upper_leg + self.lower_leg)


@dataclass
class ServoParams:
    """MG90D at ~5 V (Mini360 bucks, voltage not measured) [ds, est].

    Values are at the servo output shaft; the 2:1 hip-roll gear is applied on top.
    """
    stall_torque: float = 0.21          # N*m (2.1-2.2 kg*cm at 4.8-5 V) [ds]
    no_load_speed: float = math.radians(60) / 0.10   # rad/s (0.10 s/60 deg at 4.8 V) [ds]
    kp: float = 3.0                     # N*m/rad: saturates at ~4 deg error, typical for hobby servos [est]
    kd: float = 0.004                   # N*m*s/rad, internal damping of the control loop [est]
    deadband_us: float = 5.0            # MG90 class deadband (~0.5 deg) [ds]
    pwm_period: float = 0.020           # s, 50 Hz servo frame; the servo samples the pulse once per frame
    armature: float = 2.0e-5            # kg*m^2 reflected rotor inertia at the output shaft [est]
    friction: float = 0.004             # N*m Coulomb friction at the output shaft (gear train) [est]
    min_angle: float = 20.0             # deg, usable range from servoMainProfile [code]
    max_angle: float = 160.0


# servoMainProfile in model.h: pulse widths at 20, 30, 50, ..., 150, 160 deg [code]
SERVO_PROFILE_DEG = (20, 30, 50, 70, 90, 110, 130, 150, 160)
SERVO_PROFILE_US = (780, 890, 1090, 1300, 1500, 1700, 1900, 2110, 2200)


@dataclass
class HalParams:
    """LR_legHal per joint (same for every leg in model.h) [code]."""
    mid: tuple = (math.pi / 2, math.pi / 2, math.pi / 2)
    ratio: tuple = (2.0, 1.0, 1.0)   # 2:1 on hip roll, confirmed by Gleb
    # Calibrated trims from the model.h comment, degrees, per leg (LF, RF, LH, RH).
    # They compensate horn misalignment on the real robot; the sim robot has no
    # misalignment, so they default to zero and are only kept for reference.
    real_trim_deg: tuple = (
        (-11.000790, -3.093972, 2.979381),
        (-0.572958, 4.927437, -3.093972),
        (9.969466, 0.000000, 2.979381),
        (4.927437, 0.000000, -7.104677),
    )


@dataclass
class Environment:
    timestep: float = 0.0005            # s, physics step
    foot_friction: float = 0.8          # TPU shoe on a hard floor [est]
    floor_friction: float = 0.8
    joint_limit_margin: float = 0.0


@dataclass
class DogParams:
    geometry: Geometry = field(default_factory=Geometry)
    masses: Masses = field(default_factory=Masses)
    servo: ServoParams = field(default_factory=ServoParams)
    hal: HalParams = field(default_factory=HalParams)
    env: Environment = field(default_factory=Environment)
    legs: tuple = LEGS
    imu_pos: tuple = (0.0, 0.0, 6.0)    # mm, MPU9250 on the perfboard in the middle frame [est]
