"""
The new hexapod (design "Recommended", 2026-11-04), not built yet: 3D printed
hexagon body, 18 MG90S driving the joints directly (no printed gears), 2x 18650
at the bottom centre, a PCA9685 + 3 Mini360 board standing along each side, the
ESP32 standing across the front, GY-91 under the top cover in the middle,
2 VL53L5CX at the front, lever switch feet.

Firmware: src/esp32-legged-robot/model/hexapod2 (MODEL_HEXAPOD2). There are
no CAD parts yet, so the sim model is made of simple shapes (mjcf.py builds
them when params.parts == "simple"), sized like the design concept (artifact
"Direct-drive hexapod concept"):

  legs      coxa 60, femur 50, tibia 60 mm; neutral foot 69 mm out from the
            coxa axis, 100 mm below it [code]
  body      coxa axes (+-35, +-58) and (+-45, 0) [code]; plates are the hexagon
            through them, 14 mm bigger all round [design]
  mass      ~815 g [design estimate, sim/tools/hexapod_leg_sweep.py]

Servo directions follow the firmware's inverse flags (hal_mapping): the
servos aren't mounted yet, so there is nothing to check them against.
"""

import dataclasses
import math

from .params import (HexapodParams, HexLegConfig, HexGeometry, HexMasses, HexHalParams,
                     LINEAR_500_2500, mg90s)

# config.h of model/hexapod2 [code]
FRONT_BODY, FRONT_FOOT = (35.0, 58.0), (88.71, 101.32)
REAR_BODY, REAR_FOOT = (35.0, -58.0), (70.65, -117.08)
MIDDLE_BODY, MIDDLE_FOOT = (45.0, 0.0), (114.0, 0.0)
FOOT_Z = 100.0

# model_data.h: right legs mirrored, femur and tibia servos turn the other way [code, assumed]
_INV_L = (False, False, False)
_INV_R = (False, True, True)


def _leg(name, body, foot, right):
    sx = 1 if right else -1
    return HexLegConfig(name, (sx * body[0], body[1], 0.0), (sx * foot[0], foot[1], -FOOT_Z),
                        inverse_abg=_INV_R if right else _INV_L)


LEGS = (
    _leg("LF", FRONT_BODY, FRONT_FOOT, False),
    _leg("LM", MIDDLE_BODY, MIDDLE_FOOT, False),
    _leg("LR", REAR_BODY, REAR_FOOT, False),
    _leg("RF", FRONT_BODY, FRONT_FOOT, True),
    _leg("RM", MIDDLE_BODY, MIDDLE_FOOT, True),
    _leg("RR", REAR_BODY, REAR_FOOT, True),
)

PLATE_INFLATE = 14.0    # body plates reach 14 mm past the coxa axes [design]
BOTTOM_Z = -28.0        # bottom plate under the femur axis (body origin) [design]
TOP_Z = 16.0            # top cover [design]


def geometry():
    return HexGeometry(coxa=60.0, femur=50.0, tibia=60.0, femur_axis_z=-BOTTOM_Z,
                       torso_size=(110.0, 150.0, TOP_Z + 3 - BOTTOM_Z),
                       torso_z=(TOP_Z + 3 + BOTTOM_Z) / 2,
                       foot_radius=5.0,       # lever switch pad [est]
                       femur_width=7.0, tibia_width=5.0)


def masses(total_g=815.0):
    """Grams. Legs: coxa bracket 8 + femur servo 13.4; femur 7 + tibia servo 13.4;
    tibia 6 + lever switch 2 [est]. Body: the rest (6 coxa servos, cells, boards,
    plates, wires), so the robot weighs total_g."""
    m = HexMasses(torso=0.0, battery=100.0, electronics=55.0, coxa=21.4, femur=20.4, tibia=8.0)
    return dataclasses.replace(m, torso=total_g - 6 * (m.coxa + m.femur + m.tibia))


def params(volts=5.0, total_g=815.0) -> HexapodParams:
    """The new hexapod with MG90S at `volts` (4.8-6.0)."""
    return HexapodParams(geometry=geometry(), masses=masses(total_g), servo=mg90s(volts),
                         hal=HexHalParams(ratio=(1.0, 1.0, 1.0)), legs=LEGS,
                         imu_pos=(0.0, 0.0, TOP_Z - 3.0), profile=LINEAR_500_2500, parts="simple")


def plate_outline():
    """Hexagon of the body plates (x, y) in mm, around the coxa axes."""
    order = ("LF", "LM", "LR", "RR", "RM", "RF")
    by = {leg.name: leg.body for leg in LEGS}
    pts = []
    for n in order:
        x, y = by[n][0], by[n][1]
        r = math.hypot(x, y)
        pts.append((x + x / r * PLATE_INFLATE, y + y / r * PLATE_INFLATE))
    return pts
