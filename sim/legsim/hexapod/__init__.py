"""MuJoCo simulation of the ESP32 hexapod (6 legs x 3 joints, MG90S direct drive)."""
from .params import HexapodParams, LEGS, mg90s, ptk_7465mg
from .robot import SimHexapod
from .mjcf import build_xml
