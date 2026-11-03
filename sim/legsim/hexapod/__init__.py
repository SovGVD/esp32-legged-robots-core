"""MuJoCo simulation of the ESP32 hexapod (6 legs x 3 geared joints)."""
from .params import HexapodParams, LEGS
from .robot import SimHexapod
from .mjcf import build_xml
