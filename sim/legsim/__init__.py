"""MuJoCo simulation of the ESP32 small robot dog."""
from .params import DogParams, LEGS
from .robot import SimDog
from .mjcf import build_xml
