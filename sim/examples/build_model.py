"""Regenerate models/dog.xml and models/hexapod.xml from the params (open them in MuJoCo's `simulate` app)."""
import os, sys
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from legsim.mjcf import build_xml
from legsim.hexapod.mjcf import build_xml as build_hexapod_xml

models = os.path.join(os.path.dirname(__file__), "..", "models")
for name, xml in (("dog.xml", build_xml(meshdir="meshes")), ("hexapod.xml", build_hexapod_xml(meshdir="meshes/hexapod"))):
    out = os.path.join(models, name)
    with open(out, "w") as f:
        f.write(xml)
    print("wrote", os.path.normpath(out))
