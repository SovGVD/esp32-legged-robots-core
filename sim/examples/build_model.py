"""Regenerate models/dog.xml from legsim/params.py (open it in MuJoCo's `simulate` app)."""
import os, sys
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from legsim.mjcf import build_xml

out = os.path.join(os.path.dirname(__file__), "..", "models", "dog.xml")
with open(out, "w") as f:
    f.write(build_xml(meshdir="meshes"))
print("wrote", os.path.normpath(out))
