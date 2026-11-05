"""Rebuild the whole model, in the order the parts depend on each other.
Coxa, femur and tibia each link to the ones before them, so rebuilding one means
rebuilding those after it. Run inside FreeCAD with execute_code."""
import os
import runpy

HERE = os.path.dirname(os.path.abspath(__file__))
for script in ("build_params.py", "build_dummies.py", "build_coxa.py", "build_femur.py", "build_tibia.py"):
    print("==", script)
    runpy.run_path(os.path.join(HERE, script), run_name="__main__")
