"""Rebuild the whole model, in the order the parts depend on each other.
Coxa, femur and tibia each link to the ones before them, and all of them use
Params aliases, so old parts are cleared first: a renamed alias would otherwise
stop the rebuild. Run inside FreeCAD with execute_code."""
import os
import runpy

HERE = os.path.dirname(os.path.abspath(__file__))
common = runpy.run_path(os.path.join(HERE, "common.py"))
common["remove_built"](common["get_doc"](), ("Body", "Tibia", "Femur", "CoxaLink", "CoxaPrim", "CoxaCheck"))
for script in ("build_params.py", "build_dummies.py", "build_coxa.py", "build_femur.py", "build_tibia.py", "build_body.py"):
    print("==", script)
    runpy.run_path(os.path.join(HERE, script), run_name="__main__")
