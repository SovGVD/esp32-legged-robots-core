"""Rebuild the whole model, in the order the parts depend on each other.
Printed parts use Params aliases and link to each other, so old parts are cleared
first: a renamed alias would otherwise stop the rebuild. Run inside FreeCAD with execute_code."""
import os
import runpy

HERE = os.path.dirname(os.path.abspath(__file__))
common = runpy.run_path(os.path.join(HERE, "common.py"))
common["remove_built"](common["get_doc"](), ("BodyCheck", "BodyPrim", "Body", "TibiaCheck", "TibiaPrim", "Tibia", "FemurCheck", "FemurPrim", "Femur", "CoxaCheck", "CoxaPrim", "CoxaLink"))
for script in ("build_params.py", "build_dummies.py", "build_femur.py"):
    print("==", script)
    runpy.run_path(os.path.join(HERE, script), run_name="__main__")
