"""
Copy the hexapod parts used by the sim from Gleb's STL exports into
models/meshes/hexapod/, decimated so the repo stays small.

  python tools/prepare_hexapod_meshes.py /path/to/hexapod/stl

The STLs keep their FreeCAD frames (mm); legsim/hexapod/assembly.py places them.
Needs `pip install trimesh fast-simplification`.
"""
import os
import sys

import numpy as np
import trimesh

PARTS = {
    # sim name: (source file, max faces)
    "body_bottom": ("tmp-hexapod-Body-bottom.stl", 12000),
    "body_top": ("tmp-hexapod-Body-top.stl", 12000),
    "coxa": ("tmp-hexapod-Coxa.stl", 8000),
    "coxa_gear": ("tmp-hexapod-CoxaGear.stl", 3000),
    "femur": ("tmp-Femur.stl", 8000),
    "tibia_side1": ("tmp-hexapod-Tibia_side1.stl", 5000),
    "tibia_side2": ("tmp-hexapod-Tibia_side2.stl", 5000),
    "tibia_tip": ("tmp-hexapod-TibiaTip.stl", 1500),
    "servo_gear": ("hexapod-ServoGear_MG90S_clone.stl", 1500),
}

src = sys.argv[1]
out = os.path.join(os.path.dirname(__file__), "..", "models", "meshes", "hexapod")
os.makedirs(out, exist_ok=True)
for name, (file, faces) in PARTS.items():
    m = trimesh.load(os.path.join(src, file))
    m.merge_vertices()
    n0 = len(m.faces)
    if n0 > faces:
        import fast_simplification
        # agg 5: the screw holes and thin walls keep it above the target, but higher
        # values pull spikes out of the flat plates
        v, f = fast_simplification.simplify(m.vertices, m.faces, target_count=faces, agg=5)
        m = trimesh.Trimesh(v, f)
    path = os.path.join(out, name + ".stl")
    m.export(path)
    print(f"{name}: {n0} -> {len(m.faces)} faces, {os.path.getsize(path) // 1024} KB, bbox {np.round(m.bounds, 1).tolist()}")
