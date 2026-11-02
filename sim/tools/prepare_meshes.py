"""
Copy the printable parts used by the sim from the Thingiverse 4822059 STLs
into models/meshes/, decimated so the repo stays small.

  python tools/prepare_meshes.py /path/to/thingiverse-4822059/files

The STLs are kept in their original part frames (mm); legsim/mjcf.py places them.
"""
import os
import sys

import numpy as np
import trimesh

PARTS = {
    # part: max faces
    "leg_top": 9000,
    "leg_bottom": 8000,
    "leg_shoes_print_with_flexible": 4300,
    "shoulder_part1": 7400,
    "shoulder_part2": 6100,
    "servo_gear": 2400,
    "legs_holder_part1": 12000,
    "legs_holder_part2": 11700,
    "body": 10000,
}

src = sys.argv[1]
out = os.path.join(os.path.dirname(__file__), "..", "models", "meshes")
os.makedirs(out, exist_ok=True)
for name, faces in PARTS.items():
    m = trimesh.load(os.path.join(src, name + ".stl"))
    n0 = len(m.faces)
    if n0 > faces:
        import fast_simplification
        v, f = fast_simplification.simplify(m.vertices, m.faces, target_reduction=1 - faces / n0)
        m = trimesh.Trimesh(v, f)
    path = os.path.join(out, name + ".stl")
    m.export(path)
    print(f"{name}: {n0} -> {len(m.faces)} faces, {os.path.getsize(path) // 1024} KB, bbox {np.round(m.bounds, 1).tolist()}")
