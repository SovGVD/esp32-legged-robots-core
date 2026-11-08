"""Cross-sections of the tibia's tip with the switch and the presser in place (foot hanging):
across the split (XZ) and along it (YZ). Draws views/tip_section.png. Run after build_tibia.py."""
import os
import runpy

import FreeCAD as App

HERE = os.path.dirname(os.path.abspath(__file__))
common = runpy.run_path(os.path.join(HERE, globals().get("COMMON", "common.py")))
doc = App.getDocument("hexapod") if App.listDocuments().get("hexapod") else App.openDocument(os.path.join(HERE, "hexapod.FCStd"))
S = doc.getObject("Params")
v = lambda a: float(getattr(S.get(a), "Value", S.get(a)))
solids_of = common["solids_of"]
sw_place = doc.getObject("TibiaCheck_switch").Placement
parts = [("horn half", solids_of(doc, ["Tibia_Horn"]), "#d9a441"), ("bearing half", solids_of(doc, ["Tibia_Bearing"]), "#c98b2e"),
         ("presser", solids_of(doc, ["Tibia_Presser"]), "#5d6d7e"),
         ("switch", solids_of(doc, ["Microswitch_body", "Microswitch_button"], sw_place), "#222")]

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon
import Part

z0, z1 = v("tip_foot_z") - v("tip_foot_r") - 1, v("tip_wire_z0") + 6
fig, axes = plt.subplots(1, 3, figsize=(16, 7))
for ax, (normal, d, title, ix) in zip(axes, ((App.Vector(0, 1, 0), 0.01, "across the split (y 0.01)", 0),
                                             (App.Vector(0, 1, 0), 0.6, "across, y 0.6 (the lead slots)", 0),
                                             (App.Vector(1, 0, 0), 0.05, "along the split (x 0.05)", 1))):
    for name, solids, col in parts:
        for s in solids:
            # the cut as filled faces (holes open), drawn as triangles: no outline tricks
            box = Part.makeBox(40, 40, 120, App.Vector(-20, -20, -100))
            plane = Part.makePlane(200, 200, App.Vector(-100, d, 50) if ix == 0 else App.Vector(d, -100, -150),
                                   normal, App.Vector(1, 0, 0) if ix == 0 else App.Vector(0, 1, 0))
            cut = s.common(plane)
            for f in cut.Faces:
                pts, tris = f.tessellate(0.02)
                for t in tris:
                    ax.add_patch(Polygon([((pts[k].x if ix == 0 else pts[k].y), pts[k].z) for k in t], closed=True,
                                         facecolor=col, edgecolor=col, lw=0.2))
                for w in f.Wires:
                    q = [((p_.x if ix == 0 else p_.y), p_.z) for p_ in w.discretize(Distance=0.05)]
                    ax.plot([c[0] for c in q], [c[1] for c in q], color="k", lw=0.4)
    ax.set_xlim(-9, 9)
    ax.set_ylim(z0, z1)
    ax.set_aspect("equal")
    ax.grid(alpha=0.25)
    ax.set_title(title, fontsize=10)
    ax.set_xlabel("x, mm" if ix == 0 else "y, mm")
fig.suptitle("Tibia tip, foot hanging: the old hexapod's switch pocket, lead slots, pin and TibiaTip presser, copied as they are", fontsize=10)
path = os.path.join(HERE, "views", "tip_section.png")
fig.savefig(path, dpi=110, bbox_inches="tight")
print(path)
