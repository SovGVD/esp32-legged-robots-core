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

z0, z1 = v("tip_foot_z") - v("tip_foot_r") - 1, v("switch_top_z") + 3
fig, axes = plt.subplots(1, 2, figsize=(12, 7))
for ax, (normal, title, ix) in zip(axes, ((App.Vector(0, 1, 0.0), "across the split (cut at y 0.01)", 0),
                                          (App.Vector(1, 0, 0), "along the split (cut at x 0)", 1))):
    d = 0.01 if ix == 0 else 0.0
    for name, solids, col in parts:
        for s in solids:
            for w in s.slice(normal, d):
                pts = [(p.x if ix == 0 else p.y, p.z) for p in w.discretize(Distance=0.1)]
                ax.add_patch(Polygon(pts, closed=True, facecolor=col, edgecolor="k", lw=0.4, alpha=0.9))
    ax.set_xlim(-9, 9)
    ax.set_ylim(z0, z1)
    ax.set_aspect("equal")
    ax.grid(alpha=0.25)
    ax.set_title(title, fontsize=10)
    ax.set_xlabel("x, mm" if ix == 0 else "y, mm")
fig.suptitle("Tibia tip, foot hanging: gap %.2f to the button, presses it %.2f, then the flange stops on the plastic\n"
             "stem %.1f in a %.2f hole, flange %.1f in %.2f; switch ledges %.2f a side" % (
                 v("tip_gap"), v("tip_press"), v("tip_stem_w"), v("tip_hole_w"), v("tip_flange_x"), v("tip_cavity_x"),
                 v("sw_w") / 2 - v("tip_hole_w") / 2), fontsize=10)
path = os.path.join(HERE, "views", "tip_section.png")
fig.savefig(path, dpi=110, bbox_inches="tight")
print(path)
