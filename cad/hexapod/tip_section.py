"""Cross-sections of the tibia's tip with the switch and the presser: the foot hanging (the
presser resting on its pin) and pressed (pushed up until it first touches the tibia), across
the split (XZ) and along it (YZ). Measures the gap to the button and how far it gets pushed.
Draws views/tip_section.png. Run after build_tibia.py."""
import os
import runpy

import FreeCAD as App
import Part

HERE = os.path.dirname(os.path.abspath(__file__))
common = runpy.run_path(os.path.join(HERE, globals().get("COMMON", "common.py")))
doc = App.getDocument("hexapod") if App.listDocuments().get("hexapod") else App.openDocument(os.path.join(HERE, "hexapod.FCStd"))
S = doc.getObject("Params")
v = lambda a: float(getattr(S.get(a), "Value", S.get(a)))
solids_of, clash = common["solids_of"], common["clash"]
sw_place = doc.getObject("TibiaCheck_switch").Placement
halves = solids_of(doc, ["Tibia_Horn"]) + solids_of(doc, ["Tibia_Bearing"])
button = solids_of(doc, ["Microswitch_button"], sw_place)
switch = solids_of(doc, ["Microswitch_body", "Microswitch_button"], sw_place)
presser = solids_of(doc, ["Tibia_Presser"])


def moved(shapes, dz):
    out = []
    for s in shapes:
        c = s.copy()
        c.translate(App.Vector(0, 0, dz))
        out.append(c)
    return out


def first(test, lo, hi, step):
    """The first offset from lo toward hi where test(offset) is True, to 0.005 mm."""
    z = lo
    while (z < hi if step > 0 else z > hi) and not test(z):
        z += step
    a, b = z - step, z
    for _ in range(8):
        m = (a + b) / 2
        a, b = (a, m) if test(m) else (m, b)
    return b


# hanging: lowered until its slot sits on the pin (just before they would overlap)
hang = first(lambda z: clash(moved(presser, z), halves) > 1e-3, 0.0, -2.0, -0.01) + 0.01
# pressed: raised from there until it touches the tibia again
stop = first(lambda z: clash(moved(presser, hang + z), halves) > 1e-3, 0.0, 3.0, 0.02)
touch = first(lambda z: clash(moved(presser, hang + z), button) > 1e-4, 0.0, 3.0, 0.02)
push = stop - touch
hit = [h for h in halves if moved(presser, hang + stop + 0.01)[0].common(h).Volume > 1e-4]
where = Part.makeCompound([moved(presser, hang + stop + 0.01)[0].common(h) for h in hit]).BoundBox
print("it stops on the tibia at z %.2f..%.2f, |x| up to %.2f" % (where.ZMin, where.ZMax, max(abs(where.XMin), abs(where.XMax))))
print("presser hangs %.2f below its modelled place; rises %.2f until it stops on the tibia; meets the button after %.2f: "
      "pushes it %.2f (button travel %.2f)" % (-hang, stop, touch, push, v("sw_travel")))

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon

z0, z1 = v("tip_foot_z") - v("tip_foot_r") - 1, v("tip_wire_z0") + 6
cuts = ((App.Vector(0, 1, 0), 0.01, "across the split (y 0.01)", 0), (App.Vector(1, 0, 0), 0.05, "along the split (x 0.05)", 1))
states = (("foot hanging: %.2f to the button" % touch, hang), ("pressed %.2f: the button pushed %.2f of its %.2f" % (stop, push, v("sw_travel")), hang + stop))
fig, axes = plt.subplots(2, 2, figsize=(12, 14))
for r, (state, dz) in enumerate(states):
    parts = [(halves, "#d9a441"), (moved(presser, dz), "#5d6d7e"), (switch, "#222")]
    for c, (normal, d, title, ix) in enumerate(cuts):
        ax = axes[r][c]
        for solids, col in parts:
            for s in solids:
                # the cut as filled faces (holes stay open), drawn as triangles
                plane = Part.makePlane(200, 200, App.Vector(-100, d, 50) if ix == 0 else App.Vector(d, -100, -150),
                                       normal, App.Vector(1, 0, 0) if ix == 0 else App.Vector(0, 1, 0))
                for f in s.common(plane).Faces:
                    pts, tris = f.tessellate(0.02)
                    for t in tris:
                        ax.add_patch(Polygon([((pts[k].x if ix == 0 else pts[k].y), pts[k].z) for k in t], closed=True,
                                             facecolor=col, edgecolor=col, lw=0.2, alpha=0.85 if col == "#5d6d7e" else 1))
                    for w in f.Wires:
                        q = [((p_.x if ix == 0 else p_.y), p_.z) for p_ in w.discretize(Distance=0.05)]
                        ax.plot([p_[0] for p_ in q], [p_[1] for p_ in q], color="k", lw=0.4)
        ax.set_xlim(-9, 9)
        ax.set_ylim(z0, z1)
        ax.set_aspect("equal")
        ax.grid(alpha=0.25)
        ax.set_title("%s\n%s" % (state, title), fontsize=9)
        ax.set_xlabel("x, mm" if ix == 0 else "y, mm")
fig.suptitle("Tibia tip: the old hexapod's switch pocket, lead slot, pin and TibiaTip presser (foot made %.0f)" % v("tip_foot_r"), fontsize=10)
path = os.path.join(HERE, "views", "tip_section.png")
fig.savefig(path, dpi=100, bbox_inches="tight")
print(path)
