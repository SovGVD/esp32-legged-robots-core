"""The whole robot standing at the neutral pose (feet 69 mm out, 100 mm down), drawn with
matplotlib into views/robot.png. Run inside FreeCAD after build_body.py: the body and
everything in it come from Body_Check; each right leg's femur and tibia are added along its
joint chain, and the left legs are the right ones mirrored (they are printed that way)."""
import os

import FreeCAD as App
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

HERE = os.path.dirname(os.path.abspath(__file__))
doc = App.getDocument("hexapod")
S = doc.getObject("Params")
LIGHT = np.array([0.35, -0.5, 0.8]) / np.linalg.norm([0.35, -0.5, 0.8])
DEFLECTION = 0.35
FEMUR_DOWN, KNEE = 58.3, -41.6          # neutral pose, as in render_views.py
BLUE, GREEN, PALE, DARK, SERVO, GROUND = (0.2, 0.55, 0.75), (0.35, 0.65, 0.35), (0.72, 0.88, 0.72), (0.25, 0.25, 0.25), (0.16, 0.2, 0.5), (0.85, 0.85, 0.82)


def v(alias):
    x = S.get(alias)
    return float(getattr(x, "Value", x))


Pl = lambda x, y, z, rot=None: App.Placement(App.Vector(x, y, z), rot or App.Rotation())
Ry = lambda deg: App.Rotation(App.Vector(0, 1, 0), deg)
Rz = lambda deg: App.Rotation(App.Vector(0, 0, 1), deg)
KNEE_ROT = App.Rotation(App.Matrix(0, 1, 0, 0, 0, 0, 1, 0, 1, 0, 0, 0, 0, 0, 0, 1))
HIP_ROT = KNEE_ROT.multiply(Rz(180))
SERVO_PARTS = ["MG90S_body", "MG90S_tabs", "MG90S_turret", "MG90S_shaft"]

pieces = []          # (shape in the body frame, colour)


def colour(obj):
    return tuple(obj.ViewObject.ShapeColor[:3])


# the body and everything in it, as Body_Check places them
for link in doc.getObject("Body_Check").Group:
    target = link.LinkedObject
    children = target.Group if target.TypeId == "App::Part" else [target]
    for child in children:
        if not hasattr(child, "Shape") or not child.Shape.Solids or child.Name in ("Bearing_ring", "Bearing_bore"):
            continue
        s = child.Shape.copy()
        s.Placement = link.Placement.multiply(s.Placement)
        pieces.append((s, colour(child)))

# femur and tibia of each right leg, then the same mirrored for the left
femur_at = Pl(v("coxa_len"), 0, 0, Ry(FEMUR_DOWN))
tibia_at = femur_at.multiply(Pl(v("femur_len"), 0, 0, Ry(KNEE)))
s_y = v("joint_seat_y")
LEG = [("Femur_Main", femur_at, BLUE), ("Tibia_Horn", tibia_at, GREEN), ("Tibia_Bearing", tibia_at, PALE), ("Tibia_Presser", tibia_at, DARK)]
for n in SERVO_PARTS:
    LEG.append((n, femur_at.multiply(Pl(0, s_y, 0, HIP_ROT)), SERVO))
    LEG.append((n, femur_at.multiply(Pl(v("femur_len"), s_y, 0, KNEE_ROT)), SERVO))
feet = []
for tag in ("RF", "RM", "RR"):
    coxa = doc.getObject("BodyCheck_coxa_" + tag).Placement
    for name, at, col in LEG:
        s = doc.getObject(name).Shape.copy()
        s.Placement = coxa.multiply(at).multiply(s.Placement)
        pieces.append((s, col))
        pieces.append((s.mirror(App.Vector(0, 0, 0), App.Vector(1, 0, 0)), col))
    foot = coxa.multiply(tibia_at).multVec(App.Vector(0, 0, -v("tibia_len")))
    feet += [foot, App.Vector(-foot.x, foot.y, foot.z)]
ground_z = min(f.z for f in feet)
print("feet:", ", ".join("(%.0f, %.0f, %.0f)" % (f.x, f.y, f.z) for f in feet))

tris, cols = [], []
for shape, col in pieces:
    verts, faces = shape.tessellate(DEFLECTION)
    if not faces:
        continue
    pts = np.array([[p.x, p.y, p.z] for p in verts])
    t = pts[np.array(faces)]
    n = np.cross(t[:, 1] - t[:, 0], t[:, 2] - t[:, 0])
    n /= np.maximum(np.linalg.norm(n, axis=1, keepdims=True), 1e-9)
    tris.append(t)
    cols.append(np.clip(np.outer(0.45 + 0.55 * np.abs(n @ LIGHT), col), 0, 1))
g = 150.0
tris.append(np.array([[[-g, -g, ground_z], [g, -g, ground_z], [g, g, ground_z]], [[-g, -g, ground_z], [g, g, ground_z], [-g, g, ground_z]]]))
cols.append(np.array([GROUND, GROUND]))
tris, cols = np.vstack(tris), np.vstack(cols)
print("triangles:", len(tris))

body = tris[:-2].reshape(-1, 3)
lo, hi = body.min(axis=0), body.max(axis=0)
mid, span = (lo + hi) / 2, (hi - lo).max() / 2 * 0.92
VIEWS = (("front right, above", 25, -55), ("rear left, above", 25, 125), ("front", 0, -90), ("right side", 0, 0), ("top", 90, -90), ("below", -25, -55))
fig = plt.figure(figsize=(21, 14), facecolor="white")
for k, (label, elev, azim) in enumerate(VIEWS):
    ax = fig.add_subplot(2, 3, k + 1, projection="3d")
    c = cols if elev >= 0 else cols[:-2]
    ax.add_collection3d(Poly3DCollection(tris if elev >= 0 else tris[:-2], facecolors=c, edgecolors="none", linewidths=0))
    ax.set_xlim(mid[0] - span, mid[0] + span)
    ax.set_ylim(mid[1] - span, mid[1] + span)
    ax.set_zlim(mid[2] - span, mid[2] + span)
    ax.set_box_aspect((1, 1, 1))
    ax.view_init(elev=elev, azim=azim)
    ax.set_proj_type("ortho")
    ax.set_axis_off()
    ax.set_title(label, fontsize=13)
fig.suptitle("Hexapod, neutral pose: %.0f x %.0f x %.0f mm (feet to the top of the ToF mount)" % (hi[0] - lo[0], hi[1] - lo[1], hi[2] - ground_z), fontsize=15)
fig.subplots_adjust(left=0, right=1, top=0.94, bottom=0, wspace=0, hspace=0.05)
path = os.path.join(HERE, "views", "robot.png")
fig.savefig(path, dpi=80)
plt.close(fig)
print(path)
