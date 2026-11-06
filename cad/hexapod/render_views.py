"""Pictures of parts from all around, into views/ (not in git), drawn straight from the
geometry with matplotlib, so the FreeCAD window isn't touched. Run inside FreeCAD; each
target is a list of (object name, placement or None, colour)."""
import os

import FreeCAD as App
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, "views")
os.makedirs(OUT, exist_ok=True)
doc = App.getDocument("hexapod")
SIDES = (("above, right", 28, -60), ("above, left", 28, 120), ("below", -30, -60), ("front", 0, -90), ("rear", 0, 90),
         ("right", 0, 0), ("left", 0, 180), ("top", 90, -90))
LIGHT = np.array([0.35, -0.5, 0.8])
LIGHT = LIGHT / np.linalg.norm(LIGHT)


def triangles(shape, placement):
    s = shape.copy()
    if placement is not None:
        s.Placement = placement.multiply(s.Placement)
    verts, faces = s.tessellate(0.15)
    v = np.array([[p.x, p.y, p.z] for p in verts])
    return v[np.array(faces)]


def draw(name, items):
    tris, cols = [], []
    for obj_name, placement, color in items:
        t = triangles(doc.getObject(obj_name).Shape, placement)
        n = np.cross(t[:, 1] - t[:, 0], t[:, 2] - t[:, 0])
        n = n / np.maximum(np.linalg.norm(n, axis=1, keepdims=True), 1e-9)
        shade = 0.45 + 0.55 * np.abs(n @ LIGHT)
        tris.append(t)
        cols.append(np.clip(np.outer(shade, color), 0, 1))
    tris, cols = np.vstack(tris), np.vstack(cols)
    lo, hi = tris.reshape(-1, 3).min(axis=0), tris.reshape(-1, 3).max(axis=0)
    mid, span = (lo + hi) / 2, (hi - lo).max() / 2
    fig = plt.figure(figsize=(18, 9), facecolor="white")
    for k, (label, elev, azim) in enumerate(SIDES):
        ax = fig.add_subplot(2, 4, k + 1, projection="3d")
        ax.add_collection3d(Poly3DCollection(tris, facecolors=cols, edgecolors="none", linewidths=0))
        ax.set_xlim(mid[0] - span, mid[0] + span)
        ax.set_ylim(mid[1] - span, mid[1] + span)
        ax.set_zlim(mid[2] - span, mid[2] + span)
        ax.set_box_aspect((1, 1, 1))
        ax.view_init(elev=elev, azim=azim)
        ax.set_proj_type("ortho")
        ax.set_axis_off()
        ax.set_title(label, fontsize=11)
    fig.suptitle("%s, %.0f x %.0f x %.0f mm" % (name, hi[0] - lo[0], hi[1] - lo[1], hi[2] - lo[2]), fontsize=13)
    fig.subplots_adjust(left=0, right=1, top=0.93, bottom=0, wspace=0, hspace=0.04)
    path = os.path.join(OUT, name.replace(" ", "_") + ".png")
    fig.savefig(path, dpi=85)
    plt.close(fig)
    return path


GOLD, BLUE, GREEN, PALE, GREY, DARK, SERVO, SILVER = (0.88, 0.63, 0.11), (0.2, 0.55, 0.75), (0.35, 0.65, 0.35), (0.72, 0.88, 0.72), (0.8, 0.8, 0.77), (0.25, 0.25, 0.25), (0.16, 0.2, 0.5), (0.75, 0.75, 0.78)
Pl = lambda x, y, z, rot=None: App.Placement(App.Vector(x, y, z), rot or App.Rotation())
Ry = lambda deg: App.Rotation(App.Vector(0, 1, 0), deg)
Rz = lambda deg: App.Rotation(App.Vector(0, 0, 1), deg)
S = doc.getObject("Params")


def v(alias):
    x = S.get(alias)
    return float(getattr(x, "Value", x))


KNEE = App.Rotation(App.Matrix(0, 1, 0, 0,  0, 0, 1, 0,  1, 0, 0, 0,  0, 0, 0, 1))
HIP = KNEE.multiply(Rz(180))
SERVO_PARTS = ["MG90S_body", "MG90S_tabs", "MG90S_turret", "MG90S_shaft"]
# a whole right leg at the neutral pose: foot 69 mm out, 100 mm down
femur_at = Pl(v("coxa_len"), 0, 0, Ry(58.3))
tibia_at = femur_at.multiply(Pl(v("femur_len"), 0, 0, Ry(-41.6)))
s_y = v("joint_seat_y")
leg = [("Coxa_Main", None, GOLD), ("Femur_Main", femur_at, BLUE), ("Tibia_Horn", tibia_at, GREEN), ("Tibia_Bearing", tibia_at, PALE), ("Tibia_Presser", tibia_at, DARK)]
for n in SERVO_PARTS:
    leg.append((n, Pl(0, 0, v("coxa_seat_z"), Rz(90)), SERVO))
    leg.append((n, femur_at.multiply(Pl(0, s_y, 0, HIP)), SERVO))
    leg.append((n, femur_at.multiply(Pl(v("femur_len"), s_y, 0, KNEE)), SERVO))
TARGETS = {
    "coxa": [("Coxa_Main", None, GOLD)],
    "femur": [("Femur_Main", None, BLUE)],
    "tibia": [("Tibia_Horn", None, GREEN), ("Tibia_Bearing", None, PALE), ("Tibia_Presser", None, DARK)],
    "whole leg, neutral pose": leg,
    "body top": [("Body_Top", None, GREY)],
    "body bottom": [("Body_Bottom", None, GREY)],
    "tof mount": [("Body_ToF_Mount", None, GREY)],
}
check_pl = lambda link: doc.getObject("BodyCheck_" + link).Placement
front = [("Body_Top", None, GREY), ("Body_ToF_Mount", None, (0.55, 0.55, 0.6))]
for link, part_name in (("tof_l", "Dummy_VL53L5CX"), ("tof_r", "Dummy_VL53L5CX"), ("rocker", "Dummy_Rocker")):
    for child in doc.getObject(part_name).Group:
        if hasattr(child, "Shape") and child.Shape.Solids:
            front.append((child.Name, check_pl(link), tuple(child.ViewObject.ShapeColor[:3])))
for link in ("coxa_RF", "coxa_LF"):
    front.append((doc.getObject("BodyCheck_" + link).LinkedObject.Name, check_pl(link), GOLD))
TARGETS["top plate with ToF mount, switch, front coxas"] = front
cart = [("Battery_Cartridge", None, (0.55, 0.6, 0.75)), ("Battery_Latch", check_pl("latch_f"), DARK), ("Battery_Latch", check_pl("latch_r"), DARK)]
for link, part_name in (("cells", "Dummy_18650_pair"), ("xt30", "Dummy_XT30")):
    for child in doc.getObject(part_name).Group:
        if hasattr(child, "Shape") and child.Shape.Solids:
            cart.append((child.Name, check_pl(link), tuple(child.ViewObject.ShapeColor[:3])))
TARGETS["battery cartridge with holder, XT30 and latches"] = cart
TARGETS["battery cartridge alone"] = [("Battery_Cartridge", None, (0.55, 0.6, 0.75))]
TARGETS["bottom plate with cartridge in"] = [("Body_Bottom", None, GREY), ("Battery_Cartridge", None, (0.55, 0.6, 0.75)),
                                           ("Battery_Latch", check_pl("latch_f"), DARK), ("Battery_Latch", check_pl("latch_r"), DARK)]
TARGETS["fit test"] = [(o.Name, None, (0.85, 0.55, 0.2)) for o in doc.getObject("FitTest").Group] if doc.getObject("FitTest") else []
if globals().get("ONLY"):
    TARGETS = {k: TARGETS[k] for k in ONLY}
for name, items in TARGETS.items():
    print(draw(name, items))
print("foot at", tibia_at.multVec(App.Vector(0, 0, -v("tibia_len"))))
