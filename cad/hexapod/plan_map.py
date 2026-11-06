"""Top-down map between the plates: where the coxa links and their servos swing (+-40 deg),
where the electronics stand, and the path an inner wall around the electronics could take.
Run inside FreeCAD after build_body.py; draws views/tub_plan.png and prints the conflicts.

Everything is flattened to 2D: each solid is sliced at a few heights between the plates and
the slices are joined; a coxa's outline is then turned through its swing."""
import math
import os
import runpy

import FreeCAD as App
import Part

HERE = os.path.dirname(os.path.abspath(__file__))
common = runpy.run_path(os.path.join(HERE, "common.py"))
doc = App.getDocument("hexapod")
S = doc.getObject("Params")


def v(alias):
    x = S.get(alias)
    return float(getattr(x, "Value", x))


top_bot, top0 = v("body_plate_top"), v("top_plate_z0") - v("rib_h")     # the free height: plate top to the rib bottoms
LEVELS = [top_bot + 0.3 + k * (top0 - top_bot - 0.6) / 6 for k in range(7)]
CLEAR = float(globals().get("CLEAR", 1.5))


def flat(solid):
    """A slab's outline as a face (with holes) at z 0."""
    wires = solid.slice(App.Vector(0, 0, 1), 0.5)
    face = Part.makeFace(wires, "Part::FaceMakerBullseye")
    face.translate(App.Vector(0, 0, -0.5))
    return face


def slab(face):
    return face.extrude(App.Vector(0, 0, 1))


def footprint(shapes):
    """The outline from above of some solids between the plates, as a 1 mm slab at z 0..1: each solid
    sliced at a few heights, the slices made slabs and joined (joining flat faces only stacks them)."""
    slabs = []
    for sh in shapes:
        for z in LEVELS:
            if not (sh.BoundBox.ZMin < z < sh.BoundBox.ZMax):
                continue
            for w in sh.slice(App.Vector(0, 0, 1), z):
                if w.isClosed():
                    try:
                        f = Part.Face(w)
                        f.translate(App.Vector(0, 0, -z))
                        slabs.append(slab(f))
                    except Exception:
                        pass
    if not slabs:
        return None
    return (slabs[0].fuse(slabs[1:]) if len(slabs) > 1 else slabs[0]).removeSplitter()


def placed(link_name):
    link = doc.getObject(link_name)
    target = link.LinkedObject
    kids = target.Group if target.TypeId == "App::Part" else [target]
    out = []
    for c in kids:
        if hasattr(c, "Shape") and c.Shape.Solids and c.Name not in ("Bearing_ring", "Bearing_bore"):
            s = c.Shape.copy()
            s.Placement = link.Placement.multiply(s.Placement)
            out.append(s)
    return out


# ------------------------------------------------------------ where the legs swing
legs = {}
for o in doc.getObject("Body_Check").Group:
    if not o.Name.startswith("BodyCheck_coxa_"):
        continue
    tag = o.Name.split("_")[-1]
    base = footprint(placed(o.Name) + placed("BodyCheck_servo_" + tag))
    c = o.Placement.Base
    turns = []
    for deg in range(-40, 41, 5):
        f = base.copy()
        f.rotate(App.Vector(c.x, c.y, 0), App.Vector(0, 0, 1), deg)
        turns.append(f)
    legs[tag] = turns[0].fuse(turns[1:]).removeSplitter()
    print("leg %s swing: %.0f mm2" % (tag, legs[tag].Area))
swing = list(legs.values())[0].fuse(list(legs.values())[1:]).removeSplitter()

# ------------------------------------------------------------ the electronics
inside = {}
for key, link in (("PCA9685 R", "BodyCheck_pca_r"), ("PCA9685 L", "BodyCheck_pca_l"), ("ESP32", "BodyCheck_esp32"),
                  ("GY-91", "BodyCheck_gy91"), ("INA219", "BodyCheck_ina219"), ("switch", "BodyCheck_rocker"),
                  ("battery", "BodyCheck_cells"), ("cartridge", "BodyCheck_cartridge"), ("XT30", "BodyCheck_xt30")):
    if doc.getObject(link):
        f = footprint(placed(link))
        if f:
            inside[key] = f

# ------------------------------------------------------------ the plate and a wall path
bot = doc.getObject("Body_Bottom").Shape
edge = max(bot.slice(App.Vector(0, 0, 1), top_bot - 0.5), key=lambda w: w.BoundBox.DiagonalLength)
plate = Part.Face(edge)
plate.translate(App.Vector(0, 0, -(top_bot - 0.5)))
room = slab(plate.makeOffset2D(-2.0))                           # 2 mm inside the plate's edge
keep_out = slab(flat(swing).makeOffset2D(CLEAR))
wall_room = flat(room.cut(keep_out).removeSplitter())
swing, keep_out = flat(swing), flat(keep_out)
inside = {k: flat(f) for k, f in inside.items()}
pieces = sorted(wall_room.Faces, key=lambda f: -f.Area)
main = pieces[0]
print("room for the electronics: %.0f mm2 in the main piece, %d small pieces cut off" % (main.Area, len(pieces) - 1))
for key, f in inside.items():
    out = f.cut(main)
    clash = f.common(keep_out)
    print("%-10s outside the wall: %5.1f mm2, inside a leg's swing (+%.1f): %5.1f mm2" % (key, out.Area, CLEAR, clash.Area))

# ------------------------------------------------------------ the picture
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon


def draw(ax, face, **kw):
    for f in face.Faces:
        pts = [(p.x, p.y) for p in f.OuterWire.discretize(Distance=0.8)]
        ax.add_patch(Polygon(pts, closed=True, **kw))


fig, ax = plt.subplots(figsize=(10, 11))
draw(ax, plate, facecolor="#e8e8e4", edgecolor="#888", lw=1)
draw(ax, keep_out, facecolor="#f5b041", edgecolor="none", alpha=0.35)
draw(ax, swing, facecolor="#e67e22", edgecolor="#a04000", alpha=0.6, lw=0.6)
colors = {"PCA9685 R": "#7d3c98", "PCA9685 L": "#7d3c98", "ESP32": "#222", "GY-91": "#7d3c98", "INA219": "#2e86c1",
          "switch": "#111", "battery": "#1e8449", "cartridge": "#5d6d7e", "XT30": "#d4ac0d"}
for key, f in inside.items():
    draw(ax, f, facecolor=colors.get(key, "#444"), edgecolor="none", alpha=0.75)
for f in pieces[:1]:
    pts = [(p.x, p.y) for p in f.OuterWire.discretize(Distance=0.5)]
    ax.plot([p[0] for p in pts] + [pts[0][0]], [p[1] for p in pts] + [pts[0][1]], color="#117a65", lw=2.2)
ax.set_aspect("equal")
ax.set_xlim(plate.BoundBox.XMin - 25, plate.BoundBox.XMax + 25)
ax.set_ylim(plate.BoundBox.YMin - 10, plate.BoundBox.YMax + 10)
ax.set_title("Between the plates, from above (front up): legs' swing +-40 deg (orange, + %.1f mm clearance pale),\n"
             "electronics, and the room an inner wall could wrap (green line)" % CLEAR, fontsize=10)
ax.grid(alpha=0.2)
path = os.path.join(HERE, "views", "tub_plan.png")
fig.savefig(path, dpi=110, bbox_inches="tight")
plt.close(fig)
print(path)
