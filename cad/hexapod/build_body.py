"""The body: 2 plates, with the legs' servos outside it.

  Body_Bottom  hexagon plate, 6 coxa bearing seats, holes for the battery holder, rails
               for the standing boards, columns
  Body_Top     hexagon plate, a pocket for each coxa servo's horn, thick around it

The columns take the stock MG90S screws from above. Each coxa link's shaft runs in a
bearing in the bottom plate, and its servo's horn is fixed in the top plate.

Body_Check places everything; the end of this script swings every coxa link over
+-40 deg looking for collisions. Body frame: origin on the centre at femur axis height,
X right, Y forward.
"""
import math
import os
import runpy

import FreeCAD as App

HERE = os.path.dirname(os.path.abspath(__file__))
common = runpy.run_path(os.path.join(HERE, "common.py"))
doc = common["get_doc"]()
COL_BOT, COL_TOP = (0.7, 0.7, 0.68), (0.85, 0.85, 0.82)
Z180 = App.Rotation(App.Vector(0, 0, 1), 180)
PCA_RIGHT = App.Rotation(App.Matrix(0, 0, 1, 0,  1, 0, 0, 0,  0, 1, 0, 0,  0, 0, 0, 1))    # board X to Y, Y to Z, Z to X: pins out, bends down
ESP_FRONT = App.Rotation(App.Matrix(1, 0, 0, 0,  0, 0, 1, 0,  0, -1, 0, 0,  0, 0, 0, 1))   # module side forward


def p(alias):
    return "Params." + alias


common["remove_built"](doc, ("BodyCheck", "BodyPrim", "Body"))
part = doc.addObject("App::Part", "Body")
part.setExpression("Placement.Base.y", p("catalog_row2_y") + " * 5")
check = doc.addObject("App::Part", "Body_Check")
check.setExpression("Placement.Base.y", p("catalog_row2_y") + " * 7")
P = common["Prims"](doc, part, "BodyPrim")
box, cyl = P.box, P.cyl
S = doc.getObject("Params")


def val(alias):
    v = S.get(alias)
    return float(getattr(v, "Value", v))


front_yaw = float(S.get("front_leg_yaw").getValueAs("deg"))
rear_yaw = float(S.get("rear_leg_yaw").getValueAs("deg"))
AXES = (("RF", 1, 1, "front", front_yaw, ""), ("RM", 1, 0, "mid", 0.0, ""), ("RR", 1, -1, "rear", rear_yaw, ""),
        ("LF", -1, 1, "front", 180 - front_yaw, "_L"), ("LM", -1, 0, "mid", 180.0, "_L"), ("LR", -1, -1, "rear", 180 - rear_yaw, "_L"))


def axis_exprs(sx, sy, kind):
    if kind == "mid":
        return "%d * %s" % (sx, p("coxa_mid_x")), "0 mm"
    return "%d * %s" % (sx, p("coxa_front_x")), "%d * %s" % (sy, p("coxa_front_y"))


mx, cy = p("body_mx"), p("body_cy")
leg_x = "%s - %s" % (mx, p("body_cx"))
plate_top = p("body_plate_top")


def hexagon(name, z0, t):
    solid = [box(name, "2 * %s" % mx, "2 * %s" % cy, t, "-" + mx, "-" + cy, z0)]
    corners = []
    for tag, sx, sy in (("fr", 1, 1), ("fl", -1, 1), ("rr", 1, -1), ("rl", -1, -1)):
        corners.append(P.gusset("%s_corner_%s" % (name, tag), ("%d * %s" % (sx, mx), "%d * %s" % (sy, cy), "(%s) - 1 mm" % z0),
                                ("x", -sx), ("y", -sy), leg_x, cy, "z", "(%s) + 2 mm" % t))
    return solid, corners


# ------------------------------------------------------------ bottom print
bot_solid, bot_cut = hexagon("bot", p("body_bottom_z"), p("body_plate_t"))
for tag, sx, sy, kind, _, __ in AXES:
    x, y = axis_exprs(sx, sy, kind)
    bot_cut.append(cyl("seat_" + tag, p("bearing_seat_d"), "%s + 1 mm" % p("bearing_t"), x, y, "%s - %s" % (plate_top, p("bearing_t"))))
    bot_cut.append(cyl("bore_" + tag, p("fem_boss_d"), "%s + 2 mm" % p("body_plate_t"), x, y, "%s - 1 mm" % p("body_bottom_z")))
for tag, sign in (("f", 1), ("r", -1)):
    bot_cut.append(cyl("holder_" + tag, p("holder_screw_d"), "%s + 2 mm" % p("body_plate_t"), "0 mm",
                       "%s + %d * %s / 2" % (p("holder_y"), sign, p("holder_hole_span")), "%s - 1 mm" % p("body_bottom_z")))
COLUMNS = []
for sx in (1, -1):
    for sy in (1, -1):
        COLUMNS.append(("f%d%d" % (sx + 1, sy + 1), "%d * %s" % (sx, p("col_front_x")), "%d * %s" % (sy, p("col_front_y"))))
        COLUMNS.append(("s%d%d" % (sx + 1, sy + 1), "%d * %s" % (sx, p("col_side_x")), "%d * %s" % (sy, p("col_side_y"))))
col_h = "%s - (%s)" % (p("top_plate_z0"), plate_top)
for tag, x, y in COLUMNS:
    bot_solid.append(cyl("col_" + tag, p("col_d"), col_h, x, y, plate_top))
    bot_solid.append(cyl("foot_" + tag, p("col_foot_d"), p("col_foot_h"), x, y, plate_top))
    bot_cut.append(cyl("pilot_" + tag, p("mg90_screw_pilot_d"), "%s + 1 mm" % p("col_screw_depth"), x, y, "%s - %s" % (p("top_plate_z0"), p("col_screw_depth"))))
# rails: 2 pairs per standing board, 1 pair for the ESP32
rail_t, rail_h, rail_len, rfit = p("rail_t"), p("rail_h"), p("rail_len"), p("rail_fit")
for side, sign in (("r", 1), ("l", -1)):
    cx = "%d * (%s - %s / 2)" % (sign, p("pca_x"), p("pca_t"))   # the PCB's middle: pca_x is its pin side
    cy_b = "%d * %s" % (sign, p("pca_y"))
    # in the free spots between the pin groups: at both ends and in the wide gap
    for n, off in (("a", "-%s / 2 + 0.3 mm" % p("pca_len")), ("b", "%s / 2 - 0.3 mm - %s" % (p("pca_len"), rail_len)), ("c", "5 mm")):
        by = "(%s) + (%s)" % (cy_b, off) if sign > 0 else "(%s) - (%s) - %s" % (cy_b, off, rail_len)
        gap = "%s / 2 + %s" % (p("pca_t"), rfit)
        bot_solid.append(box("rail_%s%s_in" % (side, n), rail_t, rail_len, rail_h, "(%s) - (%s) - %s" % (cx, gap, rail_t), by, plate_top))
        bot_solid.append(box("rail_%s%s_out" % (side, n), rail_t, rail_len, rail_h, "(%s) + (%s)" % (cx, gap), by, plate_top))
gap = "(%s + %s)" % (p("esp_t"), rfit)
for n, xo in (("a", "-%s / 2 + 8 mm" % p("esp_len")), ("b", "%s / 2 - 8 mm - %s" % (p("esp_len"), rail_len)), ("c", "-%s / 2" % rail_len)):
    bot_solid.append(box("rail_e%s_back" % n, rail_len, rail_t, rail_h, xo, "%s - %s - %s" % (p("esp_y"), gap, rail_t), plate_top))
    bot_solid.append(box("rail_e%s_front" % n, rail_len, rail_t, rail_h, xo, "%s + %s" % (p("esp_y"), rfit), plate_top))
bottom_raw = P.cut("Body_Bottom_raw", bot_solid, bot_cut, COL_BOT, mirror=False)

# ------------------------------------------------------------ top print
top_solid, top_cut = hexagon("top", p("top_plate_z0"), p("top_plate_t"))
boss_h = "%s - %s" % (p("top_plate_z1"), p("top_boss_z0"))
hub_z = p("top_boss_z0")
arm = "%s - %s / 2" % (p("horn_reach"), p("horn_tip_w"))
for tag, sx, sy, kind, yaw, _ in AXES:
    x, y = axis_exprs(sx, sy, kind)
    top_solid.append(cyl("boss_" + tag, p("top_boss_d"), boss_h, x, y, hub_z))
    # the horn's arm points in along the link, yaw + 180
    a = math.radians(yaw + 180)
    ca, sa = math.cos(a), math.sin(a)
    # thick strip around the arm
    strip = "%s + 2.5 mm" % p("horn_reach")
    w = "%s + 5 mm" % p("horn_tip_w")
    s = box("strip_" + tag, strip, w, boss_h, "(%s) + %.6f * (%s) / 2" % (x, sa, w), "(%s) - %.6f * (%s) / 2" % (y, ca, w), hub_z)
    s.Placement = App.Placement(s.Placement.Base, App.Rotation(App.Vector(0, 0, 1), yaw + 180))
    top_solid.append(s)
    pk = box("pk_arm_" + tag, arm, p("horn_tip_w"), p("horn_hub_h"), "(%s) + %.6f * %s / 2" % (x, sa, p("horn_tip_w")), "(%s) - %.6f * %s / 2" % (y, ca, p("horn_tip_w")), hub_z)
    pk.Placement = App.Placement(pk.Placement.Base, App.Rotation(App.Vector(0, 0, 1), yaw + 180))
    top_cut.append(pk)
    top_cut.append(cyl("pk_tip_" + tag, p("horn_tip_w"), p("horn_hub_h"), "(%s) + %.6f * (%s)" % (x, ca, arm), "(%s) + %.6f * (%s)" % (y, sa, arm), hub_z))
    top_cut.append(cyl("pk_hub_" + tag, p("horn_hub_d"), p("horn_hub_h"), x, y, hub_z))
    through = "%s + 1 mm" % boss_h
    top_cut.append(cyl("access_" + tag, "5 mm", through, x, y, hub_z))
    for n, r in (("a", p("horn_screw_r1")), ("b", p("horn_screw_r2"))):
        top_cut.append(cyl("hs_%s_%s" % (n, tag), p("horn_screw_d"), through, "(%s) + %.6f * %s" % (x, ca, r), "(%s) + %.6f * %s" % (y, sa, r), hub_z))
for tag, x, y in COLUMNS:
    top_cut.append(cyl("clear_" + tag, "2.2 mm", "%s + 2 mm" % p("top_plate_t"), x, y, "%s - 1 mm" % p("top_plate_z0")))
top_raw = P.cut("Body_Top_raw", top_solid, top_cut, COL_TOP, mirror=False)
doc.recompute()

ch = val("fem_edge_ch")
bz0, btop = val("body_bottom_z"), val("body_plate_top")
tz0, tz1 = val("top_boss_z0"), val("top_plate_z1")
axes_xy = []
for _, sx, sy, kind, __, ___ in AXES:
    axes_xy.append((sx * val("coxa_mid_x"), 0.0) if kind == "mid" else (sx * val("coxa_front_x"), sy * val("coxa_front_y")))


def near_axis(e, limit):
    c = e.BoundBox.Center
    return any(math.hypot(c.x - ax, c.y - ay) < limit for ax, ay in axes_xy)


rail_top = btop + val("rail_h") + 1e-3
board_x = val("pca_x") - val("pca_t") / 2
board_gap = val("pca_t") / 2 + val("rail_fit") + val("rail_t") + 0.1
esp_lo = val("esp_y") - val("esp_t") - val("rail_fit") - val("rail_t") - 0.1
esp_hi = val("esp_y") + val("rail_fit") + val("rail_t") + 0.1


def bottom_skip(e):
    """No chamfers on the rails: on their inside corners one would fill the board's slot."""
    bb = e.BoundBox
    low = bb.ZMin >= btop - 1e-3 and bb.ZMax <= rail_top
    at_board = abs(abs(bb.Center.x) - board_x) < board_gap and abs(bb.Center.y) < 45
    at_esp = esp_lo <= bb.Center.y <= esp_hi and abs(bb.Center.x) < 26
    return near_axis(e, 7) or (low and (at_board or at_esp))


def top_skip(e):
    bb = e.BoundBox
    return bb.ZMin < tz0 + val("horn_hub_h") + 1e-3 and near_axis(e, val("horn_reach") + 0.5) and bb.ZMax <= tz0 + val("horn_hub_h") + 1e-3


def finish(name, raw, skip, color):
    obj = doc.addObject("Part::Chamfer", name)
    part.addObject(obj)
    obj.Base = raw
    print(name)
    obj.Edges = common["chamfer_edges"](raw.Shape, ch, skip, min_length=3)
    raw.ViewObject.Visibility = False
    obj.ViewObject.ShapeColor = color
    return obj


bottom = finish("Body_Bottom", bottom_raw, bottom_skip, COL_BOT)
top = finish("Body_Top", top_raw, top_skip, COL_TOP)
common["save"](doc)


# ------------------------------------------------------------ everything in place
def link(name, target, base, rot=None):
    obj = doc.addObject("App::Link", "BodyCheck_" + name)
    check.addObject(obj)
    obj.LinkedObject = doc.getObject(target)
    obj.Placement = App.Placement(App.Vector(*base), rot or App.Rotation())
    return obj


seat = val("coxa_seat_z")
bear_z = val("coxa_floor_z0") - val("fem_boss_h") - val("bearing_t")
link("bottom", "Body_Bottom", (0, 0, 0))
link("top", "Body_Top", (0, 0, 0))
SERVO_LEG = App.Rotation(App.Vector(0, 0, 1), 90)
for tag, sx, sy, kind, yaw, side in AXES:
    x, y = axes_xy[[a[0] for a in AXES].index(tag)]
    rot = App.Rotation(App.Vector(0, 0, 1), yaw)
    link("coxa_" + tag, "Coxa_Main" + side, (x, y, 0), rot)
    link("servo_" + tag, "Dummy_MG90S", (x, y, seat), rot.multiply(SERVO_LEG))
    link("horn_" + tag, "Dummy_Horn", (x, y, seat), rot.multiply(Z180))
    link("bearing_" + tag, "Dummy_Bearing", (x, y, bear_z))
cz, px, py, pz = val("cell_z"), val("pca_x"), val("pca_y"), val("pca_z")
link("cells", "Dummy_18650_pair", (0, val("holder_y"), cz))
link("pca_r", "Dummy_PCA9685", (px, py, pz), PCA_RIGHT)
link("pca_l", "Dummy_PCA9685", (-px, -py, pz), Z180.multiply(PCA_RIGHT))
link("esp32", "Dummy_ESP32_devkit", (0, val("esp_y"), val("esp_z")), ESP_FRONT)
link("gy91", "Dummy_GY91", (0, 0, val("gy91_z")))
common["save"](doc)

# ------------------------------------------------------------ report
solids_of, clash = common["solids_of"], common["clash"]
SERVO = ["MG90S_body", "MG90S_tabs", "MG90S_turret", "MG90S_shaft"]
HORN = ["Horn_hub", "Horn_arm", "Horn_tip"]
d = val("pla_density")
print("bottom %.1f g, top %.1f g solid; valid %s %s; solids %d %d" % (
    bottom.Shape.Volume / 1000 * d, top.Shape.Volume / 1000 * d, bottom.Shape.isValid(), top.Shape.isValid(), len(bottom.Shape.Solids), len(top.Shape.Solids)))
bot_s, top_s = solids_of(doc, ["Body_Bottom"]), solids_of(doc, ["Body_Top"])
Pl = lambda base, rot=None: App.Placement(App.Vector(*base), rot or App.Rotation())
names = lambda part_name: [c.Name for c in doc.getObject(part_name).Group if c.Name != "Bearing_ring" and c.Name != "Bearing_bore"]
inside = {
    "cells": solids_of(doc, names("Dummy_18650_pair"), Pl((0, val("holder_y"), cz))),
    "pca r": solids_of(doc, names("Dummy_PCA9685"), Pl((px, py, pz), PCA_RIGHT)),
    "pca l": solids_of(doc, names("Dummy_PCA9685"), Pl((-px, -py, pz), Z180.multiply(PCA_RIGHT))),
    "esp32": solids_of(doc, names("Dummy_ESP32_devkit"), Pl((0, val("esp_y"), val("esp_z")), ESP_FRONT)),
    "gy91": solids_of(doc, names("Dummy_GY91"), Pl((0, 0, val("gy91_z")))),
}
static = {}
keys = list(inside)
for i, k in enumerate(keys):
    for plate, s in (("bottom", bot_s), ("top", top_s)):
        v = clash(inside[k], s)
        if v > 0.001:
            static[(plate, k)] = round(v, 2)
    for other in keys[i + 1:]:
        v = clash(inside[k], inside[other])
        if v > 0.001:
            static[(k, other)] = round(v, 2)
print("plates and boards overlap mm3: %s; plates %.3f" % (static or "none", clash(bot_s, top_s)))
fits = {}
sweep = {}
for tag, sx, sy, kind, yaw, side in AXES:
    x, y = axes_xy[[a[0] for a in AXES].index(tag)]
    fits[tag] = clash(solids_of(doc, HORN, Pl((x, y, seat), App.Rotation(App.Vector(0, 0, 1), yaw).multiply(Z180))), top_s) + clash(solids_of(doc, ["Bearing_body"], Pl((x, y, bear_z))), bot_s)
    for deg in (-40, -20, 0, 20, 40):
        rot = App.Rotation(App.Vector(0, 0, 1), yaw + deg)
        coxa = solids_of(doc, ["Coxa_Main" + side], Pl((x, y, 0), rot))
        servo = solids_of(doc, SERVO, Pl((x, y, seat), rot.multiply(SERVO_LEG)))
        targets = dict(inside)
        targets["bottom"] = bot_s
        targets["top"] = top_s
        for name, group in targets.items():
            v = clash(coxa + servo, group)
            if v > 0.001:
                sweep.setdefault((tag, name), []).append((deg, round(v, 1)))
print("horn and bearing fits mm3: %s" % {k: round(v, 3) for k, v in fits.items()})
print("coxa sweep +-40 deg: %s" % (sweep or "no collisions"))
print("body height %.1f; top plate %.1f to %.1f; horn z %.1f" % (val("top_plate_z1") - val("body_bottom_z"), val("top_boss_z0"), val("top_plate_z1"), val("coxa_horn_z")))
