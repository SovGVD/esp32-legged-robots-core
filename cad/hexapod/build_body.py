"""The body's bottom print: hexagon plate, 6 pivot pins, 6 coxa servo towers.

Nothing on a leg swings within 37 mm of its coxa axis between the lower hub
(z -20.5) and the top plate (z +9), so each servo sits on a shelf at tab level in
that free space, held by walls that stay out of the lower plate's sweep.

Coxa servos, shaft up, body along Y: front toward -Y, rear toward +Y. The middle
ones run toward the gap between their board's pin groups: right toward +Y, left
toward -Y, because the left board is the right one turned 180 deg.

Body_Check places everything around it, and the end of this script swings every
coxa link over +-40 deg looking for collisions.
"""
import math
import os
import runpy

import FreeCAD as App

HERE = os.path.dirname(os.path.abspath(__file__))
common = runpy.run_path(os.path.join(HERE, "common.py"))
doc = common["get_doc"]()
PLA_BODY = (0.75, 0.75, 0.72)
Z180 = App.Rotation(App.Vector(0, 0, 1), 180)
Z90 = App.Rotation(App.Vector(0, 0, 1), 90)
SERVO_IN_LEG = App.Rotation(App.Matrix(0, 1, 0, 0,  0, 0, 1, 0,  1, 0, 0, 0,  0, 0, 0, 1))
PCA_RIGHT = App.Rotation(App.Matrix(0, 0, 1, 0,  1, 0, 0, 0,  0, 1, 0, 0,  0, 0, 0, 1))    # board X to Y, Y to Z, Z to X
ESP_FRONT = App.Rotation(App.Matrix(1, 0, 0, 0,  0, 0, 1, 0,  0, -1, 0, 0,  0, 0, 0, 1))   # module side forward


def p(alias):
    return "Params." + alias


common["remove_built"](doc, ("Body",))
part = doc.addObject("App::Part", "Body")
part.setExpression("Placement.Base.y", p("catalog_row2_y") + " * 5")
check = doc.addObject("App::Part", "Body_Check")
check.setExpression("Placement.Base.y", p("catalog_row2_y") + " * 7")
P = common["Prims"](doc, part, "BodyPrim")
plate_top, plate_z, plate_t = p("body_plate_top"), p("body_bottom_z"), p("body_plate_t")
seat_z = p("coxa_servo_seat_z")
shelf_t, wall_t = p("tower_shelf_t"), p("tower_wall_t")
shelf_z = "%s - %s" % (seat_z, shelf_t)
wall_h = "%s - (%s)" % (shelf_z, plate_top)
hw = "%s / 2 + %s" % (p("mg90_body_w"), p("mg90_fit"))
ch = p("print_chamfer")


def flip(sign, lo, hi):
    """Low end and length of a range mirrored by sign."""
    return ("(%s)" % lo if sign > 0 else "-(%s)" % hi), "(%s) - (%s)" % (hi, lo)


def rbox(name, sx, sy, x0, x1, y0, y1, z0, h):
    xl, lx = flip(sx, x0, x1)
    yl, ly = flip(sy, y0, y1)
    return P.box(name, lx, ly, h, xl, yl, z0)


def rcyl(name, sx, sy, d, h, x, y, z):
    return P.cyl(name, d, h, "%d * (%s)" % (sx, x), "%d * (%s)" % (sy, y), z)


def gusset_x(name, sx, sy, x, y0, y1, dx):
    """Wall foot chamfer along Y at x, on the plate, leg toward dx."""
    yl, ly = flip(sy, y0, y1)
    return P.gusset(name, ("%d * (%s)" % (sx, x), yl, plate_top), ("x", dx * sx), ("z", 1), ch, ch, "y", ly)


def gusset_y(name, sx, sy, y, x0, x1, dy):
    xl, lx = flip(sx, x0, x1)
    return P.gusset(name, (xl, "%d * (%s)" % (sy, y), plate_top), ("y", dy * sy), ("z", 1), ch, ch, "x", lx)


# ------------------------------------------------------------ plate: a box with its 4 corners cut off
solid = [P.box("plate", "2 * %s" % p("body_mx"), "2 * %s" % p("body_cy"), plate_t,
               "-" + p("body_mx"), "-" + p("body_cy"), plate_z)]
holes = []
leg_x, leg_y = "%s - %s" % (p("body_mx"), p("body_cx")), p("body_cy")
len_z = "%s + 2 mm" % plate_t
low_z = "%s - 1 mm" % plate_z
for tag, sx, sy in (("fr", 1, 1), ("fl", -1, 1), ("rr", 1, -1), ("rl", -1, -1)):
    holes.append(P.gusset("corner_" + tag, ("%d * %s" % (sx, p("body_mx")), "%d * %s" % (sy, p("body_cy")), low_z),
                          ("x", -sx), ("y", -sy), leg_x, leg_y, "z", len_z))
for tag, sign in (("l", -1), ("r", 1)):
    holes.append(P.cyl("cell_seat_" + tag, p("cell_seat_d"), p("cell_len"), "%d * %s" % (sign, p("cell_x")),
                       "-%s / 2" % p("cell_len"), p("cell_z"), along_y=True))

# ------------------------------------------------------------ pins and towers
pins = (("fr", 1, 1, "front"), ("fl", -1, 1, "front"), ("rr", 1, -1, "rear"), ("rl", -1, -1, "rear"), ("mr", 1, 1, "mid"), ("ml", -1, -1, "mid"))
for tag, sx, sy, kind in pins:
    ax = p("coxa_mid_x") if kind == "mid" else p("coxa_front_x")
    ay = "0 mm" if kind == "mid" else p("coxa_front_y")
    solid.append(rcyl("pin_" + tag, sx, sy, p("coxa_pin_d"), p("body_pin_h"), ax, ay, plate_top))
    inner0, inner1 = "%s - (%s) - %s" % (ax, hw, wall_t), "%s - (%s)" % (ax, hw)
    outer0, outer1 = "%s + (%s)" % (ax, hw), "%s + (%s) + %s" % (ax, hw, wall_t)
    # unflipped, the corner towers' servo runs toward -Y and the middle one toward +Y
    d = 1 if kind == "mid" else -1
    far = "%s + %d * %s" % (ay, d, p("mg90_tab_far"))
    near = "%s - %d * %s" % (ay, d, p("mg90_tab_near"))
    lo, hi = (near, far) if d > 0 else (far, near)
    solid.append(rbox("shelf_" + tag, sx, sy, inner0, outer1, lo, hi, shelf_z, shelf_t))
    body_far = "%s + %d * (%s - %s + %s)" % (ay, d, p("mg90_body_len"), p("mg90_shaft_end"), p("mg90_fit"))
    body_near = "%s - %d * (%s + %s)" % (ay, d, p("mg90_shaft_end"), p("mg90_fit"))
    wlo, whi = (body_near, body_far) if d > 0 else (body_far, body_near)
    holes.append(rbox("window_" + tag, sx, sy, "%s - (%s)" % (ax, hw), "%s + (%s)" % (ax, hw), wlo, whi, "%s - 1 mm" % shelf_z, "%s + 2 mm" % shelf_t))
    for hname, off in (("near", "-%d * %s" % (d, p("mg90_hole_near"))), ("far", "%d * %s" % (d, p("mg90_hole_far")))):
        holes.append(rcyl("hole_%s_%s" % (hname, tag), sx, sy, p("mg90_screw_pilot_d"), "%s + 2 mm" % shelf_t, ax, "%s + (%s)" % (ay, off), "%s - 1 mm" % shelf_z))
    if kind == "mid":
        y0, y1 = p("tower_mid_y0"), p("tower_mid_y1")
        end = "%s + %s" % (p("mg90_tab_far"), wall_t)
        solid.append(rbox("wall_%s" % tag, sx, sy, inner0, inner1, y0, y1, plate_top, wall_h))
        solid.append(rbox("end_%s" % tag, sx, sy, inner0, outer1, p("mg90_tab_far"), end, plate_top, wall_h))
        solid.append(gusset_x("ch_wall_" + tag, sx, sy, inner1, y0, y1, 1))
        solid.append(gusset_y("ch_end_" + tag, sx, sy, p("mg90_tab_far"), inner0, outer1, -1))
    else:
        y0, y1 = far, "%s - %s" % (ay, p("tower_gap"))
        solid.append(rbox("wall_in_%s" % tag, sx, sy, inner0, inner1, y0, y1, plate_top, wall_h))
        solid.append(rbox("wall_out_%s" % tag, sx, sy, outer0, outer1, y0, y1, plate_top, wall_h))
        solid.append(gusset_x("ch_in_" + tag, sx, sy, inner1, y0, y1, 1))
        solid.append(gusset_x("ch_out_" + tag, sx, sy, outer0, y0, y1, -1))

body = P.cut("Body_Bottom", solid, holes, PLA_BODY, mirror=False)
common["save"](doc)

# ------------------------------------------------------------ everything in place
S = doc.getObject("Params")


def val(alias):
    v = S.get(alias)
    return float(getattr(v, "Value", v))


def link(name, target, base, rot=None):
    obj = doc.addObject("App::Link", "BodyCheck_" + name)
    check.addObject(obj)
    obj.LinkedObject = doc.getObject(target)
    obj.Placement = App.Placement(App.Vector(*base), rot or App.Rotation())
    return obj


SERVO = ["MG90S_body", "MG90S_tabs", "MG90S_turret", "MG90S_shaft"]
HORN = ["Horn_hub", "Horn_arm", "Horn_tip"]
solids_of, clash = common["solids_of"], common["clash"]
seat = val("coxa_servo_seat_z")
cx, cy, mx = val("coxa_front_x"), val("coxa_front_y"), val("coxa_mid_x")
front_yaw = float(S.get("front_leg_yaw").getValueAs("deg"))
rear_yaw = float(S.get("rear_leg_yaw").getValueAs("deg"))
LEGS = {
    "RF": (cx, cy, front_yaw, Z180, ""), "RM": (mx, 0, 0.0, App.Rotation(), ""), "RR": (cx, -cy, rear_yaw, App.Rotation(), ""),
    "LF": (-cx, cy, 180 - front_yaw, Z180, "_L"), "LM": (-mx, 0, 180.0, Z180, "_L"), "LR": (-cx, -cy, 180 - rear_yaw, App.Rotation(), "_L"),
}
# the servo dummy's body runs toward its -Y: identity = toward -Y, Z180 = toward +Y
SERVO_ROT = {"RF": App.Rotation(), "LF": App.Rotation(), "RR": Z180, "LR": Z180, "RM": Z180, "LM": App.Rotation()}
link("bottom", "Body_Bottom", (0, 0, 0))
for leg, (x, y, yaw, _, side) in LEGS.items():
    rot = App.Rotation(App.Vector(0, 0, 1), yaw)
    link("lower_" + leg, "CoxaLink_Lower" + side, (x, y, 0), rot)
    link("top_" + leg, "CoxaLink_Top" + side, (x, y, 0), rot)
    link("servo_" + leg, "Dummy_MG90S", (x, y, seat), SERVO_ROT[leg])
    link("horn_" + leg, "Dummy_Horn", (x, y, seat), rot)
    link("bearing_" + leg, "Dummy_Bearing", (x, y, val("coxa_low_z")))
link("cells", "Dummy_18650_pair", (0, 0, val("cell_z")))
link("pca_r", "Dummy_PCA9685", (val("pca_x"), 0, val("pca_z")), PCA_RIGHT)
link("pca_l", "Dummy_PCA9685", (-val("pca_x"), 0, val("pca_z")), Z180.multiply(PCA_RIGHT))
link("esp32", "Dummy_ESP32_devkit", (0, val("esp_y"), val("esp_z")), ESP_FRONT)
link("gy91", "Dummy_GY91", (0, 0, val("gy91_z")))
common["save"](doc)

# ------------------------------------------------------------ report
body_s = solids_of(doc, ["Body_Bottom"])
print("body bottom %.1f g solid; valid %s; solids %d" % (body.Shape.Volume / 1000 * val("pla_density"), body.Shape.isValid(), len(body.Shape.Solids)))
PCA = [c.Name for c in doc.getObject("Dummy_PCA9685").Group]
ESP = [c.Name for c in doc.getObject("Dummy_ESP32_devkit").Group]
CELLS = [c.Name for c in doc.getObject("Dummy_18650_pair").Group]
GY = [c.Name for c in doc.getObject("Dummy_GY91").Group]
Pl = lambda base, rot=None: App.Placement(App.Vector(*base), rot or App.Rotation())
parts = {
    "cells": solids_of(doc, CELLS, Pl((0, 0, val("cell_z")))),
    "pca r": solids_of(doc, PCA, Pl((val("pca_x"), 0, val("pca_z")), PCA_RIGHT)),
    "pca l": solids_of(doc, PCA, Pl((-val("pca_x"), 0, val("pca_z")), Z180.multiply(PCA_RIGHT))),
    "esp32": solids_of(doc, ESP, Pl((0, val("esp_y"), val("esp_z")), ESP_FRONT)),
    "gy91": solids_of(doc, GY, Pl((0, 0, val("gy91_z")))),
}
for leg, (x, y, _, __, ___) in LEGS.items():
    parts["servo " + leg] = solids_of(doc, SERVO, Pl((x, y, seat), SERVO_ROT[leg]))
names = list(parts)
static = {}
for name in names:
    v = clash(parts[name], body_s)
    if v > 0.001:
        static[("body", name)] = round(v, 2)
    for other in names[names.index(name) + 1:]:
        v = clash(parts[name], parts[other])
        if v > 0.001:
            static[(name, other)] = round(v, 2)
print("static overlaps mm3: %s" % (static or "none"))

sweep = {}
for leg, (x, y, yaw, _, side) in LEGS.items():
    for deg in (-40, -20, 0, 20, 40):
        place = Pl((x, y, 0), App.Rotation(App.Vector(0, 0, 1), yaw + deg))
        moving = solids_of(doc, ["CoxaLink_Lower" + side, "CoxaLink_Top" + side], place)
        horn = solids_of(doc, HORN, Pl((x, y, seat), App.Rotation(App.Vector(0, 0, 1), yaw + deg)))
        for name, group in list(parts.items()) + [("body", body_s)]:
            v = clash(moving, group)
            if name == "servo " + leg:
                v += 0
            elif name.startswith("servo"):
                v += clash(horn, group)
            if v > 0.001:
                sweep.setdefault((leg, name), []).append((deg, round(v, 1)))
print("coxa sweep +-40 deg: %s" % (sweep or "no collisions"))
print("clearance: top plate to cover %.2f, lower plate to bottom plate %.2f" % (val("coxa_cover_gap"), val("coxa_low_clear")))
