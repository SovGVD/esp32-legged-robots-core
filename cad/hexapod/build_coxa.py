"""Step 3: the coxa link, two prints, built from primitives driven by Params.

  CoxaLink_Lower  lower plate, bearing seat, femur servo wall
  CoxaLink_Top    top plate over the horn, bolted on with 2 screws

Leg frame: origin on the coxa axis at femur axis height, +X along the link,
Z up. CoxaLink_Check adds the dummies around it, for looking and for the
interference test at the end of this script.
"""
import math
import os
import runpy

import FreeCAD as App
import Part

HERE = os.path.dirname(os.path.abspath(__file__))
common = runpy.run_path(os.path.join(HERE, "common.py"))
doc = common["get_doc"]()
PLA = (0.88, 0.63, 0.11)
PLA_TOP = (0.85, 0.82, 0.74)


def p(alias):
    return "Params." + alias


for o in [o for o in doc.Objects if o.Name.startswith(("CoxaLink", "CoxaPrim", "CoxaCheck"))]:
    doc.removeObject(o.Name)

part = doc.addObject("App::Part", "CoxaLink")
part.setExpression("Placement.Base.y", p("catalog_row2_y"))
check = doc.addObject("App::Part", "CoxaLink_Check")
check.setExpression("Placement.Base.y", p("catalog_row2_y") + " * 2")


def _prim(kind, name, x, y, z):
    obj = doc.addObject(kind, "CoxaPrim_" + name)
    part.addObject(obj)
    for axis, expr in zip("xyz", (x, y, z)):
        obj.setExpression("Placement.Base." + axis, expr)
    obj.ViewObject.Visibility = False
    return obj


def box(name, lx, ly, lz, x, y, z):
    """x, y, z = low corner."""
    obj = _prim("Part::Box", name, x, y, z)
    obj.setExpression("Length", lx)
    obj.setExpression("Width", ly)
    obj.setExpression("Height", lz)
    return obj


def ybox(name, x0, lx, ly, y0, z0, lz):
    return box(name, lx, ly, lz, x0, y0, z0)


def cyl(name, d, h, x, y, z, along_y=False):
    obj = _prim("Part::Cylinder", name, x, y, z)
    obj.setExpression("Radius", "(%s) / 2" % d)
    obj.setExpression("Height", h)
    if along_y:
        obj.Placement = App.Placement(obj.Placement.Base, App.Rotation(App.Vector(1, 0, 0), -90))
    return obj


def fuse(name, items):
    obj = doc.addObject("Part::MultiFuse", name)
    part.addObject(obj)
    obj.Shapes = items
    obj.Refine = True
    for i in items:
        i.ViewObject.Visibility = False
    obj.ViewObject.Visibility = False
    return obj


def cut(name, base, tool, color):
    obj = doc.addObject("Part::Cut", name)
    part.addObject(obj)
    obj.Base, obj.Tool = base, tool
    obj.Refine = True
    base.ViewObject.Visibility = tool.ViewObject.Visibility = False
    obj.ViewObject.ShapeColor = color
    return obj


W = p("coxa_beam_w")
half_w = "-%s / 2" % W
low_z = p("coxa_low_z")
beam_t = p("coxa_beam_t")
end = p("coxa_end")
wall_top = p("coxa_wall_top_z")
seat_y = p("coxa_seat_y")
wall_y = "%s - %s" % (seat_y, p("coxa_wall_t"))
wall_mid = "%s - %s / 2" % (seat_y, p("coxa_wall_t"))

# ------------------------------------------------------------ lower print
lower_solid = [
    box("low_beam", end, W, beam_t, "0 mm", half_w, low_z),
    cyl("low_hub", p("coxa_low_hub_d"), p("coxa_low_hub_t"), "0 mm", "0 mm", low_z),
    box("wall", "%s - %s" % (end, p("coxa_wall_x0")), p("coxa_wall_t"), "%s - (%s)" % (wall_top, low_z),
        p("coxa_wall_x0"), wall_y, low_z),
]
boss_z = "%s - %s" % (wall_top, p("coxa_boss_h"))
for n in "12":
    lower_solid.append(box("boss" + n, p("coxa_boss_len"), p("coxa_boss_w"), p("coxa_boss_h"),
                           "%s - %s / 2" % (p("coxa_boss_x" + n), p("coxa_boss_len")),
                           "%s - %s / 2" % (wall_mid, p("coxa_boss_w")), boss_z))

# femur servo: shaft at x = coxa_len, the body runs inward
shaft_x = p("coxa_len")
win_len = "%s + 2 * %s" % (p("mg90_body_len"), p("mg90_fit"))
win_h = "%s + 2 * %s" % (p("mg90_body_w"), p("mg90_fit"))
hole_mid = "%s + %s - %s / 2" % (shaft_x, p("mg90_shaft_end"), p("mg90_body_len"))
lower_cut = [
    cyl("seat", p("bearing_seat_d"), p("bearing_t"), "0 mm", "0 mm", low_z),
    cyl("bore", p("coxa_bore_d"), p("coxa_low_hub_t"), "0 mm", "0 mm", low_z),
    box("window", win_len, "%s + 2 mm" % p("coxa_wall_t"), win_h,
        "%s + %s + %s - %s" % (shaft_x, p("mg90_shaft_end"), p("mg90_fit"), win_len),
        "%s - 1 mm" % wall_y, "-(%s) / 2" % win_h),
]
for name, sign in (("tab_hole_a", "-"), ("tab_hole_b", "+")):
    lower_cut.append(cyl(name, p("mg90_screw_pilot_d"), "%s + 2 mm" % p("coxa_wall_t"),
                         "%s %s %s / 2" % (hole_mid, sign, p("mg90_screw_spacing")),
                         "%s - 1 mm" % wall_y, "0 mm", along_y=True))
for n in "12":
    lower_cut.append(cyl("pilot" + n, p("mg90_screw_pilot_d"), p("coxa_boss_h"), p("coxa_boss_x" + n), wall_mid, boss_z))

lower = cut("CoxaLink_Lower", fuse("CoxaPrim_LowerFuse", lower_solid), fuse("CoxaPrim_LowerCut", lower_cut), PLA)

# ------------------------------------------------------------ top print
top_z = p("coxa_top_z")
tail = p("coxa_tail_c")
pocket_z = p("coxa_top_low_z")
block_h = "%s - %s" % (top_z, pocket_z)
top_solid = [
    box("top_beam", end, W, beam_t, "0 mm", half_w, wall_top),
    box("top_block", "2 * %s" % tail, W, block_h, "-" + tail, half_w, pocket_z),
    cyl("top_tail_a", W, block_h, "-" + tail, "0 mm", pocket_z),
    cyl("top_tail_b", W, block_h, tail, "0 mm", pocket_z),
]
arm = "%s - %s" % (p("horn_span"), p("horn_tip_w"))
through = "%s + 1 mm" % block_h
top_cut = [
    box("pk_long", arm, p("horn_tip_w"), p("coxa_horn_pocket"), "-(%s) / 2" % arm, "-%s / 2" % p("horn_tip_w"), pocket_z),
    cyl("pk_tip_a", p("horn_tip_w"), p("coxa_horn_pocket"), "(%s) / 2" % arm, "0 mm", pocket_z),
    cyl("pk_tip_b", p("horn_tip_w"), p("coxa_horn_pocket"), "-(%s) / 2" % arm, "0 mm", pocket_z),
    box("pk_short", p("horn_short_w"), p("horn_short_span"), p("coxa_horn_pocket"),
        "-%s / 2" % p("horn_short_w"), "-%s / 2" % p("horn_short_span"), pocket_z),
    cyl("pk_hub", p("horn_hub_d"), p("coxa_horn_pocket"), "0 mm", "0 mm", pocket_z),
    cyl("access", p("coxa_access_d"), through, "0 mm", "0 mm", pocket_z),
    cyl("horn_screw_a", p("horn_screw_d"), through, p("horn_screw_r"), "0 mm", pocket_z),
    cyl("horn_screw_b", p("horn_screw_d"), through, "-" + p("horn_screw_r"), "0 mm", pocket_z),
]
for n in "12":
    top_cut.append(cyl("plate_screw" + n, p("coxa_plate_screw_d"), "%s + 1 mm" % beam_t, p("coxa_boss_x" + n), wall_mid, wall_top))

top = cut("CoxaLink_Top", fuse("CoxaPrim_TopFuse", top_solid), fuse("CoxaPrim_TopCut", top_cut), PLA_TOP)
common["save"](doc)

# ------------------------------------------------------------ check: the link among its dummies
FEMUR_ROT = App.Rotation(App.Matrix(0, 1, 0, 0,  0, 0, 1, 0,  1, 0, 0, 0,  0, 0, 0, 1))   # servo Y to link X, servo Z to link Y


def link(name, target, base, rot=None):
    obj = doc.addObject("App::Link", "CoxaCheck_" + name)
    check.addObject(obj)
    obj.LinkedObject = target
    for axis, expr in zip("xyz", base):
        obj.setExpression("Placement.Base." + axis, expr)
    if rot:
        obj.Placement = App.Placement(obj.Placement.Base, rot)
    return obj


link("lower", lower, ("0 mm", "0 mm", "0 mm"))
link("top", top, ("0 mm", "0 mm", "0 mm"))
link("coxa_servo", doc.getObject("Dummy_MG90S"), ("0 mm", "0 mm", p("coxa_servo_seat_z")))
link("femur_servo", doc.getObject("Dummy_MG90S"), (p("coxa_len"), seat_y, "0 mm"), FEMUR_ROT)
link("bearing", doc.getObject("Dummy_Bearing"), ("0 mm", "0 mm", low_z))
common["save"](doc)

# ------------------------------------------------------------ report
S = doc.getObject("Params")


def val(alias):
    v = S.get(alias)
    return float(getattr(v, "Value", v))


def dummy_shape(name, placement):
    solids = [c.Shape.copy() for c in doc.getObject(name).Group if c.Name.rsplit("_", 1)[-1] != "bore"]
    shape = Part.makeCompound(solids)
    shape.Placement = placement
    return shape


def overlap(a, b):
    return a.common(b).Volume


femur = dummy_shape("Dummy_MG90S", App.Placement(App.Vector(val("coxa_len"), val("coxa_seat_y"), 0), FEMUR_ROT))
bearing = dummy_shape("Dummy_Bearing", App.Placement(App.Vector(0, 0, val("coxa_low_z")), App.Rotation()))
horn_free = ("body", "tabs", "turret", "shaft")
coxa_body = Part.makeCompound([doc.getObject("MG90S_" + n).Shape.copy() for n in horn_free])
coxa_horn = Part.makeCompound([c.Shape.copy() for c in doc.getObject("Dummy_MG90S").Group if "horn" in c.Name])
print("lower %.2f cm3, %.1f g solid; top %.2f cm3, %.1f g solid" % (
    lower.Shape.Volume / 1000, lower.Shape.Volume / 1000 * val("pla_density"),
    top.Shape.Volume / 1000, top.Shape.Volume / 1000 * val("pla_density")))
print("valid", lower.Shape.isValid(), top.Shape.isValid(), "solids", len(lower.Shape.Solids), len(top.Shape.Solids))
print("low plate to body plate %.2f, top plate to cover %.2f, servo seat z %.2f" % (
    val("coxa_low_clear"), val("coxa_cover_gap"), val("coxa_servo_seat_z")))
print("overlap mm3: femur servo/lower %.3f, femur servo/top %.3f, bearing/lower %.3f" % (
    overlap(femur, lower.Shape), overlap(femur, top.Shape), overlap(bearing, lower.Shape)))
horn_place = App.Placement(App.Vector(0, 0, val("coxa_servo_seat_z")), App.Rotation())
worst = 0.0
for deg in (-60, -40, -20, 0, 20, 40, 60):
    body = coxa_body.copy()
    body.Placement = App.Placement(App.Vector(0, 0, val("coxa_servo_seat_z")), App.Rotation(App.Vector(0, 0, 1), deg))
    worst = max(worst, overlap(body, lower.Shape), overlap(body, top.Shape))
print("overlap mm3: coxa servo body/link over +-60 deg %.3f" % worst)
horn = coxa_horn.copy()
horn.Placement = horn_place
print("overlap mm3: coxa horn/lower %.3f, coxa horn/top %.3f" % (overlap(horn, lower.Shape), overlap(horn, top.Shape)))
