"""The coxa link, two prints, built from primitives driven by Params.

  CoxaLink_Lower  lower plate, coxa bearing seat, femur servo wall, femur bearing cap
  CoxaLink_Top    top plate over the coxa horn, bolted on with 2 screws

These are the right leg's; the _L mirrors are the left leg's, so the femur
horns of both sides face forward, as the firmware's inverse flags assume.

Leg frame: origin on the coxa axis at femur axis height, +X along the link,
Z up, femur horn on +Y. CoxaLink_Check adds the dummies around it, for looking
and for the interference test at the end.
"""
import os
import runpy

import FreeCAD as App
import Part

HERE = os.path.dirname(os.path.abspath(__file__))
common = runpy.run_path(os.path.join(HERE, "common.py"))
doc = common["get_doc"]()
PLA = (0.88, 0.63, 0.11)
PLA_TOP = (0.85, 0.82, 0.74)
# servo Y to link X, servo Z to link Y: the shaft along the joint axis, the body inward
SERVO_IN_LEG = App.Rotation(App.Matrix(0, 1, 0, 0,  0, 0, 1, 0,  1, 0, 0, 0,  0, 0, 0, 1))


def p(alias):
    return "Params." + alias


common["remove_built"](doc, ("TibiaCheck", "FemurCheck", "CoxaLink", "CoxaPrim", "CoxaCheck"))
part = doc.addObject("App::Part", "CoxaLink")
part.setExpression("Placement.Base.y", p("catalog_row2_y"))
check = doc.addObject("App::Part", "CoxaLink_Check")
check.setExpression("Placement.Base.y", p("catalog_row2_y") + " * 2")
P = common["Prims"](doc, part, "CoxaPrim")
box, cyl = P.box, P.cyl

W = p("coxa_beam_w")
half_w = "-%s / 2" % W
low_z = p("coxa_low_z")
beam_t = p("coxa_beam_t")
end = p("coxa_end")
wall_top = p("coxa_wall_top_z")
seat_y = p("coxa_seat_y")
wall_y = "%s - %s" % (seat_y, p("coxa_wall_t"))
wall_mid = "%s - %s / 2" % (seat_y, p("coxa_wall_t"))
shaft_x = p("coxa_len")
plate_top = "%s + %s" % (low_z, beam_t)
hub = p("joint_hub_d")
cap_y0, cap_y1 = p("joint_cap_y0"), p("joint_cap_y1")
cap_top = "(%s) / 2" % hub
cap_x0 = "%s - (%s) / 2" % (shaft_x, hub)
rib_x0 = "%s + %s + %s" % (shaft_x, p("mg90_shaft_end"), p("mg90_fit"))
above_plate = "%s - (%s)" % (cap_top, plate_top)
rib_t = p("joint_rib_t")
rib_x1 = "%s + %s" % (rib_x0, rib_t)
ch = p("print_chamfer")

# ------------------------------------------------------------ lower print
lower_solid = [
    box("low_beam", p("coxa_low_end"), W, beam_t, "0 mm", half_w, low_z),
    box("low_wide", "%s - (%s)" % (p("coxa_low_end"), cap_x0), "%s - (%s)" % (half_w, cap_y0), beam_t, cap_x0, cap_y0, low_z),
    cyl("low_hub", p("coxa_low_hub_d"), p("coxa_low_hub_t"), "0 mm", "0 mm", low_z),
    box("wall", "%s - %s" % (end, p("coxa_wall_x0")), p("coxa_wall_t"), "%s - (%s)" % (wall_top, plate_top),
        p("coxa_wall_x0"), wall_y, plate_top),
    box("cap", "%s + %s - (%s)" % (rib_x0, rib_t, cap_x0), p("joint_cap_t"), above_plate, cap_x0, cap_y0, plate_top),
    box("rib", rib_t, "%s - (%s)" % (wall_y, cap_y1), above_plate, rib_x0, cap_y1, plate_top),
    # internal chamfers: wall and cap onto the plate, rib into the wall and the cap
    P.gusset("ch_wall_a", (p("coxa_wall_x0"), wall_y, plate_top), ("y", -1), ("z", 1), ch, ch, "x",
             "%s - %s" % (p("coxa_low_end"), p("coxa_wall_x0"))),
    P.gusset("ch_wall_b", (p("coxa_wall_x0"), seat_y, plate_top), ("y", 1), ("z", 1), ch, ch, "x",
             "%s - %s" % (p("coxa_low_end"), p("coxa_wall_x0"))),
    P.gusset("ch_cap", (cap_x0, cap_y1, plate_top), ("y", 1), ("z", 1), ch, ch, "x", "%s - (%s)" % (p("coxa_low_end"), cap_x0)),
    P.gusset("ch_rib_wall", (rib_x1, wall_y, plate_top), ("x", 1), ("y", -1), ch, ch, "z", above_plate),
    P.gusset("ch_rib_cap", (rib_x1, cap_y1, plate_top), ("x", 1), ("y", 1), ch, ch, "z", above_plate),
]
boss_z = "%s - %s" % (wall_top, p("coxa_boss_h"))
for n in "12":
    lower_solid.append(box("boss" + n, p("coxa_boss_len"), p("coxa_boss_w"), p("coxa_boss_h"),
                           "%s - %s / 2" % (p("coxa_boss_x" + n), p("coxa_boss_len")),
                           "%s - %s / 2" % (wall_mid, p("coxa_boss_w")), boss_z))

win_len = "%s + 2 * %s" % (p("mg90_body_len"), p("mg90_fit"))
win_h = "%s + 2 * %s" % (p("mg90_body_w"), p("mg90_fit"))
hole_mid = "%s + %s - %s / 2" % (shaft_x, p("mg90_shaft_end"), p("mg90_body_len"))
lower_cut = [
    cyl("seat", p("bearing_seat_d"), p("bearing_t"), "0 mm", "0 mm", low_z),
    cyl("bore", p("coxa_bore_d"), p("coxa_low_hub_t"), "0 mm", "0 mm", low_z),
    box("window", win_len, "%s + 2 mm" % p("coxa_wall_t"), win_h,
        "%s + %s + %s - (%s)" % (shaft_x, p("mg90_shaft_end"), p("mg90_fit"), win_len),
        "%s - 1 mm" % wall_y, "-(%s) / 2" % win_h),
    cyl("cap_seat", p("bearing_seat_d"), p("bearing_t"), shaft_x, cap_y0, "0 mm", along_y=True),
    cyl("cap_bore", p("coxa_bore_d"), p("joint_cap_t"), shaft_x, cap_y0, "0 mm", along_y=True),
]
lower_cut.append(box("wall_hole", p("wall_hole_len"), "%s + 2 mm" % p("coxa_wall_t"), p("wall_hole_h"),
                     "%s - %s / 2" % (hole_mid, p("wall_hole_len")), "%s - 1 mm" % wall_y,
                     "-(%s) / 2 - %s - 1.5 mm" % (win_h, p("wall_hole_h"))))
for name, sign in (("tab_hole_a", "-"), ("tab_hole_b", "+")):
    lower_cut.append(cyl(name, p("mg90_screw_pilot_d"), "%s + 2 mm" % p("coxa_wall_t"),
                         "%s %s %s / 2" % (hole_mid, sign, p("mg90_screw_spacing")),
                         "%s - 1 mm" % wall_y, "0 mm", along_y=True))
for n in "12":
    lower_cut.append(cyl("pilot" + n, p("mg90_screw_pilot_d"), p("coxa_boss_h"), p("coxa_boss_x" + n), wall_mid, boss_z))
# No lattice in these plates: they carry the twist. FEA with holes: 11.9 MPa over most of the part
# and 0.67 mm against 8.0 and 0.48, to save about 0.3 g.
lower = P.cut("CoxaLink_Lower", lower_solid, lower_cut, PLA)

# ------------------------------------------------------------ top print
top_z = p("coxa_top_z")
block_end = p("coxa_block_end")
pocket_z = p("coxa_top_low_z")
block_h = "%s - %s" % (top_z, pocket_z)
top_solid = [
    box("top_beam", end, W, beam_t, "0 mm", half_w, wall_top),
    box("top_block", block_end, W, block_h, "0 mm", half_w, pocket_z),
    cyl("top_round", W, block_h, "0 mm", "0 mm", pocket_z),
    # thick horn block tapering into the plate
    P.gusset("top_taper", (block_end, half_w, wall_top), ("x", 1), ("z", -1), p("arm_taper_len"), "%s - %s" % (wall_top, pocket_z), "y", W),
]
arm = "%s - %s / 2" % (p("horn_reach"), p("horn_tip_w"))
through = "%s + 1 mm" % block_h
top_cut = [
    box("pk_arm", arm, p("horn_tip_w"), p("coxa_horn_pocket"), "0 mm", "-%s / 2" % p("horn_tip_w"), pocket_z),
    cyl("pk_tip", p("horn_tip_w"), p("coxa_horn_pocket"), arm, "0 mm", pocket_z),
    cyl("pk_hub", p("horn_hub_d"), p("coxa_horn_pocket"), "0 mm", "0 mm", pocket_z),
    cyl("access", p("coxa_access_d"), through, "0 mm", "0 mm", pocket_z),
    cyl("horn_screw_a", p("horn_screw_d"), through, p("horn_screw_r1"), "0 mm", pocket_z),
    cyl("horn_screw_b", p("horn_screw_d"), through, p("horn_screw_r2"), "0 mm", pocket_z),
]
for n in "12":
    top_cut.append(cyl("plate_screw" + n, p("coxa_plate_screw_d"), "%s + 1 mm" % beam_t, p("coxa_boss_x" + n), wall_mid, wall_top))
top = P.cut("CoxaLink_Top", top_solid, top_cut, PLA_TOP)
common["save"](doc)


# ------------------------------------------------------------ the link among its dummies
def link(name, target, base, rot=None):
    obj = doc.addObject("App::Link", "CoxaCheck_" + name)
    check.addObject(obj)
    obj.LinkedObject = target
    for axis, expr in zip("xyz", base):
        obj.setExpression("Placement.Base." + axis, expr)
    if rot:
        obj.Placement = App.Placement(obj.Placement.Base, rot)
    return obj


BEARING_IN_CAP = App.Rotation(App.Vector(1, 0, 0), -90)
link("lower", lower, ("0 mm", "0 mm", "0 mm"))
link("top", top, ("0 mm", "0 mm", "0 mm"))
link("coxa_servo", doc.getObject("Dummy_MG90S"), ("0 mm", "0 mm", p("coxa_servo_seat_z")))
link("coxa_horn", doc.getObject("Dummy_Horn"), ("0 mm", "0 mm", p("coxa_servo_seat_z")))
link("femur_servo", doc.getObject("Dummy_MG90S"), (shaft_x, seat_y, "0 mm"), SERVO_IN_LEG)
link("femur_horn", doc.getObject("Dummy_Horn"), (shaft_x, seat_y, "0 mm"), SERVO_IN_LEG.multiply(App.Rotation(App.Vector(0, 0, 1), 90)))
link("bearing", doc.getObject("Dummy_Bearing"), ("0 mm", "0 mm", low_z))
link("femur_bearing", doc.getObject("Dummy_Bearing"), (shaft_x, cap_y0, "0 mm"), BEARING_IN_CAP)
common["save"](doc)

# ------------------------------------------------------------ report
S = doc.getObject("Params")


def val(alias):
    v = S.get(alias)
    return float(getattr(v, "Value", v))


SERVO = ["MG90S_body", "MG90S_tabs", "MG90S_turret", "MG90S_shaft"]
HORN = ["Horn_hub", "Horn_arm", "Horn_tip"]
solids_of, clash = common["solids_of"], common["clash"]
link_lower, link_top = solids_of(doc, ["CoxaLink_Lower"]), solids_of(doc, ["CoxaLink_Top"])
coxa_seat = App.Placement(App.Vector(0, 0, val("coxa_servo_seat_z")), App.Rotation())
femur_seat = App.Placement(App.Vector(val("coxa_len"), val("coxa_seat_y"), 0), SERVO_IN_LEG)
femur_horn = femur_seat.multiply(App.Placement(App.Vector(), App.Rotation(App.Vector(0, 0, 1), 90)))
report = {
    "femur servo": clash(solids_of(doc, SERVO, femur_seat), link_lower + link_top),
    "coxa horn": clash(solids_of(doc, HORN, coxa_seat), link_lower + link_top),
    "coxa bearing": clash(solids_of(doc, ["Bearing_body"], App.Placement(App.Vector(0, 0, val("coxa_low_z")), App.Rotation())), link_lower),
    "femur bearing": clash(solids_of(doc, ["Bearing_body"], App.Placement(App.Vector(val("coxa_len"), val("joint_cap_y0"), 0), BEARING_IN_CAP)), link_lower),
}
worst = 0.0
for deg in range(-60, 61, 20):
    place = App.Placement(App.Vector(0, 0, val("coxa_servo_seat_z")), App.Rotation(App.Vector(0, 0, 1), deg))
    worst = max(worst, clash(solids_of(doc, SERVO, place), link_lower + link_top))
report["coxa servo, link +-60 deg"] = worst
print("lower %.1f g solid, top %.1f g solid; valid %s %s; solids %d %d" % (
    lower.Shape.Volume / 1000 * val("pla_density"), top.Shape.Volume / 1000 * val("pla_density"),
    lower.Shape.isValid(), top.Shape.isValid(), len(lower.Shape.Solids), len(top.Shape.Solids)))
print("overlap mm3: " + ", ".join("%s %.3f" % kv for kv in report.items()))
