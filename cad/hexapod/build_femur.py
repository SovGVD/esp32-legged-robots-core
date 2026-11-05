"""The femur, two prints, built from primitives driven by Params.

  Femur_Main  pin arm, web, tibia servo box with its bearing cap
  Femur_Horn  horn arm on the femur servo, bolted to the web with 2 screws

Right leg; the _L mirrors are the left leg's. Femur frame: origin on the femur
axis, +X to the tibia axis at neutral, Z up when level, horn on +Y.
Femur_Check adds the coxa link and the servos, and the end of this script
swings the femur over the servo's whole range looking for collisions.
"""
import os
import runpy

import FreeCAD as App
import Part

HERE = os.path.dirname(os.path.abspath(__file__))
common = runpy.run_path(os.path.join(HERE, "common.py"))
doc = common["get_doc"]()
PLA = (0.2, 0.55, 0.75)
PLA_ARM = (0.65, 0.82, 0.9)
SERVO_IN_LEG = App.Rotation(App.Matrix(0, 1, 0, 0,  0, 0, 1, 0,  1, 0, 0, 0,  0, 0, 0, 1))
BEARING_IN_CAP = App.Rotation(App.Vector(1, 0, 0), -90)


def p(alias):
    return "Params." + alias


common["remove_built"](doc, ("Femur", "FemurPrim", "FemurCheck"))
part = doc.addObject("App::Part", "Femur")
part.setExpression("Placement.Base.x", p("catalog_part_pitch"))
part.setExpression("Placement.Base.y", p("catalog_row2_y"))
check = doc.addObject("App::Part", "Femur_Check")
check.setExpression("Placement.Base.x", p("catalog_part_pitch"))
check.setExpression("Placement.Base.y", p("catalog_row2_y") + " * 3")
P = common["Prims"](doc, part, "FemurPrim")
box, cyl = P.box, P.cyl

W = p("coxa_beam_w")
half_w = "-%s / 2" % W
axis_x = p("femur_len")
web0, web1 = p("femur_web_x0"), p("femur_web_x1")
arm_y0, arm_y1 = p("joint_arm_y0"), p("joint_arm_y1")
cap_y0, cap_y1 = p("joint_cap_y0"), p("joint_cap_y1")
seat_y = p("coxa_seat_y")
wall_y = "%s - %s" % (seat_y, p("coxa_wall_t"))
box_end = p("femur_box_end")
box_h = p("femur_box_h")
half_h = "-%s / 2" % box_h
beam_y = p("joint_beam_y")
top_y = p("joint_arm_top_y")
hub = p("joint_hub_d")
flange_t = p("femur_flange_t")
flange_y = "%s - (%s)" % (p("femur_flange_y1"), cap_y0)
box_len = "%s - %s" % (box_end, web1)
inner_z = "%s / 2 - %s" % (box_h, flange_t)
rib_x0 = "%s + %s + %s" % (axis_x, p("mg90_shaft_end"), p("mg90_fit"))
cap_x0 = "%s - (%s) / 2" % (axis_x, hub)
rib_t = p("joint_rib_t")
rib_x1 = "%s + %s" % (rib_x0, rib_t)
ch = p("print_chamfer")

# ------------------------------------------------------------ main print
main_solid = [
    box("arm", web1, p("joint_arm_t"), W, "0 mm", arm_y0, half_w),
    cyl("hub", hub, p("joint_arm_t"), "0 mm", arm_y0, "0 mm", along_y=True),
    cyl("pin", p("coxa_pin_d"), p("joint_pin_len"), "0 mm", arm_y1, "0 mm", along_y=True),
    box("web", "%s - %s" % (web1, web0), "%s - (%s)" % (beam_y, arm_y0), box_h, web0, arm_y0, half_h),
    box("boss", "%s - %s" % (web0, p("femur_boss_x0")), "%s - %s" % (beam_y, p("femur_boss_y0")), W,
        p("femur_boss_x0"), p("femur_boss_y0"), half_w),
    box("wall", box_len, p("coxa_wall_t"), box_h, web1, wall_y, half_h),
    box("flange_top", box_len, flange_y, flange_t, web1, cap_y0, inner_z),
    box("flange_low", box_len, flange_y, flange_t, web1, cap_y0, half_h),
    box("cap", "%s + %s - (%s)" % (rib_x0, rib_t, cap_x0), p("joint_cap_t"), box_h, cap_x0, cap_y0, half_h),
    box("rib", rib_t, "%s - (%s)" % (wall_y, cap_y1), box_h, rib_x0, cap_y1, half_h),
    # internal chamfers: arm into the web, web into the wall and flanges, rib into the wall and the cap
    P.gusset("ch_arm", (web1, arm_y1, half_w), ("x", 1), ("y", 1), ch, ch, "z", W),
    P.gusset("ch_web_wall", (web1, wall_y, half_h), ("x", 1), ("y", -1), ch, ch, "z", box_h),
    P.gusset("ch_web_top", (web1, cap_y0, inner_z), ("x", 1), ("z", -1), ch, ch, "y", "%s - (%s)" % (wall_y, cap_y0)),
    P.gusset("ch_web_low", (web1, cap_y0, "-(%s)" % inner_z), ("x", 1), ("z", 1), ch, ch, "y", "%s - (%s)" % (wall_y, cap_y0)),
    P.gusset("ch_rib_wall", (rib_x1, wall_y, half_h), ("x", 1), ("y", -1), ch, ch, "z", box_h),
    P.gusset("ch_rib_cap", (rib_x1, cap_y1, half_h), ("x", 1), ("y", 1), ch, ch, "z", box_h),
]
win_len = "%s + 2 * %s" % (p("mg90_body_len"), p("mg90_fit"))
win_h = "%s + 2 * %s" % (p("mg90_body_w"), p("mg90_fit"))
hole_mid = "%s + %s - %s / 2" % (axis_x, p("mg90_shaft_end"), p("mg90_body_len"))
tab_x = "%s + %s - %s / 2 - %s / 2" % (axis_x, p("mg90_shaft_end"), p("mg90_body_len"), p("mg90_tab_total"))
main_cut = [
    # notch in the web for the tibia servo tab
    box("tab_notch", "%s - (%s) + %s + 1 mm" % (web1, tab_x, p("mg90_fit")), "%s + 2 * %s" % (p("mg90_tab_t"), p("mg90_fit")), win_h,
        "%s - %s" % (tab_x, p("mg90_fit")), "%s - %s" % (seat_y, p("mg90_fit")), "-(%s) / 2" % win_h),
    box("window", win_len, "%s + 2 mm" % p("coxa_wall_t"), win_h,
        "%s + %s + %s - (%s)" % (axis_x, p("mg90_shaft_end"), p("mg90_fit"), win_len),
        "%s - 1 mm" % wall_y, "-(%s) / 2" % win_h),
    cyl("cap_seat", p("bearing_seat_d"), p("bearing_t"), axis_x, cap_y0, "0 mm", along_y=True),
    cyl("cap_bore", p("coxa_bore_d"), p("joint_cap_t"), axis_x, cap_y0, "0 mm", along_y=True),
]
for name, sign in (("tab_hole_a", "-"), ("tab_hole_b", "+")):
    main_cut.append(cyl(name, p("mg90_screw_pilot_d"), "%s + 2 mm" % p("coxa_wall_t"),
                        "%s %s %s / 2" % (hole_mid, sign, p("mg90_screw_spacing")),
                        "%s - 1 mm" % wall_y, "0 mm", along_y=True))
pilot_y = "%s - %s" % (beam_y, p("coxa_boss_h"))
for name, z in (("pilot_a", p("femur_screw_z")), ("pilot_b", "-" + p("femur_screw_z"))):
    main_cut.append(cyl(name, p("mg90_screw_pilot_d"), p("coxa_boss_h"), p("femur_screw_x"), pilot_y, z, along_y=True))
main = P.cut("Femur_Main", main_solid, main_cut, PLA)

# ------------------------------------------------------------ horn arm print
tail = p("coxa_tail_c")
low_y = p("joint_horn_low_y")
block_t = "%s - %s" % (top_y, low_y)
beam_t = p("coxa_beam_t")
horn_solid = [
    box("h_beam", web1, beam_t, W, "0 mm", beam_y, half_w),
    box("h_block", "2 * %s + %s / 2" % (tail, W), block_t, W, "-" + tail, low_y, half_w),
    cyl("h_tail", W, block_t, "-" + tail, low_y, "0 mm", along_y=True),
]
arm = "%s - %s" % (p("horn_span"), p("horn_tip_w"))
pocket = p("coxa_horn_pocket")
through = "%s + 1 mm" % block_t
horn_cut = [
    box("pk_long", arm, pocket, p("horn_tip_w"), "-(%s) / 2" % arm, low_y, "-%s / 2" % p("horn_tip_w")),
    cyl("pk_tip_a", p("horn_tip_w"), pocket, "(%s) / 2" % arm, low_y, "0 mm", along_y=True),
    cyl("pk_tip_b", p("horn_tip_w"), pocket, "-(%s) / 2" % arm, low_y, "0 mm", along_y=True),
    box("pk_short", p("horn_short_w"), pocket, p("horn_short_span"), "-%s / 2" % p("horn_short_w"), low_y,
        "-%s / 2" % p("horn_short_span")),
    cyl("pk_hub", p("horn_hub_d"), pocket, "0 mm", low_y, "0 mm", along_y=True),
    cyl("access", p("coxa_access_d"), through, "0 mm", low_y, "0 mm", along_y=True),
    cyl("horn_screw_a", p("horn_screw_d"), through, p("horn_screw_r"), low_y, "0 mm", along_y=True),
    cyl("horn_screw_b", p("horn_screw_d"), through, "-" + p("horn_screw_r"), low_y, "0 mm", along_y=True),
]
for name, z in (("arm_screw_a", p("femur_screw_z")), ("arm_screw_b", "-" + p("femur_screw_z"))):
    horn_cut.append(cyl(name, p("coxa_plate_screw_d"), "%s + 1 mm" % beam_t, p("femur_screw_x"), beam_y, z, along_y=True))
horn = P.cut("Femur_Horn", horn_solid, horn_cut, PLA_ARM)
common["save"](doc)


# ------------------------------------------------------------ the femur among its neighbours
def link(name, target, base, rot=None):
    obj = doc.addObject("App::Link", "FemurCheck_" + name)
    check.addObject(obj)
    obj.LinkedObject = target
    for axis, expr in zip("xyz", base):
        obj.setExpression("Placement.Base." + axis, expr)
    if rot:
        obj.Placement = App.Placement(obj.Placement.Base, rot)
    return obj


coxa_x = "-" + p("coxa_len")
link("main", main, ("0 mm", "0 mm", "0 mm"))
link("horn", horn, ("0 mm", "0 mm", "0 mm"))
link("coxa_lower", doc.getObject("CoxaLink_Lower"), (coxa_x, "0 mm", "0 mm"))
link("coxa_top", doc.getObject("CoxaLink_Top"), (coxa_x, "0 mm", "0 mm"))
HORN_90 = SERVO_IN_LEG.multiply(App.Rotation(App.Vector(0, 0, 1), 90))
link("femur_servo", doc.getObject("Dummy_MG90S"), ("0 mm", seat_y, "0 mm"), SERVO_IN_LEG)
link("femur_horn", doc.getObject("Dummy_Horn"), ("0 mm", seat_y, "0 mm"), HORN_90)
link("tibia_servo", doc.getObject("Dummy_MG90S"), (axis_x, seat_y, "0 mm"), SERVO_IN_LEG)
link("tibia_horn", doc.getObject("Dummy_Horn"), (axis_x, seat_y, "0 mm"), HORN_90)
link("hip_bearing", doc.getObject("Dummy_Bearing"), ("0 mm", cap_y0, "0 mm"), BEARING_IN_CAP)
link("knee_bearing", doc.getObject("Dummy_Bearing"), (axis_x, cap_y0, "0 mm"), BEARING_IN_CAP)
common["save"](doc)

# ------------------------------------------------------------ report
S = doc.getObject("Params")


def val(alias):
    v = S.get(alias)
    return float(getattr(v, "Value", v))


SERVO = ["MG90S_body", "MG90S_tabs", "MG90S_turret", "MG90S_shaft"]
HORN = ["Horn_hub", "Horn_arm", "Horn_short_arm", "Horn_tip_a", "Horn_tip_b"]
solids_of, clash = common["solids_of"], common["clash"]
seat, cap, fl, cl = val("coxa_seat_y"), val("joint_cap_y0"), val("femur_len"), val("coxa_len")
turn_90 = App.Placement(App.Vector(), App.Rotation(App.Vector(0, 0, 1), 90))
hip_place = App.Placement(App.Vector(0, seat, 0), SERVO_IN_LEG)
knee_place = App.Placement(App.Vector(fl, seat, 0), SERVO_IN_LEG)
main_s, horn_s = solids_of(doc, ["Femur_Main"]), solids_of(doc, ["Femur_Horn"])
print("main %.1f g solid, horn %.1f g solid; valid %s %s; solids %d %d" % (
    main.Shape.Volume / 1000 * val("pla_density"), horn.Shape.Volume / 1000 * val("pla_density"),
    main.Shape.isValid(), horn.Shape.isValid(), len(main.Shape.Solids), len(horn.Shape.Solids)))
print("overlap mm3: tibia servo %.3f, tibia horn with main %.3f, femur horn %.3f, knee bearing %.3f, hip bearing %.3f, main with horn arm %.3f" % (
    clash(solids_of(doc, SERVO, knee_place), main_s + horn_s),
    clash(solids_of(doc, HORN, knee_place.multiply(turn_90)), main_s),
    clash(solids_of(doc, HORN, hip_place.multiply(turn_90)), main_s + horn_s),
    clash(solids_of(doc, ["Bearing_body"], App.Placement(App.Vector(fl, cap, 0), BEARING_IN_CAP)), main_s),
    clash(solids_of(doc, ["Bearing_body"], App.Placement(App.Vector(0, cap, 0), BEARING_IN_CAP)), main_s),
    clash(main_s, horn_s)))

# the moving femur against everything that stays with the coxa link
to_femur = App.Placement(App.Vector(-cl, 0, 0), App.Rotation())
fixed = {
    "coxa lower": solids_of(doc, ["CoxaLink_Lower"], to_femur),
    "coxa top": solids_of(doc, ["CoxaLink_Top"], to_femur),
    "femur servo": solids_of(doc, SERVO, hip_place),
    "hip bearing": solids_of(doc, ["Bearing_body"], App.Placement(App.Vector(0, cap, 0), BEARING_IN_CAP)),
}
moving = {
    "main": ["Femur_Main"], "horn arm": ["Femur_Horn"], "tibia servo": SERVO,
}
hits = {}
for deg in range(-90, 91, 10):
    swing = App.Placement(App.Vector(), App.Rotation(App.Vector(0, 1, 0), deg))
    for mname, names in moving.items():
        place = swing.multiply(knee_place) if mname == "tibia servo" else swing
        group = solids_of(doc, names, place)
        for fname, f in fixed.items():
            v = clash(group, f)
            if v > 0.001:
                hits.setdefault((mname, fname), []).append((deg, round(v, 1)))
print("femur sweep -90..+90 (+ = down): %s" % (hits or "no collisions"))
