"""The tibia, three prints, built from primitives driven by Params.

  Tibia_Main  pin arm, cross plate, shank with the switch bore
  Tibia_Horn  horn arm on the tibia servo, bolted to the cross plate with 2 screws
  Tibia_Tip   foot tip, floating on a screw across the shank: the load lifts it
              and its stem presses the switch button, as the old TibiaTip did

Right leg; the _L mirrors are the left leg's. Tibia frame: origin on the knee
axis, -Z along the tibia, horn on +Y. Tibia_Check adds the femur and the switch,
and the end of this script swings the knee looking for collisions.
"""
import os
import runpy

import FreeCAD as App

HERE = os.path.dirname(os.path.abspath(__file__))
common = runpy.run_path(os.path.join(HERE, "common.py"))
doc = common["get_doc"]()
PLA = (0.35, 0.65, 0.35)
PLA_ARM = (0.72, 0.88, 0.72)
PLA_TIP = (0.25, 0.25, 0.25)
SERVO_IN_LEG = App.Rotation(App.Matrix(0, 1, 0, 0,  0, 0, 1, 0,  1, 0, 0, 0,  0, 0, 0, 1))
BEARING_IN_CAP = App.Rotation(App.Vector(1, 0, 0), -90)
SWITCH_DOWN = App.Rotation(App.Vector(1, 0, 0), 180)


def p(alias):
    return "Params." + alias


common["remove_built"](doc, ("Tibia", "TibiaPrim", "TibiaCheck"))
part = doc.addObject("App::Part", "Tibia")
part.setExpression("Placement.Base.x", "2 * " + p("catalog_part_pitch"))
part.setExpression("Placement.Base.y", p("catalog_row2_y"))
check = doc.addObject("App::Part", "Tibia_Check")
check.setExpression("Placement.Base.x", "2 * " + p("catalog_part_pitch"))
check.setExpression("Placement.Base.y", p("catalog_row2_y") + " * 3")
P = common["Prims"](doc, part, "TibiaPrim")
box, cyl = P.box, P.cyl

W = p("coxa_beam_w")
half_w = "-%s / 2" % W
arm_y0, arm_y1 = p("joint_arm_y0"), p("joint_arm_y1")
beam_y, top_y, low_y = p("joint_beam_y"), p("joint_arm_top_y"), p("joint_horn_low_y")
cross_top = p("tibia_cross_top_z")
cross_low = "%s - %s" % (cross_top, p("tibia_cross_t"))
sw = p("tibia_shank_w")
half_sw = "-%s / 2" % sw
sh_end = p("tibia_shank_end_z")
sh_h = "%s - (%s)" % (cross_low, sh_end)
ch = p("print_chamfer")
boss_top = "%s + %s" % (cross_top, p("tibia_boss_h"))
bore = p("switch_fit")
beam_t = p("coxa_beam_t")

# ------------------------------------------------------------ main print
main_solid = [
    box("arm", W, p("joint_arm_t"), "-(%s)" % cross_low, half_w, arm_y0, cross_low),
    cyl("hub", p("joint_hub_d"), p("joint_arm_t"), "0 mm", arm_y0, "0 mm", along_y=True),
    cyl("pin", p("coxa_pin_d"), p("joint_pin_len"), "0 mm", arm_y1, "0 mm", along_y=True),
    box("cross", W, "%s - (%s)" % (beam_y, arm_y0), p("tibia_cross_t"), half_w, arm_y0, cross_low),
    box("boss", W, "%s - %s" % (beam_y, p("tibia_boss_y0")), p("tibia_boss_h"), half_w, p("tibia_boss_y0"), cross_top),
    box("shank", sw, sw, sh_h, half_sw, half_sw, sh_end),
    # internal chamfers: shank into the cross plate, arm and boss onto the cross plate
    P.gusset("ch_sh_a", (half_sw, "(%s) / 2" % sw, cross_low), ("y", 1), ("z", -1), ch, ch, "x", sw),
    P.gusset("ch_sh_b", (half_sw, half_sw, cross_low), ("y", -1), ("z", -1), ch, ch, "x", sw),
    P.gusset("ch_arm", (half_w, arm_y1, cross_top), ("y", 1), ("z", 1), ch, ch, "x", W),
    P.gusset("ch_boss", (half_w, p("tibia_boss_y0"), cross_top), ("y", -1), ("z", 1), ch, ch, "x", W),
]
slot_y = p("switch_slot_y")
slot_x = "%s / 2 - 0.1 mm" % bore
main_cut = [
    # the switch comes in from below and stops at the end of the bore
    box("bore", bore, bore, "%s - (%s)" % (p("switch_top_z"), sh_end), "-%s / 2" % bore, "-%s / 2" % bore, sh_end),
    box("lead_slot", "%s / 2 - (%s) + 1 mm" % (sw, slot_x), "2 * %s" % slot_y, "%s - (%s) - 0.5 mm" % (cross_low, p("switch_btn_z")),
        slot_x, "-" + slot_y, p("switch_btn_z")),
    cyl("tip_screw", p("mg90_screw_pilot_d"), "%s + 2 mm" % sw, "0 mm", "%s - 1 mm" % half_sw, p("tip_screw_z"), along_y=True),
]
pilot_y = "%s - %s" % (beam_y, p("coxa_boss_h"))
screw_z = "%s + %s / 2" % (cross_top, p("tibia_boss_h"))
for name, x in (("pilot_a", p("tibia_screw_x")), ("pilot_b", "-" + p("tibia_screw_x"))):
    main_cut.append(cyl(name, p("mg90_screw_pilot_d"), p("coxa_boss_h"), x, pilot_y, screw_z, along_y=True))
main = P.cut("Tibia_Main", main_solid, main_cut, PLA)

# ------------------------------------------------------------ horn arm print: long horn arm along Z
tail = p("coxa_tail_c")
block_t = "%s - %s" % (top_y, low_y)
horn_solid = [
    box("h_beam", W, beam_t, "-(%s)" % cross_top, half_w, beam_y, cross_top),
    box("h_block", W, block_t, "2 * %s + %s / 2" % (tail, W), half_w, low_y, "-(%s + %s / 2)" % (tail, W)),
    cyl("h_tail", W, block_t, "0 mm", low_y, tail, along_y=True),
]
arm = "%s - %s" % (p("horn_span"), p("horn_tip_w"))
pocket = p("coxa_horn_pocket")
through = "%s + 1 mm" % block_t
horn_cut = [
    box("pk_long", p("horn_tip_w"), pocket, arm, "-%s / 2" % p("horn_tip_w"), low_y, "-(%s) / 2" % arm),
    cyl("pk_tip_a", p("horn_tip_w"), pocket, "0 mm", low_y, "(%s) / 2" % arm, along_y=True),
    cyl("pk_tip_b", p("horn_tip_w"), pocket, "0 mm", low_y, "-(%s) / 2" % arm, along_y=True),
    box("pk_short", p("horn_short_span"), pocket, p("horn_short_w"), "-%s / 2" % p("horn_short_span"), low_y, "-%s / 2" % p("horn_short_w")),
    cyl("pk_hub", p("horn_hub_d"), pocket, "0 mm", low_y, "0 mm", along_y=True),
    cyl("access", p("coxa_access_d"), through, "0 mm", low_y, "0 mm", along_y=True),
    cyl("horn_screw_a", p("horn_screw_d"), through, "0 mm", low_y, p("horn_screw_r"), along_y=True),
    cyl("horn_screw_b", p("horn_screw_d"), through, "0 mm", low_y, "-" + p("horn_screw_r"), along_y=True),
]
for name, x in (("arm_screw_a", p("tibia_screw_x")), ("arm_screw_b", "-" + p("tibia_screw_x"))):
    horn_cut.append(cyl(name, p("coxa_plate_screw_d"), "%s + 1 mm" % beam_t, x, beam_y, screw_z, along_y=True))
horn = P.cut("Tibia_Horn", horn_solid, horn_cut, PLA_ARM)

# ------------------------------------------------------------ tip print
stem = p("tip_stem_w")
foot_z = p("tip_foot_z")
tip_solid = [
    box("stem", stem, stem, "%s - (%s)" % (p("tip_stem_top_z"), foot_z), "-%s / 2" % stem, "-%s / 2" % stem, foot_z),
    P.sphere("foot", p("tip_foot_r"), "0 mm", "0 mm", foot_z),
]
tip_cut = [cyl("tip_hole", p("tip_hole_d"), "%s + 2 mm" % stem, "0 mm", "-%s / 2 - 1 mm" % stem, p("tip_hole_z"), along_y=True)]
tip = P.cut("Tibia_Tip", tip_solid, tip_cut, PLA_TIP)
common["save"](doc)


# ------------------------------------------------------------ the tibia among its neighbours
def link(name, target, base, rot=None):
    obj = doc.addObject("App::Link", "TibiaCheck_" + name)
    check.addObject(obj)
    obj.LinkedObject = target
    for axis, expr in zip("xyz", base):
        obj.setExpression("Placement.Base." + axis, expr)
    if rot:
        obj.Placement = App.Placement(obj.Placement.Base, rot)
    return obj


seat_y = p("coxa_seat_y")
fem_x = "-" + p("femur_len")
link("main", main, ("0 mm", "0 mm", "0 mm"))
link("horn", horn, ("0 mm", "0 mm", "0 mm"))
link("tip", tip, ("0 mm", "0 mm", "0 mm"))
link("switch", doc.getObject("Dummy_Microswitch"), ("0 mm", "0 mm", p("switch_top_z")), SWITCH_DOWN)
link("femur_main", doc.getObject("Femur_Main"), (fem_x, "0 mm", "0 mm"))
link("femur_horn", doc.getObject("Femur_Horn"), (fem_x, "0 mm", "0 mm"))
link("tibia_servo", doc.getObject("Dummy_MG90S"), ("0 mm", seat_y, "0 mm"), SERVO_IN_LEG)
link("tibia_horn", doc.getObject("Dummy_Horn"), ("0 mm", seat_y, "0 mm"), SERVO_IN_LEG)
link("knee_bearing", doc.getObject("Dummy_Bearing"), ("0 mm", p("joint_cap_y0"), "0 mm"), BEARING_IN_CAP)
common["save"](doc)

# ------------------------------------------------------------ report
S = doc.getObject("Params")


def val(alias):
    v = S.get(alias)
    return float(getattr(v, "Value", v))


SERVO = ["MG90S_body", "MG90S_tabs", "MG90S_turret", "MG90S_shaft"]
HORN = ["Horn_hub", "Horn_arm", "Horn_short_arm", "Horn_tip_a", "Horn_tip_b"]
SWITCH = ["Microswitch_body", "Microswitch_button", "Microswitch_lead_a", "Microswitch_lead_b"]
solids_of, clash = common["solids_of"], common["clash"]
seat, cap, fl = val("coxa_seat_y"), val("joint_cap_y0"), val("femur_len")
servo_place = App.Placement(App.Vector(0, seat, 0), SERVO_IN_LEG)
switch_place = App.Placement(App.Vector(0, 0, val("switch_top_z")), SWITCH_DOWN)
main_s, horn_s, tip_s = solids_of(doc, ["Tibia_Main"]), solids_of(doc, ["Tibia_Horn"]), solids_of(doc, ["Tibia_Tip"])
density = val("pla_density")
print("main %.1f g, horn %.1f g, tip %.1f g solid; valid %s %s %s; solids %d %d %d" % (
    main.Shape.Volume / 1000 * density, horn.Shape.Volume / 1000 * density, tip.Shape.Volume / 1000 * density,
    main.Shape.isValid(), horn.Shape.isValid(), tip.Shape.isValid(), len(main.Shape.Solids), len(horn.Shape.Solids), len(tip.Shape.Solids)))
print("overlap mm3: switch/main %.3f, switch/tip %.3f, horn %.3f, main/horn arm %.3f, main/tip %.3f, knee bearing/pin arm %.3f" % (
    clash(solids_of(doc, SWITCH, switch_place), main_s), clash(solids_of(doc, SWITCH, switch_place), tip_s),
    clash(solids_of(doc, HORN, servo_place), main_s + horn_s), clash(main_s, horn_s), clash(main_s, tip_s),
    clash(solids_of(doc, ["Bearing_body"], App.Placement(App.Vector(0, cap, 0), BEARING_IN_CAP)), main_s)))
button_down = val("switch_btn_z")
print("foot bottom z %.2f (tibia_len %.0f); button end %.2f, stem top %.2f, press travel %.2f" % (
    tip.Shape.BoundBox.ZMin, val("tibia_len"), button_down, tip.Shape.BoundBox.ZMax, val("tip_press_travel")))

to_knee = App.Placement(App.Vector(-fl, 0, 0), App.Rotation())
fixed = {
    "femur main": solids_of(doc, ["Femur_Main"], to_knee),
    "femur horn arm": solids_of(doc, ["Femur_Horn"], to_knee),
    "tibia servo": solids_of(doc, SERVO, servo_place),
}
hits = {}
for deg in range(-90, 91, 10):
    swing = App.Placement(App.Vector(), App.Rotation(App.Vector(0, 1, 0), deg))
    for mname, group in (("main", ["Tibia_Main"]), ("horn arm", ["Tibia_Horn"]), ("tip", ["Tibia_Tip"])):
        moved = solids_of(doc, group, swing)
        for fname, f in fixed.items():
            v = clash(moved, f)
            if v > 0.001:
                hits.setdefault((mname, fname), []).append((deg, round(v, 1)))
print("knee sweep -90..+90 (+ = folding toward the femur): %s" % (hits or "no collisions"))
