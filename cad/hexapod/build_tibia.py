"""The tibia: 2 flat halves split on the leg plane, screwed together, and a presser.

  Tibia_Horn     +Y half, the tibia servo's horn sits in its pocket
  Tibia_Bearing  -Y half, the femur's knee shaft runs in the bearing in it
  Tibia_Presser  the round foot, trapped between the halves by its flange

The switch sits inside the tip, button down; when the foot takes load the presser
rises 0.5 mm and pushes the button. Its 2 wires leave in a groove on the split plane.
Each half prints flat on its split face.

Right leg; the _L mirrors are the left leg's. Tibia frame: origin on the knee axis,
-Z down the tibia, horn on +Y. Tibia_Check holds it with the femur; the end of this
script swings the knee looking for collisions.
"""
import os
import runpy

import FreeCAD as App

HERE = os.path.dirname(os.path.abspath(__file__))
common = runpy.run_path(os.path.join(HERE, "common.py"))
doc = common["get_doc"]()
COL_A, COL_B, COL_TIP = (0.35, 0.65, 0.35), (0.72, 0.88, 0.72), (0.25, 0.25, 0.25)
KNEE_SERVO = App.Rotation(App.Matrix(0, 1, 0, 0,  0, 0, 1, 0,  1, 0, 0, 0,  0, 0, 0, 1))
HORN_DOWN = KNEE_SERVO.multiply(App.Rotation(App.Vector(0, 0, 1), 180))
BEARING_ALONG_Y = App.Rotation(App.Vector(1, 0, 0), -90)
SWITCH_DOWN = App.Rotation(App.Vector(1, 0, 0), 180)


def p(alias):
    return "Params." + alias


common["remove_built"](doc, ("TibiaCheck", "TibiaPrim", "Tibia"))
part = doc.addObject("App::Part", "Tibia")
part.setExpression("Placement.Base.x", "2 * " + p("catalog_part_pitch"))
part.setExpression("Placement.Base.y", p("catalog_row2_y"))
check = doc.addObject("App::Part", "Tibia_Check")
check.setExpression("Placement.Base.x", "2 * " + p("catalog_part_pitch"))
check.setExpression("Placement.Base.y", p("catalog_row2_y") + " * 3")
P = common["Prims"](doc, part, "TibiaPrim")
box, cyl = P.box, P.cyl

aw = p("tib_arm_w")
half_a = "-%s / 2" % aw
arm_t = p("tib_arm_t")
cross_top, cross_t = p("tib_cross_z"), p("tib_cross_t")
cross_low = "%s - %s" % (cross_top, cross_t)
sw, half_t = p("tib_shank_w"), p("tib_half_t")
half_s = "-%s / 2" % sw
end_z = p("tib_shank_end_z")
sh_h = "%s - (%s)" % (cross_low, end_z)
g = p("tib_gusset")
reach = "%s + %s" % (p("horn_reach"), "2.5 mm")

# ------------------------------------------------------------ +Y half: the horn side
low_y = p("tib_horn_low_y")
top_y = "%s + %s" % (p("joint_horn_y"), p("tib_horn_roof"))
block_t = "%s - %s" % (top_y, low_y)
arm_y = "%s - %s" % (top_y, arm_t)
horn_solid = [
    box("a_arm", aw, arm_t, "-(%s)" % cross_low, half_a, arm_y, cross_low),
    box("a_block", aw, block_t, reach, half_a, low_y, "-(%s)" % reach),
    cyl("a_round", aw, block_t, "0 mm", low_y, "0 mm", along_y=True),
    box("a_cross", aw, top_y, cross_t, half_a, "0 mm", cross_low),
    box("a_shank", sw, half_t, sh_h, half_s, "0 mm", end_z),
    P.gusset("a_g1", (half_a, arm_y, cross_top), ("y", -1), ("z", 1), g, g, "x", aw),
    P.gusset("a_g2", (half_s, half_t, cross_low), ("y", 1), ("z", -1), g, g, "x", sw),
]
# ------------------------------------------------------------ -Y half: the bearing side
y1 = p("tib_bear_y1")
seat_end = p("tib_bear_seat_end")
bear_y0 = "%s - %s" % (seat_end, p("tib_bear_roof"))
hub_t = "%s - (%s)" % (y1, bear_y0)
bear_solid = [
    box("b_arm", aw, arm_t, "-(%s)" % cross_low, half_a, "%s - %s" % (y1, arm_t), cross_low),
    box("b_block", aw, hub_t, reach, half_a, bear_y0, "-(%s)" % reach),
    cyl("b_round", aw, hub_t, "0 mm", bear_y0, "0 mm", along_y=True),
    box("b_cross", aw, "-(%s)" % bear_y0, cross_t, half_a, bear_y0, cross_low),
    box("b_shank", sw, half_t, sh_h, half_s, "-" + half_t, end_z),
    P.gusset("b_g1", (half_a, y1, cross_top), ("y", 1), ("z", 1), g, g, "x", aw),
    P.gusset("b_g2", (half_s, "-" + half_t, cross_low), ("y", -1), ("z", -1), g, g, "x", sw),
]

# ------------------------------------------------------------ cuts shared by the shank halves
hole_w = p("tip_hole_w")
half_h = "-%s / 2" % hole_w
cz0, cz1 = p("tip_cavity_z0"), p("tip_cavity_z1")
body_z, top_z = p("switch_body_z"), p("switch_top_z")
wall_cut = lambda name: [
    box(name + "_stem", hole_w, hole_w, "%s - (%s) + 1 mm" % (cz0, end_z), half_h, half_h, "%s - 1 mm" % end_z),
    box(name + "_cavity", p("tip_cavity_x"), hole_w, "%s - (%s)" % (cz1, cz0), "-%s / 2" % p("tip_cavity_x"), half_h, cz0),
    box(name + "_btn", p("switch_btn_bore"), hole_w, "%s - (%s)" % (body_z, cz1), "-%s / 2" % p("switch_btn_bore"), half_h, cz1),
    box(name + "_switch", hole_w, hole_w, "%s - (%s)" % (top_z, body_z), half_h, half_h, body_z),
    # wire groove on the split plane: out of the switch pocket, up and out at the +X side
    # room for the switch's 2 leads, which then get bent and soldered
    box(name + "_leads", "%s / 2 - %s / 2 - 0.4 mm" % (sw, hole_w), "2 * %s" % p("switch_slot_y"), "1.2 mm", "%s / 2" % hole_w,
        "-" + p("switch_slot_y"), "%s - %s - 0.5 mm" % (top_z, p("sw_lead_z"))),
    box(name + "_wire_up", p("wire_w"), p("wire_t"), "%s - (%s)" % (p("wire_exit_z"), body_z), "%s / 2 - 0.1 mm" % hole_w,
        "-%s / 2" % p("wire_t"), body_z),
    box(name + "_wire_out", "%s / 2 - %s / 2 + 1 mm" % (sw, hole_w), p("wire_t"), p("wire_w"), "%s / 2" % hole_w,
        "-%s / 2" % p("wire_t"), "%s - %s" % (p("wire_exit_z"), p("wire_w"))),
]
screws = (("s1", p("tib_screw_a_x"), p("tib_screw_a_z")), ("s2", "-" + p("tib_screw_a_x"), p("tib_screw_a_z")),
          ("s3", p("tib_screw_b_x"), p("tib_screw_b_z")))

# ------------------------------------------------------------ the horn pocket
arm = "%s - %s / 2" % (p("horn_reach"), p("horn_tip_w"))
pocket = p("horn_hub_h")
through = "%s + 1 mm" % block_t
horn_cut = wall_cut("ca") + [
    box("pk_arm", p("horn_tip_w"), pocket, arm, "-%s / 2" % p("horn_tip_w"), low_y, "-(%s)" % arm),
    cyl("pk_tip", p("horn_tip_w"), pocket, "0 mm", low_y, "-(%s)" % arm, along_y=True),
    cyl("pk_hub", p("horn_hub_d"), pocket, "0 mm", low_y, "0 mm", along_y=True),
    cyl("access", "5 mm", through, "0 mm", low_y, "0 mm", along_y=True),
    cyl("horn_screw_a", p("horn_screw_d"), through, "0 mm", low_y, "-" + p("horn_screw_r1"), along_y=True),
    cyl("horn_screw_b", p("horn_screw_d"), through, "0 mm", low_y, "-" + p("horn_screw_r2"), along_y=True),
]
for tag, x, z in screws:
    horn_cut.append(cyl("clear_" + tag, p("tib_screw_clear"), "%s + 2 mm" % half_t, x, "-1 mm", z, along_y=True))

bear_cut = wall_cut("cb") + [
    cyl("seat", p("bearing_seat_d"), "%s - (%s)" % (y1, seat_end), "0 mm", seat_end, "0 mm", along_y=True),
    cyl("bore", p("fem_boss_d"), "%s + 1 mm" % p("tib_bear_roof"), "0 mm", "%s - 1 mm" % bear_y0, "0 mm", along_y=True),
]
for tag, x, z in screws:
    bear_cut.append(cyl("pilot_" + tag, p("tib_screw_d"), "%s + 1 mm" % half_t, x, "-(%s) - 1 mm" % half_t, z, along_y=True))

horn = P.cut("Tibia_Horn_raw", horn_solid, horn_cut, COL_A, mirror=False)
bear = P.cut("Tibia_Bearing_raw", bear_solid, bear_cut, COL_B, mirror=False)

# ------------------------------------------------------------ the presser
stem = p("tip_stem_w")
foot_z = p("tip_foot_z")
tip_solid = [
    box("stem", stem, stem, "%s - (%s)" % (p("tip_flange_top"), foot_z), "-%s / 2" % stem, "-%s / 2" % stem, foot_z),
    box("flange", p("tip_flange_x"), stem, p("tip_flange_h"), "-%s / 2" % p("tip_flange_x"), "-%s / 2" % stem, cz0),
    P.sphere("foot", p("tip_foot_r"), "0 mm", "0 mm", foot_z),
]
presser = doc.addObject("Part::MultiFuse", "Tibia_Presser")
part.addObject(presser)
presser.Shapes = tip_solid
presser.Refine = True
for o in tip_solid:
    o.ViewObject.Visibility = False
presser.ViewObject.ShapeColor = COL_TIP
doc.recompute()

# ------------------------------------------------------------ chamfers on everything but the fits
S = doc.getObject("Params")


def val(alias):
    v = S.get(alias)
    return float(getattr(v, "Value", v))


ch = val("fem_edge_ch")
stem_lim = val("tib_shank_w") / 2 + 0.1   # the lead slots reach out through the shank wall
cav0, top = val("tib_shank_end_z") - 1e-3, val("switch_top_z") + 1e-3
pocket_y = val("tib_horn_low_y") - 1e-3
pocket_z = val("horn_reach") + 0.5
pocket_x = val("horn_hub_d") / 2 + 0.5


def fit_edge(e):
    """Inner parts of the tip and the split face: they must stay exact."""
    bb = e.BoundBox
    in_tip = abs(bb.XMin) <= stem_lim and abs(bb.XMax) <= stem_lim and bb.ZMin >= cav0 and bb.ZMax <= top and abs(bb.YMin) < 4 and abs(bb.YMax) < 4
    on_split = bb.YLength < 1e-4 and abs(bb.YMin) < 1e-4
    # the horn pocket: a chamfer on its inside corners would add material under the horn
    in_pocket = bb.YMin >= pocket_y and bb.ZMin >= -pocket_z and abs(bb.XMin) <= pocket_x and abs(bb.XMax) <= pocket_x and bb.ZMax <= pocket_x
    in_seat = bb.YMax < -19 and abs(bb.XMin) <= 7 and abs(bb.XMax) <= 7 and abs(bb.ZMin) <= 7 and abs(bb.ZMax) <= 7
    return in_tip or on_split or in_pocket or in_seat


def finish(name, raw, color):
    obj = doc.addObject("Part::Chamfer", name)
    part.addObject(obj)
    obj.Base = raw
    print(name)
    obj.Edges = common["chamfer_edges"](raw.Shape, ch, fit_edge)
    raw.ViewObject.Visibility = False
    obj.ViewObject.ShapeColor = color
    mirror = doc.addObject("Part::Mirroring", name + "_L")
    part.addObject(mirror)
    mirror.Source = obj
    mirror.Normal = App.Vector(0, 1, 0)
    mirror.ViewObject.Visibility = False
    mirror.ViewObject.ShapeColor = color
    return obj


horn_final = finish("Tibia_Horn", horn, COL_A)
bear_final = finish("Tibia_Bearing", bear, COL_B)
common["save"](doc)


def link(name, target, base, rot=None):
    obj = doc.addObject("App::Link", "TibiaCheck_" + name)
    check.addObject(obj)
    obj.LinkedObject = target
    for axis, expr in zip("xyz", base):
        obj.setExpression("Placement.Base." + axis, expr)
    if rot:
        obj.Placement = App.Placement(obj.Placement.Base, rot)
    return obj


seat_y = p("joint_seat_y")
bearing_y = "%s - %s - %s" % (p("fem_floor_y0"), p("fem_boss_h"), p("bearing_t"))
link("horn", horn_final, ("0 mm", "0 mm", "0 mm"))
link("bearing", bear_final, ("0 mm", "0 mm", "0 mm"))
link("presser", presser, ("0 mm", "0 mm", "0 mm"))
link("switch", doc.getObject("Dummy_Microswitch"), ("0 mm", "0 mm", top_z), SWITCH_DOWN)
link("femur", doc.getObject("Femur_Main"), ("-" + p("femur_len"), "0 mm", "0 mm"))
link("servo", doc.getObject("Dummy_MG90S"), ("0 mm", seat_y, "0 mm"), KNEE_SERVO)
link("servo_horn", doc.getObject("Dummy_Horn"), ("0 mm", seat_y, "0 mm"), HORN_DOWN)
link("knee_bearing", doc.getObject("Dummy_Bearing"), ("0 mm", bearing_y, "0 mm"), BEARING_ALONG_Y)
common["save"](doc)

# ------------------------------------------------------------ report
solids_of, clash = common["solids_of"], common["clash"]
SERVO = ["MG90S_body", "MG90S_tabs", "MG90S_turret", "MG90S_shaft"]
HORN = ["Horn_hub", "Horn_arm", "Horn_tip"]
SWITCH = ["Microswitch_body", "Microswitch_button", "Microswitch_lead_a", "Microswitch_lead_b"]
a_s, b_s, t_s = solids_of(doc, ["Tibia_Horn"]), solids_of(doc, ["Tibia_Bearing"]), solids_of(doc, ["Tibia_Presser"])
s_y, f_len, cap = val("joint_seat_y"), val("femur_len"), val("fem_floor_y0") - val("fem_boss_h") - val("bearing_t")
Pl = lambda x, y, z, rot: App.Placement(App.Vector(x, y, z), rot)
sw_place = Pl(0, 0, val("switch_top_z"), SWITCH_DOWN)
d = val("pla_density")
print("horn half %.1f g, bearing half %.1f g, presser %.1f g solid; valid %s %s %s; foot bottom %.2f; press travel %.2f" % (
    horn_final.Shape.Volume / 1000 * d, bear_final.Shape.Volume / 1000 * d, presser.Shape.Volume / 1000 * d,
    horn_final.Shape.isValid(), bear_final.Shape.isValid(), presser.Shape.isValid(), presser.Shape.BoundBox.ZMin,
    val("tip_travel") - val("tip_gap")))
print("overlap mm3: halves %.3f, switch %.3f, presser %.3f, switch/presser %.3f, servo horn %.3f, knee bearing %.3f" % (
    clash(a_s, b_s), clash(solids_of(doc, SWITCH, sw_place), a_s + b_s), clash(t_s, a_s + b_s), clash(solids_of(doc, SWITCH, sw_place), t_s),
    clash(solids_of(doc, HORN, Pl(0, s_y, 0, HORN_DOWN)), a_s + b_s),
    clash(solids_of(doc, ["Bearing_body"], Pl(0, cap, 0, BEARING_ALONG_Y)), b_s)))
to_knee = App.Placement(App.Vector(-f_len, 0, 0), App.Rotation())
fixed = {"femur": solids_of(doc, ["Femur_Main"], to_knee), "knee servo": solids_of(doc, SERVO, Pl(0, s_y, 0, KNEE_SERVO)),
         "hip servo": solids_of(doc, SERVO, Pl(-f_len, s_y, 0, KNEE_SERVO.multiply(App.Rotation(App.Vector(0, 0, 1), 180))))}
hits = {}
for deg in range(-90, 91, 10):
    swing = App.Placement(App.Vector(), App.Rotation(App.Vector(0, 1, 0), deg))
    for mname, names in (("horn half", ["Tibia_Horn"]), ("bearing half", ["Tibia_Bearing"]), ("presser", ["Tibia_Presser"])):
        moved = solids_of(doc, names, swing)
        for fname, f in fixed.items():
            v = clash(moved, f)
            if v > 0.001:
                hits.setdefault((mname, fname), []).append((deg, round(v, 1)))
print("knee sweep -90..+90 (+ = folding toward the femur): %s" % (hits or "no collisions"))
