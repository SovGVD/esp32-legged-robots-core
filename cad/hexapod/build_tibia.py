"""The tibia: 2 halves split on the leg plane, joined with the stock MG90S screws, and
a presser. A real shape, not stacked boxes: from the front a rounded triangle from
the knee circle to the tip circle; from the side a solid wedge from each knee arm down
to the foot; the tip itself round, the same circle as the half-sphere foot. The switch sits inside the tip; its leads bend up into a hollow in the
middle of the split, which opens into the fork, so no slot cuts an outer wall.

  Tibia_Horn     +Y half, the tibia servo's horn is glued in its pocket
  Tibia_Bearing  -Y half, the femur's knee shaft runs in the bearing in it
  Tibia_Presser  the half-sphere foot, trapped between the halves by its flange

Each half prints flat on its split face. Right leg; the _L mirrors are the left leg's.
Tibia frame: origin on the knee axis, -Z down the tibia, horn on +Y.

Sizes come from Params when this script runs: re-run it after changing them.
"""
import os
import runpy

import FreeCAD as App
import Part

HERE = os.path.dirname(os.path.abspath(__file__))
common = runpy.run_path(os.path.join(HERE, "common.py"))
doc = common["get_doc"]()
COL_A, COL_B, COL_TIP = (0.35, 0.65, 0.35), (0.72, 0.88, 0.72), (0.25, 0.25, 0.25)
KNEE_SERVO = App.Rotation(App.Matrix(0, 1, 0, 0,  0, 0, 1, 0,  1, 0, 0, 0,  0, 0, 0, 1))
HIP_SERVO = KNEE_SERVO.multiply(App.Rotation(App.Vector(0, 0, 1), 180))
HORN_DOWN = HIP_SERVO
BEARING_ALONG_Y = App.Rotation(App.Vector(1, 0, 0), -90)
SWITCH_DOWN = App.Rotation(App.Vector(1, 0, 0), 180)
S = doc.getObject("Params")


def v(alias):
    x = S.get(alias)
    return float(getattr(x, "Value", x))


common["remove_built"](doc, ("TibiaCheck", "TibiaPrim", "Tibia"))
part = doc.addObject("App::Part", "Tibia")
part.setExpression("Placement.Base.x", "2 * Params.catalog_part_pitch")
part.setExpression("Placement.Base.y", "Params.catalog_row2_y")
check = doc.addObject("App::Part", "Tibia_Check")
check.setExpression("Placement.Base.x", "2 * Params.catalog_part_pitch")
check.setExpression("Placement.Base.y", "Params.catalog_row2_y * 3")
prism, block, rod = common["prism"], common["block"], common["rod"]

w = v("tib_arm_w") / 2
foot_r, foot_z, end_z = v("tip_foot_r"), v("tip_foot_z"), v("tib_shank_end_z")
half_t = v("tib_half_t")
low_y = v("tib_horn_low_y")
top_y = v("joint_horn_y") + v("tib_horn_roof")
y1 = v("tib_bear_y1")
seat_end = v("tib_bear_seat_end")
y0 = seat_end - v("tib_bear_roof")

# the triangle from the front: knee circle to tip circle
front = prism([(-w, w, w), (w, w, w), (foot_r, end_z, 0), (-foot_r, end_z, 0)], "xz", y0 - 1, top_y + 1)
# the wedges from the side: rounded corners all the way
side_a = prism([(0, v("tib_a_apex_z"), 0), (low_y, v("tib_a_inner_z"), 8), (low_y, w + 1, 0), (top_y, w + 1, 0),
                (top_y, v("tib_a_slope_z"), 25), (half_t, end_z, 0), (0, end_z, 0)], "yz", -w - 1, w + 1)
side_b = prism([(0, v("tib_b_apex_z"), 0), (y1, v("tib_b_inner_z"), 8), (y1, w + 1, 0), (y0, w + 1, 0),
                (y0, v("tib_b_slope_z"), 25), (-half_t, end_z, 0), (0, end_z, 0)], "yz", -w - 1, w + 1)
# the tip is round, the same circle as the half-sphere foot, widening as it goes up
cone_z, cone_r = v("tib_tip_cone_z"), v("tib_tip_cone_r")
round_tip = Part.makeCone(foot_r, cone_r, cone_z - end_z, App.Vector(0, 0, end_z)).fuse(block(-40, 40, -40, 40, cone_z, w + 2))
solid_a = front.common(side_a).common(round_tip).removeSplitter()
solid_b = front.common(side_b).common(round_tip).removeSplitter()

# ------------------------------------------------------------ the tip's insides, in both halves
hw = v("tip_hole_w") / 2
cz0, cz1 = v("tip_cavity_z0"), v("tip_cavity_z1")
body_z, top_z = v("switch_body_z"), v("switch_top_z")
cx = v("tip_cavity_x") / 2
bx = v("switch_btn_bore") / 2
tip_cuts = [
    rod(v("tip_hole_w"), cz0 - end_z + 1, (0, 0, end_z - 1), "z"),   # round, for a thick wall around it
    block(-cx, cx, -hw, hw, cz0, cz1),
    block(-bx, bx, -hw, hw, cz1, body_z),
    block(-hw, hw, -hw, hw, body_z, top_z),
    block(-v("wire_void_x") / 2, v("wire_void_x") / 2, -v("wire_void_y") / 2, v("wire_void_y") / 2, top_z, v("wire_exit_z") + 8),
]
screws = [(v("tib_screw_a_x"), v("tib_screw_a_z")), (-v("tib_screw_a_x"), v("tib_screw_a_z")),
          (v("tib_screw_b_x"), v("tib_screw_b_z")), (-v("tib_screw_b_x"), v("tib_screw_b_z"))]
arm = v("horn_reach") - v("horn_tip_w") / 2
tip_w = v("horn_tip_w")
pocket = v("horn_hub_h") + v("horn_fit")      # the horn plus a little room for glue
cuts_a = list(tip_cuts) + [
    block(-v("horn_pocket_w") / 2, v("horn_pocket_w") / 2, low_y, low_y + pocket, -arm, 0),
    rod(v("horn_pocket_w"), pocket, (0, low_y, -arm), "y"),
    rod(v("horn_pocket_d"), pocket, (0, low_y, 0), "y"),
    rod(5.0, top_y - low_y + 1, (0, low_y, 0), "y"),
]
for x, z in screws:
    cuts_a.append(rod(v("tib_screw_clear"), v("tib_screw_head_y") + 1, (x, -1, z), "y"))
    cuts_a.append(rod(v("tib_screw_head_d"), top_y, (x, v("tib_screw_head_y"), z), "y"))
cuts_b = list(tip_cuts) + [
    rod(v("bearing_seat_d"), y1 - seat_end + 1, (0, seat_end, 0), "y"),
    rod(v("fem_boss_d"), v("tib_bear_roof") + 1, (0, y0 - 1, 0), "y"),
]
for x, z in screws:
    cuts_b.append(rod(v("tib_screw_d"), half_t + 1, (x, -half_t - 1, z), "y"))

half_a = solid_a.cut(Part.makeCompound(cuts_a)).removeSplitter()
half_b = solid_b.cut(Part.makeCompound(cuts_b)).removeSplitter()
print("raw solids", len(half_a.Solids), len(half_b.Solids), half_a.isValid(), half_b.isValid())

# ------------------------------------------------------------ round the outer edges
edge_r = v("coxa_edge_r")


def keep_sharp(e):
    bb = e.BoundBox
    on_split = bb.YLength < 1e-4 and abs(bb.YMin) < 1e-4
    inside_tip = abs(bb.XMin) <= cx + 0.2 and abs(bb.XMax) <= cx + 0.2 and abs(bb.YMin) <= hw + 0.2 and abs(bb.YMax) <= hw + 0.2
    pocket_area = abs(bb.XMin) <= v("horn_hub_d") / 2 + 0.2 and abs(bb.XMax) <= v("horn_hub_d") / 2 + 0.2 and bb.YMin > low_y - 0.1 and bb.ZMin > -v("horn_reach") - 0.2
    near_knee = bb.YMax < y1 + 0.1 and abs(bb.XMin) <= 7 and abs(bb.XMax) <= 7 and abs(bb.ZMin) <= 7 and abs(bb.ZMax) <= 7
    screw = e.Curve.TypeId == "Part::GeomCircle" and e.Curve.Radius < 3
    return on_split or inside_tip or pocket_area or near_knee or screw


print("horn half")
horn_shape = common["smooth"](half_a, edge_r, keep_sharp)
print("bearing half")
bear_shape = common["smooth"](half_b, edge_r, keep_sharp)
horn = common["as_part"](doc, part, "Tibia_Horn", horn_shape, COL_A)
bear = common["as_part"](doc, part, "Tibia_Bearing", bear_shape, COL_B)
common["add_mirror"](doc, part, horn, COL_A)
common["add_mirror"](doc, part, bear, COL_B)

# ------------------------------------------------------------ the presser
stem = v("tip_stem_w") / 2
ball = Part.makeSphere(foot_r, App.Vector(0, 0, foot_z), App.Vector(0, 0, 1), -90, 0, 360)
presser_shape = ball.fuse([rod(v("tip_stem_w"), v("tip_flange_top") - foot_z, (0, 0, foot_z), "z"),
                           block(-v("tip_flange_x") / 2, v("tip_flange_x") / 2, -stem, stem, cz0, v("tip_flange_top"))]).removeSplitter()
presser = common["as_part"](doc, part, "Tibia_Presser", presser_shape, COL_TIP)
common["save"](doc)


def link(name, target, base, rot=None):
    obj = doc.addObject("App::Link", "TibiaCheck_" + name)
    check.addObject(obj)
    obj.LinkedObject = target
    obj.Placement = App.Placement(App.Vector(*base), rot or App.Rotation())
    return obj


s_y, f_len = v("joint_seat_y"), v("femur_len")
cap = v("fem_floor_y0") - v("fem_boss_h") - v("bearing_t")
link("horn", horn, (0, 0, 0))
link("bearing", bear, (0, 0, 0))
link("presser", presser, (0, 0, 0))
link("switch", doc.getObject("Dummy_Microswitch"), (0, 0, top_z), SWITCH_DOWN)
link("femur", doc.getObject("Femur_Main"), (-f_len, 0, 0))
link("servo", doc.getObject("Dummy_MG90S"), (0, s_y, 0), KNEE_SERVO)
link("servo_horn", doc.getObject("Dummy_Horn"), (0, s_y, 0), HORN_DOWN)
link("knee_bearing", doc.getObject("Dummy_Bearing"), (0, cap, 0), BEARING_ALONG_Y)
common["save"](doc)

solids_of, clash = common["solids_of"], common["clash"]
SERVO = ["MG90S_body", "MG90S_tabs", "MG90S_turret", "MG90S_shaft"]
HORN = ["Horn_hub", "Horn_arm", "Horn_tip"]
SWITCH = ["Microswitch_body", "Microswitch_button"]   # its leads get bent up into the hollow
a_s, b_s, t_s = solids_of(doc, ["Tibia_Horn"]), solids_of(doc, ["Tibia_Bearing"]), solids_of(doc, ["Tibia_Presser"])
Pl = lambda x, y, z, rot=None: App.Placement(App.Vector(x, y, z), rot or App.Rotation())
sw_place = Pl(0, 0, top_z, SWITCH_DOWN)
d = v("pla_density")
print("horn half %.1f g, bearing half %.1f g, presser %.1f g solid; valid %s %s %s; foot bottom %.2f; press travel %.2f" % (
    horn_shape.Volume / 1000 * d, bear_shape.Volume / 1000 * d, presser_shape.Volume / 1000 * d,
    horn_shape.isValid(), bear_shape.isValid(), presser_shape.isValid(), presser_shape.BoundBox.ZMin, v("tip_travel") - v("tip_gap")))
print("overlap mm3: halves %.3f, switch %.3f, presser %.3f, switch/presser %.3f, servo horn %.3f, knee bearing %.3f" % (
    clash(a_s, b_s), clash(solids_of(doc, SWITCH, sw_place), a_s + b_s), clash(t_s, a_s + b_s), clash(solids_of(doc, SWITCH, sw_place), t_s),
    clash(solids_of(doc, HORN, Pl(0, s_y, 0, HORN_DOWN)), a_s + b_s),
    clash(solids_of(doc, ["Bearing_body"], Pl(0, cap, 0, BEARING_ALONG_Y)), b_s)))
fixed = {"femur": solids_of(doc, ["Femur_Main"], Pl(-f_len, 0, 0)), "knee servo": solids_of(doc, SERVO, Pl(0, s_y, 0, KNEE_SERVO)),
         "hip servo": solids_of(doc, SERVO, Pl(-f_len, s_y, 0, HIP_SERVO))}
hits = {}
for deg in range(-90, 91, 10):
    swing = App.Placement(App.Vector(), App.Rotation(App.Vector(0, 1, 0), deg))
    for mname, names in (("horn half", ["Tibia_Horn"]), ("bearing half", ["Tibia_Bearing"]), ("presser", ["Tibia_Presser"])):
        moved = solids_of(doc, names, swing)
        for fname, f in fixed.items():
            x = clash(moved, f)
            if x > 0.001:
                hits.setdefault((mname, fname), []).append(deg)
print("knee sweep, angles that collide (+ = folding toward the femur): %s" % (hits or "none"))
