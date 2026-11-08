"""The tibia: 2 halves split on the leg plane, joined with the stock MG90S screws, and
a presser. A real shape, not stacked boxes: from the front a rounded triangle from
the knee circle to the tip circle; from the side a solid wedge from each knee arm down
to the foot; the tip itself round. The foot switch and its presser are the old (gear)
hexapod's, copied as they are because they work after printing: the empty space around
the switch in the old tibia, and its TibiaTip presser (import_old_tip.py, old_tip/*.brep),
moved onto this tip. The switch's leads bend up into its lead slots and on into a hollow
in the middle of the split, which opens into the fork, so no slot cuts an outer wall.

  Tibia_Horn     +Y half, the tibia servo's horn is glued in its pocket
  Tibia_Bearing  -Y half, the femur's knee shaft runs in the bearing in it
  Tibia_Presser  the old TibiaTip on a bigger foot, held between the halves by a pin through its slot

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
foot_r, foot_z, end_z = v("tib_tip_end_r"), v("tip_foot_z"), v("tib_shank_end_z")
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


def old_tip(name, dy):
    """A shape from old_tip/, moved from the old tibia's frame onto this tip."""
    sh = Part.Shape()
    sh.read(os.path.join(HERE, "old_tip", name))
    sh.translate(App.Vector(0, dy, v("tip_old_dz")))
    return sh


voids = old_tip("voids.brep", -12.0)        # the old tibia's axis was at y 12
vb = voids.BoundBox
top_z = v("switch_top_z")
# only the -X lead slot: the +X one ran up next to the lower screw (the switch's pins on that side get cut off).
# Solid from the middle of the old tongue over the switch to the +X side, level with the tongue's bottom
# on the split face (0.71 above the switch), so the tongue blends into the body
voids = voids.cut(block(0.0, vb.XMax + 1, vb.YMin - 1, vb.YMax + 1, top_z + 0.71, vb.ZMax + 1)).removeSplitter()
tip_cuts = [
    voids,
    block(-v("wire_void_x") / 2, v("wire_void_x") / 2, -v("wire_void_y") / 2, v("wire_void_y") / 2, v("tip_wire_z0"), v("wire_exit_z") + 8),
]
# 2 screws on a diagonal hold the halves (4 crowded the tip with head recesses)
screws = [(v("tib_screw_a_x"), v("tib_screw_a_z")), (-v("tib_screw_b_x"), v("tib_screw_b_z"))]
arm = v("horn_reach") - v("horn_tip_w") / 2
tip_w = v("horn_tip_w")
pocket = v("horn_hub_h") + v("horn_fit")      # the horn plus a little room for glue
cuts_a = list(tip_cuts) + [
    common["horn_pocket"](v("horn_pocket_d") / 2, v("horn_pocket_w") / 2, arm, pocket, (0, low_y, 0), (0, 0, -1), (0, 1, 0)),
    rod(5.0, top_y - low_y + 1, (0, low_y, 0), "y"),
]
for x, z in screws:
    cuts_a.append(rod(v("tib_screw_clear"), v("tib_screw_head_y") + 1, (x, -1, z), "y"))
    cuts_a.append(rod(v("tib_screw_head_d"), top_y, (x, v("tib_screw_head_y"), z), "y"))
cuts_b = list(tip_cuts) + [
    rod(v("bearing_seat_d"), y1 - seat_end + 1, (0, seat_end, 0), "y"),
    rod(v("bearing_push_d"), v("tib_bear_roof") + 1, (0, y0 - 1, 0), "y"),
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
    inside_tip = bb.XMin >= vb.XMin - 0.01 and bb.XMax <= vb.XMax + 0.01 and bb.YMin >= vb.YMin - 0.01 and bb.YMax <= vb.YMax + 0.01
    pocket_area = abs(bb.XMin) <= v("horn_hub_d") / 2 + 0.2 and abs(bb.XMax) <= v("horn_hub_d") / 2 + 0.2 and bb.YMin > low_y - 0.1 and bb.ZMin > -v("horn_reach") - 0.2
    near_knee = bb.YMax < y1 + 0.1 and abs(bb.XMin) <= 7 and abs(bb.XMax) <= 7 and abs(bb.ZMin) <= 7 and abs(bb.ZMax) <= 7
    screw = e.Curve.TypeId == "Part::GeomCircle" and e.Curve.Radius < 3
    return on_split or inside_tip or pocket_area or near_knee or screw


print("horn half")
horn_shape = common["smooth"](half_a, edge_r, keep_sharp)
print("bearing half")
bear_shape = common["smooth"](half_b, edge_r, keep_sharp)


def split_face(e):
    """Edges in the split plane stay sharp (the halves meet there), except the screw holes' mouths."""
    bb = e.BoundBox
    try:
        small_hole = e.Curve.TypeId == "Part::GeomCircle" and 2 * e.Curve.Radius <= v("lead_small_d") + 1e-6
    except Exception:
        small_hole = False
    # inside the tip (flange cavity, the switch's ledges, its pocket) stays sharp: a lead-in there eats the ledges
    in_tip = (bb.XMin >= vb.XMin - 0.01 and bb.XMax <= vb.XMax + 0.01 and bb.YMin >= vb.YMin - 0.01 and bb.YMax <= vb.YMax + 0.01
              and bb.ZMin > end_z + 0.1 and bb.ZMax < vb.ZMax + 0.01)
    return (bb.YLength < 1e-4 and abs(bb.YMin) < 1e-4 and not small_hole) or in_tip


print("lead-ins")
horn_shape = common["lead_ins"](horn_shape, v("lead_small"), v("lead_big"), v("lead_small_d"), split_face)
bear_shape = common["lead_ins"](bear_shape, v("lead_small"), v("lead_big"), v("lead_small_d"), split_face)
horn = common["as_part"](doc, part, "Tibia_Horn", horn_shape, COL_A)
bear = common["as_part"](doc, part, "Tibia_Bearing", bear_shape, COL_B)
common["add_mirror"](doc, part, horn, COL_A)
common["add_mirror"](doc, part, bear, COL_B)

# ------------------------------------------------------------ the presser
# the old TibiaTip above its foot (its axis was already at y 0), on a foot as big as the shank's end:
# the old one was a ball of 4 cut flat 1 mm above its centre (old z -105), this one the same with tip_foot_r
_old = Part.Shape()
_old.read(os.path.join(HERE, "old_tip", "presser.brep"))
# (|y| <= 2.95: drops its "forPrinter" pad, a 0.1 x 0.1 x 3 sliver that pokes out of the stem into the tibia)
_top = _old.common(Part.makeBox(20, 5.9, 20, App.Vector(-10, -2.95, -105.0)))
_ball = Part.makeSphere(v("tip_foot_r"), App.Vector(0, 0, -106.0)).common(Part.makeBox(30, 30, 30, App.Vector(-15, -15, -135.0)))
presser_shape = _top.fuse(_ball).removeSplitter()
presser_shape.translate(App.Vector(0, 0, v("tip_old_dz")))
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
print("horn half %.1f g, bearing half %.1f g, presser %.1f g solid; valid %s %s %s; foot bottom %.2f" % (
    horn_shape.Volume / 1000 * d, bear_shape.Volume / 1000 * d, presser_shape.Volume / 1000 * d,
    horn_shape.isValid(), bear_shape.isValid(), presser_shape.isValid(), presser_shape.BoundBox.ZMin))
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
