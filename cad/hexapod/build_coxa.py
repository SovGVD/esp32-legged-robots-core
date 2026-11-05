"""The coxa link: one print carrying the coxa servo. A real shape, not stacked boxes: from
above a horseshoe, round around the coxa axis and curving out to the fork around the
femur; from the side an arch under it. The servo sits in a pocket cut into that.

The servo's horn fixes to the body's top plate. A floor under the servo carries a shaft
that runs in the bearing in the body's bottom plate. Horns are glued into their pockets.
In the fork the femur servo's horn sits in the +Y arm and the femur's shaft runs in a bearing in the -Y arm. The cables of
the 3 servos lie in a groove in the top face, held by 2 small bridges.

Right leg; Coxa_Main_L is the left leg's. Leg frame: origin on the coxa axis at femur axis
height, +X out along the link, Z up. Sizes come from Params when this script runs:
re-run it after changing them.
"""
import os
import runpy

import FreeCAD as App
import Part

HERE = os.path.dirname(os.path.abspath(__file__))
common = runpy.run_path(os.path.join(HERE, "common.py"))
doc = common["get_doc"]()
PLA = (0.88, 0.63, 0.11)
SERVO_ROT = App.Rotation(App.Vector(0, 0, 1), 90)       # servo Y to -X: its body runs toward +X, shaft up
HORN_IN = App.Rotation(App.Vector(0, 0, 1), 180)        # coxa horn arm toward the body
KNEE = App.Rotation(App.Matrix(0, 1, 0, 0,  0, 0, 1, 0,  1, 0, 0, 0,  0, 0, 0, 1))
FEMUR_HORN = KNEE.multiply(App.Rotation(App.Vector(0, 0, 1), -90))
HIP_SERVO = KNEE.multiply(App.Rotation(App.Vector(0, 0, 1), 180))
BEARING_ALONG_Y = App.Rotation(App.Vector(1, 0, 0), -90)
S = doc.getObject("Params")


def v(alias):
    x = S.get(alias)
    return float(getattr(x, "Value", x))


common["remove_built"](doc, ("CoxaCheck", "CoxaPrim", "Coxa"))
part = doc.addObject("App::Part", "Coxa")
part.setExpression("Placement.Base.y", "Params.catalog_row2_y")
check = doc.addObject("App::Part", "Coxa_Check")
check.setExpression("Placement.Base.y", "Params.catalog_row2_y * 3")
prism, block, rod = common["prism"], common["block"], common["rod"]

fl = v("coxa_len")
arm_r = v("coxa_arm_w") / 2
end_x = fl + arm_r
a_y0, a_y1 = v("coxa_arm_a_y0"), v("coxa_arm_a_y1")
b_y1 = v("coxa_arm_b_y1")
b_y0 = v("coxa_arm_b_y0")
rear = v("coxa_rear_r")
waist_x, waist_r = v("coxa_waist_x"), v("coxa_waist_r")
fork_x = v("coxa_cross_x1")
fork_r = v("coxa_fork_r")
z0, z1 = v("coxa_plate_z0"), v("coxa_plate_z1")
floor_z0 = v("coxa_floor_z0")
floor_in = floor_z0 + v("coxa_floor_t")
seat = v("coxa_seat_z")

# the horseshoe from above
straight = v("coxa_straight_x")
above = prism([(-rear, rear, rear), (straight, rear, 25), (waist_x, a_y1, 25), (end_x, a_y1, 0), (end_x, a_y0, 0), (fork_x, a_y0, fork_r),
               (fork_x, b_y1, fork_r), (end_x, b_y1, 0), (end_x, b_y0, 0), (waist_x, b_y0, 25), (straight, -rear, 25), (-rear, -rear, rear)],
              "xy", floor_z0 - 1, arm_r + 1)
# the arch from the side, with a round end around the femur axis
side = prism([(-rear, z1, 3), (v("coxa_top_x"), z1, 20), (fork_x, arm_r, 20), (end_x, arm_r, arm_r), (end_x, -arm_r, arm_r),
              (fork_x, -arm_r, 12), (v("coxa_arch_x1"), z0, 10), (v("coxa_arch_x0"), floor_z0, 6), (-rear, floor_z0, 4)],
             "xz", b_y0 - 1, a_y1 + 1)
shape = above.common(side).removeSplitter()

# ------------------------------------------------------------ cut the pockets into it
fit = v("mg90_fit")
hw = v("mg90_body_w") / 2 + fit
near = v("mg90_shaft_end") + fit
far = v("mg90_body_len") - v("mg90_shaft_end") + fit
tab_near = v("mg90_tab_near") + fit
tab_far = v("mg90_tab_far") + fit
tip_w = v("horn_tip_w")
pocket = v("horn_hub_h")
arm = v("horn_reach") - tip_w / 2
through = a_y1 - a_y0 + 1
cuts = [
    block(-near, far, -hw, hw, floor_in, z1 + 1),                # the servo's body, down to the floor
    block(-tab_near, tab_far, -hw, hw, seat, z1 + 1),            # its tabs
    rod(v("mg90_screw_pilot_d"), v("fem_plate_floor") + 1, (-v("mg90_hole_near"), 0, z0 - 1), "z"),
    rod(v("mg90_screw_pilot_d"), v("fem_plate_floor") + 1, (v("mg90_hole_far"), 0, z0 - 1), "z"),
    # the femur servo's horn, its arm toward the coxa axis
    block(fl - arm, fl, a_y0, a_y0 + pocket, -tip_w / 2, tip_w / 2),
    rod(tip_w, pocket, (fl - arm, a_y0, 0), "y"),
    rod(v("horn_hub_d"), pocket, (fl, a_y0, 0), "y"),
    rod(5.0, through, (fl, a_y0, 0), "y"),
    # the femur's bearing
    rod(v("bearing_seat_d"), b_y1 - v("tib_bear_seat_end") + 1, (fl, v("tib_bear_seat_end"), 0), "y"),
    rod(v("fem_boss_d"), v("tib_bear_roof") + 1, (fl, b_y0 - 1, 0), "y"),
]
# the cable groove in the top face, running out through the fork wall
g_y0 = v("coxa_groove_y")
g_y1 = g_y0 + v("cable_w") + 0.2
g_x0 = v("coxa_groove_x0")
cuts.append(block(g_x0, fork_x + 1, g_y0, g_y1, z1 - v("coxa_groove_d"), arm_r + 2))
# hollow it from underneath, leaving walls all around and the top under the cable groove
t = v("coxa_shell_t")
hx0, hx1 = v("coxa_hollow_x0"), v("coxa_hollow_x1")
hy = v("coxa_hollow_y0")
cuts.append(prism([(hx0, g_y0 - t, 3), (hx1, g_y0 - t, 3), (hx1, b_y0 + t, 5), (hx0, -hy, 4)], "xy", floor_z0 - 2, z1 - t))
# under the plate the rear is narrower: it swings next to the board's pins
skirt = block(-rear - 1, v("coxa_arch_x0") + 1, -rear - 8, rear + 8, floor_z0 - 2, z0).cut(
    prism([(-v("coxa_skirt_x"), 9.2, 4), (v("coxa_arch_x0") + 2, 9.2, 0), (v("coxa_arch_x0") + 2, -9.2, 0), (-v("coxa_skirt_x"), -9.2, 4)], "xy", floor_z0 - 3, z0 + 1))
cuts.append(skirt)
shape = shape.cut(Part.makeCompound(cuts)).removeSplitter()
# 2 small bridges over the groove
bridges = [block(x, x + 3, g_y0 - 0.1, g_y1 + 0.1, z1 - 1.4, z1) for x in (g_x0 + 6, g_x0 + 16)]
# the shaft under the floor, on its spacer
shaft = rod(v("fem_boss_d"), v("fem_boss_h"), (0, 0, floor_z0 - v("fem_boss_h")), "z").fuse(
    rod(v("coxa_shaft_d"), v("fem_shaft_len"), (0, 0, floor_z0 - v("fem_boss_h") - v("fem_shaft_len")), "z"))
shape = shape.fuse(bridges + [shaft]).removeSplitter()
print("raw solids", len(shape.Solids), shape.isValid())

# ------------------------------------------------------------ round the outer edges


def keep_sharp(e):
    bb = e.BoundBox
    servo = abs(bb.YMin) <= hw + 0.2 and abs(bb.YMax) <= hw + 0.2 and bb.ZMin >= floor_in - 0.2 and bb.XMin >= -tab_near - 0.2 and bb.XMax <= tab_far + 0.2
    horn = bb.YMin >= a_y0 - 0.1 and bb.YMax <= a_y0 + pocket + 0.1 and bb.XMin >= fl - v("horn_reach") - 0.2
    bearing = bb.YMax < b_y1 + 0.1 and abs(bb.XMin - fl) <= 7 and abs(bb.XMax - fl) <= 7 and abs(bb.ZMin) <= 7 and abs(bb.ZMax) <= 7
    groove = bb.YMin >= g_y0 - 0.2 and bb.YMax <= g_y1 + 0.2 and bb.ZMin >= z1 - v("coxa_groove_d") - 0.2
    small_round = e.Curve.TypeId == "Part::GeomCircle" and e.Curve.Radius < 5
    shaft_area = bb.ZMax <= floor_z0 + 0.1 and abs(bb.XMin) < 6 and abs(bb.XMax) < 6
    return servo or horn or bearing or groove or small_round or shaft_area


main_shape = common["smooth"](shape, v("coxa_edge_r"), keep_sharp)
main = common["as_part"](doc, part, "Coxa_Main", main_shape, PLA)
common["add_mirror"](doc, part, main, PLA)
common["save"](doc)


def link(name, target, base, rot=None):
    obj = doc.addObject("App::Link", "CoxaCheck_" + name)
    check.addObject(obj)
    obj.LinkedObject = target
    obj.Placement = App.Placement(App.Vector(*base), rot or App.Rotation())
    return obj


bz = floor_z0 - v("fem_boss_h") - v("bearing_t")
link("main", main, (0, 0, 0))
link("servo", doc.getObject("Dummy_MG90S"), (0, 0, seat), SERVO_ROT)
link("horn", doc.getObject("Dummy_Horn"), (0, 0, seat), HORN_IN)
link("bearing", doc.getObject("Dummy_Bearing"), (0, 0, bz))
link("femur", doc.getObject("Femur_Main"), (fl, 0, 0))
common["save"](doc)

solids_of, clash = common["solids_of"], common["clash"]
SERVO = ["MG90S_body", "MG90S_tabs", "MG90S_turret", "MG90S_shaft"]
HORN = ["Horn_hub", "Horn_arm", "Horn_tip"]
Pl = lambda x, y, z, rot=None: App.Placement(App.Vector(x, y, z), rot or App.Rotation())
main_s = solids_of(doc, ["Coxa_Main"])
s_y = v("joint_seat_y")
b_y = v("fem_floor_y0") - v("fem_boss_h") - v("bearing_t")
bb = main_shape.BoundBox
print("coxa %.1f g solid; valid %s; solids %d; x %.1f to %.1f, y %.1f to %.1f, z %.1f to %.1f" % (
    main_shape.Volume / 1000 * v("pla_density"), main_shape.isValid(), len(main_shape.Solids), bb.XMin, bb.XMax, bb.YMin, bb.YMax, bb.ZMin, bb.ZMax))
print("overlap mm3: coxa servo %.3f, coxa horn %.3f, coxa bearing %.3f, femur horn %.3f, femur bearing %.3f" % (
    clash(solids_of(doc, SERVO, Pl(0, 0, seat, SERVO_ROT)), main_s), clash(solids_of(doc, HORN, Pl(0, 0, seat, HORN_IN)), main_s),
    clash(solids_of(doc, ["Bearing_body"], Pl(0, 0, bz)), main_s),
    clash(solids_of(doc, HORN, Pl(fl, s_y, 0, FEMUR_HORN)), main_s),
    clash(solids_of(doc, ["Bearing_body"], Pl(fl, b_y, 0, BEARING_ALONG_Y)), main_s)))
fixed = {"coxa": main_s, "coxa servo": solids_of(doc, SERVO, Pl(0, 0, seat, SERVO_ROT))}
hits = {}
for deg in range(-90, 91, 10):
    swing = Pl(fl, 0, 0, App.Rotation(App.Vector(0, 1, 0), deg))
    moved = {"femur": solids_of(doc, ["Femur_Main"], swing),
             "hip servo": solids_of(doc, SERVO, swing.multiply(Pl(0, s_y, 0, HIP_SERVO))),
             "knee servo": solids_of(doc, SERVO, swing.multiply(Pl(v("femur_len"), s_y, 0, KNEE)))}
    for mname, group in moved.items():
        for fname, f in fixed.items():
            x = clash(group, f)
            if x > 0.001:
                hits.setdefault((mname, fname), []).append(deg)
print("femur sweep, angles that collide: %s" % (hits or "none"))
