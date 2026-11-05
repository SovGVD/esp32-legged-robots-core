"""The coxa link: one print carrying the coxa servo, after the femur.

The servo is screwed into the link and turns with it, with its horn fixed to the
body's top plate. A strap under the servo carries a shaft that runs in the bearing in
the body's bottom plate. At the outer end a fork goes around the femur: the femur
servo's horn sits in the +Y arm, and the femur's shaft runs in a bearing in the -Y arm.

The cables of all 3 servos run in a channel on the plate's +Y face, into the body.

Right leg; Coxa_Main_L is the left leg's. Leg frame: origin on the coxa axis at femur
axis height, +X out along the link, Z up. The end of this script swings the femur
through its range looking for collisions.
"""
import os
import runpy

import FreeCAD as App

HERE = os.path.dirname(os.path.abspath(__file__))
common = runpy.run_path(os.path.join(HERE, "common.py"))
doc = common["get_doc"]()
PLA = (0.88, 0.63, 0.11)
SERVO_ROT = App.Rotation(App.Vector(0, 0, 1), 90)       # servo Y to -X: its body runs toward +X, shaft up
HORN_IN = App.Rotation(App.Vector(0, 0, 1), 180)        # coxa horn arm toward the body
FEMUR_HORN = App.Rotation(App.Matrix(0, 1, 0, 0,  0, 0, 1, 0,  1, 0, 0, 0,  0, 0, 0, 1)).multiply(App.Rotation(App.Vector(0, 0, 1), -90))
BEARING_ALONG_Y = App.Rotation(App.Vector(1, 0, 0), -90)


def p(alias):
    return "Params." + alias


common["remove_built"](doc, ("CoxaCheck", "CoxaPrim", "Coxa"))
part = doc.addObject("App::Part", "Coxa")
part.setExpression("Placement.Base.y", p("catalog_row2_y"))
check = doc.addObject("App::Part", "Coxa_Check")
check.setExpression("Placement.Base.y", p("catalog_row2_y") + " * 3")
P = common["Prims"](doc, part, "CoxaPrim")
box, cyl = P.box, P.cyl

z0, z1 = p("coxa_plate_z0"), p("coxa_plate_z1")
plate_h = "%s - (%s)" % (z1, z0)
half = p("coxa_half_w")
width = "2 * %s" % half
px0 = p("coxa_plate_x0")
cx0, cx1 = p("coxa_cross_x0"), p("coxa_cross_x1")
floor_z, floor_t = p("coxa_floor_z0"), p("coxa_floor_t")
wall_t = p("fem_wall_t")
inner = "%s - %s" % (half, wall_t)
bx0 = p("fem_box_x0")
strap_x1 = p("fem_strap_x1")
strap_len = "%s - (%s)" % (strap_x1, bx0)
depth = "(%s) - (%s)" % (z0, floor_z)
taper = p("fem_strap_taper")
ch2 = p("print_chamfer")
fl = p("coxa_len")
aw = p("coxa_arm_w")
half_a = "-%s / 2" % aw
a_y0, a_y1 = p("coxa_arm_a_y0"), p("coxa_arm_a_y1")
b_y0, b_y1 = p("coxa_arm_b_y0"), p("coxa_arm_b_y1")
arm_t = p("coxa_arm_t")
reach = "%s + 2.5 mm" % p("horn_reach")

solid = [
    box("plate", "%s - (%s)" % (cx1, px0), width, plate_h, px0, "-" + half, z0),
    box("end", wall_t, width, depth, bx0, "-" + half, floor_z),
    box("floor", strap_len, width, floor_t, bx0, "-" + half, floor_z),
    box("side_a", strap_len, wall_t, depth, bx0, inner, floor_z),
    box("side_b", strap_len, wall_t, depth, bx0, "-" + half, floor_z),
    P.gusset("taper_a", (strap_x1, inner, z0), ("x", 1), ("z", -1), taper, taper, "y", wall_t),
    P.gusset("taper_b", (strap_x1, "-" + half, z0), ("x", 1), ("z", -1), taper, taper, "y", wall_t),
    P.gusset("ch_end", (bx0, "-" + half, z0), ("x", -1), ("z", -1), ch2, ch2, "y", width),
    # the fork: a wall across, and 2 arms
    box("cross", "%s - %s" % (cx1, cx0), "%s - (%s)" % (a_y1, b_y0), aw, cx0, b_y0, half_a),
    box("arm_a", "%s + %s / 2 - %s" % (fl, aw, cx0), arm_t, aw, cx0, "%s - %s" % (a_y1, arm_t), half_a),
    box("arm_b", "%s + %s / 2 - %s" % (fl, aw, cx0), arm_t, aw, cx0, "(%s) - %s" % (b_y1, arm_t), half_a),
    box("block_a", reach, "%s - (%s)" % (a_y1, a_y0), aw, "%s - %s" % (fl, reach), a_y0, half_a),
    cyl("round_a", aw, "%s - (%s)" % (a_y1, a_y0), fl, a_y0, "0 mm", along_y=True),
    box("block_b", "%s / 2 + 3 mm" % aw, "%s - (%s)" % (b_y1, b_y0), aw, "%s - 3 mm - %s / 2" % (fl, aw), b_y0, half_a),
    cyl("round_b", aw, "%s - (%s)" % (b_y1, b_y0), fl, b_y0, "0 mm", along_y=True),
    # gussets where the plate meets the wall, and the wall meets the arms
    P.gusset("g_top", (cx0, "-" + half, z1), ("x", -1), ("z", 1), p("tib_gusset"), p("tib_gusset"), "y", width),
    P.gusset("g_low", (cx0, "-" + half, z0), ("x", -1), ("z", -1), p("tib_gusset"), p("tib_gusset"), "y", width),
    P.gusset("g_arm_a", (cx1, "%s - %s" % (a_y1, arm_t), half_a), ("x", 1), ("y", -1), p("tib_gusset"), p("tib_gusset"), "z", aw),
    P.gusset("g_arm_b", (cx1, b_y1, half_a), ("x", 1), ("y", 1), p("tib_gusset"), p("tib_gusset"), "z", aw),
    # shaft under the floor: down into the bearing in the body
    cyl("boss", p("fem_boss_d"), p("fem_boss_h"), "0 mm", "0 mm", "%s - %s" % (floor_z, p("fem_boss_h"))),
    cyl("shaft", p("coxa_shaft_d"), p("fem_shaft_len"), "0 mm", "0 mm", "%s - %s - %s" % (floor_z, p("fem_boss_h"), p("fem_shaft_len"))),
]
# cable channel on the +Y face, with fingers
chan_w, chan_d = p("coxa_chan_w"), p("coxa_chan_depth")
mid = "((%s) + (%s)) / 2" % (z0, z1)
c0, c1 = "%s - %s / 2" % (mid, chan_w), "%s + %s / 2" % (mid, chan_w)
chan_len = "%s - (%s)" % (cx0, p("coxa_chan_x0"))
rail_a, rail_b = "%s - (%s)" % (c0, z0), "(%s) - (%s)" % (z1, c1)
solid += [
    box("rail_a", chan_len, chan_d, rail_a, p("coxa_chan_x0"), half, z0),
    box("rail_b", chan_len, chan_d, rail_b, p("coxa_chan_x0"), half, c1),
]
finger_y = "%s + %s" % (half, chan_d)
for n, side in enumerate((0, 1, 0, 1)):
    x = "%s + %d * ((%s) - %s) / 3" % (p("coxa_chan_x0"), n, chan_len, p("chan_finger_len"))
    if side == 0:
        solid.append(box("finger_%d" % n, p("chan_finger_len"), p("chan_finger_t"), "%s + %s" % (rail_a, p("chan_finger_over")), x, finger_y, z0))
    else:
        solid.append(box("finger_%d" % n, p("chan_finger_len"), p("chan_finger_t"), "%s + %s" % (rail_b, p("chan_finger_over")), x, finger_y, "%s - %s" % (c1, p("chan_finger_over"))))

fit = p("mg90_fit")
win_w = "%s + 2 * %s" % (p("mg90_body_w"), fit)
win_len = "%s + 2 * %s" % (p("mg90_body_len"), fit)
near = "%s + %s" % (p("mg90_shaft_end"), fit)
tab_len = "%s + 2 * %s" % (p("mg90_tab_total"), fit)
tab_near = "%s + %s" % (p("mg90_tab_near"), fit)
seat = p("coxa_seat_z")
holes = [
    box("win", win_len, win_w, "%s + 2 mm" % plate_h, "-(%s)" % near, "-(%s) / 2" % win_w, "%s - 1 mm" % z0),
    box("tabs", tab_len, win_w, "%s + 1 mm" % p("fem_rim_h"), "-(%s)" % tab_near, "-(%s) / 2" % win_w, seat),
    cyl("tab_near", p("mg90_screw_pilot_d"), "%s + 2 mm" % p("fem_plate_floor"), "-" + p("mg90_hole_near"), "0 mm", "%s - 1 mm" % z0),
    cyl("tab_far", p("mg90_screw_pilot_d"), "%s + 2 mm" % p("fem_plate_floor"), p("mg90_hole_far"), "0 mm", "%s - 1 mm" % z0),
    # plate corners behind the axis
    P.gusset("corner_a", (px0, half, "%s - 1 mm" % z0), ("x", 1), ("y", -1), p("fem_corner"), p("fem_corner"), "z", "%s + 2 mm" % plate_h),
    P.gusset("corner_b", (px0, "-" + half, "%s - 1 mm" % z0), ("x", 1), ("y", 1), p("fem_corner"), p("fem_corner"), "z", "%s + 2 mm" % plate_h),
    # room for the cables through the wall, from the femur
    box("cable_notch", "%s - %s + 2 mm" % (cx1, cx0), chan_d, chan_w, "%s - 1 mm" % cx0, half, c0),
    # the femur servo's horn pocket, its arm toward the coxa axis
    box("pk_arm", "%s - %s / 2" % (p("horn_reach"), p("horn_tip_w")), p("horn_hub_h"), p("horn_tip_w"),
        "%s - %s + %s / 2" % (fl, p("horn_reach"), p("horn_tip_w")), a_y0, "-%s / 2" % p("horn_tip_w")),
    cyl("pk_tip", p("horn_tip_w"), p("horn_hub_h"), "%s - %s + %s / 2" % (fl, p("horn_reach"), p("horn_tip_w")), a_y0, "0 mm", along_y=True),
    cyl("pk_hub", p("horn_hub_d"), p("horn_hub_h"), fl, a_y0, "0 mm", along_y=True),
    cyl("access", "5 mm", "%s + %s" % (p("tib_horn_roof"), p("horn_hub_h")), fl, a_y0, "0 mm", along_y=True),
    cyl("horn_a", p("horn_screw_d"), "%s + %s + 1 mm" % (p("tib_horn_roof"), p("horn_hub_h")), "%s - %s" % (fl, p("horn_screw_r1")), a_y0, "0 mm", along_y=True),
    cyl("horn_b", p("horn_screw_d"), "%s + %s + 1 mm" % (p("tib_horn_roof"), p("horn_hub_h")), "%s - %s" % (fl, p("horn_screw_r2")), a_y0, "0 mm", along_y=True),
    # the femur's bearing
    cyl("seat", p("bearing_seat_d"), "(%s) - (%s)" % (b_y1, p("tib_bear_seat_end")), fl, p("tib_bear_seat_end"), "0 mm", along_y=True),
    cyl("bore", p("fem_boss_d"), "%s + 1 mm" % p("tib_bear_roof"), fl, "%s - 1 mm" % b_y0, "0 mm", along_y=True),
]
raw = P.cut("Coxa_Raw", solid, holes, PLA, mirror=False)
doc.recompute()

S = doc.getObject("Params")


def val(alias):
    v = S.get(alias)
    return float(getattr(v, "Value", v))


half_win = val("mg90_body_w") / 2 + val("mg90_fit") + 1e-3
floor_in = val("coxa_floor_z0") + val("coxa_floor_t") - 1e-3
rim = val("coxa_plate_z1") + 1e-3
tab_x0, tab_x1 = -val("mg90_tab_near") - val("mg90_fit") - 1e-3, val("mg90_tab_far") + val("mg90_fit") + 1e-3
fl_v = val("coxa_len")
ch = val("fem_edge_ch")


def fit_edge(e):
    bb = e.BoundBox
    servo = abs(bb.YMin) <= half_win and abs(bb.YMax) <= half_win and bb.ZMin >= floor_in and bb.ZMax <= rim and bb.XMin >= tab_x0 and bb.XMax <= tab_x1
    horn_pocket = bb.YMin >= val("coxa_arm_a_y0") - 1e-3 and bb.XMin >= fl_v - val("horn_reach") - 0.5 and abs(bb.ZMin) <= 6 and abs(bb.ZMax) <= 6
    seat_area = bb.YMax < -19 and abs(bb.XMin - fl_v) <= 7 and abs(bb.XMax - fl_v) <= 7 and abs(bb.ZMin) <= 7 and abs(bb.ZMax) <= 7
    return servo or horn_pocket or seat_area


step = doc.addObject("Part::Chamfer", "Coxa_Edges")
part.addObject(step)
step.Base = raw
step.Edges = common["chamfer_edges"](raw.Shape, ch, fit_edge)
raw.ViewObject.Visibility = False
doc.recompute()


def lead_edges(shape):
    """Lead-ins on the servo's way in: the pocket mouth and the ends of the hole."""
    size = val("fem_fit_ch")
    seat_z = val("coxa_seat_z")
    ends = (-val("mg90_shaft_end") - val("mg90_fit"), val("mg90_body_len") - val("mg90_shaft_end") + val("mg90_fit"))
    wanted = []
    for i, e in enumerate(shape.Edges):
        bb = e.BoundBox
        if e.Curve.TypeId != "Part::GeomLine" or bb.ZLength > 1e-4 or e.Length < 2:
            continue
        if abs(bb.YMin) > half_win or abs(bb.YMax) > half_win or bb.XMin < tab_x0 - 1 or bb.XMax > tab_x1 + 1:
            continue
        if abs(bb.ZMin - val("coxa_plate_z1")) < 1e-4:
            wanted.append(i)
        elif abs(bb.ZMin - seat_z) < 1e-4 and bb.XLength < 1e-4 and min(abs(bb.XMin - x) for x in ends) < 1e-3:
            wanted.append(i)

    def works(items):
        try:
            r = shape.makeChamfer(size, [shape.Edges[i] for i in items])
            return r.isValid() and len(r.Solids) == 1
        except Exception:
            return False

    kept = wanted if works(wanted) else []
    if not kept:
        for i in wanted:
            if works(kept + [i]):
                kept.append(i)
    print("  lead-ins: %d of %d" % (len(kept), len(wanted)))
    return [(i + 1, size, size) for i in kept]


lead = doc.addObject("Part::Chamfer", "Coxa_Fit")
part.addObject(lead)
lead.Base = step
lead.Edges = lead_edges(step.Shape)
step.ViewObject.Visibility = False
doc.recompute()
tip_z = val("coxa_floor_z0") - val("fem_boss_h") - val("fem_shaft_len")
boss_z = val("coxa_floor_z0") - val("fem_boss_h")
edges = []
for i, e in enumerate(lead.Shape.Edges):
    bb = e.BoundBox
    if e.Curve.TypeId == "Part::GeomCircle" and bb.ZLength < 1e-4:
        if abs(bb.ZMin - tip_z) < 1e-4:
            edges.append((i + 1, val("fem_shaft_ch"), val("fem_shaft_ch")))
        elif abs(bb.ZMin - boss_z) < 1e-4 and abs(e.Curve.Radius - val("fem_boss_d") / 2) < 1e-4:
            edges.append((i + 1, val("fem_boss_ch"), val("fem_boss_ch")))
main = doc.addObject("Part::Chamfer", "Coxa_Main")
part.addObject(main)
main.Base = lead
main.Edges = edges
lead.ViewObject.Visibility = False
main.ViewObject.ShapeColor = PLA
mirror = doc.addObject("Part::Mirroring", "Coxa_Main_L")
part.addObject(mirror)
mirror.Source = main
mirror.Normal = App.Vector(0, 1, 0)
mirror.ViewObject.Visibility = False
mirror.ViewObject.ShapeColor = PLA
common["save"](doc)


def link(name, target, base, rot=None):
    obj = doc.addObject("App::Link", "CoxaCheck_" + name)
    check.addObject(obj)
    obj.LinkedObject = target
    for axis, expr in zip("xyz", base):
        obj.setExpression("Placement.Base." + axis, expr)
    if rot:
        obj.Placement = App.Placement(obj.Placement.Base, rot)
    return obj


bz = "%s - %s - %s" % (floor_z, p("fem_boss_h"), p("bearing_t"))
link("main", main, ("0 mm", "0 mm", "0 mm"))
link("servo", doc.getObject("Dummy_MG90S"), ("0 mm", "0 mm", seat), SERVO_ROT)
link("horn", doc.getObject("Dummy_Horn"), ("0 mm", "0 mm", seat), HORN_IN)
link("bearing", doc.getObject("Dummy_Bearing"), ("0 mm", "0 mm", bz))
link("femur", doc.getObject("Femur_Main"), (fl, "0 mm", "0 mm"))
common["save"](doc)

solids_of, clash = common["solids_of"], common["clash"]
SERVO = ["MG90S_body", "MG90S_tabs", "MG90S_turret", "MG90S_shaft"]
HORN = ["Horn_hub", "Horn_arm", "Horn_tip"]
Pl = lambda x, y, z, rot=None: App.Placement(App.Vector(x, y, z), rot or App.Rotation())
main_s = solids_of(doc, ["Coxa_Main"])
seat_v, bz_v, s_y = val("coxa_seat_z"), val("coxa_floor_z0") - val("fem_boss_h") - val("bearing_t"), val("joint_seat_y")
b_y = val("fem_floor_y0") - val("fem_boss_h") - val("bearing_t")
print("coxa %.1f g solid; valid %s; solids %d; %d + %d + %d chamfers; bb %s" % (
    main.Shape.Volume / 1000 * val("pla_density"), main.Shape.isValid(), len(main.Shape.Solids), len(step.Edges), len(lead.Edges), len(main.Edges),
    [round(x, 1) for x in (main.Shape.BoundBox.XMin, main.Shape.BoundBox.XMax, main.Shape.BoundBox.YMin, main.Shape.BoundBox.YMax, main.Shape.BoundBox.ZMin, main.Shape.BoundBox.ZMax)]))
print("overlap mm3: coxa servo %.3f, coxa bearing %.3f, femur horn %.3f, femur bearing %.3f; horn face z %.1f, seat z %.1f" % (
    clash(solids_of(doc, SERVO, Pl(0, 0, seat_v, SERVO_ROT)), main_s),
    clash(solids_of(doc, ["Bearing_body"], Pl(0, 0, bz_v)), main_s),
    clash(solids_of(doc, HORN, Pl(fl_v, s_y, 0, FEMUR_HORN)), main_s),
    clash(solids_of(doc, ["Bearing_body"], Pl(fl_v, b_y, 0, BEARING_ALONG_Y)), main_s),
    val("coxa_horn_z"), seat_v))
KNEE_SERVO = App.Rotation(App.Matrix(0, 1, 0, 0,  0, 0, 1, 0,  1, 0, 0, 0,  0, 0, 0, 1))
hip = KNEE_SERVO.multiply(App.Rotation(App.Vector(0, 0, 1), 180))
fixed = {"coxa": main_s, "coxa servo": solids_of(doc, SERVO, Pl(0, 0, seat_v, SERVO_ROT))}
hits = {}
for deg in range(-90, 91, 10):
    swing = Pl(fl_v, 0, 0, App.Rotation(App.Vector(0, 1, 0), deg))
    moved = {"femur": solids_of(doc, ["Femur_Main"], swing),
             "hip servo": solids_of(doc, SERVO, swing.multiply(Pl(0, s_y, 0, hip))),
             "knee servo": solids_of(doc, SERVO, swing.multiply(Pl(val("femur_len"), s_y, 0, KNEE_SERVO)))}
    for mname, group in moved.items():
        for fname, f in fixed.items():
            v = clash(group, f)
            if v > 0.001:
                hits.setdefault((mname, fname), []).append((deg, round(v, 1)))
print("femur sweep -90..+90 (+ = down): %s" % (hits or "no collisions"))
