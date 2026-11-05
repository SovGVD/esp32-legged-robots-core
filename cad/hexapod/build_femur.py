"""The femur: one print holding the femur and tibia servos in line, after the biped's
leg_bottom_v2. A thick plate has a through-hole for each servo and a pocket for its
tabs. The servo bodies hang out bare between the 2 end straps: screwed in, the servos
stiffen the part. Their horns are on +Y and fix to the next part (coxa fork, tibia).
Opposite each horn a strap reaches across the servo's bottom to a shaft that runs in
a bearing in that part: Ø9 x 1 spacer, Ø8 x 3 shaft.

Print with the rim on the bed; the strap floors are then short bridges.

Right leg; Femur_Main_L is the left leg's. Femur frame: origin on the femur axis,
+X to the knee, Z up with the femur level.

The outer chamfers are picked by position when this script runs, so re-run it after
changing Params.
"""
import os
import runpy

import FreeCAD as App

HERE = os.path.dirname(os.path.abspath(__file__))
common = runpy.run_path(os.path.join(HERE, "common.py"))
doc = common["get_doc"]()
PLA = (0.2, 0.55, 0.75)
KNEE_SERVO = App.Rotation(App.Matrix(0, 1, 0, 0,  0, 0, 1, 0,  1, 0, 0, 0,  0, 0, 0, 1))      # body runs toward -X
HIP_SERVO = KNEE_SERVO.multiply(App.Rotation(App.Vector(0, 0, 1), 180))                       # body runs toward +X
HORN_TO_COXA = KNEE_SERVO.multiply(App.Rotation(App.Vector(0, 0, 1), -90))                    # horn arm toward -X
HORN_TO_TIBIA = KNEE_SERVO.multiply(App.Rotation(App.Vector(0, 0, 1), 180))                   # horn arm toward -Z
BEARING_ALONG_Y = App.Rotation(App.Vector(1, 0, 0), -90)


def p(alias):
    return "Params." + alias


common["remove_built"](doc, ("FemurCheck", "FemurPrim", "Femur"))
part = doc.addObject("App::Part", "Femur")
part.setExpression("Placement.Base.x", p("catalog_part_pitch"))
part.setExpression("Placement.Base.y", p("catalog_row2_y"))
check = doc.addObject("App::Part", "Femur_Check")
check.setExpression("Placement.Base.x", p("catalog_part_pitch"))
check.setExpression("Placement.Base.y", p("catalog_row2_y") + " * 3")
P = common["Prims"](doc, part, "FemurPrim")
box, cyl = P.box, P.cyl

fl = p("femur_len")
seat = p("joint_seat_y")
y0, y1 = p("fem_plate_y0"), p("fem_plate_y1")
plate_h = "%s - (%s)" % (y1, y0)
floor_y0 = p("fem_floor_y0")
floor_t = p("fem_floor_t")
wall_t = p("fem_wall_t")
half = p("fem_half_h")
height = "2 * %s" % half
px0, px1 = p("fem_plate_x0"), p("fem_plate_x1")
bx0 = p("fem_box_x0")
inner_z = "%s - %s" % (half, wall_t)
ch = p("print_chamfer")
fit = p("mg90_fit")
depth = "(%s) - (%s)" % (y0, floor_y0)
strap_len = "%s - (%s)" % (p("fem_strap_x1"), bx0)
taper = p("fem_strap_taper")

solid = [box("plate", "%s - (%s)" % (px1, px0), plate_h, height, px0, y0, "-" + half)]
for tag, sign, axis in (("hip", 1, "0 mm"), ("knee", -1, fl)):
    # everything of a strap as seen from its own shaft: `sign` turns it around for the knee
    x_of = lambda off: "(%s) + %d * (%s)" % (axis, sign, off)
    lo = lambda a, b: x_of(a) if sign > 0 else x_of(b)
    solid += [
        box("end_" + tag, wall_t, depth, height, lo(bx0, "%s + %s" % (bx0, wall_t)), floor_y0, "-" + half),
        box("floor_" + tag, strap_len, floor_t, height, lo(bx0, p("fem_strap_x1")), floor_y0, "-" + half),
        box("side_top_" + tag, strap_len, depth, wall_t, lo(bx0, p("fem_strap_x1")), floor_y0, inner_z),
        box("side_low_" + tag, strap_len, depth, wall_t, lo(bx0, p("fem_strap_x1")), floor_y0, "-" + half),
        # the side walls run out into the plate instead of stopping square
        P.gusset("taper_top_" + tag, (x_of(p("fem_strap_x1")), y0, inner_z), ("x", sign), ("y", -1), taper, taper, "z", wall_t),
        P.gusset("taper_low_" + tag, (x_of(p("fem_strap_x1")), y0, "-" + half), ("x", sign), ("y", -1), taper, taper, "z", wall_t),
        # the end wall into the plate's overhang
        P.gusset("ch_end_" + tag, (x_of(bx0), y0, "-" + half), ("x", -sign), ("y", -1), ch, ch, "z", height),
    ]
    boss_y = "%s - %s" % (floor_y0, p("fem_boss_h"))
    solid.append(cyl("boss_" + tag, p("fem_boss_d"), p("fem_boss_h"), axis, boss_y, "0 mm", along_y=True))
    solid.append(cyl("shaft_" + tag, p("fem_shaft_d"), p("fem_shaft_len"), axis, "%s - %s" % (boss_y, p("fem_shaft_len")), "0 mm", along_y=True))

win_h = "%s + 2 * %s" % (p("mg90_body_w"), fit)
win_len = "%s + 2 * %s" % (p("mg90_body_len"), fit)
near = "%s + %s" % (p("mg90_shaft_end"), fit)
tab_len = "%s + 2 * %s" % (p("mg90_tab_total"), fit)
tab_near = "%s + %s" % (p("mg90_tab_near"), fit)
through = "%s + 2 mm" % plate_h
holes = [
    box("win_hip", win_len, through, win_h, "-(%s)" % near, "%s - 1 mm" % y0, "-(%s) / 2" % win_h),
    box("win_knee", win_len, through, win_h, "%s + %s - (%s)" % (fl, near, win_len), "%s - 1 mm" % y0, "-(%s) / 2" % win_h),
    box("tabs_hip", tab_len, "%s + 1 mm" % p("fem_rim_h"), win_h, "-(%s)" % tab_near, seat, "-(%s) / 2" % win_h),
    box("tabs_knee", tab_len, "%s + 1 mm" % p("fem_rim_h"), win_h, "%s + %s - (%s)" % (fl, tab_near, tab_len), seat, "-(%s) / 2" % win_h),
]
for tag, x in (("hip_near", "-" + p("mg90_hole_near")), ("hip_far", p("mg90_hole_far")),
               ("knee_near", "%s + %s" % (fl, p("mg90_hole_near"))), ("knee_far", "%s - %s" % (fl, p("mg90_hole_far")))):
    holes.append(cyl("tab_" + tag, p("mg90_screw_pilot_d"), "%s + 2 mm" % p("fem_plate_floor"), x, "%s - 1 mm" % y0, "0 mm", along_y=True))
# plate corners at the ends cut off
corner = p("fem_corner")
low_y = "%s - 1 mm" % y0
for tag, x, sx, z, sz in (("a", px0, 1, half, -1), ("b", px0, 1, "-" + half, 1), ("c", px1, -1, half, -1), ("d", px1, -1, "-" + half, 1)):
    holes.append(P.gusset("corner_" + tag, (x, low_y, z), ("x", sx), ("z", sz), corner, corner, "y", through))
body = P.cut("Femur_Body", solid, holes, PLA, mirror=False)
doc.recompute()


# ------------------------------------------------------------ outer chamfers, picked by position
S = doc.getObject("Params")


def val(alias):
    v = S.get(alias)
    return float(getattr(v, "Value", v))


def straight_edges(shape):
    """Every straight edge, inside corners too, except where a servo must fit: those
    stay sharp. Edges the kernel can't chamfer with the others are left out, counted."""
    ch = val("fem_edge_ch")
    limit = val("mg90_body_w") / 2 + val("mg90_fit") + 1e-3
    floor_in, top = val("fem_floor_y1"), val("fem_plate_y1")

    def servo_fit(e):
        """Inside the space the servos take up: window, tab pocket, strap interior."""
        bb = e.BoundBox
        return abs(bb.ZMin) <= limit and abs(bb.ZMax) <= limit and floor_in - 1e-3 <= bb.YMin and bb.YMax <= top + 1e-3

    wanted = [i for i, e in enumerate(shape.Edges)
              if e.Curve.TypeId == "Part::GeomLine" and e.Length > 1.5 * ch and not servo_fit(e)]
    wanted.sort(key=lambda i: -shape.Edges[i].Length)   # keep the ones that show the most

    def works(items):
        try:
            result = shape.makeChamfer(ch, [shape.Edges[i] for i in items])
            return result.isValid() and len(result.Solids) == 1
        except Exception:
            return False

    if works(wanted):
        kept = wanted
    else:
        kept = []
        for i in wanted:
            if works(kept + [i]):
                kept.append(i)
    print("chamfers: %d of %d straight edges" % (len(kept), len(wanted)))
    return [(i + 1, ch, ch) for i in kept]


def shaft_edges(shape):
    tip = val("fem_floor_y0") - val("fem_boss_h") - val("fem_shaft_len")
    boss = val("fem_floor_y0") - val("fem_boss_h")
    out = []
    for i, e in enumerate(shape.Edges):
        bb = e.BoundBox
        if e.Curve.TypeId != "Part::GeomCircle" or bb.YLength > 1e-4:
            continue
        if abs(bb.YMin - tip) < 1e-4:
            out.append((i + 1, val("fem_shaft_ch"), val("fem_shaft_ch")))
        elif abs(bb.YMin - boss) < 1e-4 and abs(e.Curve.Radius - val("fem_boss_d") / 2) < 1e-4:
            out.append((i + 1, val("fem_boss_ch"), val("fem_boss_ch")))
    return out


step = doc.addObject("Part::Chamfer", "Femur_Edges")
part.addObject(step)
step.Base = body
step.Edges = straight_edges(body.Shape)
body.ViewObject.Visibility = False
doc.recompute()
main = doc.addObject("Part::Chamfer", "Femur_Main")
part.addObject(main)
main.Base = step
main.Edges = shaft_edges(step.Shape)
step.ViewObject.Visibility = False
main.ViewObject.ShapeColor = PLA
mirror = doc.addObject("Part::Mirroring", "Femur_Main_L")
part.addObject(mirror)
mirror.Source = main
mirror.Normal = App.Vector(0, 1, 0)
mirror.ViewObject.Visibility = False
mirror.ViewObject.ShapeColor = PLA
common["save"](doc)


def link(name, target, base, rot=None):
    obj = doc.addObject("App::Link", "FemurCheck_" + name)
    check.addObject(obj)
    obj.LinkedObject = target
    for axis, expr in zip("xyz", base):
        obj.setExpression("Placement.Base." + axis, expr)
    if rot:
        obj.Placement = App.Placement(obj.Placement.Base, rot)
    return obj


bearing_y = "%s - %s - %s" % (floor_y0, p("fem_boss_h"), p("bearing_t"))
link("main", main, ("0 mm", "0 mm", "0 mm"))
link("hip_servo", doc.getObject("Dummy_MG90S"), ("0 mm", seat, "0 mm"), HIP_SERVO)
link("hip_horn", doc.getObject("Dummy_Horn"), ("0 mm", seat, "0 mm"), HORN_TO_COXA)
link("knee_servo", doc.getObject("Dummy_MG90S"), (fl, seat, "0 mm"), KNEE_SERVO)
link("knee_horn", doc.getObject("Dummy_Horn"), (fl, seat, "0 mm"), HORN_TO_TIBIA)
link("hip_bearing", doc.getObject("Dummy_Bearing"), ("0 mm", bearing_y, "0 mm"), BEARING_ALONG_Y)
link("knee_bearing", doc.getObject("Dummy_Bearing"), (fl, bearing_y, "0 mm"), BEARING_ALONG_Y)
common["save"](doc)

solids_of, clash = common["solids_of"], common["clash"]
SERVO = ["MG90S_body", "MG90S_tabs", "MG90S_turret", "MG90S_shaft"]
HORN = ["Horn_hub", "Horn_arm", "Horn_tip"]
s_y, f_len, b_y = val("joint_seat_y"), val("femur_len"), val("fem_floor_y0") - val("fem_boss_h") - val("bearing_t")
main_s = solids_of(doc, ["Femur_Main"])
Pl = lambda x, y, rot: App.Placement(App.Vector(x, y, 0), rot)
hip_s, knee_s = solids_of(doc, SERVO, Pl(0, s_y, HIP_SERVO)), solids_of(doc, SERVO, Pl(f_len, s_y, KNEE_SERVO))
bb = main.Shape.BoundBox
print("femur %.1f g solid; valid %s; solids %d; %d + %d chamfered edges; %.1f x %.1f x %.1f, y %.1f to %.1f" % (
    main.Shape.Volume / 1000 * val("pla_density"), main.Shape.isValid(), len(main.Shape.Solids), len(step.Edges), len(main.Edges),
    bb.XLength, bb.YLength, bb.ZLength, bb.YMin, bb.YMax))
print("overlap mm3: hip servo %.3f, knee servo %.3f, servos with each other %.3f, horns %.3f, bearings %.3f" % (
    clash(hip_s, main_s), clash(knee_s, main_s), clash(hip_s, knee_s),
    clash(solids_of(doc, HORN, Pl(0, s_y, HORN_TO_COXA)) + solids_of(doc, HORN, Pl(f_len, s_y, HORN_TO_TIBIA)), main_s),
    clash(solids_of(doc, ["Bearing_body"], Pl(0, b_y, BEARING_ALONG_Y)) + solids_of(doc, ["Bearing_body"], Pl(f_len, b_y, BEARING_ALONG_Y)), main_s)))
print("rim top %.1f, horn hub starts %.1f" % (val("fem_plate_y1"), val("joint_horn_y") - val("horn_hub_h")))
