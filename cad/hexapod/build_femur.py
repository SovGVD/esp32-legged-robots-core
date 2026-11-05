"""The femur: one print, a closed box holding the femur and tibia servos in line.

The servos are part of the structure: screwed to the plate, they stiffen the box.
Their horns are on +Y and fix to the next part (coxa fork, tibia). Opposite each
horn a shaft on the floor runs in a bearing in that part.

Print with the plate on the bed: the walls rise straight up and the floor bridges
the 12.4 mm between them.

Right leg; Femur_Main_L is the left leg's. Femur frame: origin on the femur axis,
+X to the knee, Z up with the femur level.
"""
import os
import runpy

import FreeCAD as App

HERE = os.path.dirname(os.path.abspath(__file__))
common = runpy.run_path(os.path.join(HERE, "common.py"))
doc = common["get_doc"]()
PLA = (0.2, 0.55, 0.75)
# servo Y to +X, servo Z to +Y: the body runs toward -X
KNEE_SERVO = App.Rotation(App.Matrix(0, 1, 0, 0,  0, 0, 1, 0,  1, 0, 0, 0,  0, 0, 0, 1))
HIP_SERVO = KNEE_SERVO.multiply(App.Rotation(App.Vector(0, 0, 1), 180))       # body runs toward +X
HORN_TO_COXA = KNEE_SERVO.multiply(App.Rotation(App.Vector(0, 0, 1), -90))    # horn arm toward -X
HORN_TO_TIBIA = KNEE_SERVO.multiply(App.Rotation(App.Vector(0, 0, 1), 180))   # horn arm toward -Z
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
plate_t = p("fem_plate_t")
wall_t = p("fem_wall_t")
plate_y = "%s - %s" % (seat, plate_t)
floor_y0, floor_y1 = p("fem_floor_y0"), p("fem_floor_y1")
half = p("fem_half_h")
height = "2 * %s" % half
x0, x1 = p("fem_box_x0"), p("fem_box_x1")
inner_z = "%s - %s" % (half, wall_t)
ch = p("print_chamfer")
fit = p("mg90_fit")
wall_depth = "%s - (%s)" % (seat, floor_y0)
gap0 = "%s - %s + %s" % (p("mg90_body_len"), p("mg90_shaft_end"), fit)            # femur servo's far end
gap1 = "%s - (%s)" % (fl, gap0)                                                    # tibia servo's far end
gap_len = "%s - (%s)" % (gap1, gap0)
hole_gap = "(%s) - %s" % (gap_len, ch)

solid = [
    box("plate", "%s - (%s)" % (p("fem_plate_x1"), p("fem_plate_x0")), plate_t, height, p("fem_plate_x0"), plate_y, "-" + half),
    box("floor", "%s - (%s)" % (x1, x0), p("fem_floor_t"), height, x0, floor_y0, "-" + half),
    box("side_top", "%s - (%s)" % (x1, x0), wall_depth, wall_t, x0, floor_y0, inner_z),
    box("side_low", "%s - (%s)" % (x1, x0), wall_depth, wall_t, x0, floor_y0, "-" + half),
    box("end_a", wall_t, wall_depth, height, x0, floor_y0, "-" + half),
    box("end_b", wall_t, wall_depth, height, "%s - %s" % (x1, wall_t), floor_y0, "-" + half),
    # internal chamfers in the middle compartment, where no servo sits: floor and plate into the side walls
    P.gusset("ch_floor_top", (gap0, floor_y1, inner_z), ("y", 1), ("z", -1), ch, ch, "x", gap_len),
    P.gusset("ch_floor_low", (gap0, floor_y1, "-(%s)" % inner_z), ("y", 1), ("z", 1), ch, ch, "x", gap_len),
    P.gusset("ch_plate_top", (gap0, plate_y, inner_z), ("y", -1), ("z", -1), ch, ch, "x", gap_len),
    P.gusset("ch_plate_low", (gap0, plate_y, "-(%s)" % inner_z), ("y", -1), ("z", 1), ch, ch, "x", gap_len),
    # outside: the end walls into the plate's overhang
    P.gusset("ch_end_a", (x0, plate_y, "-" + half), ("x", -1), ("y", -1), ch, ch, "z", height),
    P.gusset("ch_end_b", (x1, plate_y, "-" + half), ("x", 1), ("y", -1), ch, ch, "z", height),
]
for tag, x in (("hip", "0 mm"), ("knee", fl)):
    boss_y = "%s - %s" % (floor_y0, p("fem_boss_h"))
    solid.append(cyl("boss_" + tag, p("fem_boss_d"), p("fem_boss_h"), x, boss_y, "0 mm", along_y=True))
    solid.append(cyl("shaft_" + tag, p("fem_shaft_d"), p("fem_shaft_len"), x, "%s - %s" % (boss_y, p("fem_shaft_len")), "0 mm", along_y=True))

win_h = "%s + 2 * %s" % (p("mg90_body_w"), fit)
through = "%s + 2 mm" % plate_t
win_len = "%s + 2 * %s" % (p("mg90_body_len"), fit)
near = "%s + %s" % (p("mg90_shaft_end"), fit)
holes = [
    box("win_hip", win_len, through, win_h, "-(%s)" % near, "%s - 1 mm" % plate_y, "-(%s) / 2" % win_h),
    box("win_knee", win_len, through, win_h, "%s + %s - (%s)" % (fl, near, win_len), "%s - 1 mm" % plate_y, "-(%s) / 2" % win_h),
    # cable hole in the top wall, between the servos
    box("wire", p("fem_wire_len"), p("fem_wire_h"), "%s + 2 mm" % wall_t, "(%s) / 2 - %s / 2" % (fl, p("fem_wire_len")),
        "(%s + %s) / 2 - %s / 2" % (floor_y1, plate_y, p("fem_wire_h")), "%s - 1 mm" % inner_z),
]
for tag, x in (("hip_near", "-" + p("mg90_hole_near")), ("hip_far", p("mg90_hole_far")),
               ("knee_near", "%s + %s" % (fl, p("mg90_hole_near"))), ("knee_far", "%s - %s" % (fl, p("mg90_hole_far")))):
    holes.append(cyl("tab_" + tag, p("mg90_screw_pilot_d"), through, x, "%s - 1 mm" % plate_y, "0 mm", along_y=True))
main = P.cut("Femur_Main", solid, holes, PLA)
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

S = doc.getObject("Params")


def val(alias):
    v = S.get(alias)
    return float(getattr(v, "Value", v))


solids_of, clash = common["solids_of"], common["clash"]
SERVO = ["MG90S_body", "MG90S_tabs", "MG90S_turret", "MG90S_shaft"]
HORN = ["Horn_hub", "Horn_arm", "Horn_tip"]
s_y, f_len, b_y = val("joint_seat_y"), val("femur_len"), val("fem_floor_y0") - val("fem_boss_h") - val("bearing_t")
main_s = solids_of(doc, ["Femur_Main"])
Pl = lambda x, y, rot: App.Placement(App.Vector(x, y, 0), rot)
hip_s, knee_s = solids_of(doc, SERVO, Pl(0, s_y, HIP_SERVO)), solids_of(doc, SERVO, Pl(f_len, s_y, KNEE_SERVO))
print("femur %.1f g solid; valid %s; solids %d; box %.1f x %.1f x %.1f" % (
    main.Shape.Volume / 1000 * val("pla_density"), main.Shape.isValid(), len(main.Shape.Solids),
    main.Shape.BoundBox.XLength, main.Shape.BoundBox.YLength, main.Shape.BoundBox.ZLength))
print("overlap mm3: hip servo %.3f, knee servo %.3f, servos with each other %.3f, horns %.3f, bearings %.3f" % (
    clash(hip_s, main_s), clash(knee_s, main_s), clash(hip_s, knee_s),
    clash(solids_of(doc, HORN, Pl(0, s_y, HORN_TO_COXA)) + solids_of(doc, HORN, Pl(f_len, s_y, HORN_TO_TIBIA)), main_s),
    clash(solids_of(doc, ["Bearing_body"], Pl(0, b_y, BEARING_ALONG_Y)) + solids_of(doc, ["Bearing_body"], Pl(f_len, b_y, BEARING_ALONG_Y)), main_s)))
gap = (val("femur_len") - val("mg90_tab_far")) - val("mg90_tab_far")
print("between the servo tabs %.2f mm; between their bodies %.2f mm" % (gap, val("femur_len") - 2 * (val("mg90_body_len") - val("mg90_shaft_end"))))
