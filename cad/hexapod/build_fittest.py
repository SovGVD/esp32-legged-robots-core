"""Fit test: small pieces with the exact fits of the real parts, to print first and try the
bought parts in. Every size comes from Params, so a fit that comes out tight or loose is one
parameter to change for all parts. Exports print/fit_test.stl, everything flat on the bed.

  A  MG90S pocket as in the femur and coxa: body hole, tab pocket, 2 screw pilots, lead-ins.
     Try: the servo drops in, the tabs sit flat, the stock screws bite.        mg90_fit, fem_fit_ch
  B  horn pocket as in the tibia, coxa and top plate: hub, arm and shaft hole.
     Try: the horn drops in with a little play for glue.                        horn_fit
  C  bearing seat as in the body, femur and tibia, plus a shaft on its spacer.
     Try: the MR128K presses into the seat; the shaft turns in its inner ring.  bearing_seat_d, bearing_id
  D  column end with a screw pilot, and a plate with the clearance hole.
     Try: a stock screw through the plate bites into the column, its head flush.  screw_clear_d, screw_head_d/_h
  E  board slots as in the bottom plate: PCA9685 (6 mm high) and ESP32 (3.5), one piece each.
     Try: the boards slide in by hand and stand without wobbling.               rail_fit
  F  power switch cutout in a 1.6 mm panel.
     Try: the KCD1 snaps in and holds.                                          pwr_cut_w, pwr_cut_len, pwr_panel_t

Pieces already approved in a printed test (APPROVED below) are left out of the STL; ALL=True puts
them back. Run inside FreeCAD after build_params.py.
"""
import os
import runpy

import FreeCAD as App
import Mesh
import MeshPart
import Part

HERE = os.path.dirname(os.path.abspath(__file__))
common = runpy.run_path(os.path.join(HERE, "common.py"))
doc = common["get_doc"]()
S = doc.getObject("Params")
block, rod = common["block"], common["rod"]


def v(alias):
    x = S.get(alias)
    return float(getattr(x, "Value", x))


common["remove_built"](doc, ("FitTest",))
part = doc.addObject("App::Part", "FitTest")
part.setExpression("Placement.Base.y", "Params.catalog_row2_y * 9")
WALL = 2.4
pieces = {}


def top_bottom_chamfer(shape, size, z_levels):
    """Chamfer the horizontal edges lying in the given heights: lead-ins and softened outlines."""
    edges = [e for e in shape.Edges if e.BoundBox.ZLength < 1e-4 and any(abs(e.BoundBox.ZMin - z) < 1e-4 for z in z_levels)]
    try:
        return shape.makeChamfer(size, edges)
    except Exception:
        return shape


# ------------------------------------------------------------ A: MG90S pocket
fit = v("mg90_fit")
bw = v("mg90_body_w") / 2 + v("mg90_fit_w")
near, far = v("mg90_shaft_end") + fit, v("mg90_body_len") - v("mg90_shaft_end") + fit
t_near, t_far = v("mg90_tab_near") + fit, v("mg90_tab_far") + fit
floor, rim = v("fem_plate_floor"), v("fem_rim_h")
h = floor + rim
a = block(-t_near - WALL, t_far + WALL, -bw - WALL, bw + WALL, 0, h).cut(Part.makeCompound([
    block(-near, far, -bw, bw, -1, h + 1),                                   # the body, through
    block(-t_near, t_far, -bw, bw, floor, h + 1),                            # the tabs, from the top
    rod(v("mg90_screw_pilot_d"), floor + 2, (-v("mg90_hole_near"), 0, -1), "z"),
    rod(v("mg90_screw_pilot_d"), floor + 2, (v("mg90_hole_far"), 0, -1), "z")])).removeSplitter()
pieces["A_servo_pocket"] = a

# ------------------------------------------------------------ B: horn pocket
pd, pw = v("horn_pocket_d"), v("horn_pocket_w")
arm = v("horn_reach") - v("horn_tip_w") / 2
depth = v("horn_hub_h") + v("horn_fit")
roof = v("tib_horn_roof")
bh = depth + roof
b = block(-pd / 2 - WALL, arm + pw / 2 + WALL, -pd / 2 - WALL, pd / 2 + WALL, 0, bh).cut(Part.makeCompound([
    common["horn_pocket"](pd / 2, pw / 2, arm, depth + 1, (0, 0, roof), (1, 0, 0), (0, 0, 1)),
    rod(5.0, bh + 2, (0, 0, -1), "z")])).removeSplitter()
pieces["B_horn_pocket"] = b

# ------------------------------------------------------------ C: bearing seat and shaft
seat_d, bt = v("bearing_seat_d"), v("bearing_t")
c_h = bt + 1.5
c = rod(seat_d + 2 * WALL, c_h, (0, 0, 0), "z").cut(Part.makeCompound([
    rod(seat_d, bt + 1, (0, 0, c_h - bt), "z"),
    rod(v("bearing_push_d"), c_h + 2, (0, 0, -1), "z")])).removeSplitter()    # push the bearing out from below
shaft = rod(14, 2, (0, 0, 0), "z").fuse([rod(v("fem_boss_d"), v("fem_boss_h"), (0, 0, 2), "z"),
                                         rod(v("fem_shaft_d"), v("fem_shaft_len"), (0, 0, 2 + v("fem_boss_h")), "z")]).removeSplitter()
tip = [e for e in shaft.Edges if abs(e.BoundBox.ZMin - (2 + v("fem_boss_h") + v("fem_shaft_len"))) < 1e-4 and e.BoundBox.ZLength < 1e-4]
pieces["C_bearing_seat"] = c
pieces["C_shaft"] = shaft.makeChamfer(v("fem_shaft_ch"), tip)

# ------------------------------------------------------------ D: column end and screw plate
d_len = 12.0
col = rod(v("col_d"), d_len, (0, 0, 0), "z").cut(rod(v("mg90_screw_pilot_d"), v("col_screw_depth") + 1, (0, 0, d_len - v("col_screw_depth")), "z"))
pieces["D_column"] = col
d_t = v("top_plate_t") + v("col_top_floor")          # the top plate at a column
pieces["D_plate"] = block(-7, 7, -7, 7, 0, d_t).cut(Part.makeCompound([
    rod(v("screw_clear_d"), d_t + 2, (0, 0, -1), "z"),
    rod(v("screw_head_d"), v("screw_head_h") + 1, (0, 0, d_t - v("screw_head_h")), "z")]))

# ------------------------------------------------------------ E: board slots
rt, rl, rf = v("rail_t"), 10.0, v("rail_fit")
base_t = v("bot_skin_t") + 1.2
for name, board_t, rh in (("E_pca_slot", v("pca_t"), v("rail_h")), ("E_esp_slot", v("esp_t"), v("esp_rail_h"))):
    gap = board_t / 2 + rf
    e = block(-8, 8, -5, 5, 0, base_t).fuse([block(-rl / 2, rl / 2, -gap - rt, -gap, base_t, base_t + rh),
                                            block(-rl / 2, rl / 2, gap, gap + rt, base_t, base_t + rh)])
    pieces[name] = e.removeSplitter()

# ------------------------------------------------------------ F: switch cutout
sw, sl = v("pwr_cut_w"), v("pwr_cut_len")
pieces["F_switch_panel"] = block(-sw / 2 - 5, sw / 2 + 5, -sl / 2 - 5, sl / 2 + 5, 0, v("pwr_panel_t")).cut(
    block(-sw / 2, sw / 2, -sl / 2, sl / 2, -1, 3))

# ------------------------------------------------------------ only what still needs trying
# Pieces whose fit the user has approved in a printed fit test; they stay here (ALL=True prints them
# again) but leave the STL.
APPROVED = {
    "B_horn_pocket": "fit test 2: perfect (tapered horn, flattened to 3.8)",
    "D_column": "fit test 2: perfect",
    "D_plate": "fit test 2: perfect (2.5 clearance, 5.6 x 1.4 counterbore)",
    "E_esp_slot": "fit test 2: good (0.05 a side)",
}
if not globals().get("ALL"):
    for name in APPROVED:
        pieces.pop(name, None)
print("printing:", ", ".join(pieces))

# ------------------------------------------------------------ lead-ins on every hole, pocket and outline
for name in list(pieces):
    print(name)
    pieces[name] = common["lead_ins"](pieces[name], v("lead_small"), v("lead_big"), v("lead_small_d"))

# ------------------------------------------------------------ pack them into a compact block and export
# shelves: the tallest (in Y) first, each row filled up to ROW_W, 5 mm apart, centred on the bed origin
ROW_W, GAP = 80.0, 5.0
order = sorted(pieces.items(), key=lambda kv: -kv[1].BoundBox.YLength)
rows, row, row_w = [], [], 0.0
for name, shape in order:
    w = shape.BoundBox.XLength
    if row and row_w + GAP + w > ROW_W:
        rows.append(row)
        row, row_w = [], 0.0
    row_w += (GAP if row else 0.0) + w
    row.append((name, shape))
rows.append(row)
spots, y = [], 0.0
for row in rows:
    x = 0.0
    depth = max(s.BoundBox.YLength for _, s in row)
    for name, shape in row:
        spots.append((name, shape, x, y))
        x += shape.BoundBox.XLength + GAP
    y -= depth + GAP
placed, d = [], v("pla_density")
for name, shape, x, y in spots:
    bb = shape.BoundBox
    s = shape.copy()
    s.translate(App.Vector(x - bb.XMin, y - bb.YMax, -bb.ZMin))
    obj = common["as_part"](doc, part, "FitTest_" + name, s, (0.85, 0.55, 0.2))
    placed.append(s)
    print("%-18s %5.1f x %4.1f x %4.1f mm  %.1f g  valid %s" % (name, bb.XLength, bb.YLength, bb.ZLength, s.Volume / 1000 * d, s.isValid()))
common["save"](doc)
out_dir = os.path.join(HERE, "print")
os.makedirs(out_dir, exist_ok=True)
whole = Part.makeCompound(placed)
c = whole.BoundBox.Center
whole.translate(App.Vector(-c.x, -c.y, 0))
print("block %.0f x %.0f mm" % (whole.BoundBox.XLength, whole.BoundBox.YLength))
mesh = MeshPart.meshFromShape(Shape=whole, LinearDeflection=0.02, AngularDeflection=0.1, Relative=False)
path = os.path.join(out_dir, "fit_test.stl")
mesh.write(path)
print("total %.1f g solid, %d triangles -> %s" % (whole.Volume / 1000 * d, mesh.CountFacets, path))
