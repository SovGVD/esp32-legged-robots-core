"""The body: 2 plates, with the legs' servos outside it. Not a plain hexagon: a wavy
outline with a lobe around each coxa axis and scallops between the legs. Ribs stiffen
both plates against twisting: along the border between the legs, and across the inside.

  Body_Bottom  6 coxa bearing seats, holes for the battery holder, rails for the
               standing boards, ribs
  Body_Top     a pocket for each coxa servo's horn (glued in), thick around it, ribs
  Body_Column  8 of these, printed apart so the plates print fast; a stock MG90S screw
               through each plate into each end

The plates are thin skins with ribs, full thickness only around bearings and screws. Body_Check places everything; the end
of this script swings every coxa link over +-40 deg looking for collisions. Body frame:
origin on the centre at femur axis height, X right, Y forward. Sizes come from Params
when this script runs: re-run it after changing them.
"""
import math
import os
import runpy

import FreeCAD as App
import Part

HERE = os.path.dirname(os.path.abspath(__file__))
common = runpy.run_path(os.path.join(HERE, "common.py"))
doc = common["get_doc"]()
COL_BOT, COL_TOP = (0.7, 0.7, 0.68), (0.85, 0.85, 0.82)
Z180 = App.Rotation(App.Vector(0, 0, 1), 180)
PCA_RIGHT = App.Rotation(App.Matrix(0, 0, 1, 0,  1, 0, 0, 0,  0, 1, 0, 0,  0, 0, 0, 1))    # board X to Y, Y to Z, Z to X: pins out, bends down
ESP_FRONT = App.Rotation(App.Matrix(1, 0, 0, 0,  0, 0, 1, 0,  0, -1, 0, 0,  0, 0, 0, 1))   # module side forward
S = doc.getObject("Params")


def v(alias):
    x = S.get(alias)
    return float(getattr(x, "Value", x))


common["remove_built"](doc, ("BodyCheck", "BodyPrim", "Body"))
part = doc.addObject("App::Part", "Body")
part.setExpression("Placement.Base.y", "Params.catalog_row2_y * 5")
check = doc.addObject("App::Part", "Body_Check")
check.setExpression("Placement.Base.y", "Params.catalog_row2_y * 7")
prism, block, rod = common["prism"], common["block"], common["rod"]

front_yaw = float(S.get("front_leg_yaw").getValueAs("deg"))
rear_yaw = float(S.get("rear_leg_yaw").getValueAs("deg"))
mid_x, cx, cy = v("coxa_mid_x"), v("coxa_front_x"), v("coxa_front_y")
AXES = (("RF", cx, cy, front_yaw, ""), ("RM", mid_x, 0.0, 0.0, ""), ("RR", cx, -cy, rear_yaw, ""),
        ("LF", -cx, cy, 180 - front_yaw, "_L"), ("LM", -mid_x, 0.0, 180.0, "_L"), ("LR", -cx, -cy, 180 - rear_yaw, "_L"))
reach = v("plate_inflate")

# the wavy outline, right side from front to rear, then mirrored
lobe = 12.0
right = [(cx * 0.55, cy + reach + 3, 26), (cx + reach * 0.7, cy + reach * 0.9, lobe), (cx + reach + 2, cy - 4, lobe),
         (cx + 2, cy - 26, 14), (mid_x + reach, 12, 9), (mid_x + reach, -12, 9), (cx + 2, -(cy - 26), 14),
         (cx + reach + 2, -(cy - 4), lobe), (cx + reach * 0.7, -(cy + reach * 0.9), lobe), (cx * 0.55, -(cy + reach + 3), 26)]
outline = [(0.0, cy + reach - 2, 40)] + right + [(0.0, -(cy + reach - 2), 40)] + [(-x, y, r) for x, y, r in reversed(right)]
inset = v("rib_inset")
inner = [(x * inset, y * inset, max(r * inset - 2, 1)) for x, y, r in outline]


def plate_prism(z0, z1):
    return prism(outline, "xy", z0, z1)


def ring(z0, z1):
    """The border rib: the outline minus a smaller copy, opened around the legs."""
    r = prism(outline, "xy", z0, z1).cut(prism(inner, "xy", z0 - 1, z1 + 1))
    openings = []
    for _, x, y, yaw, __ in AXES:
        openings.append(rod(2 * v("rib_open_r"), z1 - z0 + 2, (x, y, z0 - 1), "z"))
        spread = v("rib_open_deg")
        a0, a1 = math.radians(yaw - spread), math.radians(yaw + spread)
        far = 90.0
        openings.append(prism([(x, y, 0), (x + far * math.cos(a0), y + far * math.sin(a0), 0), (x + far * math.cos(yaw * math.pi / 180) * 1.3, y + far * math.sin(yaw * math.pi / 180) * 1.3, 0),
                               (x + far * math.cos(a1), y + far * math.sin(a1), 0)], "xy", z0 - 1, z1 + 1))
    return r.cut(Part.makeCompound(openings))


def rib(x0, y0, x1, y1, z0, z1):
    length = math.hypot(x1 - x0, y1 - y0)
    t = v("rib_t")
    b = Part.makeBox(length, t, z1 - z0, App.Vector(0, -t / 2, 0))
    b.Placement = App.Placement(App.Vector(x0, y0, z0), App.Rotation(App.Vector(0, 0, 1), math.degrees(math.atan2(y1 - y0, x1 - x0))))
    return b


COLUMNS = [(sx * v("col_front_x"), sy * v("col_front_y")) for sx in (1, -1) for sy in (1, -1)] + \
          [(sx * v("col_side_x"), sy * v("col_side_y")) for sx in (1, -1) for sy in (1, -1)]

# ------------------------------------------------------------ bottom print
z_bot, top_bot = v("body_bottom_z"), v("body_plate_top")
top0, top1 = v("top_plate_z0"), v("top_plate_z1")
bot_parts = [ring(top_bot, top_bot + v("rib_bot_h"))]
fx, fy = v("col_front_x"), v("col_front_y")
sx_, sy_ = v("col_side_x"), v("col_side_y")
h_b = top_bot + v("rib_bot_h")
# ribs on the bottom plate: where nothing stands, in front of the ESP32 and behind the holder
bot_parts += [rib(-fx, fy, fx, fy, top_bot, h_b), rib(-fx, -fy, fx, -fy, top_bot, h_b)]
skin = top_bot - v("body_skin_t")
for _, x, y, __, ___ in AXES:
    bot_parts.append(rod(v("bear_boss_d"), top_bot - z_bot, (x, y, z_bot), "z"))
for x, y in COLUMNS:
    bot_parts.append(rod(v("col_ring_d"), top_bot + v("col_ring_h") - z_bot, (x, y, z_bot), "z"))
# rails for the standing boards
rail_t, rail_h, rail_len, rfit = v("rail_t"), v("rail_h"), v("rail_len"), v("rail_fit")
pca_len, pca_x, pca_y, pca_t = v("pca_len"), v("pca_x"), v("pca_y"), v("pca_t")
for sign in (1, -1):
    bx = sign * (pca_x - pca_t / 2)
    gap = pca_t / 2 + rfit
    for off in (-pca_len / 2 + 0.3, pca_len / 2 - 0.3 - rail_len, 5.0):
        y0 = sign * pca_y + (off if sign > 0 else -off - rail_len)
        bot_parts.append(block(bx - gap - rail_t, bx - gap, y0, y0 + rail_len, top_bot, top_bot + rail_h))
        bot_parts.append(block(bx + gap, bx + gap + rail_t, y0, y0 + rail_len, top_bot, top_bot + rail_h))
esp_y, esp_len = v("esp_y"), v("esp_len")
for xo in (-esp_len / 2 + 8, esp_len / 2 - 8 - rail_len, -rail_len / 2):
    bot_parts.append(block(xo, xo + rail_len, esp_y - v("esp_t") - rfit - rail_t, esp_y - v("esp_t") - rfit, top_bot, top_bot + v("esp_rail_h")))
    bot_parts.append(block(xo, xo + rail_len, esp_y + rfit, esp_y + rfit + rail_t, top_bot, top_bot + v("esp_rail_h")))
# ------------------------------------------------------------ battery cartridge, in from below
hy, hl, hw, htab = v("holder_y"), v("holder_len"), v("holder_w"), v("holder_tab")
cfit, cwall = v("cart_fit"), v("cart_wall_t")
c_half = hw / 2 + cfit + cwall                    # floor half width
c_front = hy + hl / 2 + htab + cfit               # the floor reaches past the holder's solder tabs
c_rear = hy - hl / 2 - htab - cfit
t_half, t_end = v("cart_tongue_w") / 2, v("cart_tongue_y1")


def cart_outline(g):
    """The cartridge's floor seen from below, grown by g: the holder's footprint and the tongue behind it."""
    return [(-c_half - g, c_front + g, 3 + g), (c_half + g, c_front + g, 3 + g), (c_half + g, c_rear - g, 3 + g),
            (t_half + g, c_rear - g, max(1.5 - g, 0)), (t_half + g, t_end - g, 1 + g), (-t_half - g, t_end - g, 1 + g),
            (-t_half - g, c_rear - g, max(1.5 - g, 0)), (-c_half - g, c_rear - g, 3 + g)]


lw, lt, llen = v("latch_w"), v("latch_t"), v("latch_len")
LATCH_FRONT = (0.0, c_front - v("latch_overlap") + lw / 2 + llen)      # pivots; the closed latch points at the cartridge
LATCH_REAR = (0.0, t_end + v("latch_overlap") - lw / 2 - llen)
xw, xt_, xp = v("xt30_w"), v("xt30_t"), v("xt30_pocket_wall")
x_y, x_z0 = v("xt30_y"), top_bot + v("xt30_lift")
fem_z0, fem_z1 = x_z0 + v("xt30_male_h"), x_z0 + v("xt30_mated_h")
bot_parts.append(prism(cart_outline(cfit + v("cart_band_w")), "xy", z_bot, top_bot))
for px, py in (LATCH_FRONT, LATCH_REAR):
    bot_parts.append(rod(v("latch_boss_d") if False else 7.0, top_bot - z_bot, (px, py, z_bot), "z"))
bot_parts.append(rod(7.0, 4.0, (LATCH_REAR[0], LATCH_REAR[1], top_bot), "z"))      # thread for the rear latch's screw
post_x = t_half + cfit + 0.5 + 1.5
for sx in (1, -1):
    bot_parts.append(block(sx * post_x - 1.5, sx * post_x + 1.5, x_y - 2.5, x_y + 2.5, top_bot, fem_z1 + v("xt30_bridge_t")))
bot_parts.append(block(-post_x - 1.5, post_x + 1.5, x_y - 2.5, x_y + 2.5, fem_z1, fem_z1 + v("xt30_bridge_t")))
bot_parts.append(block(-xw / 2 - xp, xw / 2 + xp, x_y - xt_ / 2 - xp, x_y + xt_ / 2 + xp, fem_z0 + 2, fem_z1))
bottom_solid = plate_prism(skin, top_bot).fuse(bot_parts).removeSplitter()

bot_cuts = []
for _, x, y, __, ___ in AXES:
    bot_cuts.append(rod(v("bearing_seat_d"), v("bearing_t") + 1, (x, y, top_bot - v("bearing_t")), "z"))
    bot_cuts.append(rod(v("fem_boss_d"), v("body_plate_t") + 2, (x, y, z_bot - 1), "z"))
for x, y in COLUMNS:
    bot_cuts.append(rod(v("col_d") + v("col_fit"), v("rib_bot_h") + 1, (x, y, top_bot), "z"))
    bot_cuts.append(rod(2.2, top_bot - z_bot + 1, (x, y, z_bot - 0.5), "z"))
    bot_cuts.append(rod(v("col_head_d"), v("col_head_h") + 1, (x, y, z_bot - 1), "z"))
# notch the border rib behind the PCA9685 pin groups (outside the slots), with a little room
for sign in (1, -1):
    x0 = pca_x + pca_t / 2 + rfit + rail_t + 0.5
    x1 = pca_x + pca_t / 2 + v("pca_grp_h") + 1.0
    for n in "123":
        gy = sign * (pca_y + v("pca_grp_x" + n))
        half = v("pca_grp_w") / 2 + 0.8
        bot_cuts.append(block(min(sign * x0, sign * x1), max(sign * x0, sign * x1), gy - half, gy + half, top_bot, top_bot + v("rib_bot_h") + 1))
bot_cuts.append(prism(cart_outline(cfit), "xy", z_bot - 1, top_bot + 1))
for px, py in (LATCH_FRONT, LATCH_REAR):
    bot_cuts.append(rod(v("mg90_screw_pilot_d"), 10, (px, py, z_bot - 1), "z"))
bot_cuts.append(block(-xw / 2 - 0.15, xw / 2 + 0.15, x_y - xt_ / 2 - 0.15, x_y + xt_ / 2 + 0.15, fem_z0, fem_z1))   # female XT30, glued
bot_cuts.append(rod(4.0, 10, (0, x_y, fem_z1 - 1), "z"))                                                            # its wires, up
bottom_shape = bottom_solid.cut(Part.makeCompound(bot_cuts)).removeSplitter()

# ------------------------------------------------------------ top print
r_low = top0 - v("rib_h")
top_parts = [ring(r_low, top0)]
# ribs under the hood: the border's front and rear, column to column, and an X around the GY-91
top_parts += [rib(-fx, fy, fx, fy, r_low, top0), rib(-fx, -fy, fx, -fy, r_low, top0)]
for sign in (1, -1):
    top_parts += [rib(sign * fx, fy, sign * sx_, sy_, r_low, top0), rib(sign * fx, -fy, sign * sx_, -sy_, r_low, top0)]
    top_parts += [rib(sign * 35.5, sy_, sign * 35.5, -sy_, r_low, top0)]
    top_parts += [rib(sign * fx, fy, -sign * 9.5, 13, r_low, top0), rib(sign * fx, -fy, -sign * 9.5, -13, r_low, top0)]
top_parts += [rib(-9.5, 13, 9.5, 13, r_low, top0), rib(-9.5, -13, 9.5, -13, r_low, top0), rib(-9.5, -13, -9.5, 13, r_low, top0), rib(9.5, -13, 9.5, 13, r_low, top0)]
boss_h = top1 - v("top_boss_z0")
top_cuts = []
for x, y in COLUMNS:
    top_parts.append(rod(v("col_ring_d"), v("col_ring_h"), (x, y, top0 - v("col_ring_h")), "z"))
tip_w = v("horn_tip_w")
arm = v("horn_reach") - tip_w / 2
pocket = v("horn_hub_h")
hub_z = v("top_boss_z0")
for _, x, y, yaw, __ in AXES:
    top_parts.append(rod(v("top_boss_d"), boss_h, (x, y, hub_z), "z"))
    a = math.radians(yaw + 180)
    c, s = math.cos(a), math.sin(a)
    strip = Part.makeBox(v("horn_reach") + 2.5, tip_w + 5, boss_h, App.Vector(0, -(tip_w + 5) / 2, 0))
    strip.Placement = App.Placement(App.Vector(x, y, hub_z), App.Rotation(App.Vector(0, 0, 1), yaw + 180))
    top_parts.append(strip)
    pk = Part.makeBox(arm, tip_w, pocket, App.Vector(0, -tip_w / 2, 0))
    pk.Placement = App.Placement(App.Vector(x, y, hub_z), App.Rotation(App.Vector(0, 0, 1), yaw + 180))
    top_cuts.append(pk)
    top_cuts.append(rod(tip_w, pocket, (x + c * arm, y + s * arm, hub_z), "z"))
    top_cuts.append(rod(v("horn_hub_d"), pocket, (x, y, hub_z), "z"))
    top_cuts.append(rod(5.0, boss_h + 1, (x, y, hub_z), "z"))
for x, y in COLUMNS:
    top_cuts.append(rod(2.2, v("top_plate_t") + 2, (x, y, top0 - 1), "z"))
    top_cuts.append(rod(v("col_d") + v("col_fit"), v("rib_h") + 1, (x, y, r_low - 1), "z"))   # the column stands here, its top against the plate
top_shape = plate_prism(top0, top1).fuse(top_parts).common(plate_prism(r_low - 1, top1 + 1).fuse(
    [rod(v("top_boss_d"), boss_h, (x, y, hub_z), "z") for _, x, y, __, ___ in AXES])).cut(Part.makeCompound(top_cuts)).removeSplitter()
print("raw", len(bottom_shape.Solids), len(top_shape.Solids), bottom_shape.isValid(), top_shape.isValid())

# ------------------------------------------------------------ round the plates' outlines
edge_r = v("body_edge_r")


def bottom_sharp(e):
    """Only the plate's own outer rim gets rounded; the seats, rails and rings stay."""
    bb = e.BoundBox
    return not (bb.ZLength < 1e-4 and abs(bb.ZMin - skin) < 1e-4)


def top_sharp(e):
    bb = e.BoundBox
    return not (bb.ZLength < 1e-4 and abs(bb.ZMin - top1) < 1e-4) or (e.Curve.TypeId == "Part::GeomCircle" and e.Curve.Radius < 8)


print("bottom")
bottom = common["as_part"](doc, part, "Body_Bottom", common["smooth"](bottom_shape, edge_r, lambda e: bottom_sharp(e) or (e.Curve.TypeId == "Part::GeomCircle" and e.Curve.Radius < 8)), COL_BOT)
print("top")
top_round = common["smooth"](top_shape, edge_r, top_sharp)


# ------------------------------------------------------------ ToF brackets and the power switch, after the rounding
def tof_frame(side):
    """Sensor frame in the body: origin on the sensor, Z where it looks, Y up (pin row), X = Y x Z."""
    yaw = math.radians(v("tof_yaw")) * (1 if side < 0 else -1)          # + = left: each turned outward
    tilt = math.radians(v("tof_tilt_left") if side < 0 else v("tof_tilt_right"))
    f = App.Vector(-math.sin(yaw) * math.cos(tilt), math.cos(yaw) * math.cos(tilt), -math.sin(tilt))
    r = App.Vector(math.cos(yaw), math.sin(yaw), 0)
    u = r.cross(f)
    x = u.cross(f)
    m = App.Matrix(x.x, u.x, f.x, 0, x.y, u.y, f.y, 0, x.z, u.z, f.z, 0, 0, 0, 0, 1)
    return App.Placement(App.Vector(side * v("tof_x"), v("tof_y"), v("tof_z")), App.Rotation(m))


t_pcb, off, fit = v("tof_t"), v("tof_sensor_off"), v("tof_fit")
span, ear = v("tof_hole_span"), v("tof_ear_d")
half_w = span / 2 + ear / 2 + v("tof_back_margin")
base_z1 = top1 + v("tof_base_t")
msx, msy = v("tof_mount_screw_x"), v("tof_mount_screw_y")
backs, fronts, mount_cuts = [], [], []
for side in (-1, 1):
    pl = tof_frame(side)
    back = block(-half_w, half_w, -40, off + v("tof_wid") / 2, -t_pcb - v("tof_back_t"), -t_pcb)
    back.Placement = pl
    backs.append(back)
    front = block(-half_w - 3, half_w + 3, -60, 60, -t_pcb, 60)        # nothing of the mount in front of a board
    front.Placement = pl
    fronts.append(front)
    for sx in (-1, 1):
        pilot = rod(v("mg90_screw_pilot_d"), v("tof_screw_depth") + 0.5, (sx * span / 2, off, -t_pcb - v("tof_screw_depth")), "z")
        pilot.Placement = pl
        mount_cuts.append(pilot)
over_plate = plate_prism(top1 - 1, base_z1 + 40)
base = block(-msx - 5, msx + 5, msy - 5, v("tof_y") + 4, top1, base_z1)      # a bridge between the 2 backs, around the screws and wires
bosses = [rod(v("tof_mount_boss_d"), 8, (sx * msx, msy, top1), "z") for sx in (-1, 1)]
mount = base.fuse(bosses + [b.common(block(-40, 40, 0, 100, top1, 60)) for b in backs]).common(over_plate)
mount = mount.cut(Part.makeCompound(fronts)).removeSplitter()
mount_cuts += [rod(v("mg90_screw_pilot_d"), v("tof_mount_depth") + 0.5, (sx * msx, msy, top1 - 0.5), "z") for sx in (-1, 1)]
mount_cuts.append(rod(v("tof_wire_d"), 50, (0, v("tof_wire_y"), top1 - 1), "z"))
mount = mount.cut(Part.makeCompound(mount_cuts)).removeSplitter()
try:
    picked = common["chamfer_edges"](mount, 0.6, lambda e: False)
    mount = mount.makeChamfer(0.6, [mount.Edges[i - 1] for i, _, __ in picked]) if picked else mount
except Exception as exc:
    print("mount chamfer skipped:", exc)
top_holes = [rod(2.2, top1 - top0 + 2, (sx * msx, msy, top0 - 1), "z") for sx in (-1, 1)]
top_holes.append(rod(v("tof_wire_d"), v("rib_h") + top1 - top0 + 2, (0, v("tof_wire_y"), top0 - v("rib_h") - 1), "z"))
px_, py_, pf = v("pwr_x"), v("pwr_y"), v("pwr_fit")
pw, plen, fr = v("pwr_body_w") + 2 * pf, v("pwr_body_len") + 2 * pf, v("pwr_frame")
pwr_cut = [block(px_ - pw / 2, px_ + pw / 2, py_ - plen / 2, py_ + plen / 2, top0 - v("rib_h") - 1, top1 + 1),
           block(px_ - pw / 2 - fr, px_ + pw / 2 + fr, py_ - plen / 2 - fr, py_ + plen / 2 + fr, top0 - v("rib_h") - 1, top1 - v("pwr_panel_t"))]
top_final = top_round.cut(Part.makeCompound(top_holes + pwr_cut)).removeSplitter()
print("top: solids %d valid %s; ToF mount: solids %d valid %s %.1f g" % (len(top_final.Solids), top_final.isValid(), len(mount.Solids), mount.isValid(), mount.Volume / 1000 * v("pla_density")))
top = common["as_part"](doc, part, "Body_Top", top_final, COL_TOP)
tof_mount = common["as_part"](doc, part, "Body_ToF_Mount", mount, COL_TOP)
col_len = top0 - top_bot
col_raw = rod(v("col_d"), col_len, (0, 0, 0), "z").cut(Part.makeCompound([
    rod(v("mg90_screw_pilot_d"), v("col_screw_depth") + 1, (0, 0, -1), "z"),
    rod(v("mg90_screw_pilot_d"), v("col_screw_depth") + 1, (0, 0, col_len - v("col_screw_depth")), "z")]))
col_edges = [e for e in col_raw.Edges if e.Curve.TypeId == "Part::GeomCircle" and abs(e.Curve.Radius - v("col_d") / 2) < 1e-4]
column = common["as_part"](doc, part, "Body_Column", col_raw.makeChamfer(v("col_end_ch"), col_edges), COL_BOT)

# the cartridge: floor flush with the plate's underside, low walls around the holder, lugs for the latches,
# bosses for the holder's 2 screws, a tongue with the male XT30
cart = prism(cart_outline(0), "xy", skin, top_bot)
h_in = (hw / 2 + cfit, hl / 2 + cfit)
walls = block(-c_half, c_half, hy - h_in[1] - cwall, hy + h_in[1] + cwall, top_bot, top_bot + v("cart_side_h")).cut(
    block(-h_in[0], h_in[0], hy - h_in[1], hy + h_in[1], top_bot - 1, top_bot + 20))
walls = walls.cut(Part.makeCompound([block(-c_half - 1, c_half + 1, hy + hl / 2, hy + hl, top_bot + v("cart_end_h"), top_bot + 20),
                                    block(-c_half - 1, c_half + 1, hy - hl, hy - hl / 2, top_bot + v("cart_end_h"), top_bot + 20)]))
lug_w, lug_l = v("cart_lug_w") / 2, v("cart_lug_len")
pillar = block(-xw / 2 - xp, xw / 2 + xp, x_y - xt_ / 2 - xp, x_y + xt_ / 2 + xp, top_bot, fem_z0 - 2)
cart = cart.fuse([walls, pillar,
                  block(-lug_w, lug_w, c_front - lug_l, c_front, z_bot, skin),
                  block(-lug_w, lug_w, t_end, t_end + lug_l, z_bot, skin)] +
                 [rod(v("holder_boss_d"), skin - z_bot, (0, hy + sg * v("holder_hole_span") / 2, z_bot), "z") for sg in (1, -1)]).removeSplitter()
cart = cart.cut(Part.makeCompound(
    [block(-xw / 2 - 0.15, xw / 2 + 0.15, x_y - xt_ / 2 - 0.15, x_y + xt_ / 2 + 0.15, x_z0, fem_z0),            # male XT30, glued
     block(-3, 3, x_y, x_y + xt_ / 2 + xp + 1, x_z0, x_z0 + 4)] +                                           # its solder cups' wires, toward the holder
    [rod(v("holder_screw_d"), 20, (0, hy + sg * v("holder_hole_span") / 2, z_bot - 1), "z") for sg in (1, -1)])).removeSplitter()
cartridge = common["as_part"](doc, part, "Battery_Cartridge", cart, COL_BOT)
latch_raw = prism([(-lw / 2, 0, lw / 2 - 0.01), (lw / 2, 0, lw / 2 - 0.01), (lw / 2, llen, lw / 2 - 0.01), (-lw / 2, llen, lw / 2 - 0.01)], "xy", -lt, 0).fuse(
    rod(lw, lt, (0, 0, -lt), "z")).fuse(rod(lw, lt, (0, llen, -lt), "z")).removeSplitter().cut(rod(2.2, lt + 2, (0, 0, -lt - 1), "z"))
latch = common["as_part"](doc, part, "Battery_Latch", latch_raw, COL_BOT)
print("cartridge %.1f g, latch %.1f g; valid %s %s; opening %.1f x %.1f, y %.1f to %.1f" % (
    cart.Volume / 1000 * v("pla_density"), latch_raw.Volume / 1000 * v("pla_density"), cart.isValid(), latch_raw.isValid(),
    2 * (c_half + cfit), c_front - t_end + 2 * cfit, t_end - cfit, c_front + cfit))
common["save"](doc)


# ------------------------------------------------------------ everything in place
def link(name, target, base, rot=None):
    obj = doc.addObject("App::Link", "BodyCheck_" + name)
    check.addObject(obj)
    obj.LinkedObject = doc.getObject(target)
    obj.Placement = App.Placement(App.Vector(*base), rot or App.Rotation())
    return obj


seat = v("coxa_seat_z")
bear_z = v("coxa_floor_z0") - v("fem_boss_h") - v("bearing_t")
link("bottom", "Body_Bottom", (0, 0, 0))
link("top", "Body_Top", (0, 0, 0))
for n, (x, y) in enumerate(COLUMNS):
    link("column_%d" % n, "Body_Column", (x, y, top_bot))
SERVO_LEG = App.Rotation(App.Vector(0, 0, 1), 90)
for tag, x, y, yaw, side in AXES:
    rot = App.Rotation(App.Vector(0, 0, 1), yaw)
    link("coxa_" + tag, "Coxa_Main" + side, (x, y, 0), rot)
    link("servo_" + tag, "Dummy_MG90S", (x, y, seat), rot.multiply(SERVO_LEG))
    link("horn_" + tag, "Dummy_Horn", (x, y, seat), rot.multiply(Z180))
    link("bearing_" + tag, "Dummy_Bearing", (x, y, bear_z))
cz, px, py, pz = v("cell_z"), v("pca_x"), v("pca_y"), v("pca_z")
link("cells", "Dummy_18650_pair", (0, v("holder_y"), cz))
link("pca_r", "Dummy_PCA9685", (px, py, pz), PCA_RIGHT)
link("pca_l", "Dummy_PCA9685", (-px, -py, pz), Z180.multiply(PCA_RIGHT))
link("esp32", "Dummy_ESP32_devkit", (0, esp_y, v("esp_z")), ESP_FRONT)
link("gy91", "Dummy_GY91", (0, 0, v("gy91_z")))
link("tof_mount", "Body_ToF_Mount", (0, 0, 0))
link("cartridge", "Battery_Cartridge", (0, 0, 0))
link("xt30", "Dummy_XT30", (0, x_y, x_z0))
link("latch_f", "Battery_Latch", (LATCH_FRONT[0], LATCH_FRONT[1], z_bot), Z180)
link("latch_r", "Battery_Latch", (LATCH_REAR[0], LATCH_REAR[1], z_bot))
link("rocker", "Dummy_Rocker", (v("pwr_x"), v("pwr_y"), top1))
for side in (-1, 1):
    tl = link("tof_%s" % ("l" if side < 0 else "r"), "Dummy_VL53L5CX", (0, 0, 0))
    tl.Placement = tof_frame(side)
common["save"](doc)

# ------------------------------------------------------------ report
solids_of, clash = common["solids_of"], common["clash"]
SERVO = ["MG90S_body", "MG90S_tabs", "MG90S_turret", "MG90S_shaft"]
HORN = ["Horn_hub", "Horn_arm", "Horn_tip"]
d = v("pla_density")
print("bottom %.1f g, top %.1f g, 8 columns %.1f g solid; valid %s %s; solids %d %d" % (
    bottom.Shape.Volume / 1000 * d, top.Shape.Volume / 1000 * d, 8 * column.Shape.Volume / 1000 * d, bottom.Shape.isValid(), top.Shape.isValid(), len(bottom.Shape.Solids), len(top.Shape.Solids)))
bot_s, top_s = solids_of(doc, ["Body_Bottom"]), solids_of(doc, ["Body_Top"])
col_s = []
for x, y in COLUMNS:
    col_s += solids_of(doc, ["Body_Column"], App.Placement(App.Vector(x, y, top_bot), App.Rotation()))
print("columns overlap the plates mm3: %.3f" % clash(col_s, bot_s + top_s))
Pl = lambda base, rot=None: App.Placement(App.Vector(*base), rot or App.Rotation())
names = lambda part_name: [c.Name for c in doc.getObject(part_name).Group if c.Name not in ("Bearing_ring", "Bearing_bore")]
inside = {
    "cells": solids_of(doc, names("Dummy_18650_pair"), Pl((0, v("holder_y"), cz))),
    "pca r": solids_of(doc, names("Dummy_PCA9685"), Pl((px, py, pz), PCA_RIGHT)),
    "pca l": solids_of(doc, names("Dummy_PCA9685"), Pl((-px, -py, pz), Z180.multiply(PCA_RIGHT))),
    "esp32": solids_of(doc, names("Dummy_ESP32_devkit"), Pl((0, esp_y, v("esp_z")), ESP_FRONT)),
    "gy91": solids_of(doc, names("Dummy_GY91"), Pl((0, 0, v("gy91_z")))),
    "rocker": solids_of(doc, names("Dummy_Rocker"), Pl((v("pwr_x"), v("pwr_y"), top1))),
    "tof mount": solids_of(doc, ["Body_ToF_Mount"]),
    "cartridge": solids_of(doc, ["Battery_Cartridge"]),
    "xt30": solids_of(doc, names("Dummy_XT30"), Pl((0, x_y, x_z0))),
    "latches": solids_of(doc, ["Battery_Latch"], Pl((LATCH_FRONT[0], LATCH_FRONT[1], z_bot), Z180)) + solids_of(doc, ["Battery_Latch"], Pl((LATCH_REAR[0], LATCH_REAR[1], z_bot))),
    "tof l": solids_of(doc, names("Dummy_VL53L5CX"), tof_frame(-1)),
    "tof r": solids_of(doc, names("Dummy_VL53L5CX"), tof_frame(1)),
}
static = {}
keys = list(inside)
for i, k in enumerate(keys):
    for plate, s_ in (("bottom", bot_s), ("top", top_s)):
        x_ = clash(inside[k], s_)
        if x_ > 0.001:
            static[(plate, k)] = round(x_, 2)
    for other in keys[i + 1:]:
        x_ = clash(inside[k], inside[other])
        if x_ > 0.001:
            static[(k, other)] = round(x_, 2)
print("plates and boards overlap mm3: %s; plates %.3f" % (static or "none", clash(bot_s, top_s)))
fits, sweep = {}, {}
for tag, x, y, yaw, side in AXES:
    fits[tag] = clash(solids_of(doc, HORN, Pl((x, y, seat), App.Rotation(App.Vector(0, 0, 1), yaw).multiply(Z180))), top_s) + clash(solids_of(doc, ["Bearing_body"], Pl((x, y, bear_z))), bot_s)
    for deg in (-40, -20, 0, 20, 40):
        rot = App.Rotation(App.Vector(0, 0, 1), yaw + deg)
        coxa = solids_of(doc, ["Coxa_Main" + side], Pl((x, y, 0), rot))
        servo = solids_of(doc, SERVO, Pl((x, y, seat), rot.multiply(SERVO_LEG)))
        targets = dict(inside)
        targets["bottom"] = bot_s
        targets["top"] = top_s
        targets["columns"] = col_s
        for name, group in targets.items():
            x_ = clash(coxa + servo, group)
            if x_ > 0.001:
                sweep.setdefault((tag, name), []).append((deg, round(x_, 1)))
print("horn and bearing fits mm3: %s" % {k: round(x_, 3) for k, x_ in fits.items()})
print("coxa sweep +-40 deg: %s" % (sweep or "no collisions"))

# the cartridge comes out straight down with the latches turned 90 deg; the female XT30 stays in the body
R90 = App.Rotation(App.Vector(0, 0, 1), 90)
open_latches = solids_of(doc, ["Battery_Latch"], Pl((LATCH_FRONT[0], LATCH_FRONT[1], z_bot), Z180.multiply(R90))) + \
    solids_of(doc, ["Battery_Latch"], Pl((LATCH_REAR[0], LATCH_REAR[1], z_bot), R90))
moving = inside["cartridge"] + inside["cells"] + solids_of(doc, ["XT30_male"], Pl((0, x_y, x_z0)))
stay = {"bottom": bot_s, "columns": col_s, "pca r": inside["pca r"], "pca l": inside["pca l"], "esp32": inside["esp32"],
        "open latches": open_latches, "female xt30": solids_of(doc, ["XT30_female"], Pl((0, x_y, x_z0)))}
path = {}
for dz in [0.5 * k for k in range(1, 9)] + list(range(5, 46, 3)):
    shifted = []
    for sld in moving:
        c = sld.copy()
        c.translate(App.Vector(0, 0, -dz))
        shifted.append(c)
    for name, group in stay.items():
        x_ = clash(shifted, group)
        if x_ > 0.001:
            path.setdefault(name, []).append((dz, round(x_, 2)))
print("cartridge out, straight down 0.5..45 mm: %s" % (path or "clear"))
