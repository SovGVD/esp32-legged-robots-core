"""Step 2: simple dummies of the bought parts, one App::Part each, built from
primitives whose sizes and positions are expressions on Params.

Every dummy lies in its own part coordinates (see the Params section headers);
the assembly will place it with an App::Link. Here they only stand in a row
along X (catalog_pitch).

Run with init_globals={"PARTS": ["MG90S", ...]} to build some, or all by default.
Re-running replaces those parts.
"""
import os
import runpy

import FreeCAD as App

HERE = os.path.dirname(os.path.abspath(__file__))
common = runpy.run_path(os.path.join(HERE, "common.py"))

doc = common["get_doc"]()

BLUE, GOLD, GREY = (0.11, 0.2, 0.55), (0.75, 0.6, 0.2), (0.25, 0.25, 0.28)
BLACK, WHITE, SILVER = (0.08, 0.08, 0.08), (0.92, 0.92, 0.9), (0.72, 0.73, 0.75)
PCB_GREEN, PCB_PURPLE = (0.05, 0.35, 0.2), (0.4, 0.15, 0.5)


def p(alias):
    return "Params." + alias


class Dummy:
    def __init__(self, name, index):
        self.name = name
        # keep the part itself: links in the check assemblies point at it
        self.part = doc.getObject("Dummy_" + name) or doc.addObject("App::Part", "Dummy_" + name)
        for child in self.part.Group:
            doc.removeObject(child.Name)
        self.part.setExpression("Placement.Base.x", "%d * %s" % (index, p("catalog_pitch")))

    def _add(self, kind, sub, x, y, z, color):
        obj = doc.addObject(kind, self.name + "_" + sub)
        self.part.addObject(obj)
        for axis, expr in zip("xyz", (x, y, z)):
            obj.setExpression("Placement.Base." + axis, expr)
        obj.ViewObject.ShapeColor = color
        return obj

    def box(self, sub, lx, ly, lz, x, y, z, color):
        """Box of sizes lx, ly, lz (expressions); x, y, z is its low corner."""
        obj = self._add("Part::Box", sub, x, y, z, color)
        obj.setExpression("Length", lx)
        obj.setExpression("Width", ly)
        obj.setExpression("Height", lz)
        return obj

    def cbox(self, sub, lx, ly, lz, cx, cy, z, color):
        """Box centred on cx, cy in plan, from z upwards."""
        return self.box(sub, lx, ly, lz, "(%s) - (%s) / 2" % (cx, lx), "(%s) - (%s) / 2" % (cy, ly), z, color)

    def cyl(self, sub, d, h, x, y, z, color, along_y=False):
        """Cylinder of diameter d and length h from x, y, z along +Z (or +Y)."""
        obj = self._add("Part::Cylinder", sub, x, y, z, color)
        obj.setExpression("Radius", "%s / 2" % d)
        obj.setExpression("Height", h)
        if along_y:
            obj.Placement = App.Placement(obj.Placement.Base, App.Rotation(App.Vector(1, 0, 0), -90))
        return obj


def mg90s(i):
    d = Dummy("MG90S", i)
    w, ln = p("mg90_body_w"), p("mg90_body_len")
    y0 = "%s - %s" % (p("mg90_shaft_end"), ln)       # body runs from the shaft end to -Y
    z0 = "-" + p("mg90_below_tab")
    d.box("body", w, ln, "%s + %s" % (p("mg90_below_tab"), p("mg90_body_top")), "-%s / 2" % w, y0, z0, BLUE)
    tab_y = "%s + %s / 2 - %s / 2" % (y0, ln, p("mg90_tab_total"))
    d.box("tabs", w, p("mg90_tab_total"), p("mg90_tab_t"), "-%s / 2" % w, tab_y, "0 mm", BLUE)
    d.cyl("turret", p("mg90_turret_d"), "%s - %s" % (p("mg90_turret_top"), p("mg90_body_top")),
          "0 mm", "0 mm", p("mg90_body_top"), BLUE)
    d.cyl("shaft", p("mg90_shaft_d"), "%s - %s" % (p("mg90_shaft_top"), p("mg90_turret_top")),
          "0 mm", "0 mm", p("mg90_turret_top"), SILVER)


def horn(i):
    # own part: each joint turns its horn to its own angle on the servo
    d = Dummy("Horn", i)
    hub_z = "%s + %s" % (p("mg90_turret_top"), p("horn_gap"))
    d.cyl("hub", p("horn_hub_d"), p("horn_hub_h"), "0 mm", "0 mm", hub_z, WHITE)
    arm_z = "%s - %s" % (p("horn_top"), p("horn_arm_t"))
    # single arm along +X
    arm = "%s - %s / 2" % (p("horn_reach"), p("horn_tip_w"))
    d.box("arm", arm, p("horn_tip_w"), p("horn_arm_t"), "0 mm", "-%s / 2" % p("horn_tip_w"), arm_z, WHITE)
    d.cyl("tip", p("horn_tip_w"), p("horn_arm_t"), arm, "0 mm", arm_z, WHITE)


def cells(i):
    d = Dummy("18650_pair", i)
    y0 = "-%s / 2" % p("cell_len")
    z = "0 mm"
    d.cyl("left", p("cell_d"), p("cell_len"), "-" + p("cell_x"), y0, z, (0.15, 0.55, 0.3), along_y=True)
    d.cyl("right", p("cell_d"), p("cell_len"), p("cell_x"), y0, z, (0.15, 0.55, 0.3), along_y=True)
    # the holder around them: end blocks and side walls
    h = p("cell_d")
    wall_z = "-%s / 2" % h
    for tag, y in (("a", "-%s / 2" % p("holder_len")), ("b", "%s / 2" % p("cell_len"))):
        d.box("end_" + tag, p("holder_w"), p("holder_end"), h, "-%s / 2" % p("holder_w"), y, wall_z, GREY)
    for tag, x in (("l", "-%s / 2" % p("holder_w")), ("r", "%s / 2 - %s" % (p("holder_w"), p("holder_side")))):
        d.box("side_" + tag, p("holder_side"), p("cell_len"), h, x, "-%s / 2" % p("cell_len"), wall_z, GREY)
    for tag, x in (("l", "-%s - 1 mm" % p("cell_x")), ("r", "%s - 1 mm" % p("cell_x"))):
        d.box("tab_a" + tag, "2 mm", p("holder_tab"), "6 mm", x, "%s / 2" % p("holder_len"), "-3 mm", GOLD)
        d.box("tab_b" + tag, "2 mm", p("holder_tab"), "6 mm", x, "-%s / 2 - %s" % (p("holder_len"), p("holder_tab")), "-3 mm", GOLD)


def pca9685(i):
    d = Dummy("PCA9685", i)
    t = p("pca_t")
    d.cbox("pcb", p("pca_len"), p("pca_w"), t, "0 mm", "0 mm", "-" + t, PCB_PURPLE)
    # bent pins, one block per group: bend at the -Y edge, tips toward +Y
    gy = "-%s / 2 + %s + %s / 2" % (p("pca_w"), p("pca_grp_edge"), p("pca_grp_depth"))
    for n in "123":
        d.cbox("pins_" + n, p("pca_grp_w"), p("pca_grp_depth"), p("pca_grp_h"), p("pca_grp_x" + n), gy, "0 mm", GOLD)
    # 3 Mini360 on the back, long side across the board
    tape_z = "-(%s + %s)" % (t, p("mini_tape_t"))
    pcb_z = "%s - %s" % (tape_z, p("mini_t"))
    comp_z = "%s - %s" % (pcb_z, p("mini_comp_h"))
    for n, x in (("a", "-" + p("mini_pitch")), ("b", "0 mm"), ("c", p("mini_pitch"))):
        d.cbox("tape_" + n, p("mini_w"), p("mini_len"), p("mini_tape_t"), x, p("mini_node_y"), tape_z, (0.55, 0.7, 0.9))
        d.cbox("mini_" + n, p("mini_w"), p("mini_len"), p("mini_t"), x, p("mini_node_y"), pcb_z, PCB_GREEN)
        d.cbox("comp_" + n, p("mini_comp_w"), p("mini_comp_len"), p("mini_comp_h"), x, p("mini_node_y"), comp_z, GREY)


def mini360(i):
    d = Dummy("Mini360", i)
    t = p("mini_t")
    d.cbox("pcb", p("mini_len"), p("mini_w"), t, "0 mm", "0 mm", "-" + t, PCB_GREEN)
    d.cbox("components", p("mini_comp_len"), p("mini_comp_w"), p("mini_comp_h"), "0 mm", "0 mm", "0 mm", GREY)


def esp32(i):
    d = Dummy("ESP32_devkit", i)
    t = p("esp_t")
    d.cbox("pcb", p("esp_len"), p("esp_w"), t, "0 mm", "0 mm", "-" + t, BLACK)
    d.cbox("module", p("esp_mod_len"), p("esp_mod_w"), p("esp_mod_h"),
           "%s / 2 - %s / 2" % (p("esp_len"), p("esp_mod_len")), "0 mm", "0 mm", SILVER)
    d.cbox("usb", p("esp_usb_len"), p("esp_usb_w"), p("esp_usb_h"),
           "-(%s / 2 + %s - %s / 2)" % (p("esp_len"), p("esp_usb_over"), p("esp_usb_len")), "0 mm", "0 mm", SILVER)


def gy91(i):
    d = Dummy("GY91", i)
    t = p("gy91_t")
    d.cbox("pcb", p("gy91_w"), p("gy91_len"), t, "0 mm", "0 mm", "-" + t, PCB_PURPLE)
    d.cbox("imu", p("gy91_chip"), p("gy91_chip"), p("gy91_chip_h"), "0 mm", "0 mm", "0 mm", BLACK)


def vl53l5cx(i):
    d = Dummy("VL53L5CX", i)
    t = p("tof_t")
    off = p("tof_sensor_off")
    d.cbox("pcb", p("tof_len"), p("tof_wid"), t, "0 mm", off, "-" + t, PCB_GREEN)
    for sub, sign in (("a", ""), ("b", "-")):
        x = "%s(%s / 2)" % (sign, p("tof_hole_span"))
        d.cyl("ear_" + sub, p("tof_ear_d"), t, x, off, "-" + t, PCB_GREEN)
        # dark plug standing for the hole, proud of the ear so it shows
        d.cyl("hole_" + sub, p("tof_hole_d"), "%s + 0.1 mm" % t, x, off, "-" + t, BLACK)
    d.cbox("pin_row", p("tof_pin_row_len"), p("tof_pin_row_w"), "0.1 mm", "0 mm",
           "%s + %s" % (off, p("tof_pin_row_off")), "0 mm", GOLD)
    d.cbox("sensor", p("tof_chip_w"), p("tof_chip_h"), p("tof_chip_t"), "0 mm", "0 mm", "0 mm", BLACK)


def microswitch(i):
    d = Dummy("Microswitch", i)
    d.cbox("body", p("sw_w"), p("sw_w"), p("sw_body_h"), "0 mm", "0 mm", "0 mm", BLACK)
    d.cyl("button", p("sw_btn_d"), "%s - %s" % (p("sw_total_h"), p("sw_body_h")), "0 mm", "0 mm", p("sw_body_h"), GREY)
    lead = "%s - %s" % (p("sw_lead_span"), p("sw_w"))
    for sub, sign in (("lead_a", ""), ("lead_b", "-")):
        d.box(sub, lead, p("sw_lead_w"), p("sw_lead_t"), "%s / 2" % p("sw_w"),
              "%s(%s / 2) - %s / 2" % (sign, p("sw_lead_pitch"), p("sw_lead_w")),
              "%s - %s" % (p("sw_lead_z"), p("sw_lead_t")), SILVER)


def rocker(i):
    d = Dummy("Rocker", i)
    d.cbox("body", p("pwr_body_w"), p("pwr_body_len"), p("pwr_body_h"), "0 mm", "0 mm", "-" + p("pwr_body_h"), BLACK)
    d.cbox("bezel", p("pwr_bezel_w"), p("pwr_bezel_len"), p("pwr_bezel_t"), "0 mm", "0 mm", "0 mm", BLACK)
    d.cbox("rocker", "%s - 2 mm" % p("pwr_body_w"), "%s - 2 mm" % p("pwr_body_len"), p("pwr_rocker_h"), "0 mm", "0 mm", p("pwr_bezel_t"), GREY)
    for sub, y in (("pin_a", "-" + p("pwr_pin_pitch")), ("pin_b", "0 mm")):
        d.cbox(sub, p("pwr_pin_t"), p("pwr_pin_w"), p("pwr_pin_len"), "0 mm", y, "-(%s + %s)" % (p("pwr_body_h"), p("pwr_pin_len")), SILVER)


def bearing(i):
    d = Dummy("Bearing", i)
    d.cyl("ring", p("bearing_od"), p("bearing_t"), "0 mm", "0 mm", "0 mm", SILVER)
    bore = d.cyl("bore", p("bearing_id"), p("bearing_t"), "0 mm", "0 mm", "0 mm", BLACK)
    ring = doc.getObject("Bearing_ring")
    cut = doc.addObject("Part::Cut", "Bearing_body")
    d.part.addObject(cut)
    cut.Base, cut.Tool = ring, bore
    ring.ViewObject.Visibility = bore.ViewObject.Visibility = False
    cut.ViewObject.ShapeColor = SILVER


BUILDERS = {"MG90S": mg90s, "18650_pair": cells, "PCA9685": pca9685, "Mini360": mini360,
            "ESP32_devkit": esp32, "GY91": gy91, "VL53L5CX": vl53l5cx, "Microswitch": microswitch,
            "Bearing": bearing, "Horn": horn, "Rocker": rocker}
ORDER = list(BUILDERS)

for name in globals().get("PARTS") or ORDER:
    BUILDERS[name](ORDER.index(name))
common["save"](doc)
for name in globals().get("PARTS") or ORDER:
    bb = doc.getObject("Dummy_" + name).Shape.BoundBox if hasattr(doc.getObject("Dummy_" + name), "Shape") else None
    print(name, "x %.1f y %.1f z %.1f" % (bb.XLength, bb.YLength, bb.ZLength) if bb else "built")
