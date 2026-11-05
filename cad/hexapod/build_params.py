"""Step 1: the Params spreadsheet, the one home of every dimension.

Re-run after editing the table: cells are rewritten in place, so expressions
that use them stay valid. Then checks the leg geometry against the firmware.

Columns: A alias, B value, C source, D note.
Source says how far to trust a value:
  config.h / params.py / model_data.h / simple_parts.py   the robot design, CAD must match
  hexapodv2 / hexapod.FCStd   taken from the old models in Dropbox/robots/hexapod
  derived        formula from other cells
  TODO measure   a guess: replace it with a caliper value
"""
import math
import os
import re
import runpy

HERE = os.path.dirname(os.path.abspath(__file__))
common = runpy.run_path(os.path.join(HERE, "common.py"))

CFG, SIM, FW, SP = "config.h", "params.py", "model_data.h", "simple_parts.py"
OLD2, OLD1 = "hexapodv2", "hexapod.FCStd"
TODO, CALC = "TODO measure", "derived"

TABLE = [
    "Robot: links and leg geometry (origin: body centre at femur axis height; X right, Y forward, Z up)",
    ("coxa_len", "60 mm", CFG, "LEG_SIZE_COXA"),
    ("femur_len", "50 mm", CFG, "LEG_SIZE_FEMUR"),
    ("tibia_len", "60 mm", CFG, "LEG_SIZE_TIBIA"),
    ("coxa_front_x", "35 mm", CFG, "front coxa axis, +-x"),
    ("coxa_front_y", "58 mm", CFG, "front coxa axis"),
    ("coxa_mid_x", "45 mm", CFG, "middle coxa axis, +-x"),
    ("coxa_mid_y", "0 mm", CFG, "middle coxa axis"),
    ("coxa_rear_x", "35 mm", CFG, "rear coxa axis, +-x"),
    ("coxa_rear_y", "-58 mm", CFG, "rear coxa axis"),
    ("front_splay", "20 deg", CFG, "front legs further out than radial, out of the ToF view"),
    ("foot_out", "69 mm", CFG, "neutral foot from the coxa axis (coxa 60 + 9)"),
    ("foot_down", "100 mm", CFG, "neutral foot below the femur axis, LEG_FOOT_Z"),
    ("front_leg_yaw", "=atan2(coxa_front_y; coxa_front_x) - front_splay", CALC, "front leg direction from +X, right side"),
    ("rear_leg_yaw", "=atan2(coxa_rear_y; coxa_rear_x)", CALC, "rear leg direction from +X, right side"),
    ("foot_front_x", "=coxa_front_x + foot_out * cos(front_leg_yaw)", CALC, "config.h 88.71"),
    ("foot_front_y", "=coxa_front_y + foot_out * sin(front_leg_yaw)", CALC, "config.h 101.32"),
    ("foot_mid_x", "=coxa_mid_x + foot_out", CALC, "config.h 114"),
    ("foot_rear_x", "=coxa_rear_x + foot_out * cos(rear_leg_yaw)", CALC, "config.h 70.65"),
    ("foot_rear_y", "=coxa_rear_y + foot_out * sin(rear_leg_yaw)", CALC, "config.h -117.08"),

    "Body envelope",
    ("plate_inflate", "14 mm", SIM, "plates reach past the coxa axes"),
    ("body_bottom_z", "-28 mm", SIM, "BOTTOM_Z, bottom plate"),
    ("body_top_z", "16 mm", SIM, "TOP_Z, top cover"),
    ("top_cover_t", "3 mm", SIM, "torso_size: TOP_Z + 3"),

    "Layout of the bought parts in the body",
    ("cell_x", "9.5 mm", SP, "cell axes at +-x"),
    ("cell_z", "-17 mm", SP, "cell axis height"),
    ("pca_x", "23 mm", SP, "standing board at +-x"),
    ("pca_z", "-12 mm", SP, "standing board centre"),
    ("mini_x", "26.3 mm", SP, "Mini360 beside the board at +-x"),
    ("mini_dy", "18 mm", SP, "Mini360 at y = -dy, 0, +dy"),
    ("mini_z", "-12 mm", SP, "Mini360 centre"),
    ("logic_mini_y", "-44 mm", SP, "7th Mini360 (logic) at the rear"),
    ("esp_y", "45 mm", SP, "ESP32 across the front"),
    ("esp_z", "-12 mm", SP, "ESP32 centre"),
    ("gy91_z", "=body_top_z - 3 mm", SIM, "imu_pos: under the top cover, x = y = 0"),
    ("tof_x", "14 mm", FW, "TOF_SENSORS at +-x"),
    ("tof_y", "74 mm", FW, "TOF_SENSORS"),
    ("tof_z", "2 mm", FW, "TOF_SENSORS"),
    ("tof_yaw", "20 deg", FW, "0.35 rad, turned outward"),
    ("tof_tilt_left", "35 deg", FW, "0.61 rad down, terrain"),
    ("tof_tilt_right", "22 deg", FW, "0.384 rad down, room scan"),

    "MG90S (part origin: shaft axis at the underside of the tabs; +Z to the horn; body runs to -Y)",
    ("mg90_body_w", "12 mm", OLD2, "pocket sketch; old fit pocket 12.4"),
    ("mg90_body_len", "23 mm", OLD2, "pocket sketch; old fit pocket 23.4"),
    ("mg90_tab_total", "32.3 mm", OLD1, "tab tip to tab tip (coxaServo: 4.65 each side)"),
    ("mg90_shaft_end", "6 mm", OLD2, "shaft axis from the near end of the body"),
    ("mg90_screw_spacing", "27.86 mm", OLD1, "tab screw centres"),
    ("mg90_below_tab", "18.5 mm", TODO, "body bottom to tab underside"),
    ("mg90_tab_t", "2.5 mm", TODO, "tab thickness"),
    ("mg90_body_top", "6 mm", TODO, "tab underside to top of the rectangular body"),
    ("mg90_turret_d", "11.8 mm", TODO, "round gear turret around the shaft"),
    ("mg90_turret_top", "9.9 mm", TODO, "tab underside to top of the turret"),
    ("mg90_shaft_d", "4.8 mm", TODO, "spline outside diameter"),
    ("mg90_shaft_top", "13.6 mm", TODO, "tab underside to the shaft tip"),
    ("horn_span", "36 mm", OLD1, "two-arm horn, tip to tip (servo_horn_cut outline)"),
    ("horn_tip_w", "5.8 mm", OLD1, "arm width at the tip (servo_horn_cut outline)"),
    ("horn_hub_d", "8.1 mm", TODO, "servo_horn_cut has circles r 4.05 and r 6.1, assumed the smaller"),
    ("horn_hub_h", "4 mm", TODO, "hub height, arm top to the hub's lower face"),
    ("horn_arm_t", "1.6 mm", TODO, "arm thickness"),
    ("horn_gap", "0.3 mm", TODO, "turret top to the horn hub when screwed down"),
    ("horn_top", "=mg90_turret_top + horn_gap + horn_hub_h", CALC, "tab underside to the horn's outer face"),
    ("mg90_stack", "=mg90_below_tab + horn_top", CALC, "body bottom to the horn's outer face"),

    "18650 pair (part origin: centre of the pair; cells along Y)",
    ("cell_d", "18.4 mm", TODO, "cell diameter"),
    ("cell_len", "65 mm", TODO, "65 unprotected in simple_parts.py; protected cells ~69"),

    "PCA9685 board (part origin: PCB centre, lying flat; long side along X; +Z = component side)",
    ("pca_len", "62.5 mm", TODO, "sim 62; old pocket 63 with clearance"),
    ("pca_w", "25.4 mm", TODO, "sim 25"),
    ("pca_t", "1.6 mm", TODO, "PCB"),
    ("pca_hole_spacing", "55.86 mm", OLD1, "mount hole centres"),
    ("pca_hdr_len", "40.64 mm", TODO, "servo header block, 16 x 2.54"),
    ("pca_hdr_w", "7.62 mm", TODO, "3 rows x 2.54"),
    ("pca_hdr_h", "11.5 mm", TODO, "pin tips above the PCB, servo plugs add more"),
    ("pca_hdr_edge", "1 mm", TODO, "header block from the long edge"),
    ("pca_term_len", "10 mm", TODO, "screw terminal"),
    ("pca_term_w", "7.5 mm", TODO, "screw terminal"),
    ("pca_term_h", "10 mm", TODO, "screw terminal above the PCB"),

    "Mini360 buck (part origin: PCB centre, lying flat; +Z = component side)",
    ("mini_len", "17.2 mm", TODO, "sim 17; old pocket 18.5 with clearance"),
    ("mini_w", "11.2 mm", TODO, "sim 11; old pocket 12 with clearance"),
    ("mini_t", "1 mm", TODO, "PCB"),
    ("mini_comp_len", "13 mm", TODO, "component envelope"),
    ("mini_comp_w", "9 mm", TODO, "component envelope"),
    ("mini_comp_h", "3 mm", TODO, "tallest part above the PCB"),

    "ESP32 devkit (part origin: PCB centre, lying flat; long side along X; +Z = module side; USB at -X)",
    ("esp_len", "52 mm", TODO, "sim 52; 30-pin board assumed"),
    ("esp_w", "28 mm", TODO, "sim 28"),
    ("esp_t", "1.6 mm", TODO, "PCB"),
    ("esp_mod_len", "25.5 mm", TODO, "WROOM module"),
    ("esp_mod_w", "18 mm", TODO, "WROOM module"),
    ("esp_mod_h", "3.2 mm", TODO, "WROOM module above the PCB"),
    ("esp_usb_len", "5.6 mm", TODO, "USB socket"),
    ("esp_usb_w", "7.6 mm", TODO, "USB socket"),
    ("esp_usb_h", "2.8 mm", TODO, "USB socket above the PCB"),
    ("esp_hdr_len", "38.1 mm", TODO, "pin header, 15 x 2.54"),
    ("esp_hdr_w", "2.54 mm", TODO, "pin header"),
    ("esp_hdr_h", "11 mm", TODO, "pins below the PCB"),
    ("esp_hdr_pitch", "25.4 mm", TODO, "between the two pin rows"),

    "GY-91 (part origin: PCB centre, lying flat; long side along Y; +Z = chip side)",
    ("gy91_w", "14 mm", TODO, "sim 14"),
    ("gy91_len", "20.5 mm", TODO, "sim 20"),
    ("gy91_t", "1.6 mm", TODO, "PCB"),
    ("gy91_chip", "3 mm", TODO, "MPU9250, marks the IMU centre"),
    ("gy91_chip_h", "1 mm", TODO, "MPU9250 above the PCB"),
    ("gy91_hdr_len", "20.32 mm", TODO, "8 x 2.54 along the long edge"),
    ("gy91_hdr_w", "2.54 mm", TODO, "pin header"),
    ("gy91_hdr_h", "8.5 mm", TODO, "pins below the PCB"),

    "VL53L5CX board (part origin: sensor centre on the PCB face; +Z = where it looks)",
    ("tof_w", "13 mm", TODO, "board width, sim 13"),
    ("tof_h", "9 mm", TODO, "board height, sim 9"),
    ("tof_t", "1.6 mm", TODO, "PCB"),
    ("tof_chip_w", "6.4 mm", TODO, "sensor package, datasheet"),
    ("tof_chip_h", "3 mm", TODO, "sensor package, datasheet"),
    ("tof_chip_t", "1.5 mm", TODO, "sensor package, datasheet"),

    "Lever microswitch (part origin: body centre; lever on +Y side, pivoting at -X; terminals on -Y)",
    ("sw_len", "12.8 mm", TODO, "body along the lever"),
    ("sw_h", "6.5 mm", TODO, "body, terminal side to lever side"),
    ("sw_t", "5.8 mm", TODO, "body thickness"),
    ("sw_lever_len", "13 mm", TODO, "lever"),
    ("sw_lever_w", "3.5 mm", TODO, "lever"),
    ("sw_lever_t", "0.5 mm", TODO, "lever"),
    ("sw_lever_gap", "1.5 mm", TODO, "lever above the body when released"),
    ("sw_term_pitch", "5 mm", TODO, "3 terminals"),
    ("sw_term_len", "3.5 mm", TODO, "terminals below the body"),
    ("sw_term_w", "1.5 mm", TODO, "terminal"),
    ("sw_term_t", "0.5 mm", TODO, "terminal"),

    "Catalog: how the dummies are spread out in the document (not robot sizes)",
    ("catalog_pitch", "90 mm", "layout", "between dummies along X"),
]


def fill(sheet):
    sheet.clearAll()
    for col, text in zip("ABCD", ("alias", "value", "source", "note")):
        sheet.set(col + "1", text)
        sheet.setStyle(col + "1", "bold")
    row = 2
    for item in TABLE:
        if isinstance(item, str):
            row += 1
            sheet.set("A%d" % row, item)
            sheet.setStyle("A%d" % row, "bold")
        else:
            alias, value, source, note = item
            sheet.set("A%d" % row, alias)
            sheet.set("B%d" % row, value if value.startswith("=") else "=" + value)
            sheet.setAlias("B%d" % row, alias)
            sheet.set("C%d" % row, source)
            sheet.set("D%d" % row, note)
            if source == TODO:
                sheet.setBackground("B%d" % row, (1.0, 0.9, 0.5, 1.0))
        row += 1
    sheet.setColumnWidth("A", 170)
    sheet.setColumnWidth("B", 90)
    sheet.setColumnWidth("C", 110)
    sheet.setColumnWidth("D", 420)


def check_against_firmware(sheet):
    """The CAD must match config.h: fail loudly when it doesn't."""
    root = os.path.abspath(os.path.join(HERE, "..", ".."))
    text = open(os.path.join(root, "src/esp32-legged-robot/model/hexapod/config.h")).read()
    defs = {k: float(v) for k, v in re.findall(r"#define\s+(LEG_\w+)\s+(-?[\d.]+)", text)}
    pairs = {
        "LEG_SIZE_COXA": "coxa_len", "LEG_SIZE_FEMUR": "femur_len", "LEG_SIZE_TIBIA": "tibia_len",
        "LEG_FRONT_BODY_X": "coxa_front_x", "LEG_FRONT_BODY_Y": "coxa_front_y",
        "LEG_FRONT_FOOT_X": "foot_front_x", "LEG_FRONT_FOOT_Y": "foot_front_y",
        "LEG_MIDDLE_BODY_X": "coxa_mid_x", "LEG_MIDDLE_BODY_Y": "coxa_mid_y",
        "LEG_MIDDLE_FOOT_X": "foot_mid_x",
        "LEG_REAR_BODY_X": "coxa_rear_x", "LEG_REAR_BODY_Y": "coxa_rear_y",
        "LEG_REAR_FOOT_X": "foot_rear_x", "LEG_REAR_FOOT_Y": "foot_rear_y",
        "LEG_FOOT_Z": "foot_down",
    }
    bad = []
    for define, alias in pairs.items():
        got = float(getattr(sheet.get(alias), "Value", sheet.get(alias)))
        if abs(got - defs[define]) > 0.01:
            bad.append("%s=%g but %s=%g" % (define, defs[define], alias, got))
    if bad:
        raise ValueError("Params does not match config.h: " + "; ".join(bad))
    print("Params matches config.h: %d values checked" % len(pairs))


doc = common["get_doc"]()
sheet = doc.getObject("Params") or doc.addObject("Spreadsheet::Sheet", "Params")
fill(sheet)
doc.recompute()
check_against_firmware(sheet)
common["save"](doc)
rows = [i for i in TABLE if not isinstance(i, str)]
print("%d params, %d TODO measure, saved %s" % (len(rows), sum(i[2] == TODO for i in rows), doc.FileName))
