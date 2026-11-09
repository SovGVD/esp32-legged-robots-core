"""Step 1: the Params spreadsheet, the one home of every dimension.

Re-run after editing the table: cells are rewritten in place, so expressions
that use them stay valid. Then checks the leg geometry against the firmware.

Columns: A alias, B value, C source, D note.
Source says how far to trust a value:
  config.h / params.py / model_data.h / simple_parts.py   the robot design, CAD must match
  hexapodv2 / hexapod.FCStd   taken from the old models in Dropbox/robots/hexapod
  biped          leg_bottom_v2 in Dropbox/robots/biped/biped_maybebroken.FCStd
  design         a choice made for this CAD
  caliper        measured on the real part
  photo / drawing / web   from a picture or seller page, not measured
  not critical   a guess, but its exact size doesn't matter
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
CAL, PHOTO, DRAWING, WEB, STD, LOOSE = "caliper", "photo", "drawing", "web", "standard", "not critical"
DESIGN = "design"
BIPED = "biped"

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
    ("body_plate_t", "4 mm", DESIGN, "bottom plate, holds the coxa bearings"),

    "Layout of the bought parts in the body",
    ("cell_x", "9.9 mm", PHOTO, "cell axes at +-x, half the holder pitch"),
    ("cell_z", "=body_plate_top + cell_d / 2 + 0.2 mm", CALC, "cell axis height: the holder stands on the bottom plate"),
    ("pca_x", "27.4 mm", DESIGN, "standing board at +-x; simple_parts.py has 23. Pins outward with the bends up and the tips down, so the connectors plug in from below (the cables come up through the bottom plate), Mini360 inward; the left board is the right one turned 180 deg"),
    ("pca_z", "=body_plate_top + pca_w / 2 + 0.2 mm", CALC, "standing board centre, on the bottom plate"),
    ("pca_y", "6.1 mm", DESIGN, "right board slid along Y, left one the opposite way: the middle coxa strap swings in the gap in its pins (the board turned over: was -6.1)"),
    ("logic_mini_y", "-44 mm", SP, "7th Mini360 (logic) at the rear"),
    ("esp_y", "40.5 mm", DESIGN, "ESP32 across the front, clear of the front links' tails; simple_parts.py has 45"),
    ("esp_z", "=body_plate_top + esp_w / 2 + 0.2 mm", CALC, "ESP32 centre, on the bottom plate"),
    ("gy91_z", "=top_plate_z0 - 1.5 mm", CALC, "under the top plate, x = y = 0"),
    ("tof_x", "14 mm", FW, "TOF_SENSORS at +-x"),
    ("tof_y", "74 mm", FW, "TOF_SENSORS"),
    ("tof_z", "=top_plate_z1 + tof_base_t + 4 mm", FW, "TOF_SENSORS: 22, on the mount on the top plate (at 2 the boards hit the front coxa links)"),
    ("tof_yaw", "20 deg", FW, "0.35 rad, turned outward"),
    ("tof_tilt_left", "35 deg", FW, "0.61 rad down, terrain"),
    ("tof_tilt_right", "22 deg", FW, "0.384 rad down, room scan"),

    "MG90S (part origin: shaft axis at the underside of the tabs; +Z to the horn; body runs to -Y)",
    ("mg90_body_w", "12.15 mm", CAL, "body width"),
    ("mg90_body_len", "23.16 mm", CAL, "body length"),
    ("mg90_fit", "0.2 mm", CAL, "servo pocket clearance per side along the servo (fit test: right)"),
    ("mg90_fit_w", "0.15 mm", CAL, "servo pocket clearance per side across the servo (fit test: 0.2 a touch loose)"),
    ("mg90_tab_total", "32.39 mm", CAL, "tab tip to tab tip"),
    ("mg90_shaft_end", "6 mm", OLD2, "servoToShaft, shaft axis from the near end of the body"),
    ("mg90_screw_spacing", "27.86 mm", OLD2, "servoBetweenMountHoles in the hexapod v2 model (the user's source); servo drawing says 28"),
    ("mg90_screw_pilot_d", "1.56 mm", OLD2, "servoScrew r 0.78"),
    ("mg90_below_tab", "17.54 mm", CAL, "body bottom to the tabs' underside (the face resting on the link)"),
    ("mg90_tab_t", "2.5 mm", OLD2, "servoMount"),
    ("mg90_body_top", "6 mm", TODO, "tab underside to top of the rectangular body, under the turret"),
    ("mg90_turret_d", "=mg90_body_w", CALC, "the round turret is as wide as the servo, ~12"),
    ("mg90_turret_top", "=mg90_height - mg90_below_tab", CALC, "tab underside to the top of the servo"),
    ("mg90_height", "28.64 mm", CAL, "body bottom to the top of the round turret, without the shaft"),
    ("mg90_shaft_d", "5 mm", OLD2, "probablyServoShaft"),
    ("mg90_shaft_tip", "31.89 mm", CAL, "body bottom to the shaft's spline tip"),
    ("mg90_horn_face", "34.04 mm", CAL, "body bottom to the bought (unflattened) horn's outer face; the flattened one is 1.35 lower"),
    ("mg90_shaft_top", "=mg90_shaft_tip - mg90_below_tab", CALC, "tab underside to the shaft tip"),
    ("horn_reach", "18.3 mm", CAL, "shaft to arm tip: 21.95 overall minus half the 7.24 hub"),
    ("horn_tip_w", "4.7 mm", CAL, "the arm's round tip; the arm tapers to it from the hub (small_dog Sketch062 has the same shape)"),
    ("horn_hub_d", "7.24 mm", CAL, "hub"),
    ("horn_hub_h", "3.8 mm", DESIGN, "the horn flattened by the user: 2 mm plate + 1.8 mm on the spline (5.15 as bought)"),
    ("horn_arm_t", "2.02 mm", CAL, "arm thickness"),
    ("horn_fit", "0.15 mm", DESIGN, "clearance per side in the horn pockets (glued): printed holes come out small"),
    ("horn_pocket_d", "=horn_hub_d + 2 * horn_fit", CALC, "pocket for the hub"),
    ("horn_pocket_w", "=horn_tip_w + 2 * horn_fit", CALC, "pocket for the arm"),
    ("horn_gap", "0.25 mm", CAL, "turret top to the horn: 34.04 to the bought horn's face - 28.64 - 5.15"),
    ("horn_top", "=mg90_turret_top + horn_gap + horn_hub_h", CALC, "tab underside to the horn's outer face"),
    ("mg90_stack", "=mg90_below_tab + horn_top", CALC, "body bottom to the horn's outer face"),
    ("mg90_tab_near", "=mg90_tab_total / 2 + mg90_body_len / 2 - (mg90_body_len - mg90_shaft_end)", CALC, "shaft to the near tab tip"),
    ("mg90_tab_far", "=mg90_tab_total - mg90_tab_near", CALC, "shaft to the far tab tip"),
    ("mg90_hole_near", "=mg90_screw_spacing / 2 - (mg90_body_len / 2 - mg90_shaft_end)", CALC, "shaft to the near tab screw"),
    ("mg90_hole_far", "=mg90_screw_spacing - mg90_hole_near", CALC, "shaft to the far tab screw"),

    "Pivot bearing MR128K, opposite every horn (part origin: bearing centre on one face; axis along Z)",
    ("bearing_od", "12 mm", PHOTO, "seller picture; matches hexapodv2"),
    ("bearing_id", "8 mm", PHOTO, "seller picture; hexapodv2 inBearingHolder r 4"),
    ("bearing_t", "2.5 mm", PHOTO, "seller picture; hexapodv2 bearing pockets"),
    ("bearing_seat_d", "12.3 mm", CAL, "pocket for the outer ring: 12.2 fitted in one fit test and was too tight in the next"),
    ("bearing_push_d", "10 mm", CAL, "hole under each bearing seat: the outer ring keeps a 1.1 mm ledge and a tool can push it out (fit test)"),

    "18650 pair (part origin: centre of the pair; cells along Y)",
    ("cell_d", "18 mm", STD, "18650"),
    ("cell_len", "65 mm", STD, "18650"),

    "Battery holder, flat: 2 cells, same height as a cell (part origin: its centre)",
    ("holder_len", "77.19 mm", CAL, "caliper, body end to end"),
    ("holder_w", "39.77 mm", CAL, "caliper"),
    ("holder_end", "=(holder_len - cell_len) / 2", CALC, "end block past the cells"),
    ("holder_side", "=holder_w / 2 - cell_x - cell_d / 2", CALC, "side wall outside a cell"),
    ("holder_hole_span", "55.46 mm", CAL, "caliper: average of 58.57 across the holes' outer edges and 52.34 across their inner edges (holes 3.1)"),
    ("holder_tab", "3.7 mm", TODO, "solder tabs past each end, scaled from the seller picture"),

    "PCA9685 node, HW-052 with bent pins on top and 3 Mini360 glued on the back; I2C and power wires soldered, not modelled (part origin: PCB centre, lying flat; long side along X; +Z = pin side; pin bends at -Y)",
    ("pca_len", "62.41 mm", CAL, "PCB"),
    ("pca_w", "25.22 mm", CAL, "PCB"),
    ("pca_t", "1.55 mm", CAL, "PCB (user: 1.5-1.6); the slot at 1.6 + 0.1 was a touch loose"),
    ("pca_hole_spacing", "55.85 mm", OLD1, "servoControllerBetweenHoles, along the board"),
    ("pca_hole_across", "19.05 mm", TODO, "standard board; ~18.8 in the caliper photo"),
    ("pca_hole_d", "2.4 mm", OLD1, "servoControllerHole"),
    ("pca_grp_w", "7.62 mm", STD, "pin group, 3 channels x 2.54"),
    ("pca_grp_depth", "15.5 mm", PHOTO, "bent pins lie across the board: bend to tip"),
    ("pca_grp_h", "8.5 mm", CAL, "caliper: PCB to the top of the bent pins' black plastic"),
    ("pca_grp_edge", "1 mm", PHOTO, "bend from the long edge"),
    ("pca_grp_x1", "-22.5 mm", PHOTO, "1st pin group centre from the PCB centre"),
    ("pca_grp_x2", "-9.5 mm", PHOTO, "2nd pin group"),
    ("pca_grp_x3", "21.7 mm", PHOTO, "3rd pin group; the space around +7.4 has no pins"),
    ("mini_pitch", "12 mm", PHOTO, "3 Mini360 side by side on the back of the PCB, long side across it"),
    ("mini_node_y", "3.5 mm", PHOTO, "row centre, toward the long edge away from the pin bends"),
    ("mini_tape_t", "1 mm", TODO, "thermal pad under the Mini360 (only the total, mini_stack, is measured)"),

    "Mini360 buck (part origin: PCB centre, lying flat; +Z = component side)",
    ("mini_len", "17.4 mm", CAL, "PCB, about"),
    ("mini_w", "11 mm", CAL, "PCB, about"),
    ("mini_t", "1 mm", LOOSE, "PCB"),
    ("mini_comp_len", "13 mm", LOOSE, "component envelope"),
    ("mini_comp_w", "9 mm", LOOSE, "component envelope"),
    ("mini_stack", "4 mm", CAL, "PCA9685 PCB back to the top of the inductor: thermal pad + PCB + inductor"),
    ("mini_comp_h", "=mini_stack - mini_tape_t - mini_t", CALC, "the inductor, tallest part above the PCB"),

    "ESP32 devkit, 38 pins removed, wires soldered, USB-C (part origin: PCB centre, lying flat; long side along X; +Z = module side; USB at -X)",
    ("esp_len", "51.57 mm", CAL, "PCB, the USB-C socket ends flush with it"),
    ("esp_w", "25.75 mm", CAL, "PCB"),
    ("esp_t", "1.64 mm", CAL, "caliper, PCB"),
    ("esp_mod_len", "25.5 mm", TODO, "WROOM module"),
    ("esp_mod_w", "18 mm", TODO, "WROOM module"),
    ("esp_mod_h", "3.2 mm", CAL, "WROOM module above the PCB, caliper (about)"),
    ("esp_usb_len", "9 mm", TODO, "USB-C socket, scaled from the caliper photo"),
    ("esp_usb_w", "9 mm", TODO, "USB-C socket, scaled from the caliper photo"),
    ("esp_usb_h", "3.2 mm", TODO, "USB-C socket above the PCB"),
    ("esp_usb_over", "0 mm", CAL, "socket past the PCB end: flush"),

    "GY-91, no pins, lies flat (part origin: PCB centre, lying flat; long side along Y; +Z = chip side)",
    ("gy91_w", "14.3 mm", WEB, "seller spec 2.05 x 1.43 cm"),
    ("gy91_len", "20.5 mm", WEB, "seller spec 2.05 x 1.43 cm"),
    ("gy91_t", "1.6 mm", TODO, "PCB"),
    ("gy91_chip", "3 mm", TODO, "MPU9250, marks the IMU centre"),
    ("gy91_chip_h", "1 mm", TODO, "MPU9250 above the PCB"),

    "VL53L5CX board with 2 mounting ears, sizes scaled from a seller photo using the 2.54 pin pitch (part origin: sensor centre on the PCB face; +Z = where it looks; pin row on +Y)",
    ("tof_len", "15.87 mm", CAL, "rectangular part, along the pin row"),
    ("tof_wid", "10 mm", CAL, "rectangular part, across the pin row"),
    ("tof_t", "1.49 mm", CAL, "PCB"),
    ("tof_len_all", "24.87 mm", CAL, "ear to ear"),
    ("tof_hole_span", "19.7 mm", CAL, "mounting hole centres"),
    ("tof_hole_d", "2.94 mm", CAL, "mounting hole"),
    ("tof_ear_d", "=tof_len_all - tof_hole_span", CALC, "round ear around the hole"),
    ("tof_total_t", "6.26 mm", CAL, "cover top to the parts on the back"),
    ("tof_cover_w", "10.35 mm", CAL, "sensor's black cover, top part, along the pin row"),
    ("tof_cover_h", "5.66 mm", CAL, "that top part, across the pin row"),
    ("tof_cover_base_w", "11.9 mm", CAL, "the cover's base glued to the board, along the pin row"),
    ("tof_cover_base_h", "7.28 mm", CAL, "that base, across the pin row: runs 0.78 past the board's far edge"),
    ("tof_cover_t", "3.57 mm", CAL, "cover height above the board"),
    ("tof_cover_base_t", "1 mm", TODO, "the base's height, of the 3.57"),
    ("tof_cover_pin_gap", "3.5 mm", CAL, "board's pin-side edge to the cover's base"),
    ("tof_sensor_off", "=tof_cover_pin_gap + tof_cover_base_h / 2 - tof_wid / 2", CALC, "sensor (the cover's centre) from the PCB centre, away from the pins"),
    ("tof_back_parts_t", "=tof_total_t - tof_t - tof_cover_t", CALC, "parts on the board's back"),
    ("tof_pin_row_len", "15.24 mm", STD, "6 pins x 2.54"),
    ("tof_pin_row_w", "1.7 mm", TODO, "pad row"),
    ("tof_pin_row_off", "=tof_wid / 2 - 1.3 mm", CALC, "pad row from the PCB centre"),

    "Power switch, KCD1 rocker (LY601), 2 pins ON-OFF (part origin: centre of the panel face it snaps into, i.e. under the bezel; +Z = rocker; pins down; long side along Y)",
    ("pwr_body_w", "11.6 mm", CAL, "body width across the short side (user, at the fit test; an earlier reading of 12.49 included more)"),
    ("pwr_body_len", "18.18 mm", CAL, "body below the bezel"),
    ("pwr_clip_w", "12.96 mm", CAL, "across the snap clips: they must spring in through the cutout"),
    ("pwr_clip_len", "19.13 mm", CAL, "across the snap clips along the long side: the 18.68 cutout leaves them ~0.2 mm to catch"),
    ("pwr_bezel_w", "14.93 mm", CAL, "caliper, frame on top of the panel"),
    ("pwr_bezel_len", "20.9 mm", CAL, "caliper, frame on top of the panel (20.86 / 20.99)"),
    ("pwr_bezel_t", "2 mm", PHOTO, "frame thickness"),
    ("pwr_rocker_h", "4 mm", PHOTO, "rocker above the frame"),
    ("pwr_body_h", "13 mm", PHOTO, "body below the panel face, without pins"),
    ("pwr_pin_w", "4.8 mm", PHOTO, "quick-connect tab"),
    ("pwr_pin_t", "0.8 mm", TODO, "tab"),
    ("pwr_pin_len", "7 mm", PHOTO, "tab below the body"),
    ("pwr_pin_pitch", "7 mm", PHOTO, "2 tabs: one at the centre, one toward an end (3rd place empty)"),
    ("pwr_panel_t", "1.6 mm", TODO, "panel thickness the snap clips hold"),

    "XT30 pair, mated, axis vertical (part origin: centre of the male's underside; male below, female on top; long side along X)",
    ("xt30_w", "10.1 mm", TODO, "housing, long side"),
    ("xt30_t", "5.1 mm", TODO, "housing, short side"),
    ("xt30_male_h", "8 mm", TODO, "male housing with its solder cups, below the female's mouth"),
    ("xt30_mated_h", "16 mm", TODO, "the pair mated, solder cup to solder cup"),

    "INA219 module, the common purple one, pins off and wires soldered (part origin: middle of the PCB's bottom edge, PCB standing in the YZ plane; +X = component side; screw terminal at the top)",
    ("ina_w", "22 mm", TODO, "PCB along Y, typical seller size"),
    ("ina_h", "25 mm", TODO, "PCB height, typical seller size"),
    ("ina_t", "1.6 mm", TODO, "PCB"),
    ("ina_parts_h", "1.5 mm", TODO, "shunt resistor and chip above the PCB"),
    ("ina_term_w", "10.2 mm", TODO, "2-pin 5.08 screw terminal, along the PCB"),
    ("ina_term_len", "7.5 mm", TODO, "screw terminal footprint, up the PCB"),
    ("ina_term_out", "10 mm", TODO, "screw terminal, out from the PCB"),

    "Foot switch, 6x6 tact switch with side leads (part origin: centre of the body's underside; +Z = button; leads on +X)",
    ("sw_w", "5.98 mm", CAL, "caliper, body, square"),
    ("sw_body_h", "3.39 mm", CAL, "caliper, body without the button"),
    ("sw_total_h", "4.03 mm", CAL, "caliper, body and button"),
    ("sw_btn_d", "3.5 mm", DRAWING, "button"),
    ("sw_travel", "0.4 mm", OLD1, "clickButtonMoveLength"),
    ("sw_lead_span", "10.3 mm", DRAWING, "body edge to the lead tips, overall"),
    ("sw_lead_pitch", "4.5 mm", DRAWING, "between the 2 leads"),
    ("sw_lead_w", "0.7 mm", DRAWING, "lead"),
    ("sw_lead_t", "0.3 mm", DRAWING, "lead"),
    ("sw_lead_z", "2.15 mm", DRAWING, "underside to the top of the lead"),

    "Leg joints: each servo is part of the link that holds it; its horn is fixed to the next part in, and opposite the horn a shaft runs in a bearing there",
    ("joint_gap", "0.5 mm", DESIGN, "servo bottom to the floor of its box"),
    ("joint_shift", "=(joint_gap + fem_floor_t + fem_boss_h + bearing_t + tib_bear_roof - tib_horn_roof) / 2", CALC,
     "each servo sits this far toward its horn: the whole joint, not the servo, is centred on the leg plane, so the parts around it can be mirror images"),
    ("joint_horn_y", "=mg90_stack / 2 + joint_shift", CALC, "outer face of the horn"),
    ("joint_seat_y", "=-mg90_stack / 2 + mg90_below_tab + joint_shift", CALC, "tab underside, the servo plate's face"),
    ("print_chamfer", "2 mm", DESIGN, "internal chamfers, for PLA strength"),
    ("pla_density", "1.24", STD, "g/cm3, solid, for the weight check"),
    ("pla_E", "2160", WEB, "MPa, Bambu PLA Basic datasheet, across the layers (2680 along them)"),
    ("pla_limit", "17.5", DESIGN, "MPa, FEA limit: half of the datasheet's 35 MPa across the layers"),

    "Femur, after biped leg_bottom_v2: a thick plate with the 2 servos through it, tabs in pockets, and a strap at each end carrying the shaft (femur frame: origin on the femur axis, +X to the knee, Z up with the femur level, horns on +Y)",
    ("fem_plate_floor", "6.5 mm", DESIGN, "plate under the tab pockets; biped 4.7 was over the PLA limit between the servos in the landing FEA"),
    ("fem_rim_h", "8 mm", BIPED, "tab pocket depth, the rim around the tabs"),
    ("fem_plate_y0", "=joint_seat_y - fem_plate_floor", CALC, "underside of the plate"),
    ("fem_plate_y1", "=joint_seat_y + fem_rim_h", CALC, "top of the rim"),
    ("fem_wall_t", "3 mm", DESIGN, "strap and rim walls"),
    ("fem_floor_t", "3 mm", DESIGN, "strap floor under the servo, carries the shaft"),
    ("fem_floor_y1", "=-mg90_stack / 2 - joint_gap + joint_shift", CALC, "inner face of the floor"),
    ("fem_floor_y0", "=fem_floor_y1 - fem_floor_t", CALC, "outer face of the floor"),
    ("fem_half_h", "=mg90_body_w / 2 + mg90_fit_w + fem_wall_t", CALC, "half the height in Z"),
    ("fem_plate_x0", "=-mg90_tab_near - mg90_fit - fem_wall_t", CALC, "plate end past the femur servo tab"),
    ("fem_plate_x1", "=femur_len - fem_plate_x0", CALC, "plate end past the tibia servo tab"),
    ("fem_corner", "4 mm", DESIGN, "plate corners cut off at the ends"),
    ("fem_box_x0", "=-mg90_shaft_end - mg90_fit - fem_wall_t", CALC, "outer face of the strap's end wall"),
    ("fem_strap_x1", "9 mm", DESIGN, "strap side walls reach this far past the shaft; between the straps the servos are bare"),
    ("fem_strap_taper", "6 mm", DESIGN, "strap side walls taper into the plate"),
    ("fem_lobe_d", "18 mm", DESIGN, "round floor around the shaft"),
    ("fem_boss_d", "9 mm", BIPED, "leg_bottom_bearing_pad: spacer at the root of each shaft, on the bearing's inner ring only"),
    ("fem_boss_h", "1 mm", BIPED, "spacer"),
    ("fem_shaft_d", "=bearing_id", CALC, "shaft"),
    ("fem_shaft_len", "2.6 mm", CAL, "shaft in the bearing (2.5 thick): fit test asked for 0.4 shorter than 3"),
    ("fem_edge_ch", "1 mm", BIPED, "chamfer on the edges"),
    ("fem_fit_ch", "=lead_big", CALC, "lead-in on the servo openings: tab pocket mouth and servo hole"),
    ("fem_shaft_ch", "0.4 mm", BIPED, "chamfer on the shaft tip"),
    ("fem_boss_ch", "0.2 mm", BIPED, "chamfer on the spacer"),

    "Servo cables: thin cable only, the connector comes off. Channel along the femur's +Z face from the gap between the servos to the hip",
    ("cable_w", "4.2 mm", OLD1, "servoCableWidth, flat 3-wire cable"),
    ("cable_t", "1.8 mm", OLD1, "servoCableHeight"),
    ("chan_w", "=2 * cable_w + 0.4 mm", CALC, "2 cables side by side, femur + tibia"),
    ("sw_wire_w", "2.8 mm", TODO, "the foot switch's 2-wire lead, flat pair, across"),
    ("sw_wire_t", "1.4 mm", TODO, "that lead, thickness"),
    ("chan_depth", "=cable_t + sw_wire_t + 0.2 mm", CALC, "rail height above the face: the switch lead lies on the 2 servo cables"),
    ("chan_x0", "-4 mm", DESIGN, "channel start at the hip: from -9 its first bridge, deeper for the switch lead, hit the coxa with the femur at -45..-75 deg"),
    ("chan_x1", "30 mm", DESIGN, "channel end, in the gap between the servos"),
    ("chan_finger_len", "8 mm", DESIGN, "cable bridge along X (4 x 1.2 fingers broke off, print 1)"),
    ("chan_finger_t", "2.4 mm", DESIGN, "cable bridge thickness, across the whole channel; prints as a short bridge"),
    ("sw_tube_z", "0 mm", DESIGN, "upright tube for the foot switch's lead on the femur's knee end, past the knee axis: its centre height. The lead comes up from the tibia's fork through it and back along the +Z edge to the channel (it replaces a channel along the whole -Z face)"),
    ("sw_tube_len", "8 mm", DESIGN, "the tube's length, a third of the old channel"),
    ("sw_tube_bore", "3.6 mm", DESIGN, "its bore: the 2-wire lead, 2.8 x 1.4"),
    ("sw_tube_wall", "2.4 mm", DESIGN, "its wall (1.4 looked thin)"),
    ("sw_tube_blend", "3 mm", DESIGN, "concave fillet where the tube meets the femur's end face, both sides"),

    "Tibia: 2 flat halves split on the leg plane and screwed together; the switch sits inside the tip and a presser trapped between the halves pushes it (tibia frame: origin on the knee axis, -Z down the tibia, horn on +Y)",
    ("tib_arm_w", "18 mm", DESIGN, "arm width in X, around the bearing seat"),
    ("tib_a_slope_z", "=-horn_reach - 0.5 mm", CALC, "horn half: the outer face runs straight down to here, past the horn pocket, then slopes to the tip"),
    ("tib_b_slope_z", "=tib_a_slope_z", CALC, "bearing half: the same as the horn half, a mirror image"),
    ("tib_a_inner_z", "-22 mm", DESIGN, "inside the fork the face drops to here, then slopes to the centre"),
    ("tib_a_apex_z", "-32 mm", DESIGN, "where the inner slope reaches the leg plane"),
    ("tib_b_inner_z", "=tib_a_inner_z", CALC, "bearing half: the same as the horn half"),
    ("tib_b_apex_z", "=tib_a_apex_z", CALC, "bearing half: the same as the horn half"),
    ("tib_gusset", "3 mm", DESIGN, "gussets where arm, wall and shank meet"),
    ("tib_half_t", "6 mm", DESIGN, "each half of the shank"),
    ("tib_shank_end_z", "=-104.65 mm + tip_old_dz", OLD1, "lower end of the shank: where the old tibia's halves end (old_tip)"),
    ("tib_horn_low_y", "=joint_horn_y - horn_hub_h", CALC, "inner face at the horn: the horn sits fully in a pocket"),
    ("tib_horn_roof", "2 mm", DESIGN, "behind the horn pocket"),
    ("tib_bear_y1", "=fem_floor_y0 - 0.5 mm", CALC, "inner face of the bearing half, clear of the femur's floor"),
    ("tib_bear_seat_end", "=fem_floor_y0 - fem_boss_h - bearing_t", CALC, "bottom of the bearing seat"),
    ("tib_bear_roof", "3 mm", DESIGN, "behind the bearing seat"),
    ("tib_screw_d", "=mg90_screw_pilot_d", CALC, "pilot for the stock MG90S screws joining the halves"),
    ("tib_screw_clear", "=screw_clear_d", CALC, "clearance for those screws"),
    ("tib_screw_head_d", "=screw_head_d", CALC, "recess for the stock screw head"),
    ("tib_screw_head_y", "4 mm", DESIGN, "the head seats this far from the split face: the 9 mm stock screw then has about 3.5 mm of thread in the far half"),
    ("align_pin_d", "2 mm", OLD1, "alignment pins on one half's split face (tibia, coxa arm), as in the old tibia"),
    ("align_pin_h", "1 mm", OLD1, "those pins' height"),
    ("align_pin_ch", "0.2 mm", DESIGN, "chamfer on the pin's tip"),
    ("align_fit", "0.15 mm", DESIGN, "pin to hole a side: PLA prints the pin fat and the hole small (servo pocket fit 0.15 a side)"),
    ("align_hole_d", "=align_pin_d + 2 * align_fit", CALC, "holes for them in the other half (the old tibia's 2.1 was too tight to trust)"),
    ("align_hole_h", "=align_pin_h + 0.3 mm", CALC, "those holes' depth: the pin never bottoms out, so the halves close fully"),
    ("align_hole_ch", "0.4 mm", OLD1, "lead-in at the hole's mouth"),
    ("tib_pin_a_x", "-4.9 mm", DESIGN, "tibia alignment pins, on the bearing half, diagonal to the screws, centred in the shank wall beside the switch"),
    ("tib_pin_a_z", "-43.5 mm", DESIGN, "those pins"),
    ("tib_pin_b_x", "5.15 mm", DESIGN, "those pins"),
    ("tib_pin_b_z", "-33.5 mm", DESIGN, "those pins"),
    ("tib_screw_a_x", "4.2 mm", DESIGN, "lower screw, just above the switch, on +x (2 screws on a diagonal)"),
    ("tib_screw_a_z", "-37.5 mm", DESIGN, "those screws; clear of the old tip's lead slots below"),
    ("tib_screw_b_x", "4.4 mm", DESIGN, "upper screw, on -x"),
    ("tib_screw_b_z", "-34 mm", DESIGN, "those screws"),
    ("tib_tip_end_r", "6 mm", DESIGN, "the shank's round end; the triangle narrows to this circle at the tip"),
    ("tip_foot_r", "=tib_tip_end_r", CALC, "the presser's foot: the old TibiaTip's, made as big as the shank's end (was 4)"),
    ("tib_taper_x", "=tib_arm_w / 2 - tib_tip_end_r", CALC, "how much each side of the triangle narrows from knee to tip"),
    ("tib_tip_cone_z", "-38 mm", DESIGN, "the tip is round from its end up to here, widening"),
    ("tib_tip_cone_r", "13 mm", DESIGN, "the round tip's radius at that height"),
    ("tip_foot_z", "=-tibia_len + tip_foot_r", CALC, "centre of the round end"),
    ("old_tibia_len", "110 mm", OLD1, "the old (gear) hexapod's tibia: the frame of old_tip/*.brep, the switch and presser copied as they are"),
    ("tip_fit", "0.2 mm", DESIGN, "the old tip's pockets grown this much a side across (X, Y; heights kept): as copied the switch had 0.01 a side, the presser and pin 0.1, and the print didn't fit"),
    ("tip_presser_grow_x", "0.25 mm", DESIGN, "the presser grown this much a side in X (heights kept): it had 0.3-0.4 a side in its pockets and wobbled in the assembled tibia, 0.15 left"),
    ("tip_presser_grow_y", "0.15 mm", DESIGN, "the same in Y (it had 0.3 there), 0.15 left"),
    ("tip_foot_drop", "0.5 mm", DESIGN, "the presser's foot sits this much lower on a longer stem than the old TibiaTip's: its top clears the shank's end by 0.85, not 0.35 (print 2 stuck)"),
    ("tip_old_dz", "=106 mm + tip_foot_drop + tip_foot_r - tibia_len", CALC, "old_tip goes up this far: the foot's top (old z -105, lowered by tip_foot_drop) 1 mm above its centre, its bottom at the tibia's end"),
    ("tip_stop_z", "=-97.6 mm + tip_old_dz", OLD1, "top of the presser's pocket (tibiaCutForTip): the presser stops under it"),
    ("switch_body_z", "=-97.1 mm + tip_old_dz", OLD1, "underside of the switch's body, on its ledges (tibiaCutForButton)"),
    ("switch_top_z", "=switch_body_z + sw_body_h", CALC, "top of the switch"),
    ("tip_wire_z0", "=-90 mm + tip_old_dz", OLD1, "top of the old lead slots: the wire hollow starts here"),
    ("wire_void_x", "4.8 mm", DESIGN, "hollow from the switch's lead slots up to the fork for its 2 wires, in the middle of the split, X size"),
    ("wire_void_y", "5.6 mm", DESIGN, "that hollow, Y size: both leads, 4.5 apart"),
    ("wire_exit_z", "-29 mm", DESIGN, "the hollow runs up to here and opens into the fork"),

    "Coxa link, after the femur: a plate holding the coxa servo, shaft up; its horn fixes to the body's top plate and a shaft under its floor runs in the bearing in the bottom plate. At the outer end a fork around the femur: horn arm on +Y, bearing arm on -Y (leg frame: origin on the coxa axis at femur axis height, +X out along the link, Z up)",
    ("coxa_floor_z0", "=body_bottom_z + body_plate_t + fem_boss_h", CALC, "underside of the floor, above the spacer and the bottom plate"),
    ("coxa_floor_t", "3 mm", DESIGN, "floor under the servo"),
    ("coxa_seat_z", "=coxa_floor_z0 + coxa_floor_t + joint_gap + mg90_below_tab", CALC, "servo tab underside"),
    ("coxa_horn_z", "=coxa_seat_z + horn_top", CALC, "outer face of the coxa horn, in the body's top plate"),
    ("coxa_plate_floor", "4.7 mm", BIPED, "plate under the coxa servo's tabs (the femur's is thicker now; the coxa's FEA is fine)"),
    ("coxa_plate_z0", "=coxa_seat_z - coxa_plate_floor", CALC, "underside of the plate"),
    ("coxa_plate_z1", "=coxa_seat_z + fem_rim_h", CALC, "top of the rim"),
    ("coxa_half_w", "=fem_half_h", CALC, "half the plate width in Y"),
    ("coxa_plate_x0", "=-mg90_tab_near - mg90_fit - fem_wall_t", CALC, "plate end behind the axis"),
    ("coxa_cross_x0", "38 mm", DESIGN, "wall joining the plate and the fork: 6 mm, FEA found the coxa the softest part"),
    ("coxa_cross_x1", "44 mm", DESIGN, "stays 16 mm from the femur axis: the femur's tail clears it"),
    ("coxa_arm_w", "18 mm", DESIGN, "fork arm width in Z"),
    ("coxa_arm_t", "4 mm", DESIGN, "fork arm thin part"),
    ("coxa_arm_a_y0", "=joint_horn_y - horn_hub_h", CALC, "horn arm, inner face at the horn"),
    ("coxa_arm_a_y1", "=joint_horn_y + tib_horn_roof", CALC, "horn arm, outer face"),
    ("coxa_arm_b_y1", "=tib_bear_y1", CALC, "bearing arm, inner face"),
    ("coxa_arm_b_y0", "=tib_bear_seat_end - tib_bear_roof", CALC, "bearing arm, outer face"),
    ("coxa_shaft_d", "=bearing_id", CALC, "shaft under the floor"),
    ("coxa_chan_w", "=2 * cable_w + 0.4 mm", CALC, "channel on the plate's +Y face: 3 cables, 2 + 1"),
    ("coxa_chan_depth", "=2 * cable_t + 0.2 mm", CALC, "rail height"),
    ("coxa_servo_wall", "3 mm", DESIGN, "wall around the servo and its tabs, the slicer's infill fills the rest: 2.4 left the landing FEA at 17.7 MPa, 3 at 15.8"),
    ("coxa_side_y", "=mg90_body_w / 2 + mg90_fit_w + coxa_servo_wall", CALC, "the outline's half width around the servo"),
    ("coxa_tail_x", "=mg90_tab_near + mg90_fit + coxa_servo_wall", CALC, "the outline's rear end, behind the near tab"),
    ("coxa_rear_corner_r", "6 mm", DESIGN, "the rear corners, round the tab's corners: 6 (was 4) clears the middle legs' PCA9685 pins, which reach up to coxa height with the board turned over"),
    ("coxa_straight_x", "4 mm", DESIGN, "the outline stays a servo wall wide up to here, then widens: the far tab screw needs the material (FEA)"),
    ("coxa_waist_x", "36 mm", DESIGN, "the horn side of the outline curves out to full fork width by here"),
    ("coxa_waist_r", "40 mm", DESIGN, "that curve's radius"),
    ("coxa_fork_r", "6 mm", DESIGN, "inside corners of the fork"),
    ("coxa_arch_x0", "14 mm", DESIGN, "underside: end of the floor, seen from the side"),
    ("coxa_arch_x1", "26 mm", DESIGN, "underside: the arch reaches the plate here"),
    ("coxa_top_x", "28 mm", DESIGN, "top: rises from the rim to the fork from here"),
    ("coxa_hole_x0", "26 mm", DESIGN, "cable hole through the coxa, top to bottom, on the +Y side past the servo's tab pocket (22): the femur's 2 servo cables and the foot switch lead go down it to where the coxa servo's cable comes out under the coxa, and are zip-tied together there. Replaces the groove: its 1.4 mm bridges fell off in print 1"),
    ("coxa_hole_x1", "36 mm", DESIGN, "the hole's outer end, short of the femur's horn pocket (39.4)"),
    ("coxa_hole_y", "9.8 mm", DESIGN, "the hole's centre line"),
    ("coxa_hole_w", "6 mm", DESIGN, "the hole's width across: the 3 cables stacked are 5 mm (a 3-pin connector, 2.6 x 7.6, passes too)"),
    ("coxa_edge_r", "1.2 mm", DESIGN, "rounding on the outer edges"),
    ("coxa_skirt_x", "11.5 mm", DESIGN, "under the plate the rear is narrower: it swings next to the PCA9685's pins"),
    ("coxa_cut_y", "-9.2 mm", DESIGN, "the bearing side comes off on this plane (the servo wall's -Y face, 9.45, and the skirt's, 9.2, end on it)"),
    ("coxa_cut_x", "26 mm", DESIGN, "the bearing side is a square plate from here out, past the joint screws (the outline is squared out to the arm's face from here); in front of it the flare stays on the coxa: without it the landing FEA went over (18.2 MPa against 14.9)"),
    ("coxa_joint_y", "=coxa_cut_y + 7 mm", CALC, "how deep the screws go into the coxa behind the joint plane: pilot for 2 mm of plate under the head plus the screw's 4..7 mm"),
    ("coxa_screw_head_y", "2 mm", DESIGN, "the bearing side's screw heads seat this far from the joint face: at the tibia's 4 the stock screws were too short to pull it tight (print 1)"),
    ("coxa_flare_x", "21 mm", DESIGN, "on the bearing side the outline's curve reaches the side's flat outer face by here (corner, its round runs to x 26): with the waist at 36, as on the horn side, the side's square plate stood out 5 mm past the coxa at coxa_cut_x (print 1)"),
    ("coxa_flare_z0", "-17.5 mm", DESIGN, "under this the flare stays inside y -15 (the old outline): lower down, the arch hit the bottom plate with the middle and rear coxas at -40 deg"),
    ("coxa_pin_a_x", "31.5 mm", DESIGN, "alignment pins on the coxa's joint face, into the bearing arm, diagonal to the screws"),
    ("coxa_pin_a_z", "-3.5 mm", DESIGN, "those pins"),
    ("coxa_pin_b_x", "39.5 mm", DESIGN, "those pins"),
    ("coxa_pin_b_z", "3.5 mm", DESIGN, "those pins"),
    ("coxa_screw_a_x", "31.5 mm", DESIGN, "arm screws, along Y through the arm into the coxa, on a diagonal like the tibia's"),
    ("coxa_screw_a_z", "3.5 mm", DESIGN, "those screws"),
    ("coxa_screw_b_x", "39.5 mm", DESIGN, "those screws"),
    ("coxa_screw_b_z", "-3.5 mm", DESIGN, "those screws"),

    "Body: 2 plates. The bottom one holds the 6 coxa bearings and stands the boards and cells; the top one holds the 6 coxa horns. Columns join them, screwed with the stock servo screws (body frame: origin on the centre at femur axis height, X right, Y forward)",
    ("body_cx", "=coxa_front_x + coxa_front_x / sqrt(coxa_front_x^2 + coxa_front_y^2) * plate_inflate", CALC, "hexagon corner, plate_outline() in params.py"),
    ("body_cy", "=coxa_front_y + coxa_front_y / sqrt(coxa_front_x^2 + coxa_front_y^2) * plate_inflate", CALC, "hexagon corner"),
    ("body_mx", "=coxa_mid_x + plate_inflate", CALC, "hexagon side corner"),
    ("body_plate_top", "=body_bottom_z + body_plate_t", CALC, "top of the bottom plate"),
    ("top_roof", "2 mm", DESIGN, "top plate over the horn"),
    ("top_plate_z1", "=coxa_horn_z + top_roof", CALC, "top of the top plate"),
    ("top_plate_t", "2.4 mm", DESIGN, "top plate skin, the ribs stiffen it"),
    ("top_plate_z0", "=top_plate_z1 - top_plate_t", CALC, "underside of the top plate away from the horns"),
    ("top_boss_d", "16 mm", DESIGN, "thick round part of the top plate around each horn hub"),
    ("top_boss_z0", "=coxa_horn_z - horn_hub_h", CALC, "underside of the top plate at a horn"),
    ("col_d", "9 mm", DESIGN, "columns between the plates"),
            ("col_front_x", "14 mm", DESIGN, "front and rear columns at +-x"),
    ("col_front_y", "62 mm", DESIGN, "front and rear columns at +-y"),
    ("col_side_x", "34.5 mm", DESIGN, "side columns at +-x, in the scallops between the legs"),
    ("col_side_y", "38 mm", DESIGN, "side columns at +-y"),
    ("col_screw_depth", "8 mm", DESIGN, "pilot hole for a stock screw, in each end of a column"),
    ("holder_y", "-8 mm", DESIGN, "battery holder centre along Y, in its cartridge: the opening under it stays clear of the ESP32's slots"),
    ("holder_screw_d", "=mg90_screw_pilot_d", CALC, "pilot in the cartridge floor for the stock screws through the holder's 2 holes"),
    ("body_edge_r", "1.5 mm", DESIGN, "rounding on the plates' outlines"),
    ("rib_t", "2.4 mm", DESIGN, "ribs on both plates"),
    ("body_skin_t", "2.4 mm", DESIGN, "battery cartridge floor"),
    ("top_skin_t", "0.8 mm", DESIGN, "top plate skin between ribs: only closes the plate, the ribs carry the load; 0.8 doesn't dent under a finger"),
    ("bot_skin_t", "1.2 mm", DESIGN, "bottom plate skin between ribs, on the bed side: tougher against edges and steps"),
    ("grid_pitch", "22 mm", DESIGN, "diagonal rib grid in the plates' pockets"),
    ("grid_rib_t", "1.6 mm", DESIGN, "grid ribs"),
    ("grid_rib_h", "1.6 mm", DESIGN, "grid ribs above the skin (the top plate's pockets are only this deep anyway)"),
    ("keep_margin", "1 mm", DESIGN, "full thickness this far around every rib, boss, ring and rail"),
    ("col_bore_d", "5.8 mm", DESIGN, "columns are tubes between their screw pilots"),
    ("bear_boss_d", "18 mm", DESIGN, "full-thickness round around each coxa bearing seat"),
    ("holder_boss_d", "7 mm", DESIGN, "round under the cartridge floor at the holder's screws, down to the plate's underside"),
    ("col_ring_d", "11.4 mm", DESIGN, "ring on a plate that locates a column, and full thickness under it"),
    ("col_ring_h", "1.2 mm", DESIGN, "that ring's height"),
    ("col_head_d", "=screw_head_d", CALC, "recess for a stock screw head under the bottom plate"),
    ("col_head_h", "2 mm", DESIGN, "that recess: leaves about 7 mm of thread in the column"),
    ("ina_x", "0 mm", DESIGN, "INA219 standing across the robot at the front, between the ESP32 and the front columns: centre in X"),
    ("ina_y", "50 mm", DESIGN, "its PCB along Y, components forward (at the rear it had no room: battery opening, latch post)"),
    ("lead_small", "0.2 mm", CAL, "chamfer on the mouths of small holes (screw pilots and clearance holes, up to lead_small_d)"),
    ("lead_big", "0.4 mm", CAL, "chamfer on every other convex edge: pocket and seat mouths, slots, outlines"),
    ("lead_small_d", "3.2 mm", DESIGN, "holes up to this diameter get the small chamfer"),
    ("screw_clear_d", "2.5 mm", CAL, "clearance hole for the stock screws: 2.2 held the screw in the fit test"),
    ("screw_head_d", "5.6 mm", CAL, "counterbore for a stock screw head: Ø5, with room"),
    ("screw_head_h", "1.4 mm", CAL, "counterbore depth: the head is 1.2 and sits just under the surface"),
    ("col_top_floor", "1 mm", DESIGN, "the columns stop this far under the top plate: the plate is thicker there, under the counterbored head"),
    ("cable_slot_x", "10.5 mm", DESIGN, "cable slot through the bottom plate under each PCA9685 pin group, out from the board's pin face: the connector (7.6 across its 3 rows) and the cables leaving it"),
    ("cable_slot_y", "9.6 mm", DESIGN, "that slot along the board: the pin group (7.62) + 1 mm each side; more than 3 servo cables and the switch lead side by side (3 x 1.8 + 1.4)"),
    ("cable_slot_web", "0 mm", DESIGN, "plate left between a cable slot and a column's ring"),
    ("col_fit", "0.3 mm", DESIGN, "ring and rib clearance around a column"),
    ("col_end_ch", "0.5 mm", DESIGN, "chamfer on a column's ends"),
    ("rib_h", "6 mm", DESIGN, "rib height, top plate"),
    ("rib_bot_h", "5 mm", DESIGN, "rib height, bottom plate"),
    ("rib_inset", "0.94", DESIGN, "the border rib's inner outline, scaled from the plate's"),
    ("rib_open_r", "23 mm", DESIGN, "no rib within this radius of a coxa axis"),
    ("rib_open_deg", "82", DESIGN, "nor within this angle of a leg's neutral direction: its swing +-40, plus its width"),
    ("rail_h", "6 mm", DESIGN, "slots the standing boards slide down into; deep enough to hold them without anything from the top"),
    ("rail_t", "2.4 mm", DESIGN, "rail thickness"),
    ("esp_rail_h", "3.5 mm", DESIGN, "the ESP32's slots: its WROOM module starts 3.9 mm above the board edge"),
    ("rail_len", "4.4 mm", DESIGN, "rail length, fits between the PCA9685 pin groups"),
    ("rail_fit", "0.05 mm", CAL, "clearance per side in the board slots: the fit test printed them true to size, 0.3 was loose"),
    ("pwr_x", "0 mm", DESIGN, "power switch in the top plate, on the centre line"),
    ("pwr_y", "-40 mm", DESIGN, "at the rear, over the cells (its pins end 1.5 mm above them), in front of the battery's XT30"),
    ("pwr_cut_w", "11.8 mm", CAL, "switch cutout, short side: the switch is ~11.6 there and doesn't clip on that side"),
    ("pwr_cut_len", "19.15 mm", CAL, "switch cutout: the fit test printed 0.24 small; prints ~18.9 against 19.5 clips and an 18.18 body"),
    ("pwr_frame", "2 mm", DESIGN, "the plate thinned to pwr_panel_t this far around the cutout, from below"),
    ("cart_wall_t", "1.6 mm", DESIGN, "battery cartridge (in from below): walls around the holder"),
    ("cart_side_h", "7 mm", DESIGN, "side walls above the floor, under the Mini360s"),
    ("cart_end_h", "3 mm", DESIGN, "end walls, under the holder's solder tabs"),
    ("cart_fit", "0.3 mm", DESIGN, "the bottom plate's opening around the cartridge, per side"),
    ("cart_band_w", "4 mm", DESIGN, "full-thickness band of the bottom plate around the opening"),
    ("cart_lug_len", "4.5 mm", DESIGN, "lugs under the cartridge's 2 ends, flush with the plate's underside: the latches turn over them"),
    ("cart_lug_w", "10 mm", DESIGN, "lug width"),
    ("xt30_x", "12 mm", DESIGN, "XT30 pair lying on the right cell, strapped to the holder: plugged by hand on a lead, nothing printed takes the force; clear of the switch's pins"),
    ("latch_len", "6.5 mm", DESIGN, "turn latch: pivot (a stock screw) to the tip's centre"),
    ("latch_w", "6 mm", DESIGN, "turn latch width"),
    ("latch_t", "2 mm", DESIGN, "turn latch thickness, under the plate"),
    ("latch_overlap", "2.5 mm", DESIGN, "a closed latch's tip over the cartridge lug"),
    ("tof_base_t", "2.5 mm", DESIGN, "ToF mount: a separate print standing on the top plate's front, its base"),
    ("tof_back_t", "8 mm", DESIGN, "solid back behind each board, room for the screw pilots"),
    ("tof_back_margin", "1.5 mm", DESIGN, "that back past the board and its ears"),
    ("tof_mount_screw_x", "7 mm", DESIGN, "2 stock screws up through the top plate into the mount, at +-x"),
    ("tof_mount_screw_y", "66 mm", DESIGN, "between the front rib (y 62) and the border rib, room for the heads under the plate"),
    ("tof_mount_boss_d", "7 mm", DESIGN, "round under each of those screws, full height"),
    ("tof_wire_d", "4.5 mm", DESIGN, "hole through the mount and the plate for the 2 boards' wires, x = 0"),
    ("tof_wire_y", "66.5 mm", DESIGN, "that hole"),
    ("tof_screw_depth", "7 mm", DESIGN, "pilot for a stock screw through an ear: 9 long, 1.6 board"),
    ("tof_mount_depth", "7 mm", DESIGN, "pilot in the mount for a screw up through the 2.4 plate"),
    ("tof_fit", "0.3 mm", DESIGN, "clearance around a ToF board"),

    "Catalog: how the dummies are spread out in the document (not robot sizes)",
    ("catalog_pitch", "90 mm", "layout", "between dummies along X"),
    ("catalog_row2_y", "-150 mm", "layout", "printed parts row"),
    ("catalog_part_pitch", "130 mm", "layout", "between printed parts along X"),
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
# Parts that use a renamed alias break until build_dummies.py rebuilds them: save anyway
stale = common["broken"](doc)
doc.save()
if stale:
    print("run build_dummies.py next, stale: %s" % stale)
rows = [i for i in TABLE if not isinstance(i, str)]
print("%d params, %d TODO measure, saved %s" % (len(rows), sum(i[2] == TODO for i in rows), doc.FileName))
