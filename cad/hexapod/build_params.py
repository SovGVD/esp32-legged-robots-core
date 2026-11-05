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
    ("pca_x", "26.6 mm", DESIGN, "standing board at +-x; simple_parts.py has 23. Pins outward with the bends down, Mini360 inward; the left board is the right one turned 180 deg"),
    ("pca_z", "=body_plate_top + pca_w / 2 + 0.2 mm", CALC, "standing board centre, on the bottom plate"),
    ("pca_y", "-6.1 mm", DESIGN, "right board slid along Y, left one the opposite way: the middle coxa strap swings in the gap in its pins"),
    ("logic_mini_y", "-44 mm", SP, "7th Mini360 (logic) at the rear"),
    ("esp_y", "42.5 mm", DESIGN, "ESP32 across the front, clear of the front links' tails; simple_parts.py has 45"),
    ("esp_z", "=body_plate_top + esp_w / 2 + 0.2 mm", CALC, "ESP32 centre, on the bottom plate"),
    ("gy91_z", "=top_plate_z0 - 1.5 mm", CALC, "under the top plate, x = y = 0"),
    ("tof_x", "14 mm", FW, "TOF_SENSORS at +-x"),
    ("tof_y", "74 mm", FW, "TOF_SENSORS"),
    ("tof_z", "2 mm", FW, "TOF_SENSORS"),
    ("tof_yaw", "20 deg", FW, "0.35 rad, turned outward"),
    ("tof_tilt_left", "35 deg", FW, "0.61 rad down, terrain"),
    ("tof_tilt_right", "22 deg", FW, "0.384 rad down, room scan"),

    "MG90S (part origin: shaft axis at the underside of the tabs; +Z to the horn; body runs to -Y)",
    ("mg90_body_w", "12 mm", OLD2, "servoWidth; servo drawing says 13"),
    ("mg90_body_len", "23 mm", OLD2, "servoLength; small_dog pocket says 22.6"),
    ("mg90_fit", "0.2 mm", OLD2, "servoSpace, pocket clearance per side"),
    ("mg90_tab_total", "32.3 mm", OLD2, "servoFullLength, tab tip to tab tip; servo drawing says 34.5"),
    ("mg90_shaft_end", "6 mm", OLD2, "servoToShaft, shaft axis from the near end of the body"),
    ("mg90_screw_spacing", "27.86 mm", OLD2, "servoBetweenMountHoles; servo drawing says 28"),
    ("mg90_screw_pilot_d", "1.56 mm", OLD2, "servoScrew r 0.78"),
    ("mg90_below_tab", "17 mm", OLD2, "servoBodyToMount, body bottom to tab underside; with the tab = 19.5, same as the servo drawing"),
    ("mg90_tab_t", "2.5 mm", OLD2, "servoMount"),
    ("mg90_body_top", "6 mm", TODO, "tab underside to top of the rectangular body, under the turret"),
    ("mg90_turret_d", "11.8 mm", TODO, "round gear turret around the shaft"),
    ("mg90_turret_top", "=mg90_height - mg90_below_tab", CALC, "tab underside to the top of the servo"),
    ("mg90_height", "28.1 mm", OLD2, "servoHeight, body bottom to the top without the shaft; servo drawing says 27"),
    ("mg90_shaft_d", "5 mm", OLD2, "probablyServoShaft"),
    ("mg90_shaft_top", "15 mm", OLD2, "refTop sketch, tab underside to the shaft tip"),
    ("horn_reach", "18 mm", TODO, "single-arm horn, shaft to arm tip, scaled from the horn set photo"),
    ("horn_tip_w", "5.8 mm", OLD1, "arm width at the tip (servo_horn_cut outline)"),
    ("horn_hub_d", "10 mm", TODO, "single-arm horn hub, scaled from the horn set photo"),
    ("horn_hub_h", "4 mm", OLD1, "horn recess depth in ServoGear_MG90S_clone"),
    ("horn_screw_d", "2 mm", TODO, "clearance hole for the screw through a horn arm hole"),
    ("horn_screw_r1", "9 mm", TODO, "horn arm hole, from the shaft"),
    ("horn_screw_r2", "15 mm", TODO, "second horn arm hole"),
    ("horn_arm_t", "1.6 mm", TODO, "arm thickness"),
    ("horn_gap", "0.9 mm", OLD2, "servo top to the horn seat: 34 with a 5 mm gear - 28.1 - 5"),
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
    ("bearing_seat_d", "12.2 mm", OLD2, "onBearingHolder r 6.1, pocket for the outer ring"),

    "18650 pair (part origin: centre of the pair; cells along Y)",
    ("cell_d", "18 mm", STD, "18650"),
    ("cell_len", "65 mm", STD, "18650"),

    "Battery holder, flat: 2 cells, same height as a cell (part origin: its centre)",
    ("holder_len", "76.8 mm", PHOTO, "seller picture"),
    ("holder_w", "39.5 mm", PHOTO, "seller picture"),
    ("holder_end", "=(holder_len - cell_len) / 2", CALC, "end block past the cells"),
    ("holder_side", "=holder_w / 2 - cell_x - cell_d / 2", CALC, "side wall outside a cell"),
    ("holder_hole_span", "54 mm", TODO, "2 mounting holes on the centre line, scaled from the seller picture"),
    ("holder_tab", "3.7 mm", TODO, "solder tabs past each end, scaled from the seller picture"),

    "PCA9685 node, HW-052 with bent pins on top and 3 Mini360 glued on the back; I2C and power wires soldered, not modelled (part origin: PCB centre, lying flat; long side along X; +Z = pin side; pin bends at -Y)",
    ("pca_len", "62.41 mm", CAL, "PCB"),
    ("pca_w", "25.22 mm", CAL, "PCB"),
    ("pca_t", "1.66 mm", OLD2, "pcaPcb"),
    ("pca_hole_spacing", "55.85 mm", OLD1, "servoControllerBetweenHoles, along the board"),
    ("pca_hole_across", "19.05 mm", TODO, "standard board; ~18.8 in the caliper photo"),
    ("pca_hole_d", "2.4 mm", OLD1, "servoControllerHole"),
    ("pca_grp_w", "7.62 mm", STD, "pin group, 3 channels x 2.54"),
    ("pca_grp_depth", "15.5 mm", PHOTO, "bent pins lie across the board: bend to tip"),
    ("pca_grp_h", "10.9 mm", TODO, "PCB to the top of the bent pins; hexapodv2 cut, plugs may add more"),
    ("pca_grp_edge", "1 mm", PHOTO, "bend from the long edge"),
    ("pca_grp_x1", "-22.5 mm", PHOTO, "1st pin group centre from the PCB centre"),
    ("pca_grp_x2", "-9.5 mm", PHOTO, "2nd pin group"),
    ("pca_grp_x3", "21.7 mm", PHOTO, "3rd pin group; the space around +7.4 has no pins"),
    ("mini_pitch", "12 mm", PHOTO, "3 Mini360 side by side on the back of the PCB, long side across it"),
    ("mini_node_y", "3.5 mm", PHOTO, "row centre, toward the long edge away from the pin bends"),
    ("mini_tape_t", "1 mm", TODO, "foam tape under the Mini360"),

    "Mini360 buck (part origin: PCB centre, lying flat; +Z = component side)",
    ("mini_len", "17.4 mm", CAL, "PCB, about"),
    ("mini_w", "11 mm", CAL, "PCB, about"),
    ("mini_t", "1 mm", LOOSE, "PCB"),
    ("mini_comp_len", "13 mm", LOOSE, "component envelope"),
    ("mini_comp_w", "9 mm", LOOSE, "component envelope"),
    ("mini_comp_h", "3 mm", TODO, "the inductor, tallest part above the PCB"),

    "ESP32 devkit, 38 pins removed, wires soldered, USB-C (part origin: PCB centre, lying flat; long side along X; +Z = module side; USB at -X)",
    ("esp_len", "51.57 mm", CAL, "PCB, the USB-C socket ends flush with it"),
    ("esp_w", "25.75 mm", CAL, "PCB"),
    ("esp_t", "1.6 mm", TODO, "PCB"),
    ("esp_mod_len", "25.5 mm", TODO, "WROOM module"),
    ("esp_mod_w", "18 mm", TODO, "WROOM module"),
    ("esp_mod_h", "3.2 mm", TODO, "WROOM module above the PCB"),
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
    ("tof_len", "17 mm", TODO, "rectangular part, along the pin row"),
    ("tof_wid", "10.6 mm", TODO, "rectangular part, across the pin row"),
    ("tof_t", "1.6 mm", TODO, "PCB"),
    ("tof_hole_span", "20.3 mm", TODO, "mounting hole centres"),
    ("tof_hole_d", "3.2 mm", TODO, "mounting hole"),
    ("tof_ear_d", "5.6 mm", TODO, "round ear around the hole"),
    ("tof_sensor_off", "2 mm", TODO, "sensor from the PCB centre, away from the pins"),
    ("tof_pin_row_len", "15.24 mm", STD, "6 pins x 2.54"),
    ("tof_pin_row_w", "1.7 mm", TODO, "pad row"),
    ("tof_pin_row_off", "3.6 mm", TODO, "pad row from the PCB centre"),
    ("tof_chip_w", "6.4 mm", TODO, "sensor package, along the pin row"),
    ("tof_chip_h", "3 mm", TODO, "sensor package"),
    ("tof_chip_t", "1.5 mm", TODO, "sensor package"),

    "Foot switch, 6x6 tact switch with side leads (part origin: centre of the body's underside; +Z = button; leads on +X)",
    ("sw_w", "6 mm", DRAWING, "body, square"),
    ("sw_body_h", "3.45 mm", DRAWING, "body"),
    ("sw_total_h", "4.7 mm", TODO, "H is open on the drawing; hexapod.FCStd tibiaButtonCut 4.7"),
    ("sw_btn_d", "3.5 mm", DRAWING, "button"),
    ("sw_travel", "0.4 mm", OLD1, "clickButtonMoveLength"),
    ("sw_lead_span", "10.3 mm", DRAWING, "body edge to the lead tips, overall"),
    ("sw_lead_pitch", "4.5 mm", DRAWING, "between the 2 leads"),
    ("sw_lead_w", "0.7 mm", DRAWING, "lead"),
    ("sw_lead_t", "0.3 mm", DRAWING, "lead"),
    ("sw_lead_z", "2.15 mm", DRAWING, "underside to the top of the lead"),

    "Leg joints: each servo is part of the link that holds it; its horn is fixed to the next part in, and opposite the horn a shaft runs in a bearing there",
    ("joint_gap", "0.5 mm", DESIGN, "servo bottom to the floor of its box"),
    ("joint_horn_y", "=mg90_stack / 2", CALC, "outer face of the horn: the servo stack is centred on the leg plane"),
    ("joint_seat_y", "=-mg90_stack / 2 + mg90_below_tab", CALC, "tab underside, the servo plate's face"),
    ("print_chamfer", "2 mm", DESIGN, "internal chamfers, for PLA strength"),
    ("pla_density", "1.24", STD, "g/cm3, solid, for the weight check"),

    "Femur, after biped leg_bottom_v2: a thick plate with the 2 servos through it, tabs in pockets, and a strap at each end carrying the shaft (femur frame: origin on the femur axis, +X to the knee, Z up with the femur level, horns on +Y)",
    ("fem_plate_floor", "4.7 mm", BIPED, "plate under the tab pockets"),
    ("fem_rim_h", "8 mm", BIPED, "tab pocket depth, the rim around the tabs"),
    ("fem_plate_y0", "=joint_seat_y - fem_plate_floor", CALC, "underside of the plate"),
    ("fem_plate_y1", "=joint_seat_y + fem_rim_h", CALC, "top of the rim"),
    ("fem_wall_t", "3 mm", DESIGN, "strap and rim walls"),
    ("fem_floor_t", "3 mm", DESIGN, "strap floor under the servo, carries the shaft"),
    ("fem_floor_y1", "=-mg90_stack / 2 - joint_gap", CALC, "inner face of the floor"),
    ("fem_floor_y0", "=fem_floor_y1 - fem_floor_t", CALC, "outer face of the floor"),
    ("fem_half_h", "=mg90_body_w / 2 + mg90_fit + fem_wall_t", CALC, "half the height in Z"),
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
    ("fem_shaft_len", "3 mm", BIPED, "shaft"),
    ("fem_edge_ch", "1 mm", BIPED, "chamfer on the edges"),
    ("fem_fit_ch", "0.5 mm", DESIGN, "lead-in on the servo openings: tab pocket mouth and servo hole"),
    ("fem_shaft_ch", "0.4 mm", BIPED, "chamfer on the shaft tip"),
    ("fem_boss_ch", "0.2 mm", BIPED, "chamfer on the spacer"),

    "Servo cables: thin cable only, the connector comes off. Channel along the femur's +Z face from the gap between the servos to the hip",
    ("cable_w", "4.2 mm", OLD1, "servoCableWidth, flat 3-wire cable"),
    ("cable_t", "1.8 mm", OLD1, "servoCableHeight"),
    ("chan_w", "=2 * cable_w + 0.4 mm", CALC, "2 cables side by side, femur + tibia"),
    ("chan_depth", "=cable_t + 0.2 mm", CALC, "rail height above the face"),
    ("chan_x0", "-9 mm", DESIGN, "channel start at the hip"),
    ("chan_x1", "30 mm", DESIGN, "channel end, in the gap between the servos"),
    ("chan_finger_len", "4 mm", DESIGN, "finger along X"),
    ("chan_finger_t", "1.2 mm", DESIGN, "finger thickness; prints as a short bridge"),
    ("chan_finger_over", "3 mm", DESIGN, "finger reaches this far over the channel, leaving room to press the cable past it"),

    "Tibia: 2 flat halves split on the leg plane and screwed together; the switch sits inside the tip and a presser trapped between the halves pushes it (tibia frame: origin on the knee axis, -Z down the tibia, horn on +Y)",
    ("tib_arm_w", "18 mm", DESIGN, "arm width in X, around the bearing seat"),
    ("tib_arm_t", "4 mm", DESIGN, "thin part of the arms"),
    ("tib_cross_z", "-24 mm", DESIGN, "top of the wall joining an arm to the shank, clear of the femur"),
    ("tib_cross_t", "3 mm", DESIGN, "wall joining an arm to the shank"),
    ("tib_gusset", "3 mm", DESIGN, "gussets where arm, wall and shank meet"),
    ("tib_shank_w", "16 mm", DESIGN, "shank width in X"),
    ("tib_half_t", "6 mm", DESIGN, "each half of the shank"),
    ("tib_shank_end_z", "=tip_foot_z + 0.5 mm", CALC, "lower end of the shank, just above the half-sphere: room for the presser to rise"),
    ("tib_horn_low_y", "=joint_horn_y - horn_hub_h", CALC, "inner face at the horn: the horn sits fully in a pocket"),
    ("tib_horn_roof", "2 mm", DESIGN, "behind the horn pocket"),
    ("tib_bear_y1", "=fem_floor_y0 - 0.5 mm", CALC, "inner face of the bearing half, clear of the femur's floor"),
    ("tib_bear_seat_end", "=fem_floor_y0 - fem_boss_h - bearing_t", CALC, "bottom of the bearing seat"),
    ("tib_bear_roof", "3 mm", DESIGN, "behind the bearing seat"),
    ("tib_screw_d", "=mg90_screw_pilot_d", CALC, "pilot for the stock MG90S screws joining the halves"),
    ("tib_screw_clear", "2.2 mm", DESIGN, "clearance for those screws"),
    ("tib_screw_head_d", "5.4 mm", PHOTO, "recess for the stock screw head: Ø5 on the seller photo, 9 mm long overall"),
    ("tib_screw_head_h", "3 mm", DESIGN, "recess depth: the 9 mm stock screw then has about 4.5 mm of thread in the far half"),
    ("tib_screw_a_x", "-4.85 mm", DESIGN, "screw beside the switch, on the side away from the wires"),
    ("tib_screw_a_z", "-43.5 mm", DESIGN, "that screw"),
    ("tib_screw_b_x", "5 mm", DESIGN, "2 screws near the top of the shank, at +-x"),
    ("tib_screw_b_z", "-31.5 mm", DESIGN, "those screws"),
    ("tip_foot_r", "6 mm", DESIGN, "half-sphere foot; the triangle narrows to this circle at the tip. Sim foot_radius was 5"),
    ("tib_taper_x", "=tib_arm_w / 2 - tip_foot_r", CALC, "how much each side of the triangle narrows from knee to tip"),
    ("tip_foot_z", "=-tibia_len + tip_foot_r", CALC, "centre of the round end"),
    ("tip_stem_w", "5.98 mm", OLD1, "tibiaTipWidth, presser stem"),
    ("tip_hole_w", "6.2 mm", OLD1, "buttonWidth, stem hole and switch pocket"),
    ("tip_flange_x", "7.4 mm", DESIGN, "flange that traps the presser between the halves, full length in X"),
    ("tip_flange_h", "2 mm", DESIGN, "flange height"),
    ("tip_cavity_x", "7.8 mm", DESIGN, "cavity for the flange"),
    ("tip_floor_t", "3 mm", DESIGN, "shank under the flange"),
    ("tip_travel", "0.5 mm", DESIGN, "the presser can rise this far"),
    ("tip_gap", "0.1 mm", OLD1, "safePrinterDistance, flange to button with the foot hanging"),
    ("tip_cavity_z0", "=tib_shank_end_z + tip_floor_t", CALC, "cavity floor, the flange rests on it"),
    ("tip_flange_top", "=tip_cavity_z0 + tip_flange_h", CALC, "flange top, foot hanging"),
    ("tip_cavity_z1", "=tip_flange_top + tip_travel", CALC, "cavity top: stops the presser and takes the foot's load"),
    ("switch_btn_z", "=tip_flange_top + tip_gap", CALC, "button end, slightly inside the leg"),
    ("switch_body_z", "=switch_btn_z + sw_total_h - sw_body_h", CALC, "underside of the switch's body"),
    ("switch_top_z", "=switch_body_z + sw_body_h", CALC, "top of the switch, end of its pocket"),
    ("switch_btn_bore", "4 mm", DESIGN, "button hole through the ledge the switch sits on"),
    ("switch_slot_y", "2.8 mm", DESIGN, "slot for the switch leads, +-y"),
    ("wire_t", "2 mm", DESIGN, "groove on the split plane for the switch's 2 wires, each half 1 mm deep"),
    ("wire_w", "3 mm", DESIGN, "groove width"),
    ("wire_exit_z", "-36 mm", DESIGN, "where the wire groove leaves the shank"),

    "Coxa link, after the femur: a plate holding the coxa servo, shaft up; its horn fixes to the body's top plate and a shaft under its floor runs in the bearing in the bottom plate. At the outer end a fork around the femur: horn arm on +Y, bearing arm on -Y (leg frame: origin on the coxa axis at femur axis height, +X out along the link, Z up)",
    ("coxa_floor_z0", "=body_bottom_z + body_plate_t + fem_boss_h", CALC, "underside of the floor, above the spacer and the bottom plate"),
    ("coxa_floor_t", "3 mm", DESIGN, "floor under the servo"),
    ("coxa_seat_z", "=coxa_floor_z0 + coxa_floor_t + joint_gap + mg90_below_tab", CALC, "servo tab underside"),
    ("coxa_horn_z", "=coxa_seat_z + horn_top", CALC, "outer face of the coxa horn, in the body's top plate"),
    ("coxa_plate_z0", "=coxa_seat_z - fem_plate_floor", CALC, "underside of the plate"),
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
    ("coxa_chan_x0", "-9 mm", DESIGN, "channel end, inside the body"),

    "Body: 2 plates. The bottom one holds the 6 coxa bearings and stands the boards and cells; the top one holds the 6 coxa horns. Columns join them, screwed with the stock servo screws (body frame: origin on the centre at femur axis height, X right, Y forward)",
    ("body_cx", "=coxa_front_x + coxa_front_x / sqrt(coxa_front_x^2 + coxa_front_y^2) * plate_inflate", CALC, "hexagon corner, plate_outline() in params.py"),
    ("body_cy", "=coxa_front_y + coxa_front_y / sqrt(coxa_front_x^2 + coxa_front_y^2) * plate_inflate", CALC, "hexagon corner"),
    ("body_mx", "=coxa_mid_x + plate_inflate", CALC, "hexagon side corner"),
    ("body_plate_top", "=body_bottom_z + body_plate_t", CALC, "top of the bottom plate"),
    ("top_roof", "2 mm", DESIGN, "top plate over the horn"),
    ("top_plate_z1", "=coxa_horn_z + top_roof", CALC, "top of the top plate"),
    ("top_plate_t", "3 mm", DESIGN, "top plate away from the horns"),
    ("top_plate_z0", "=top_plate_z1 - top_plate_t", CALC, "underside of the top plate away from the horns"),
    ("top_boss_d", "16 mm", DESIGN, "thick round part of the top plate around each horn hub"),
    ("top_boss_z0", "=coxa_horn_z - horn_hub_h", CALC, "underside of the top plate at a horn"),
    ("col_d", "9 mm", DESIGN, "columns between the plates"),
    ("col_foot_d", "13 mm", DESIGN, "wider foot on the bottom plate"),
    ("col_foot_h", "3 mm", DESIGN, "column foot"),
    ("col_front_x", "14 mm", DESIGN, "front and rear columns at +-x"),
    ("col_front_y", "62 mm", DESIGN, "front and rear columns at +-y"),
    ("col_side_x", "46 mm", DESIGN, "side columns at +-x"),
    ("col_side_y", "30 mm", DESIGN, "side columns at +-y"),
    ("col_screw_depth", "7 mm", DESIGN, "pilot hole for the stock screw, from the top of a column"),
    ("holder_y", "-3.5 mm", DESIGN, "battery holder centre along Y: its solder tabs stay clear of the ESP32"),
    ("holder_screw_d", "=mg90_screw_pilot_d", CALC, "pilot in the plate for the stock screws through the holder's 2 holes"),
    ("rail_h", "4 mm", DESIGN, "rails holding the standing boards' lower edges"),
    ("rail_t", "2.4 mm", DESIGN, "rail thickness"),
    ("rail_len", "4.4 mm", DESIGN, "rail length, fits between the PCA9685 pin groups"),
    ("rail_fit", "0.2 mm", DESIGN, "clearance per side between a board and its rails"),

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
