"""Step 1: the Params spreadsheet, the one home of every dimension.

Re-run after editing the table: cells are rewritten in place, so expressions
that use them stay valid. Then checks the leg geometry against the firmware.

Columns: A alias, B value, C source, D note.
Source says how far to trust a value:
  config.h / params.py / model_data.h / simple_parts.py   the robot design, CAD must match
  hexapodv2 / hexapod.FCStd   taken from the old models in Dropbox/robots/hexapod
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
    ("body_plate_t", "3 mm", SIM, "bottom plate thickness, concept"),

    "Layout of the bought parts in the body",
    ("cell_x", "9.5 mm", SP, "cell axes at +-x"),
    ("cell_z", "-17 mm", SP, "cell axis height"),
    ("pca_x", "25 mm", DESIGN, "standing board at +-x; simple_parts.py has 23. Pins outward with the bends down, Mini360 inward; the left board is the right one turned 180 deg"),
    ("pca_z", "-12 mm", SP, "standing board centre"),
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
    ("horn_arm_t", "1.6 mm", TODO, "arm thickness"),
    ("horn_gap", "0.9 mm", OLD2, "servo top to the horn seat: 34 with a 5 mm gear - 28.1 - 5"),
    ("horn_top", "=mg90_turret_top + horn_gap + horn_hub_h", CALC, "tab underside to the horn's outer face"),
    ("mg90_stack", "=mg90_below_tab + horn_top", CALC, "body bottom to the horn's outer face"),

    "Pivot bearing MR128K, opposite every horn (part origin: bearing centre on one face; axis along Z)",
    ("bearing_od", "12 mm", PHOTO, "seller picture; matches hexapodv2"),
    ("bearing_id", "8 mm", PHOTO, "seller picture; hexapodv2 inBearingHolder r 4"),
    ("bearing_t", "2.5 mm", PHOTO, "seller picture; hexapodv2 bearing pockets"),
    ("bearing_seat_d", "12.2 mm", OLD2, "onBearingHolder r 6.1, pocket for the outer ring"),

    "18650 pair (part origin: centre of the pair; cells along Y)",
    ("cell_d", "18 mm", STD, "18650"),
    ("cell_len", "65 mm", STD, "18650"),

    "PCA9685 node, HW-052 with bent pins on top and 3 Mini360 glued on the back; the power cable is not modelled (part origin: PCB centre, lying flat; long side along X; +Z = pin side; pin bends at -Y)",
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
    ("pca_sock_len", "15.24 mm", STD, "6-pin socket on the back, 6 x 2.54"),
    ("pca_sock_w", "2.54 mm", STD, "socket"),
    ("pca_sock_h", "8.5 mm", TODO, "socket height from the back of the PCB"),
    ("pca_sock_edge", "1.4 mm", PHOTO, "pad row from the short edge"),
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

    "ESP32 devkit, 38 pins, USB-C (part origin: PCB centre, lying flat; long side along X; +Z = module side; USB at -X)",
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
    ("esp_hdr_len", "48.23 mm", CAL, "19 pins per side"),
    ("esp_hdr_w", "2.54 mm", STD, "pin header"),
    ("esp_hdr_margin", "0.6 mm", TODO, "pin row from the antenna end of the PCB, scaled from the caliper photo"),
    ("esp_hdr_h", "7.9 mm", CAL, "pins below the PCB, black plastic included"),
    ("esp_hdr_pitch", "22.86 mm", PHOTO, "between the two pin rows: ~23.2 in the caliper photo, nearest 2.54 multiple"),

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

    "Coxa link, two prints: lower plate with the femur servo wall, and a top plate over the horn (leg frame: origin on the coxa axis at femur axis height, +X out along the link, Z up). Coxa servos: shaft up, body along Y (front and middle -Y, rear +Y)",
    ("coxa_cover_gap", "1 mm", DESIGN, "top plate to the underside of the top cover"),
    ("coxa_top_z", "=body_top_z - coxa_cover_gap", CALC, "top of the link"),
    ("coxa_horn_roof", "2 mm", DESIGN, "plate above the horn pocket"),
    ("coxa_horn_z", "=coxa_top_z - coxa_horn_roof", CALC, "outer face of the coxa horn"),
    ("coxa_servo_seat_z", "=coxa_horn_z - horn_top", CALC, "coxa servo tab underside"),
    ("coxa_servo_bottom_z", "=coxa_servo_seat_z - mg90_below_tab", CALC, "coxa servo bottom"),
    ("coxa_low_gap", "0.5 mm", DESIGN, "servo bottom to the lower plate"),
    ("coxa_low_top_z", "=coxa_servo_bottom_z - coxa_low_gap", CALC, "top of the lower hub"),
    ("coxa_low_hub_t", "4 mm", DESIGN, "lower hub: bearing + 1.5 roof"),
    ("coxa_low_z", "=coxa_low_top_z - coxa_low_hub_t", CALC, "underside of the lower plate"),
    ("coxa_low_clear", "=coxa_low_z - body_bottom_z - body_plate_t", CALC, "lower plate to the body's bottom plate"),
    ("coxa_low_hub_d", "18 mm", DESIGN, "around the bearing seat"),
    ("coxa_bore_d", "9 mm", DESIGN, "through hole above the bearing"),
    ("coxa_pin_d", "=bearing_id", CALC, "body pin through the bearing"),
    ("coxa_beam_t", "2 mm", DESIGN, "thin part of the plates; FEA has plenty of margin"),
    ("coxa_beam_w", "14 mm", DESIGN, "plate width; femur fork arms pass outside it"),
    ("coxa_end", "72 mm", DESIGN, "plates end, past the femur servo tabs"),
    ("coxa_low_end", "68.5 mm", DESIGN, "lower plate ends here, so its corner stays inside femur_web_x0 when the femur swings down"),
    ("coxa_tail_wall", "2.5 mm", DESIGN, "plate past the horn tip"),
    ("coxa_block_end", "=horn_reach + coxa_tail_wall", CALC, "thick horn block ends here"),
    ("coxa_top_low_z", "=coxa_horn_z - coxa_horn_pocket", CALC, "underside of the top plate at the horn"),
    ("coxa_horn_pocket", "=horn_hub_h", OLD1, "horn sits fully in the plate, as in ServoGear_MG90S_clone"),
    ("coxa_access_d", "5 mm", TODO, "through hole for the horn screw"),
    ("horn_screw_r1", "9 mm", TODO, "horn arm hole used to screw it to the plate, from the shaft"),
    ("horn_screw_r2", "15 mm", TODO, "second horn arm hole"),
    ("horn_screw_d", "2 mm", TODO, "clearance hole for that screw"),
    ("coxa_wall_t", "3 mm", DESIGN, "femur servo wall"),
    ("coxa_wall_x0", "37 mm", DESIGN, "inner end of the wall, past the tab tip"),
    ("coxa_seat_y", "=-mg90_stack / 2 + mg90_below_tab", CALC, "femur servo tab underside: its stack is centred on the leg plane"),
    ("coxa_wall_top_z", "=coxa_top_z - coxa_beam_t", CALC, "top of the wall, the top plate lands here"),
    ("coxa_boss_len", "6 mm", DESIGN, "boss for the top plate screw"),
    ("coxa_boss_w", "6 mm", DESIGN, "boss for the top plate screw"),
    ("coxa_boss_h", "5 mm", DESIGN, "boss for the top plate screw"),
    ("coxa_boss_x1", "48 mm", DESIGN, "inner top plate screw"),
    ("coxa_boss_x2", "62 mm", DESIGN, "outer top plate screw"),
    ("coxa_plate_screw_d", "2.2 mm", DESIGN, "clearance hole for the top plate screws, M2"),
    ("pla_density", "1.24", STD, "g/cm3, solid, for the weight check"),

    "Pivot opposite every leg horn, in the leg plane frame (Y along the joint axis, stack centred on Y = 0, horn on +Y)",
    ("lattice_rib", "1.6 mm", DESIGN, "ribs left by the lattice cuts: 4 perimeters of 0.4"),
    ("lattice_s", "2.5 mm", DESIGN, "triangle legs shorter than their cell: leaves the diagonal rib"),
    ("print_chamfer", "2 mm", DESIGN, "internal chamfers where walls meet plates, for PLA strength"),
    ("joint_rib_t", "2 mm", DESIGN, "rib joining the servo wall and the bearing cap"),
    ("arm_taper_len", "8 mm", DESIGN, "thick horn block tapers down to the thin plate over this length"),
    ("wall_hole_h", "9 mm", DESIGN, "lightening hole in the coxa wall, under the servo window"),
    ("wall_hole_len", "17 mm", DESIGN, "lightening hole in the coxa wall"),
    ("joint_gap", "0.5 mm", DESIGN, "servo bottom to the bearing cap"),
    ("joint_roof", "1.5 mm", DESIGN, "cap behind the bearing"),
    ("joint_cap_t", "=bearing_t + joint_roof", CALC, "bearing cap"),
    ("joint_cap_y1", "=-mg90_stack / 2 - joint_gap", CALC, "servo side of the cap"),
    ("joint_cap_y0", "=joint_cap_y1 - joint_cap_t", CALC, "outer side of the cap, the bearing goes in from here"),
    ("joint_hub_d", "18 mm", DESIGN, "around the bearing seat and on the outer arm"),
    ("joint_arm_gap", "0.5 mm", DESIGN, "cap to the outer arm"),
    ("joint_arm_t", "2.5 mm", DESIGN, "outer arm with the pin"),
    ("joint_arm_y1", "=joint_cap_y0 - joint_arm_gap", CALC, "inner face of the outer arm"),
    ("joint_arm_y0", "=joint_arm_y1 - joint_arm_t", CALC, "outer face of the outer arm"),
    ("joint_pin_len", "=bearing_t + joint_arm_gap", CALC, "pin on the outer arm, into the bearing"),
    ("joint_horn_y", "=mg90_stack / 2", CALC, "outer face of the horn"),
    ("joint_arm_top_y", "=joint_horn_y + coxa_horn_roof", CALC, "outer face of the horn arm"),
    ("joint_horn_low_y", "=joint_horn_y - coxa_horn_pocket", CALC, "inner face of the horn arm at the horn"),
    ("joint_beam_y", "=joint_arm_top_y - coxa_beam_t", CALC, "inner face of the thin part of the horn arm"),

    "Femur, two prints: main (pin arm, web, tibia servo box) and a horn arm (femur frame: origin on the femur axis, +X to the tibia axis at neutral)",
    ("femur_web_x0", "26.5 mm", DESIGN, "web inner face; femur_web_x0 > the coxa lower plate corner radius"),
    ("femur_web_x1", "29 mm", DESIGN, "web outer face; notched for the tibia servo tab, which starts at x = 28.35"),
    ("femur_box_end", "62 mm", DESIGN, "end of the tibia servo box, past its tabs"),
    ("femur_box_h", "15.8 mm", DESIGN, "tibia servo box height, in Z"),
    ("femur_flange_t", "1.6 mm", DESIGN, "above and below the tibia servo"),
    ("femur_flange_y1", "7 mm", DESIGN, "flanges end, toward the horn"),
    ("femur_boss_x0", "=coxa_block_end", CALC, "screw boss on the web, reaches past the coxa top plate at y > 7"),
    ("femur_boss_y0", "8 mm", DESIGN, "screw boss, clear of the coxa top plate and the femur servo"),
    ("femur_screw_x", "23 mm", DESIGN, "horn arm screws"),
    ("femur_screw_z", "4 mm", DESIGN, "horn arm screws at +-z"),

    "Tibia, three prints: main (pin arm, cross plate, shank), horn arm, floating foot tip (tibia frame: origin on the knee axis, Z up with the tibia hanging straight down from a level femur)",
    ("tibia_cross_top_z", "-28 mm", DESIGN, "top of the cross plate, outside the femur box corner when the knee swings"),
    ("tibia_cross_t", "3 mm", DESIGN, "cross plate joining the fork arms and the shank"),
    ("tibia_boss_y0", "8 mm", DESIGN, "screw boss for the horn arm, clear of the femur box"),
    ("tibia_boss_h", "6 mm", DESIGN, "screw boss height above the cross plate"),
    ("tibia_screw_x", "4 mm", DESIGN, "horn arm screws at +-x"),
    ("tibia_shank_w", "10 mm", DESIGN, "square shank around the switch; sim tibia_arm 8"),
    ("tibia_shank_end_z", "-50 mm", DESIGN, "lower end of the shank"),
    ("tip_foot_r", "5 mm", SIM, "foot_radius, rounded end of the tip"),
    ("switch_fit", "6.2 mm", OLD1, "buttonWidth, square bore for the switch"),
    ("switch_slot_y", "2.8 mm", DESIGN, "slot for the switch leads at +-y"),
    ("tip_stem_w", "5.98 mm", OLD1, "tibiaTipWidth, tip stem sliding in the bore"),
    ("tip_screw_z", "=tibia_shank_end_z + 2.5 mm", CALC, "screw across the shank, holding the tip"),
    ("tip_hole_d", "2.56 mm", OLD1, "screw hole in the tip, r 1.28: the tip floats on the screw"),
    ("tip_float", "=(tip_hole_d - mg90_screw_pilot_d) / 2", CALC, "tip travel on the screw"),
    ("tip_hole_z", "=tip_screw_z - tip_float", CALC, "hole centre with the foot hanging: the screw touches the top of the hole"),
    ("tip_stem_top_z", "=tip_screw_z + 2.5 mm", CALC, "stem top, foot hanging"),
    ("tip_gap", "0.1 mm", OLD1, "safePrinterDistance, stem to button with the foot hanging"),
    ("tip_foot_z", "=-tibia_len + tip_foot_r", CALC, "centre of the rounded foot, bottom at tibia_len"),
    ("switch_btn_z", "=tip_stem_top_z + tip_gap", CALC, "button end"),
    ("switch_top_z", "=switch_btn_z + sw_total_h", CALC, "top of the switch body, end of the bore"),
    ("tip_press_travel", "=tip_float - tip_gap", CALC, "how far the tip can push the button; hexapod.FCStd clickButtonMoveLength 0.4"),

    "Body bottom print: hexagon plate, pivot pins, servo towers (body frame: origin on the centre at femur axis height, X right, Y forward)",
    ("body_cx", "=coxa_front_x + coxa_front_x / sqrt(coxa_front_x^2 + coxa_front_y^2) * plate_inflate", CALC, "hexagon corner, plate_outline() in params.py"),
    ("body_cy", "=coxa_front_y + coxa_front_y / sqrt(coxa_front_x^2 + coxa_front_y^2) * plate_inflate", CALC, "hexagon corner"),
    ("body_mx", "=coxa_mid_x + plate_inflate", CALC, "hexagon side corner"),
    ("body_plate_top", "=body_bottom_z + body_plate_t", CALC, "top of the plate"),
    ("body_pin_h", "=coxa_low_z + bearing_t - body_plate_top", CALC, "pivot pin, up through the coxa bearing"),
    ("cell_seat_d", "=cell_d + 2 * 0.2 mm", CALC, "cells sink into the plate: their centre height leaves them 1 mm under its top"),
    ("tower_wall_t", "2 mm", DESIGN, "walls under the servo shelf"),
    ("tower_shelf_t", "2.5 mm", DESIGN, "shelf under the servo tabs, from coxa_servo_seat_z down"),
    ("tower_gap", "10 mm", DESIGN, "corner tower walls stop this far from the coxa axis, clear of the hub and the link"),
    ("mg90_tab_near", "=mg90_tab_total / 2 + mg90_body_len / 2 - (mg90_body_len - mg90_shaft_end) ", CALC, "shaft to the near tab tip"),
    ("mg90_tab_far", "=mg90_tab_total - mg90_tab_near", CALC, "shaft to the far tab tip"),
    ("mg90_hole_near", "=mg90_screw_spacing / 2 - (mg90_body_len / 2 - mg90_shaft_end)", CALC, "shaft to the near tab screw"),
    ("mg90_hole_far", "=mg90_screw_spacing - mg90_hole_near", CALC, "shaft to the far tab screw"),
    ("tower_mid_y0", "9.5 mm", DESIGN, "middle tower wall starts here, clear of the coxa hub"),
    ("tower_mid_y1", "=pca_grp_x3 - pca_grp_w / 2 - 0.2 mm", CALC, "middle tower wall ends before the PCA9685's 3rd pin group"),

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
