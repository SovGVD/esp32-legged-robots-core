/**
 * New hexapod (MG90S direct drive) description: legs, servo calibration and gait. Only types from
 * lib/robot_core and macros from def.h + config.h, no Arduino, so the same
 * data can be compiled into the firmware (model.cpp), native tests and the
 * simulator bridge. Include it from exactly one .cpp file per program.
 */
#ifndef MODEL_HEXAPOD_MODEL_DATA_H
#define MODEL_HEXAPOD_MODEL_DATA_H

/**
 * Leg order LF, LM, LR, RF, RM, RR. Left legs on the left PCA9685 (channels
 * 0-10), right legs on the right one (16-26, second board), 3 channels per
 * leg with a spare between legs, like the old hexapod.
 *
 * Direct drive: gear ratio 1, every joint has the servo's 180 deg. Servo
 * middle (90 deg) = coxa along its neutral direction, femur level, tibia
 * square to the femur. The neutral stance has the femur ~58 deg down and the
 * knee ~132 deg open, 32 deg and 48 deg from the ends of the servo travel.
 *
 * Inverse flags (alpha, beta, gamma) assume the right legs are mirror images
 * of the left ones: the femur and tibia servos of the right legs turn the
 * other way. Not built yet: check each leg with the leg test before walking.
 */
leg legs[LEG_NUM] = {
  // left front leg
  {
    { LEGLF, "LF" },
    { -LEG_FRONT_BODY_X, LEG_FRONT_BODY_Y, LEG_BODY_Z },                  // default LEG POSITION ON BODY
    { -LEG_FRONT_FOOT_X, LEG_FRONT_FOOT_Y, -LEG_FOOT_Z },                 // default LEG ON GROUND: x, y, z
    { LEG_SIZE_COXA, LEG_SIZE_FEMUR, LEG_SIZE_TIBIA },
    { 0, 0, 0 },                                 // MIN ANGLES (enforced by MotionController)
    { M_PI, M_PI, M_PI },                        // MAX ANGLES
    { M_PI_2, M_PI_2, M_PI_2 },                  // calibration position
    {
      { M_PI_2, M_PI_2, M_PI_2 },                // hardware middle
      {  0,  1,  2 },                            // PCA9685 channels (16+ = second board)
      { 0, 0, 0 },                               // servo middle trim
      { 1, 1, 1 }                                // direct drive, no gears
    },
    { -LEG_FRONT_BODY_X, LEG_FRONT_BODY_Y, LEG_BODY_Z },
    { -LEG_FRONT_FOOT_X, LEG_FRONT_FOOT_Y, -LEG_FOOT_Z },
    { M_PI_2, M_PI_2, M_PI_2 },                  // leg angles, set by the HAL
    { true, true, true, false, false, false },          // inverse x, y, z, alpha, beta, gamma
    { true, 0, 0 }                               // foot sensor
  },
  // left middle leg
  {
    { LEGLM, "LM" },
    { -LEG_MIDDLE_BODY_X, LEG_MIDDLE_BODY_Y, LEG_BODY_Z },                  // default LEG POSITION ON BODY
    { -LEG_MIDDLE_FOOT_X, LEG_MIDDLE_FOOT_Y, -LEG_FOOT_Z },                 // default LEG ON GROUND: x, y, z
    { LEG_SIZE_COXA, LEG_SIZE_FEMUR, LEG_SIZE_TIBIA },
    { 0, 0, 0 },                                 // MIN ANGLES (enforced by MotionController)
    { M_PI, M_PI, M_PI },                        // MAX ANGLES
    { M_PI_2, M_PI_2, M_PI_2 },                  // calibration position
    {
      { M_PI_2, M_PI_2, M_PI_2 },                // hardware middle
      {  4,  5,  6 },                            // PCA9685 channels (16+ = second board)
      { 0, 0, 0 },                               // servo middle trim
      { 1, 1, 1 }                                // direct drive, no gears
    },
    { -LEG_MIDDLE_BODY_X, LEG_MIDDLE_BODY_Y, LEG_BODY_Z },
    { -LEG_MIDDLE_FOOT_X, LEG_MIDDLE_FOOT_Y, -LEG_FOOT_Z },
    { M_PI_2, M_PI_2, M_PI_2 },                  // leg angles, set by the HAL
    { true, true, true, false, false, false },          // inverse x, y, z, alpha, beta, gamma
    { true, 0, 0 }                               // foot sensor
  },
  // left rear leg
  {
    { LEGLR, "LR" },
    { -LEG_REAR_BODY_X, LEG_REAR_BODY_Y, LEG_BODY_Z },                  // default LEG POSITION ON BODY
    { -LEG_REAR_FOOT_X, LEG_REAR_FOOT_Y, -LEG_FOOT_Z },                 // default LEG ON GROUND: x, y, z
    { LEG_SIZE_COXA, LEG_SIZE_FEMUR, LEG_SIZE_TIBIA },
    { 0, 0, 0 },                                 // MIN ANGLES (enforced by MotionController)
    { M_PI, M_PI, M_PI },                        // MAX ANGLES
    { M_PI_2, M_PI_2, M_PI_2 },                  // calibration position
    {
      { M_PI_2, M_PI_2, M_PI_2 },                // hardware middle
      {  8,  9, 10 },                            // PCA9685 channels (16+ = second board)
      { 0, 0, 0 },                               // servo middle trim
      { 1, 1, 1 }                                // direct drive, no gears
    },
    { -LEG_REAR_BODY_X, LEG_REAR_BODY_Y, LEG_BODY_Z },
    { -LEG_REAR_FOOT_X, LEG_REAR_FOOT_Y, -LEG_FOOT_Z },
    { M_PI_2, M_PI_2, M_PI_2 },                  // leg angles, set by the HAL
    { true, true, true, false, false, false },          // inverse x, y, z, alpha, beta, gamma
    { true, 0, 0 }                               // foot sensor
  },
  // right front leg
  {
    { LEGRF, "RF" },
    { LEG_FRONT_BODY_X, LEG_FRONT_BODY_Y, LEG_BODY_Z },                  // default LEG POSITION ON BODY
    { LEG_FRONT_FOOT_X, LEG_FRONT_FOOT_Y, -LEG_FOOT_Z },                 // default LEG ON GROUND: x, y, z
    { LEG_SIZE_COXA, LEG_SIZE_FEMUR, LEG_SIZE_TIBIA },
    { 0, 0, 0 },                                 // MIN ANGLES (enforced by MotionController)
    { M_PI, M_PI, M_PI },                        // MAX ANGLES
    { M_PI_2, M_PI_2, M_PI_2 },                  // calibration position
    {
      { M_PI_2, M_PI_2, M_PI_2 },                // hardware middle
      { 16, 17, 18 },                            // PCA9685 channels (16+ = second board)
      { 0, 0, 0 },                               // servo middle trim
      { 1, 1, 1 }                                // direct drive, no gears
    },
    { LEG_FRONT_BODY_X, LEG_FRONT_BODY_Y, LEG_BODY_Z },
    { LEG_FRONT_FOOT_X, LEG_FRONT_FOOT_Y, -LEG_FOOT_Z },
    { M_PI_2, M_PI_2, M_PI_2 },                  // leg angles, set by the HAL
    { true, true, true, false, true, true },          // inverse x, y, z, alpha, beta, gamma
    { true, 0, 0 }                               // foot sensor
  },
  // right middle leg
  {
    { LEGRM, "RM" },
    { LEG_MIDDLE_BODY_X, LEG_MIDDLE_BODY_Y, LEG_BODY_Z },                  // default LEG POSITION ON BODY
    { LEG_MIDDLE_FOOT_X, LEG_MIDDLE_FOOT_Y, -LEG_FOOT_Z },                 // default LEG ON GROUND: x, y, z
    { LEG_SIZE_COXA, LEG_SIZE_FEMUR, LEG_SIZE_TIBIA },
    { 0, 0, 0 },                                 // MIN ANGLES (enforced by MotionController)
    { M_PI, M_PI, M_PI },                        // MAX ANGLES
    { M_PI_2, M_PI_2, M_PI_2 },                  // calibration position
    {
      { M_PI_2, M_PI_2, M_PI_2 },                // hardware middle
      { 20, 21, 22 },                            // PCA9685 channels (16+ = second board)
      { 0, 0, 0 },                               // servo middle trim
      { 1, 1, 1 }                                // direct drive, no gears
    },
    { LEG_MIDDLE_BODY_X, LEG_MIDDLE_BODY_Y, LEG_BODY_Z },
    { LEG_MIDDLE_FOOT_X, LEG_MIDDLE_FOOT_Y, -LEG_FOOT_Z },
    { M_PI_2, M_PI_2, M_PI_2 },                  // leg angles, set by the HAL
    { true, true, true, false, true, true },          // inverse x, y, z, alpha, beta, gamma
    { true, 0, 0 }                               // foot sensor
  },
  // right rear leg
  {
    { LEGRR, "RR" },
    { LEG_REAR_BODY_X, LEG_REAR_BODY_Y, LEG_BODY_Z },                  // default LEG POSITION ON BODY
    { LEG_REAR_FOOT_X, LEG_REAR_FOOT_Y, -LEG_FOOT_Z },                 // default LEG ON GROUND: x, y, z
    { LEG_SIZE_COXA, LEG_SIZE_FEMUR, LEG_SIZE_TIBIA },
    { 0, 0, 0 },                                 // MIN ANGLES (enforced by MotionController)
    { M_PI, M_PI, M_PI },                        // MAX ANGLES
    { M_PI_2, M_PI_2, M_PI_2 },                  // calibration position
    {
      { M_PI_2, M_PI_2, M_PI_2 },                // hardware middle
      { 24, 25, 26 },                            // PCA9685 channels (16+ = second board)
      { 0, 0, 0 },                               // servo middle trim
      { 1, 1, 1 }                                // direct drive, no gears
    },
    { LEG_REAR_BODY_X, LEG_REAR_BODY_Y, LEG_BODY_Z },
    { LEG_REAR_FOOT_X, LEG_REAR_FOOT_Y, -LEG_FOOT_Z },
    { M_PI_2, M_PI_2, M_PI_2 },                  // leg angles, set by the HAL
    { true, true, true, false, true, true },          // inverse x, y, z, alpha, beta, gamma
    { true, 0, 0 }                               // foot sensor
  },
};

// MG90S: 500-2500 us over 180 deg [datasheet], linear until measured
servoProfile servoMainProfile = {
    0,    // minAngle
    180,  // maxAngle

    500,  // degMin
    833,  // deg30
    1056, // deg50
    1278, // deg70
    1500, // deg90 - middle
    1722, // deg110
    1944, // deg130
    2167, // deg150
    2500  // degMax
};

const GaitParams GAIT_PARAMS = {
	true,     // enabled
	0.3,      // cycle time, s: tripod, each leg swings 150 ms of every 300 ms
	0.5,      // duty factor
	{ 0.25, 0.75, 0.25, 0.75, 0.25, 0.75 },   // phase offsets LF, LM, LR, RF, RM, RR (LF, LR, RM swing together)
	30,       // step height, mm (25 drifted 15-19 mm/s sideways in the simulator)
	300,      // max speed, mm/s. Simulator (MG90S datasheet at 5 V, 815 g): ~250 mm/s forward,
	          // servos at their torque limit ~2 % of the time. 400 walks ~380 but the femur
	          // servos saturate 38 % of the time and it drifts sideways
	1.6,      // max yaw rate, rad/s: simulator turns ~1.5 rad/s (86 deg/s), coxa servos at their
	          // torque limit ~5 % of the time; 1.0 turned only ~0.73
	600,      // max acceleration, mm/s^2
	6,        // max yaw acceleration, rad/s^2
	3,        // idle tolerance, mm
	false,    // foot switches: set by the terrain mode
	false,    // no static stability shift: a tripod always stands on three legs
	0.1,      // (stability blend)
	{ 1, 1, 0 },  // (stability gain)
	{ 0, 0, 0 },  // centre of mass, mm from the body origin (design: battery in the middle)
	200       // max sideways foot speed, mm/s (strafe + turn): ~130 mm/s strafing in the simulator,
	          // higher saturates the tibia servos
};

/**
 * Rough ground (with TERRAIN_PARAMS): slower tripod with twice the swing
 * time, so the servos can lift a foot over a 40 mm obstacle (a 150 ms swing
 * can't: the femur lags and the foot hits the edge). Simulator.
 */
const GaitParams GAIT_PARAMS_ROUGH = {
	true,     // enabled
	0.6,      // cycle time, s: each leg swings 300 ms
	0.5,      // duty factor: tripod
	{ 0.25, 0.75, 0.25, 0.75, 0.25, 0.75 },   // phase offsets LF, LM, LR, RF, RM, RR
	30,       // step height, mm (over flat ground; obstacles raise it)
	120,      // max speed, mm/s
	0.4,      // max yaw rate, rad/s
	400,      // max acceleration, mm/s^2
	4,        // max yaw acceleration, rad/s^2
	3,        // idle tolerance, mm
	false,    // foot switches: set by the terrain mode
	false,    // no static stability shift
	0.1,      // (stability blend)
	{ 1, 1, 0 },  // (stability gain)
	{ 0, 0, 0 },  // centre of mass, mm
	120       // max sideways foot speed, mm/s
};

// IMU leveling (HORIZON_LEVEL or CLI `level on`), not tuned for this model yet:
// the dog's values (tuned in the simulator) as a starting point
const LevelerParams LEVEL_PARAMS = {
	0.0,   // kp
	1.0,   // ki, 1/s
	0.0,   // kd, s
	0.25,  // limit, rad
	0.5,   // rate, rad/s
	0.0,   // IMU low-pass, s
	0.026  // deadband, rad
};

// Battery protection (motion.cpp): warn below lowVoltage, below
// criticalVoltage stop, sit down by SIT_HEIGHT and turn the servos off.
// Defaults are for a 2S LiPo (3.4 / 3.2 V per cell); change for your pack.
const BatteryParams BATTERY_PARAMS = {
	6.8,   // lowVoltage, V
	6.4,   // criticalVoltage, V
	2.0,   // filter, s (servo current makes the voltage sag every step)
	3.0    // holdTime, s
};

// Body lowered this much (mm) to sit down when the battery is empty
const float SIT_HEIGHT = -30;

/**
 * Rough ground (optional, off unless enabled): foot switches and ToF distance
 * sensors. The real hexapod has neither yet; the simulator uses these to test
 * the terrain code (sim/examples/terrain_walk.py).
 *
 * Two VL53L5CX (8x8 zones, 45 deg field of view) on the front of the body,
 * turned 20 deg left and right, so they see the ground where the front feet
 * step (the front legs open 20 deg wider to keep out of the view: ~2 % of the
 * zones hit a leg in the design concept). The left one looks 35 deg down
 * (terrain), the right one 22 deg down like on the old hexapod: its top
 * zone rows look nearly level, which the room scan (TOF_SCAN_PARAMS) needs.
 * With both at 35 deg the scan would need the nose ~18 deg up; the simulator
 * only gets ~11 deg out of the legs.
 */
const TofSensor TOF_SENSORS[] = {
	// position (mm, body frame), yaw (+ = left), tilt down (rad), field of view (rad), zones per side, max range (mm)
	{ { -14, 74, 2 },  0.35, 0.61, 0.785, 8, 450 },
	{ {  14, 74, 2 }, -0.35, 0.384, 0.785, 8, 450 },
};
#define TOF_SENSOR_COUNT 2

const TerrainMapParams TERRAIN_MAP_PARAMS = {
	15,    // cell size, mm (the 40x40 map covers 600 mm around the robot; 25 mm cells are too coarse to see a 40 mm gap)
	6.0,   // cells are forgotten after 6 s (odometry drifts)
	0.3,   // a lower reading moves a cell 30 % of the way (noise)
	8      // a reading 8 mm above a cell replaces it at once (obstacle top)
};

// body tilt the terrain map places ToF readings with (TerrainTilt): with the raw IMU the mapped floor crept up 10-20 mm in 10 s of walking
#define TERRAIN_TILT_MODE TERRAIN_TILT_FILTERED

const TerrainParams TERRAIN_PARAMS = {
	30,    // search depth, mm: with foot switches a foot reaches up to 30 mm below its planned touchdown
	200,   // search speed, mm/s
	12,    // clearance, mm above the highest mapped point between lift-off and touchdown
	45,    // max step, mm: mapped ground further from the leg's own ground is ignored (out of reach)
	60,    // body height follows the ground under the feet at up to 60 mm/s
	8,     // edge limit, mm: footholds next to a height change of more than 8 mm move
	60,    // up to 60 mm along the step
	0,     // foot radius, mm: the foot point is the tibia tip itself
	5,     // tolerance, mm: touchdowns closer than this to the plan keep the planned height
	0.05,  // servo latency, s (50 Hz pulses plus travel)
	20     // gap margin, mm: keep 20 mm from ground that drops away (a map cell, rounded up)
};

/**
 * 2D lidar (LD06 / LD19) for indoor mapping and walking to a point (lidar.cpp,
 * `lidar`, `goto`). On top of the body, in the middle: the scan plane is
 * ~50 mm above the body origin, ~150 mm above the floor, over the legs. Simulator only so far.
 */
#define LIDAR_SUPPORTED 1
const LidarParams LIDAR_PARAMS = {
	{ 0, 0, 50 },  // position, mm, body frame
	0.0,           // yaw of the sensor's 0 deg, rad (check with `lidar print`)
	true,          // LD06 / LD19 count clockwise
	150,           // min range, mm (closer is the robot itself)
	8000,          // max range, mm
	10             // min intensity
};

const LidarMapParams LIDAR_MAP_PARAMS = {
	100,    // cell size, mm (200 x 200 cells: 20 x 20 m, 40 KB)
	6000,   // readings up to 6 m go into the map
	20, 5,  // log-odds: + 20 where a beam ends, - 5 where it passes
	100,    // cells stay within +-100
	50,     // map update after 50 mm...
	0.1,    // ...or 6 deg
	0.25,   // matches with a mean occupancy under the points below 0.25 are thrown away
	150,    // ...or that move the odometry guess more than 150 mm
	0.25    // ...or 14 deg in one scan (0.1 s)
};

const NavParams NAV_PARAMS = {
	200,   // robot radius, mm: the feet stand ~190 mm from the centre
	100,   // + 100 mm that still costs more (doorways: walk in the middle)
	450,   // lookahead, mm
	150,   // arrived within 150 mm
	0.4,   // forward command 0.4 of GAIT_PARAMS max speed (160 mm/s)
	1.5,   // turn gain, 1/s
	0.6,   // turn in place for heading errors over 35 deg
	260,   // stop walking forward with something closer than 260 mm ahead
	1.0    // replan every second
};

/**
 * Room scans with the ToF sensors instead of a lidar (tof_scan.h, used when
 * LIDAR_TYPE is not set in config.h): the robot stops, tilts its nose up so
 * the flatter right sensor's (TOF_SENSORS) top rows look level, turns once around and
 * puts what they saw into the same map. Walking to a goal it scans every
 * scanEvery mm. Simulator only so far (sim/examples/lidar_nav.py --tof).
 */
#define TOF_SCAN_SUPPORTED 1
const TofScanParams TOF_SCAN_PARAMS = {
	0.105,  // nose up 6 deg while scanning. The top zone row looks 19.7 +- 2.8 deg above the
	        // sensor's axis: nose up = mount angle (22, right sensor) - 17 deg puts its lower edge level. The
	        // servo ranges allow ~10 deg at most, so the mount can't be much steeper than 27 deg
	0.3,    // tilt rate, rad/s
	0.45,   // turning while scanning, rad/s (slow: the sensor's frames lag a little)
	6.7,    // turn a full circle plus 23 deg of overlap
	0.07,   // zone rays within 4 deg of level are used
	150,    // min range, mm
	2500,   // max range, mm (VL53L5CX 8x8 indoors; further readings are unreliable)
	40,     // hits lower than 40 mm above the floor are the floor
	1200,   // scan again after walking this far, mm
	0.5,    // stand still 0.5 s before tilting up
	40,     // walking: anything over 40 mm high ahead stops the robot
	15      // body 15 mm lower while scanning: the legs reach a steeper tilt
};

const LidarMapParams TOF_MAP_PARAMS = {
	100,    // cell size, mm (200 x 200 cells: 20 x 20 m, 40 KB)
	2500,   // readings up to 2.5 m go into the map
	20, 5,  // log-odds: + 20 where a beam ends, - 5 where it passes
	100,    // cells stay within +-100
	50,     // map update after 50 mm...
	0.1,    // ...or 6 deg
	0.25,   // matches with a mean occupancy under the points below 0.25 are thrown away
	250,    // ...or that move the odometry guess more than 250 mm (a metre walked on odometry alone is ~150 mm off)
	0.35    // ...or 20 deg
};

#endif
