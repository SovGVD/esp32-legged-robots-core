/**
 * Dog description: legs, servo calibration and gait. Only types from
 * lib/robot_core and macros from def.h + config.h, no Arduino, so the same
 * data is compiled into the firmware (model.cpp) and the simulator bridge
 * (sim/bridge). Include it from exactly one .cpp file per program.
 */
#ifndef MODEL_DOG_MODEL_DATA_H
#define MODEL_DOG_MODEL_DATA_H

leg legs[LEG_NUM] = {
  // LEFT FRONT
  {
    {LEGLF, "LF"},
    { -LEG_BODY_X,  LEG_BODY_Y_F,   LEG_BODY_Z},
    {-LEG_POINT_X, LEG_POINT_Y_F, -LEG_POINT_Z},
    { LEG_SIZE_L1,   LEG_SIZE_L2,  LEG_SIZE_L3},
    {    0,    0,    0 },
    { M_PI, M_PI, M_PI },
    { M_PI_2, M_PI_4, M_PI_2 },
    {
      { M_PI_2, M_PI_2, M_PI_2 },             // hardware middle
      {     25,     26,     27 },             // 3 servo pins
      {      0,      0,      0 },             // servo middle trim
      {      2,      1,      1 }              // gear ratio, first and last one is 16 groves on servo and 24 groves on leg
    },
    {  -LEG_BODY_X,  LEG_BODY_Y_F,   LEG_BODY_Z },
    { -LEG_POINT_X, LEG_POINT_Y_F, -LEG_POINT_Z },
    {       M_PI_2,        M_PI_4,       M_PI_2 },
    {true, true, false, false, true, true},
    {true, 0, 0}
  },
  // RIGHT FRONT
  {
    {LEGRF, "RF"},
    { LEG_BODY_X,  LEG_BODY_Y_F,   LEG_BODY_Z},
    {LEG_POINT_X, LEG_POINT_Y_F, -LEG_POINT_Z},
    {LEG_SIZE_L1,   LEG_SIZE_L2,  LEG_SIZE_L3},
    {0, 0, 0},
    {M_PI, M_PI, M_PI},
    { M_PI_2, M_PI_4, M_PI_2 },
    {
      {M_PI_2, M_PI_2, M_PI_2},
      { 16, 18, 17},
      {  0,  0,  0},
      {  2,  1,  1}
    },
    {     LEG_BODY_X,   LEG_BODY_Y_F,      LEG_BODY_Z},
    {    LEG_POINT_X,  LEG_POINT_Y_F,    -LEG_POINT_Z},
    {M_PI_2, M_PI_4, M_PI_2},
    {false, true, false, true, false, false},
    {true, 0, 0}
  },
  // LEFT HIND
  {
    {LEGLH, "LH"},
    { -LEG_BODY_X,  -LEG_BODY_Y_H,   LEG_BODY_Z},
    {-LEG_POINT_X, -LEG_POINT_Y_H, -LEG_POINT_Z},
    { LEG_SIZE_L1,    LEG_SIZE_L2,  LEG_SIZE_L3},
    {0, 0, 0},
    {M_PI, M_PI, M_PI},
    { M_PI_2, M_PI_4, M_PI_2 },
    {
      {M_PI_2, M_PI_2, M_PI_2},
      { 13, 12, 14},
      {  0,  0,  0},
      {  2,  1,  1}
    },
    {    -LEG_BODY_X,  -LEG_BODY_Y_H,      LEG_BODY_Z},
    {   -LEG_POINT_X, -LEG_POINT_Y_H,    -LEG_POINT_Z},
    {M_PI_2, M_PI_4, M_PI_2},
    {true, true, false, true, true, true},
    {true, 0, 0}
  },
  // RIGHT HIND
  {
    {LEGRH, "RH"},
    { LEG_BODY_X,  -LEG_BODY_Y_H,   LEG_BODY_Z},
    {LEG_POINT_X, -LEG_POINT_Y_H, -LEG_POINT_Z},
    {LEG_SIZE_L1,    LEG_SIZE_L2,  LEG_SIZE_L3},
    {0, 0, 0},
    {M_PI, M_PI, M_PI},
    { M_PI_2, M_PI_4, M_PI_2 },
    {
      {M_PI_2, M_PI_2, M_PI_2},
      {  4,  2, 15},
      {  0,  0,  0},
      {  2,  1,  1}
    },
    {     LEG_BODY_X,  -LEG_BODY_Y_H,      LEG_BODY_Z},
    {    LEG_POINT_X, -LEG_POINT_Y_H,    -LEG_POINT_Z},
    {M_PI_2, M_PI_4, M_PI_2},
    {false, true, false, false, false, false},
    {true, 0, 0}
  }
};

servoProfile servoMainProfile = {
    20,   // minAngle
    160,  // maxAngle
    780,  // degMin
    890,  // deg30
    1090, // deg50
    1300, // deg70
    1500, // deg90 - middle
    1700, // deg110
    1900, // deg130
    2110, // deg150
    2200  // degMax
};

const GaitParams GAIT_PARAMS = {
	true,     // enabled
	0.4,      // cycle time, s (two pairs of diagonal legs)
	0.75,     // duty factor: each leg swings 100ms of every 400ms
	{ 0.5, 0.0, 0.0, 0.5, 0, 0 },   // phase offsets LF, RF, LH, RH: trot (diagonal pairs)
	15,       // step height, mm: MG90D servos (0.1 s/60 deg, 50 Hz pulses) can't lift a foot
	          // 30 mm and put it back in a 100 ms swing; the foot landed 50-100 ms late, the body
	          // stood on two legs that much longer and rocked over the diagonal (simulator)
	150,      // max speed, mm/s
	1.5,      // max yaw rate, rad/s
	600,      // max acceleration, mm/s^2
	6,        // max yaw acceleration, rad/s^2
	3,        // idle tolerance, mm
	false,    // no foot contact sensors
	false,    // no static stability shift: a trot is supported by a diagonal line
	0.1,      // (stability blend)
	{ 1, 1, 0 },  // (stability gain)
	{ 0, -8, 0 }, // centre of mass, mm from the body origin: the body is moved this much the other
	              // way so it sits over the middle of the feet. The parts put it ~5 mm behind the
	              // hips' centre; -8 balances best in the simulator (rocking, backward drift).
	              // Check on the robot: it balances on a ruler edge this far behind the middle.
	50            // max sideways foot speed, mm/s (strafe + turn): the hip roll servos have the 2:1
	              // gear, ~5 rad/s at the leg; a 150 mm/s strafe needed 15 rad/s, the feet landed late
	              // and the body rocked (simulator). Strafe 50 mm/s, turn in place ~0.7 rad/s.
};

/**
 * Slow statically stable walk (crawl): one leg at a time, LH, LF, RH, RF,
 * with the body shifted over the other three feet before each lift-off.
 * Select with the CLI command `gait walk`.
 */
const GaitParams GAIT_PARAMS_WALK = {
	true,     // enabled
	1.2,      // cycle time, s
	0.9,      // duty factor: one leg swings at a time (120 ms), 180 ms four-foot phases in between to shift the body
	{ 0.65, 0.15, 0.9, 0.4, 0, 0 },   // phase offsets LF, RF, LH, RH: swing order LH, LF, RH, RF
	25,       // step height, mm
	20,       // max speed, mm/s (stride ~22 mm: the legs also have to reach the shift)
	0.25,     // max yaw rate, rad/s
	300,      // max acceleration, mm/s^2
	2,        // max yaw acceleration, rad/s^2
	3,        // idle tolerance, mm
	false,    // no foot contact sensors
	true,     // keep the centre of mass over the supporting feet
	0.15,     // support weights ramp over the four-foot phases
	{ 0.7, 0, 0 },  // shift sideways only, 70 % of the way to the support centroid (~11 mm margin):
	              // the hip roll servos (2:1 gear, 20-160 deg) leave about +-23 deg of leg roll
	{ 0, -8, 0 }, // centre of mass, mm from the body origin (see GAIT_PARAMS; the fore-aft
	              // part has no effect here with the sideways-only gain)
	50            // max sideways foot speed, mm/s (see GAIT_PARAMS; this slow walk stays below it)
};

/**
 * Rough ground (with TERRAIN_PARAMS): slower trot with a longer swing, so
 * the servos can lift a foot over an obstacle before it moves forward.
 * Simulator.
 */
const GaitParams GAIT_PARAMS_ROUGH = {
	true,     // enabled
	0.6,      // cycle time, s
	0.7,      // duty factor: each leg swings 180 ms
	{ 0.5, 0.0, 0.0, 0.5, 0, 0 },   // phase offsets LF, RF, LH, RH: trot
	15,       // step height, mm (over flat ground; obstacles raise it)
	80,       // max speed, mm/s
	0.8,      // max yaw rate, rad/s
	400,      // max acceleration, mm/s^2
	4,        // max yaw acceleration, rad/s^2
	3,        // idle tolerance, mm
	false,    // foot switches: set by the terrain mode
	false,    // no static stability shift
	0.1,      // (stability blend)
	{ 1, 1, 0 },  // (stability gain)
	{ 0, -8, 0 }, // centre of mass, mm (see GAIT_PARAMS)
	40        // max sideways foot speed, mm/s
};

// IMU leveling (HORIZON_LEVEL or CLI `level on`), tuned in the simulator:
// mostly integral, so it removes a slope's steady tilt but doesn't chase
// the gait's own rocking (the IMU estimate is off by 1-3 deg while walking).
// It holds its value during a trot (see BodyLeveler::update, hold).
const LevelerParams LEVEL_PARAMS = {
	0.0,   // kp
	0.5,   // ki, 1/s
	0.0,   // kd, s
	0.25,  // limit, rad (~14 deg)
	0.5,   // rate, rad/s
	0.0,   // IMU low-pass, s (TiltEstimator already filters)
	0.026 // deadband, rad (1.5 deg): the integral ignores the walking estimate error
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
 * sensors. The real dog has neither yet; the simulator uses these to test
 * the terrain code (sim/examples/terrain_walk.py).
 *
 * One VL53L5CX (8x8 zones, 45 deg field of view) on the front of the body,
 * looking 25 deg down: it sees the ground from just ahead of the nose and, with
 * the top zone rows, nearly level (the room scan, TOF_SCAN_PARAMS, needs that:
 * at 40 deg down the legs can't tilt the nose up far enough).
 */
const TofSensor TOF_SENSORS[] = {
	// position (mm, body frame), yaw, tilt down (rad), field of view (rad), zones per side, max range (mm)
	{ { 0, 88, 0 }, 0.0, 0.436, 0.785, 8, 400 },
};
#define TOF_SENSOR_COUNT 1

const TerrainMapParams TERRAIN_MAP_PARAMS = {
	12,    // cell size, mm (the 40x40 map covers 480 mm around the robot)
	10.0,  // cells are forgotten after 10 s (odometry drifts; the hind feet reach what the sensor saw ~4 s earlier)
	0.3,   // a lower reading moves a cell 30 % of the way (noise)
	8      // a reading 8 mm above a cell replaces it at once (obstacle top)
};

// body tilt the terrain map places ToF readings with (TerrainTilt): the trot rocks faster than the filter; filtered cost the 20 mm step in the sim
#define TERRAIN_TILT_MODE TERRAIN_TILT_RAW

const TerrainParams TERRAIN_PARAMS = {
	12,    // search depth, mm: with foot switches a foot reaches up to 12 mm below its planned touchdown
	       // (the legs are 80 mm long and reach 102: deeper runs them out of reach)
	150,   // search speed, mm/s
	10,    // clearance, mm above the highest mapped point between lift-off and touchdown
	30,    // max step, mm: mapped ground further from the leg's own ground is ignored (out of reach)
	40,    // body height follows the ground under the feet at up to 40 mm/s
	6,     // edge limit, mm: footholds next to a height change of more than 6 mm move
	36,    // up to 36 mm along the step
	6,     // foot radius, mm: the foot point is the centre of the 6 mm TPU shoe ball
	5,     // tolerance, mm: touchdowns closer than this to the plan keep the planned height
	0.05,  // servo latency, s (50 Hz pulses plus travel)
	20     // gap margin, mm: keep 20 mm from a gap (foot ball radius + a map cell, rounded up)
};

/**
 * 2D lidar (LD06 / LD19) for indoor mapping and walking to a point (lidar.cpp,
 * `lidar`, `goto`). On top of the body, in the middle: the scan plane is
 * ~40 mm above the body origin, ~120 mm above the floor, over the legs and
 * the head. Simulator only so far.
 */
#define LIDAR_SUPPORTED 1
const LidarParams LIDAR_PARAMS = {
	{ 0, 0, 40 },  // position, mm, body frame
	0.0,           // yaw of the sensor's 0 deg, rad (check with `lidar print`)
	true,          // LD06 / LD19 count clockwise
	100,           // min range, mm (closer is the robot itself)
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
	130,   // robot radius, mm: half the body length plus a little
	200,   // + 200 mm that still costs more (doorways: walk in the middle)
	350,   // lookahead, mm
	120,   // arrived within 120 mm
	0.8,   // forward command 0.8 of GAIT_PARAMS max speed
	1.5,   // turn gain, 1/s
	0.6,   // turn in place for heading errors over 35 deg
	180,   // stop walking forward with something closer than 180 mm ahead
	1.0,   // replan every second
	40     // map cells under 40 log-odds (one scan's hit, from afar) are doubtful: the path may cross
	       // them, and the ToF navigator walks up and looks again first (a doorway seen at a slant)
};

/**
 * Room scans with the ToF sensors instead of a lidar (tof_scan.h, used when
 * LIDAR_TYPE is not set in config.h): the robot stops, tilts its nose up so
 * the nose sensor (TOF_SENSORS) look level, turns once around and
 * puts what they saw into the same map. Walking to a goal it scans every
 * scanEvery mm. Simulator only so far (sim/examples/lidar_nav.py --tof).
 */
#define TOF_SCAN_SUPPORTED 1
const TofScanParams TOF_SCAN_PARAMS = {
	0.14,   // nose up 8 deg while scanning. The top zone row looks 19.7 +- 2.8 deg above the
	        // sensor's axis: nose up = mount angle (25) - 17 deg puts its lower edge level. The legs
	        // reach ~15 deg at most (with the crouch), so the mount can't be much steeper than 30 deg
	0.3,    // tilt rate, rad/s
	0.45,   // turning while scanning, rad/s (slow: the sensor's frames lag a little)
	6.7,    // turn a full circle plus 23 deg of overlap
	0.07,   // zone rays within 4 deg of level are used
	120,    // min range, mm
	2500,   // max range, mm (VL53L5CX 8x8 indoors; further readings are unreliable)
	40,     // hits lower than 40 mm above the floor are the floor
	1000,   // scan again after walking this far, mm
	0.5,    // stand still 0.5 s before tilting up
	40,     // walking: anything over 40 mm high ahead stops the robot
	15,     // body 15 mm lower while scanning: the legs reach a steeper tilt
	0       // full turns every time. 2.6 = rescans sweep 150 deg (75 right, then left): in the sim that
	        // wasn't faster overall (doubtful sweeps turn on all the way) and got lost more often
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
