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

// IMU leveling (HORIZON_LEVEL or CLI `level on`), tuned in the simulator:
// mostly integral, so it removes a slope's steady tilt but doesn't chase
// the gait's own rocking (the IMU estimate is off by 1-3 deg while walking).
// It holds its value during a trot (see BodyLeveler::update, hold).
const LevelerParams LEVEL_PARAMS = {
	0.0,   // kp
	1.0,   // ki, 1/s
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

#endif
