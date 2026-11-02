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
	30,       // step height, mm
	150,      // max speed, mm/s
	1.5,      // max yaw rate, rad/s
	600,      // max acceleration, mm/s^2
	6,        // max yaw acceleration, rad/s^2
	3,        // idle tolerance, mm
	false,    // no foot contact sensors
	false,    // no static stability shift: a trot is supported by a diagonal line
	0.1,      // (stability blend)
	{ 1, 1, 0 },  // (stability gain)
	{ 0, 0, 0 }   // centre of mass, mm from the body origin
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
	{ 0, 0, 0 }   // centre of mass, mm from the body origin (battery is centred)
};

#endif
