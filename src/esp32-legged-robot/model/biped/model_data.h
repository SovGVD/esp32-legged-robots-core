/**
 * Biped description: legs, servo calibration and gait. Only types from
 * lib/robot_core and macros from def.h + config.h, no Arduino, so the same
 * data can be compiled into the firmware (model.cpp), native tests and the
 * simulator bridge. Include it from exactly one .cpp file per program.
 */
#ifndef MODEL_BIPED_MODEL_DATA_H
#define MODEL_BIPED_MODEL_DATA_H

leg legs[LEG_NUM] = {
  // left leg
  {
    { LEGL, "LEFT" },                                                                     // ID: id, title
    {  -LEG_BODY_X,    LEG_BODY_Y,   LEG_BODY_Z },                                        // default LEG POSITION ON BODY
    { -LEG_POINT_X,   LEG_POINT_Y, -LEG_POINT_Z },                                        // default LEG JOING CLOSED TO THE GROUND: x, y ,z
    {  LEG_SIZE_L1,   LEG_SIZE_L2,  LEG_SIZE_L3 },                                        // LEG SIZES (that we need):
    {    0,    0,    0,    0,    0,    0 },                                               // MIN ANGLES FOR EACH JOINT (enforced by MotionController)
    { M_PI, M_PI, M_PI, M_PI, M_PI, M_PI },                                               // MAX ANGLES FOR EACH JOINT (enforced by MotionController)
    { M_PI_2, M_PI_2, M_PI - M_PI_4, M_PI_2, M_PI - M_PI_4, M_PI_2 },                     // Calibration position
    {
      { M_PI_2, M_PI_2, M_PI_2, M_PI_2, M_PI_2, M_PI_2 },                                 // hardware middle (looks like never used, or not?)
      {      0,      1,      2,      3,      4,      5 },                                 // 6 servo channels
      {      0,      0,      0,      0,      0,      0 },                                 // servo middle trim (this will be override anyway with setup trim... why do I need it???)
      {      1,      1,      1,      1,      1,      1 }                                  // gear ratio, first and last one is 16 groves on servo and 24 groves on leg
      // @TODO WOW... i did not setup gear ratio correcly =(((
    },
    {  -LEG_BODY_X,  LEG_BODY_Y,   LEG_BODY_Z },                                          // initial LEG POSITION ON BODY, same as LEG POSITION ON BODY, not sure why I need it
    { -LEG_POINT_X, LEG_POINT_Y, -LEG_POINT_Z },                                          // initial LEG JOING CLOSED TO THE GROUND, same as above, but at least it make sense
    { M_PI_2, M_PI_2, M_PI_4, M_PI_2, M_PI_4, M_PI_2 },                                   // leg angles, will be override by HAL update
    { true, true, false, false, true, false, true, false, false },                        // inverse leg calculations (depends on servo, is servo use gear and leg position): x, y, z and 6 angles
    // not sure about alpha and zeta angles
    { true, 0, 0 }                                                                        // FOOT SENSOR: is on ground state, pin, threshold for sensor
  },
  // right leg
  {
    { LEGR, "RIGHT" },
    {  LEG_BODY_X,  LEG_BODY_Y,   LEG_BODY_Z },
    { LEG_POINT_X, LEG_POINT_Y, -LEG_POINT_Z },
    { LEG_SIZE_L1, LEG_SIZE_L2,  LEG_SIZE_L3 },
    {    0,    0,    0,    0,    0,    0 },
    { M_PI, M_PI, M_PI, M_PI, M_PI, M_PI },
    { M_PI_2, M_PI_2, M_PI - M_PI_4, M_PI_2, M_PI - M_PI_4, M_PI_2 },
    {
      { M_PI_2, M_PI_2, M_PI_2, M_PI_2, M_PI_2, M_PI_2 },
      {     15,     14,     13,     12,     11,     10 },
      {      0,      0,      0,      0,      0,      0 },
      {      1,      1,      1,      1,      1,      1 }
    },
    {  LEG_BODY_X,  LEG_BODY_Y,   LEG_BODY_Z },
    { LEG_POINT_X, LEG_POINT_Y, -LEG_POINT_Z },
    { M_PI_2, M_PI_2, M_PI_4, M_PI_2, M_PI_4, M_PI_2 },
    { false, true, false, true, false, true, false, true, true },
    { true, 0, 0 }
  }
};

servoProfile servoMainProfile = {
    0,    // minAngle
    180,  // maxAngle

    590,  // degMin
    870,  // deg30
    1080, // deg50
    1290, // deg70
    1500, // deg90 - middle
    1710, // deg110
    1890, // deg130
    2100, // deg150
    2200  // degMax
};

const GaitParams GAIT_PARAMS = {
	true,     // enabled
	4.0,      // cycle time, s
	0.75,     // duty factor: each leg swings 1s of every 4s
	{ 0.75, 0.25, 0, 0, 0, 0 },   // phase offsets LEFT, RIGHT
	40,       // step height, mm
	15,       // max speed, mm/s
	0.15,     // max yaw rate, rad/s
	30,       // max acceleration, mm/s^2
	0.3,      // max yaw acceleration, rad/s^2
	3,        // idle tolerance, mm
	false     // no foot contact sensors
};

#endif
