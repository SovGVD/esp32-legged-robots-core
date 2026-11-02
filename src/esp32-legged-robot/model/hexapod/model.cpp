/**
 * Robot description data for the hexapod model.
 * Compile time configuration (pins, sizes, hardware) lives in config.h.
 */
#include "../../robot.h"

#if ROBOT_MODEL == MODEL_HEXAPOD

leg legs[LEG_NUM] = {
  // left front leg
  {
    { LEGLF, "LF" },                                              // ID: id, title
    {   -LEG_BODY_X,     LEG_BODY_Y,     LEG_BODY_Z },            // default LEG POSITION ON BODY
    {   -LEG_FOOT_X,     LEG_FOOT_Y,    -LEG_FOOT_Z },            // default LEG ON GROUND: x, y ,z
    { LEG_SIZE_COXA, LEG_SIZE_FEMUR, LEG_SIZE_TIBIA },            // Leg part sizes
    {             0,              0,              0 },            // MIN ANGLES FOR EACH JOINT: @TODO this is not in use
    {          M_PI,           M_PI,           M_PI },            // MAX ANGLES FOR EACH JOINT: @TODO this is not in use
    {        M_PI_2,         M_PI_2,         M_PI_2 },            // Calibration position
    {
      { M_PI_2, M_PI_2, M_PI_2 },                                 // hardware middle (looks like never used, or not?)
      {      0,      1,      2 },                                 // 3 servo channels on PCA board, if this is second board, then pin + 16
      {      0,      0,      0 },                                 // servo middle trim (this will be override anyway with setup trim... why do I need it???)
      {      2,      2,    1.5 }                                  // gear ratio, first and last one is 16 groves on servo and 24 groves on leg
    },
    {  -LEG_BODY_X, LEG_BODY_Y,  LEG_BODY_Z },                    // initial LEG POSITION ON BODY, same as LEG POSITION ON BODY, not sure why I need it
    {  -LEG_FOOT_X, LEG_FOOT_Y, -LEG_FOOT_Z },                    // initial LEG JOING CLOSED TO THE GROUND, same as above, but at least it make sense
    {       M_PI_2,     M_PI_2,      M_PI_2 },                    // leg angles, will be override by HAL update
    { true, true, true },                                         // inverse leg calculations (depends on servo, is servo use gear and leg position): x, y, z and 6 angles
    { true, 0, 0 }                                                // FOOT SENSOR: is on ground state, pin, threshold for sensor
  },
  // left middle leg
  {
    { LEGLM, "LM" },
    { -LEG_MIDDLE_BODY_X, LEG_MIDDLE_BODY_Y,  LEG_MIDDLE_BODY_Z },
    { -LEG_MIDDLE_FOOT_X, LEG_MIDDLE_FOOT_Y, -LEG_MIDDLE_FOOT_Z },
    {      LEG_SIZE_COXA,    LEG_SIZE_FEMUR,     LEG_SIZE_TIBIA },
    {                  0,                 0,                  0 },
    {               M_PI,              M_PI,               M_PI },
    {             M_PI_2,            M_PI_2,             M_PI_2 },
    {
      { M_PI_2, M_PI_2, M_PI_2 },
      {      4,      5,      6 },
      {      0,      0,      0 },
      {      2,      2,    1.5 }
    },
    {  -LEG_MIDDLE_BODY_X, LEG_MIDDLE_BODY_Y,  LEG_MIDDLE_BODY_Z },
    {  -LEG_MIDDLE_FOOT_X, LEG_MIDDLE_FOOT_Y, -LEG_MIDDLE_FOOT_Z },
    {              M_PI_2,            M_PI_2,             M_PI_2 },
    { true, true, true },
    { true, 0, 0 }
  },
  // left rear leg
  {
    { LEGLR, "LR" },
    {   -LEG_BODY_X,    -LEG_BODY_Y,     LEG_BODY_Z },
    {   -LEG_FOOT_X,    -LEG_FOOT_Y,    -LEG_FOOT_Z },
    { LEG_SIZE_COXA, LEG_SIZE_FEMUR, LEG_SIZE_TIBIA },
    {             0,              0,              0 },
    {          M_PI,           M_PI,           M_PI },
    {        M_PI_2,         M_PI_2,         M_PI_2 },
    {
      { M_PI_2, M_PI_2, M_PI_2 },
      {      8,      9,     10 },
      {      0,      0,      0 },
      {      2,      2,    1.5 }
    },
    { -LEG_BODY_X, LEG_BODY_Y,  LEG_BODY_Z },
    { -LEG_FOOT_X, LEG_FOOT_Y, -LEG_FOOT_Z },
    {       M_PI_2,    M_PI_2,      M_PI_2 },
    { true, true, true },
    { true, 0, 0 }
  },

  // right front leg
  {
    { LEGRF, "RF" },
    {    LEG_BODY_X,     LEG_BODY_Y,     LEG_BODY_Z },
    {    LEG_FOOT_X,     LEG_FOOT_Y,    -LEG_FOOT_Z },
    { LEG_SIZE_COXA, LEG_SIZE_FEMUR, LEG_SIZE_TIBIA },
    {             0,              0,              0 },
    {          M_PI,           M_PI,           M_PI },
    {        M_PI_2,         M_PI_2,         M_PI_2 },
    {
      { M_PI_2, M_PI_2, M_PI_2 },
      {     16,     17,     18 },
      {      0,      0,      0 },
      {      2,      2,    1.5 }
    },
    {  LEG_BODY_X, LEG_BODY_Y,  LEG_BODY_Z },
    {  LEG_FOOT_X, LEG_FOOT_Y, -LEG_FOOT_Z },
    {      M_PI_2,     M_PI_2,      M_PI_2 },
    { true, true, true },
    { true, 0, 0 }
  },
  // right middle leg
  {
    { LEGRM, "RM" },
    { LEG_MIDDLE_BODY_X, LEG_MIDDLE_BODY_Y,  LEG_MIDDLE_BODY_Z },
    { LEG_MIDDLE_FOOT_X, LEG_MIDDLE_FOOT_Y, -LEG_MIDDLE_FOOT_Z },
    {     LEG_SIZE_COXA,    LEG_SIZE_FEMUR,     LEG_SIZE_TIBIA },
    {                 0,                 0,                  0 },
    {              M_PI,              M_PI,               M_PI },
    {            M_PI_2,            M_PI_2,             M_PI_2 },
    {
      { M_PI_2, M_PI_2, M_PI_2 },
      {     20,     21,     22 },
      {      0,      0,      0 },
      {      2,      2,    1.5 }
    },
    {  LEG_MIDDLE_BODY_X, LEG_MIDDLE_BODY_Y,  LEG_MIDDLE_BODY_Z },
    {  LEG_MIDDLE_FOOT_X, LEG_MIDDLE_FOOT_Y, -LEG_MIDDLE_FOOT_Z },
    {             M_PI_2,            M_PI_2,             M_PI_2 },
    { true, true, true },
    { true, 0, 0 }
  },
  // right rear leg
  {
    { LEGRR, "RR" },
    {    LEG_BODY_X,    -LEG_BODY_Y,     LEG_BODY_Z },
    {    LEG_FOOT_X,    -LEG_FOOT_Y,    -LEG_FOOT_Z },
    { LEG_SIZE_COXA, LEG_SIZE_FEMUR, LEG_SIZE_TIBIA },
    {             0,              0,              0 },
    {          M_PI,           M_PI,           M_PI },
    {        M_PI_2,         M_PI_2,         M_PI_2 },
    {
      { M_PI_2, M_PI_2, M_PI_2 },
      {     24,     25,     26 },                                 // second board, channels 8-10 (mirrors left rear)
      {      0,      0,      0 },
      {      2,      2,    1.5 }
    },
    { LEG_BODY_X, LEG_BODY_Y,  LEG_BODY_Z },
    { LEG_FOOT_X, LEG_FOOT_Y, -LEG_FOOT_Z },
    {      M_PI_2,    M_PI_2,      M_PI_2 },
    { true, true, true },
    { true, 0, 0 }
  },
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
	0.4,      // cycle time, s
	0.75,     // duty factor
	{ 0.5, 0.0, 0.5, 0.0, 0.5, 0.0 },   // phase offsets LF, LM, LR, RF, RM, RR (LF, LR, RM swing together)
	30,       // step height, mm
	150,      // max speed, mm/s
	1.5,      // max yaw rate, rad/s
	600,      // max acceleration, mm/s^2
	6,        // max yaw acceleration, rad/s^2
	3,        // idle tolerance, mm
	false     // no foot contact sensors
};

static HexapodLegIK modelLegIK;
LegIK &MODEL_IK = modelLegIK;

const cliCommand *modelCliCommands     = nullptr;
const uint8_t     modelCliCommandsCount = 0;

void modelSettingsPrint()
{
}

void modelSettingsLoad()
{
}

void modelSettingsSave()
{
}

#endif
