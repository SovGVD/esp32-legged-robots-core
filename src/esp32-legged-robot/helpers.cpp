#include "robot.h"

float radToDeg(float angleRad)
{
  return angleRad*RADTODEG;
}

float degToRad(float angleDeg)
{
  return angleDeg*DEGTORAD;
}

float mapf(float val, float in_min, float in_max, float out_min, float out_max) {
  return (val - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
