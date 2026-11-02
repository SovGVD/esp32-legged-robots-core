#include "robot.h"

bool HALEnabled = true;

void initHAL()
{
	Serial.println("HAL");
}

void doHAL() {
	if (isHALEnabled()) {
		if (isHalDoReady) {
			if (servoReady()) {
				servoSet();
			}
			isHalDoReady = false;
		}
	}

	doMotors();
}

void setLegPWM(leg &_leg)
{
	lrLegPulses(_leg, LEG_DOF, servoMainProfile, servoPulse[_leg.id.id]);
}

void runServoCalibrate(leg &_leg)
{
	servoPulse[_leg.id.id][ALPHA] = servoMainProfile.deg90;
	servoPulse[_leg.id.id][BETA]  = servoMainProfile.deg90;
	#if LEG_DOF > 2
		servoPulse[_leg.id.id][GAMMA] = servoMainProfile.deg90;
	#endif
	#if LEG_DOF > 3
		servoPulse[_leg.id.id][DELTA]   = servoMainProfile.deg90;
		servoPulse[_leg.id.id][EPSILON] = servoMainProfile.deg90;
		servoPulse[_leg.id.id][ZETA]    = servoMainProfile.deg90;
	#endif
}

float limitServoAngle(float angle)
{
	return lrLimitServoAngle(angle);
}

float getHALAngle(float angle, float mid, float trimAngle, float gearRatio, bool inverse)
{
	return lrHalAngle(angle, mid, trimAngle, gearRatio, inverse);
}

bool isHALEnabled()
{
  // This not correct, we are not enbale/disable HAL, but just disable servo position calculations
  return HALEnabled;
}

void enableHAL()
{
	HALEnabled = true;
}

void disableHAL()
{
	HALEnabled = false;
}
