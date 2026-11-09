#include "robot.h"

static int getAngleIdByAngleTitle(char* angleTitle);

/**
 * @TODO cliRunServoCalibrate and/or setMotorsToCalibratePosition should disable HAL
 */
void cliCalibrate()
{
	char* mode = CLI_readWord();
	if (strcmp(mode, TITLE_IMU) == 0) {
		if (!imu().ready()) {
			cliSerial->println("No IMU.");
			return;
		}
		cliSerial->println("IMU: calibrating in 5 s, keep the robot still on a flat level surface.");
		calibrateIMU();
		cliSerial->println("IMU calibrated.");
		return;
	}
	if (strcmp(mode, TITLE_LEG) == 0) {
		setMotorsToCalibratePosition();	// all servos to init robot position
		cliSerial->println("Servos at the calibration pose (`hal off` first, or the gait moves them back).");
		return;
	}
	
	cliSerial->println("Usage: calib imu|leg");
}

static int getAngleIdByAngleTitle(char* angleTitle)
{
	if (strcmp(angleTitle, TITLE_ALPHA) == 0) {
		return ALPHA;
	}
	if (strcmp(angleTitle, TITLE_BETA) == 0) {
		return BETA;
	}
	#if LEG_DOF > 2
		if (strcmp(angleTitle, TITLE_GAMMA) == 0) {
			return GAMMA;
		}
	#endif
	#if LEG_DOF > 3
		if (strcmp(angleTitle, TITLE_DELTA) == 0) {
			return DELTA;
		}
		if (strcmp(angleTitle, TITLE_EPSILON) == 0) {
			return EPSILON;
		}
		if (strcmp(angleTitle, TITLE_ZETA) == 0) {
			return ZETA;
		}
	#endif
	
	return UNKNOWN_ANGLE;
}

/**
 * @TODO cliSerial cant work like that on different cores =(
 * 
 */
void cliPwmServo()
{
	char*    legTitle = CLI_readWord();
	char*    legAngle = CLI_readWord();
	uint16_t us       = CLI_readUInt16();

	int angleId = getAngleIdByAngleTitle(legAngle);

	if (angleId == UNKNOWN_ANGLE) {
		cliSerial->printf("Unknown angle [%s]. \n", legAngle);
		return;
	}

	for (uint8_t i = 0; i < LEG_NUM; i++) {
		if (strcmp(legs[i].id.title, legTitle) == 0) {
			servoPulse[i][angleId] = us;
			return;
		}
	}

	cliSerial->println("Leg is not valid.");
	printAvailableLegs();
}

void cliAngleServo()
{
	char*  legTitle = CLI_readWord();
	char*  legAngle = CLI_readWord();
	float angle    = degToRad(CLI_readFloat());

	int angleId = getAngleIdByAngleTitle(legAngle);

	if (angleId == UNKNOWN_ANGLE) {
		cliSerial->printf("Unknown angle [%s]. \n", legAngle);
		return;
	}

	for (uint8_t i = 0; i < LEG_NUM; i++) {
		if (strcmp(legs[i].id.title, legTitle) == 0) {
			switch (angleId) {
				case ALPHA:
					legs[i].angle.alpha = angle;
					break;
				case BETA:
					legs[i].angle.beta  = angle;
					break;
				#if LEG_DOF > 2
					case GAMMA:
						legs[i].angle.gamma = angle;
						break;
				#endif
				#if LEG_DOF > 3
					case DELTA:
						legs[i].angle.delta   = angle;
						break;
					case EPSILON:
						legs[i].angle.epsilon = angle;
						break;
					case ZETA:
						legs[i].angle.zeta    = angle;
						break;
				#endif
			}
			setLegPWM(legs[i]);

			return;
		}
	}

	cliSerial->println("Leg is not valid.");
	printAvailableLegs();
}

void cliSetTrim()
{
	char*  legTitle     = CLI_readWord();
	char*  legAngle     = CLI_readWord();
	float trimValueRad = degToRad(CLI_readFloat());
	
	int angleId = getAngleIdByAngleTitle(legAngle);
	
	if (angleId == UNKNOWN_ANGLE) {
		cliSerial->printf("Unknown angle [%s]. \n", legAngle);
		return;
	}
	cliSerial->printf("Angle %f rad, %f deg.\n", trimValueRad, radToDeg(trimValueRad));
	
	for (uint8_t i = 0; i < LEG_NUM; i++) {
		if (strcmp(legs[i].id.title, legTitle) == 0) {
			switch (angleId) {
				case ALPHA:
					legs[i].hal.trim.alpha = trimValueRad;
					return;
				case BETA:
					legs[i].hal.trim.beta  = trimValueRad;
					return;
				#if LEG_DOF > 2
					case GAMMA:
						legs[i].hal.trim.gamma = trimValueRad;
						return;
				#endif
				#if LEG_DOF > 3
					case DELTA:
						legs[i].hal.trim.delta   = trimValueRad;
						return;
					case EPSILON:
						legs[i].hal.trim.epsilon = trimValueRad;
						return;
					case ZETA:
						legs[i].hal.trim.zeta    = trimValueRad;
						return;
				#endif
			}
		}
	}
	
	cliSerial->println("Leg is not valid.");
	printAvailableLegs();
}
