void cliHelp()
{
  for (int i = 0; i < CLI_COMMANDS; i++) {
    cliSerial->printf(" - [%-8s] %s\n", cliCommands[i].commandName, cliCommands[i].description);
  }
  for (int i = 0; i < MODEL_CLI_MENU_COMMANDS; i++) {
    cliSerial->printf(" - [%-8s] %s\n",  modelCliCommands[i].commandName, modelCliCommands[i].description);
  }
}

/**
 * @TODO cliRunServoCalibrate and/or setMotorsToCalibratePosition should disable HAL
 */
void cliCalibrate()
{
	char* mode = CLI_readWord();
	if (strcmp(mode, TITLE_IMU) == 0) {
		calibrateIMU();
		return;
	}
	if (strcmp(mode, TITLE_LEG) == 0) {
		setMotorsToCalibratePosition();	// all servos to init robot position
		return;
	}
	
	Serial.println("Unknown calibration hardware");
}

int getAngleIdByAngleTitle(char* angleTitle)
{
	if (strcmp(angleTitle, TITLE_ALPHA) == 0) {
		return ALPHA;
	}
	if (strcmp(angleTitle, TITLE_BETA) == 0) {
		return BETA;
	}
	if (strcmp(angleTitle, TITLE_GAMMA) == 0) {
		return GAMMA;
	}
	#if LEG_DOF == 6
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
	double angle    = degToRad(CLI_readFloat());

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
				case GAMMA:
					legs[i].angle.gamma = angle;
					break;
				#if LEG_DOF == 6
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
	double trimValueRad = degToRad(CLI_readFloat());
	
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
				case GAMMA:
					legs[i].hal.trim.gamma = trimValueRad;
					return;
				#if LEG_DOF == 6
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
