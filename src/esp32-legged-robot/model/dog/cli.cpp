#define MODEL_CLI_MENU_COMMANDS 1

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
	
	return UNKNOWN_ANGLE;
}

void modelCliSetTrim()
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
			}
		}
	}
	
	cliSerial->println("Leg is not valid.");
	printAvailableLegs();
}

const cliCommand modelCliCommands[MODEL_CLI_MENU_COMMANDS] = {
  { "trim", modelCliSetTrim, "Trim servos", 3 }
};
