LR_legAngle modelSettingsLoadTrimLeg(modelSettingsAngleTrim t)
{
	#ifndef LEG_DOF
		#error "LEG DOF required"
	#elif LEG_DOF == 1
		return {
			t.alpha
		};
	#elif LEG_DOF > 1
		return {
			t.alpha,
			t.beta,
			t.gamma
		};
	#elif LEG_DOF > 3
		return {
			t.alpha,
			t.beta,
			t.gamma,
			t.delta,
			t.epsilon,
			t.zeta
		};
	#else
		return {};
	#endif
}

void settingsLoadTrim()
{
	for (uint8_t i = 0; i < LEG_NUM; i++) {
		legs[i].hal.trim = modelSettingsLoadTrimLeg(settings.trim[i]);
	}
}

void settingsSaveTrim()
{
	int value = 0;
	for (uint8_t i = 0; i < LEG_NUM; i++) {
		settings.trim[i].alpha = legs[i].hal.trim.alpha;
		#if LEG_DOF > 1
			settings.trim[i].beta  = legs[i].hal.trim.beta;
			settings.trim[i].gamma = legs[i].hal.trim.gamma;
		#endif
		#if LEG_DOF > 3
			settings.trim[i].delta   = legs[i].hal.trim.delta;
			settings.trim[i].epsilon = legs[i].hal.trim.epsilon;
			settings.trim[i].zeta    = legs[i].hal.trim.zeta;
		#endif
	}
}

void cliHal()
{
	char* mode = CLI_readWord();
	if (strcmp(mode, TITLE_ON) == 0) {
		enableHAL();
	}
	if (strcmp(mode, TITLE_OFF) == 0) {
		disableHAL();
	}
	if (strcmp(mode, TITLE_STATE) == 0) {
	}

	cliDisplayState();
}

void cliDisplayState()
{
	cliSerial->print("HAL: ");
	if (isHALEnabled()) {
		cliSerial->println(TITLE_ON);
	} else {
		cliSerial->println(TITLE_OFF);
	}
}

void cliRunServoCalibrate()
{
	for (uint8_t i = 0; i < LEG_NUM; i++) {
		runServoCalibrate(legs[i]);
	}
}
