#if PWM_CONTROLLER_TYPE == DCMOTOR

#define ESP32_DCMOTOR_FREQ    30000
#define ESP32_DCMOTOR_BIT     8

void initServoHAL() {
	Serial.println("ESP32 ledc");
	for (uint8_t i = 0; i < LEG_NUM; i++) {
		int channel = i * LEG_DOF;

		ledcSetup(channel+0, ESP32_DCMOTOR_FREQ, ESP32_DCMOTOR_BIT);
		ledcAttachPin(legs[i].hal.pin.alpha, channel+0);

		ledcSetup(channel+1, ESP32_DCMOTOR_FREQ, ESP32_DCMOTOR_BIT);
		ledcAttachPin(legs[i].hal.pin.beta,  channel+1);
		#if LEG_DOF > 2
			#error "Not supported or never tested"
		#endif
	}
}

void doLeg(leg &_leg)
{
	int channel = _leg.id.id * LEG_DOF;
	ledcWrite(channel+0, (int)servoPulse[_leg.id.id][ALPHA]);
	ledcWrite(channel+1, (int)servoPulse[_leg.id.id][BETA]);
}

void disableServos()
{
	// @TODO
}

void enableServos()
{
	// @TODO
}
#endif
