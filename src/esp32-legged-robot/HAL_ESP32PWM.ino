#if PWM_CONTROLLER_TYPE == ESP32PWM
int esp32pwmServos[LEG_NUM*LEG_DOF];

void initServoHAL() {
	Serial.print("ESP32_ISR ");
	Serial.println(ESP32_ISR_SERVO_VERSION);
	ESP32_ISR_Servos.useTimer(USE_ESP32_TIMER_NO);

	/**
	* Init servos
	*/
	for (uint8_t i = 0; i < LEG_NUM; i++) {
		esp32pwmServos[i*LEG_DOF+0] = ESP32_ISR_Servos.setupServo(legs[i].hal.pin.alpha, SERVO_MIN, SERVO_MAX);
		esp32pwmServos[i*LEG_DOF+1] = ESP32_ISR_Servos.setupServo(legs[i].hal.pin.beta,  SERVO_MIN, SERVO_MAX);
		#if LEF_DOF > 2
			esp32pwmServos[i*LEG_DOF+2] = ESP32_ISR_Servos.setupServo(legs[i].hal.pin.gamma, SERVO_MIN, SERVO_MAX);
		#endif
		#if LEG_DOF > 3
			esp32pwmServos[i*LEG_DOF+3] = ESP32_ISR_Servos.setupServo(legs[i].hal.pin.delta,   SERVO_MIN, SERVO_MAX);
			esp32pwmServos[i*LEG_DOF+4] = ESP32_ISR_Servos.setupServo(legs[i].hal.pin.epsilon, SERVO_MIN, SERVO_MAX);
			esp32pwmServos[i*LEG_DOF+5] = ESP32_ISR_Servos.setupServo(legs[i].hal.pin.zeta,    SERVO_MIN, SERVO_MAX);
		#endif
	}
}

void doLeg(leg &_leg)
{
	uint8_t l = _leg.id.id*LEG_DOF;
	ESP32_ISR_Servos.setPulseWidth(esp32pwmServos[l+0], servoPulse[_leg.id.id][ALPHA]);
	ESP32_ISR_Servos.setPulseWidth(esp32pwmServos[l+1], servoPulse[_leg.id.id][BETA]);
	#if LEG_DOF > 2
		ESP32_ISR_Servos.setPulseWidth(esp32pwmServos[l+2], servoPulse[_leg.id.id][GAMMA]);
	#endif
	#if LEG_DOF > 3
		ESP32_ISR_Servos.setPulseWidth(esp32pwmServos[l+3], servoPulse[_leg.id.id][DELTA]);
		ESP32_ISR_Servos.setPulseWidth(esp32pwmServos[l+4], servoPulse[_leg.id.id][EPSILON]);
		ESP32_ISR_Servos.setPulseWidth(esp32pwmServos[l+5], servoPulse[_leg.id.id][ZETA]);
	#endif
}

void disableServos()
{
	ESP32_ISR_Servos.disableAll();
}

void enableServos()
{
	ESP32_ISR_Servos.enableAll();
}
#endif
