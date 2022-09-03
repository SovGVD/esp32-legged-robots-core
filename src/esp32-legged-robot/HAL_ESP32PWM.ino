#if PWM_CONTROLLER_TYPE == ESP32PWM
int esp32pwmServos[LEG_NUM*LEG_DOF];
uint16_t esp32pwmPulse = 0;

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
		esp32pwmServos[i*LEG_DOF+2] = ESP32_ISR_Servos.setupServo(legs[i].hal.pin.gamma, SERVO_MIN, SERVO_MAX);
		#if LEG_DOF == 6
			esp32pwmServos[i*LEG_DOF+3] = ESP32_ISR_Servos.setupServo(legs[i].hal.pin.delta,   SERVO_MIN, SERVO_MAX);
			esp32pwmServos[i*LEG_DOF+4] = ESP32_ISR_Servos.setupServo(legs[i].hal.pin.epsilon, SERVO_MIN, SERVO_MAX);
			esp32pwmServos[i*LEG_DOF+5] = ESP32_ISR_Servos.setupServo(legs[i].hal.pin.zeta,    SERVO_MIN, SERVO_MAX);
		#endif
	}
}

void setLegPWM(leg &_leg)
{
	uint8_t l = _leg.id.id*LEG_DOF;
	esp32pwmPulse = angleToPulse(limitServoAngle(getHALAngle(_leg.angle.alpha, _leg.hal.mid.alpha, _leg.hal.trim.alpha, _leg.hal.ratio.alpha, _leg.inverse.alpha)));
	ESP32_ISR_Servos.setPulseWidth(esp32pwmServos[l+0], esp32pwmPulse);
	
	esp32pwmPulse = angleToPulse(limitServoAngle(getHALAngle(_leg.angle.beta,  _leg.hal.mid.beta,  _leg.hal.trim.beta,  _leg.hal.ratio.beta,  _leg.inverse.beta )));
	ESP32_ISR_Servos.setPulseWidth(esp32pwmServos[l+1], esp32pwmPulse);

	esp32pwmPulse = angleToPulse(limitServoAngle(getHALAngle(_leg.angle.gamma, _leg.hal.mid.gamma, _leg.hal.trim.gamma, _leg.hal.ratio.gamma, _leg.inverse.gamma)));
	ESP32_ISR_Servos.setPulseWidth(esp32pwmServos[l+2], esp32pwmPulse);
	#if LEG_DOF == 6
		esp32pwmPulse = angleToPulse(limitServoAngle(getHALAngle(_leg.angle.delta, _leg.hal.mid.delta, _leg.hal.trim.delta, _leg.hal.ratio.delta, _leg.inverse.delta)));
		ESP32_ISR_Servos.setPulseWidth(esp32pwmServos[l+3], esp32pwmPulse);

		esp32pwmPulse = angleToPulse(limitServoAngle(getHALAngle(_leg.angle.epsilon, _leg.hal.mid.epsilon, _leg.hal.trim.epsilon, _leg.hal.ratio.epsilon, _leg.inverse.epsilon)));
		ESP32_ISR_Servos.setPulseWidth(esp32pwmServos[l+4], esp32pwmPulse);

		esp32pwmPulse = angleToPulse(limitServoAngle(getHALAngle(_leg.angle.zeta, _leg.hal.mid.zeta, _leg.hal.trim.zeta, _leg.hal.ratio.zeta, _leg.inverse.zeta)));
		ESP32_ISR_Servos.setPulseWidth(esp32pwmServos[l+5], esp32pwmPulse);
	#endif
}

void runServoCalibrate(leg &_leg)
{
	uint8_t l = _leg.id.id*LEG_DOF;
	for (uint8_t i = 0; i < LEG_DOF; i++) {
		ESP32_ISR_Servos.setPulseWidth(esp32pwmServos[l+i], servoMainProfile.deg90);
	}
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
