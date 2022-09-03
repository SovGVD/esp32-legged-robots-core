#if PWM_CONTROLLER_TYPE == PCA9685
uint16_t servoPulse = 0;

void initServoHAL() {
	pwm = Adafruit_PWMServoDriver(PWM_CONTROLLER_ADDRESS, Wire);
	pwm.begin();
	pwm.setOscillatorFrequency(27000000);  // The int.osc. is closer to 27MHz
	pwm.setPWMFreq(SERVO_FREQ);  // This is the maximum PWM frequency of servo
	enableServos();
}

void setLegPWM(leg &_leg)
{
	servoPulse = angleToPulse(limitServoAngle(getHALAngle(_leg.angle.alpha, _leg.hal.mid.alpha, _leg.hal.trim.alpha, _leg.hal.ratio.alpha, _leg.inverse.alpha)));
	pwm.writeMicroseconds(_leg.hal.pin.alpha, servoPulse);

	servoPulse = angleToPulse(limitServoAngle(getHALAngle(_leg.angle.beta,  _leg.hal.mid.beta,  _leg.hal.trim.beta,  _leg.hal.ratio.beta,  _leg.inverse.beta)));
	pwm.writeMicroseconds(_leg.hal.pin.beta,  servoPulse);

	servoPulse = angleToPulse(limitServoAngle(getHALAngle(_leg.angle.gamma, _leg.hal.mid.gamma, _leg.hal.trim.gamma, _leg.hal.ratio.gamma, _leg.inverse.gamma)));
	pwm.writeMicroseconds(_leg.hal.pin.gamma, servoPulse);

	#if LEG_DOF == 6
		servoPulse = angleToPulse(limitServoAngle(getHALAngle(_leg.angle.delta, _leg.hal.mid.delta, _leg.hal.trim.delta, _leg.hal.ratio.delta, _leg.inverse.delta)));
		pwm.writeMicroseconds(_leg.hal.pin.delta, servoPulse);

		servoPulse = angleToPulse(limitServoAngle(getHALAngle(_leg.angle.epsilon, _leg.hal.mid.epsilon, _leg.hal.trim.epsilon, _leg.hal.ratio.epsilon, _leg.inverse.epsilon)));
		pwm.writeMicroseconds(_leg.hal.pin.epsilon, servoPulse);

		servoPulse = angleToPulse(limitServoAngle(getHALAngle(_leg.angle.zeta, _leg.hal.mid.zeta, _leg.hal.trim.zeta, _leg.hal.ratio.zeta, _leg.inverse.zeta)));
		pwm.writeMicroseconds(_leg.hal.pin.zeta, servoPulse);
	#endif
}

void runServoCalibrate(leg &_leg)
{
	pwm.writeMicroseconds(_leg.hal.pin.alpha, servoMainProfile.deg90);
	pwm.writeMicroseconds(_leg.hal.pin.beta,  servoMainProfile.deg90);
	pwm.writeMicroseconds(_leg.hal.pin.gamma, servoMainProfile.deg90);
	#if LEG_DOF == 6
		pwm.writeMicroseconds(_leg.hal.pin.delta,   servoMainProfile.deg90);
		pwm.writeMicroseconds(_leg.hal.pin.epsilon, servoMainProfile.deg90);
		pwm.writeMicroseconds(_leg.hal.pin.zeta,    servoMainProfile.deg90);
	#endif
}

void disableServos()
{
	pwm.sleep();
}

void enableServos()
{
	pwm.wakeup();
}
#endif
