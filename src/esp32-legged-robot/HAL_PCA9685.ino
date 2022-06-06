#if PWM_CONTROLLER_TYPE == PCA9685

void initServoHAL() {
	pwm = Adafruit_PWMServoDriver();
	pwm.begin();
	pwm.setOscillatorFrequency(27000000);  // The int.osc. is closer to 27MHz
	pwm.setPWMFreq(SERVO_FREQ);  // This is the maximum PWM frequency of servo
}

uint16_t angleToPulse(double angleRad) {
	// TODO, angle expected to be between 0 and 180 deg (0 and PI)

	double pulse = mapf(angleRad, 0, M_PI, 600, 2400);
	// from Adafruit PWM lib
	double pulselength;
	pulselength = 1000000; // 1,000,000 us per second

	// Read prescale
	uint16_t prescale = pwm.readPrescale();

	// Calculate the pulse for PWM based on Equation 1 from the datasheet section
	// 7.3.5
	prescale += 1;
	pulselength *= prescale;
	pulselength /= pwm.getOscillatorFrequency();
	pulse /= pulselength;

	return pulse;
}

void setLegPWM(leg &_leg)
{
	pwm.setPWM(_leg.hal.pin.alpha,  0,  angleToPulse(limitServoAngle(getHALAngle(_leg.angle.alpha, _leg.hal.mid.alpha, _leg.hal.trim.alpha, _leg.hal.ratio.alpha, _leg.inverse.alpha))));
	pwm.setPWM(_leg.hal.pin.beta,   0,  angleToPulse(limitServoAngle(getHALAngle(_leg.angle.beta,  _leg.hal.mid.beta,  _leg.hal.trim.beta,  _leg.hal.ratio.beta,  _leg.inverse.beta ))));
	pwm.setPWM(_leg.hal.pin.gamma,  0,  angleToPulse(limitServoAngle(getHALAngle(_leg.angle.gamma, _leg.hal.mid.gamma, _leg.hal.trim.gamma, _leg.hal.ratio.gamma, _leg.inverse.gamma))));
	#if LEG_DOF == 6
		pwm.setPWM(_leg.hal.pin.delta,   0,  angleToPulse(limitServoAngle(getHALAngle(_leg.angle.delta,   _leg.hal.mid.delta,   _leg.hal.trim.delta,   _leg.hal.ratio.delta,  _leg.inverse.delta))));
		pwm.setPWM(_leg.hal.pin.epsilon, 0,  angleToPulse(limitServoAngle(getHALAngle(_leg.angle.epsilon, _leg.hal.mid.epsilon, _leg.hal.trim.epsilon, _leg.hal.ratio.epsilon, _leg.inverse.epsilon))));
		pwm.setPWM(_leg.hal.pin.zeta,    0,  angleToPulse(limitServoAngle(getHALAngle(_leg.angle.zeta,    _leg.hal.mid.zeta,    _leg.hal.trim.zeta,    _leg.hal.ratio.zeta,    _leg.inverse.zeta))));
	#endif
}

void runServoCalibrate(leg &_leg)
{
	pwm.setPWM(_leg.hal.pin.alpha, 0, servoMainProfile.deg90);
	pwm.setPWM(_leg.hal.pin.beta,  0, servoMainProfile.deg90);
	pwm.setPWM(_leg.hal.pin.gamma, 0, servoMainProfile.deg90);
	#if LEG_DOF == 6
		pwm.setPWM(_leg.hal.pin.delta,   0, servoMainProfile.deg90);
		pwm.setPWM(_leg.hal.pin.epsilon, 0, servoMainProfile.deg90);
		pwm.setPWM(_leg.hal.pin.zeta,    0, servoMainProfile.deg90);
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
