#if PWM_CONTROLLER_TYPE == PCA9685

void initServoHAL() {
	pwm = Adafruit_PWMServoDriver(PWM_CONTROLLER_ADDRESS, Wire);
	pwm.begin();
	pwm.setOscillatorFrequency(27000000);  // The int.osc. is closer to 27MHz
	pwm.setPWMFreq(SERVO_FREQ);  // This is the maximum PWM frequency of servo
	enableServos();
}

void doLeg(leg &_leg)
{
	pwm.writeMicroseconds(_leg.hal.pin.alpha, servoPulse[_leg.id.id][ALPHA]);
	pwm.writeMicroseconds(_leg.hal.pin.beta,  servoPulse[_leg.id.id][BETA]);
	#if LEG_DOF > 2
		pwm.writeMicroseconds(_leg.hal.pin.gamma, servoPulse[_leg.id.id][GAMMA]);
	#endif
	#if LEG_DOF > 3
		pwm.writeMicroseconds(_leg.hal.pin.delta,   servoPulse[_leg.id.id][DELTA]);
		pwm.writeMicroseconds(_leg.hal.pin.epsilon, servoPulse[_leg.id.id][EPSILON]);
		pwm.writeMicroseconds(_leg.hal.pin.zeta,    servoPulse[_leg.id.id][ZETA]);
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
