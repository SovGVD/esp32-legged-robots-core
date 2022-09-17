void initHAL()
{
	Serial.print("HAL ");
	initLegs();
	Serial.println();
}

void initLegs() {

}

void updateHAL() {
	updateLegs();
}

void doHAL() {
	if (isHALEnabled()) {
		if (isHalDoReady) {
			if (servoReady()) {
				servoSet();
			}
			isHalDoReady = false;
		}
	}

	doMotors();
}

void readLegsSensors() {
  // TODO that will be i2c ADC, not analog read
  // TODO enable if sensors available
  // TODO don't set to legs, this should be part of gait
  for (uint8_t i = 0; i < LEG_NUM; i++) {
    //legs[i].sensor.onGround = analogRead(legs[i].sensor.pin) > legs[i].sensor.threshold;
  }
}

void updateLegs() {
	if (!isHALEnabled()) return;

	readLegsSensors();
	for (uint8_t i = 0; i < LEG_NUM; i++) {
		legs[i].angle = ikLeg.solve(i).angle;
	}
}

void setLegPWM(leg &_leg)
{
	servoPulse[_leg.id.id][ALPHA] = angleToPulse(limitServoAngle(getHALAngle(_leg.angle.alpha, _leg.hal.mid.alpha, _leg.hal.trim.alpha, _leg.hal.ratio.alpha, _leg.inverse.alpha)));
	#if LED_DOF > 1
		servoPulse[_leg.id.id][BETA]  = angleToPulse(limitServoAngle(getHALAngle(_leg.angle.beta,  _leg.hal.mid.beta,  _leg.hal.trim.beta,  _leg.hal.ratio.beta,  _leg.inverse.beta)));
		servoPulse[_leg.id.id][GAMMA] = angleToPulse(limitServoAngle(getHALAngle(_leg.angle.gamma, _leg.hal.mid.gamma, _leg.hal.trim.gamma, _leg.hal.ratio.gamma, _leg.inverse.gamma)));
	#endif
	#if LEG_DOF > 3
		servoPulse[_leg.id.id][DELTA]   = angleToPulse(limitServoAngle(getHALAngle(_leg.angle.delta,   _leg.hal.mid.delta,   _leg.hal.trim.delta,   _leg.hal.ratio.delta,   _leg.inverse.delta)));
		servoPulse[_leg.id.id][EPSILON] = angleToPulse(limitServoAngle(getHALAngle(_leg.angle.epsilon, _leg.hal.mid.epsilon, _leg.hal.trim.epsilon, _leg.hal.ratio.epsilon, _leg.inverse.epsilon)));
		servoPulse[_leg.id.id][ZETA]    = angleToPulse(limitServoAngle(getHALAngle(_leg.angle.zeta,    _leg.hal.mid.zeta,    _leg.hal.trim.zeta,    _leg.hal.ratio.zeta,    _leg.inverse.zeta)));
	#endif
}

void runServoCalibrate(leg &_leg)
{
	servoPulse[_leg.id.id][ALPHA] = servoMainProfile.deg90;
	#if LEG_DOF > 1
		servoPulse[_leg.id.id][BETA]  = servoMainProfile.deg90;
		servoPulse[_leg.id.id][GAMMA] = servoMainProfile.deg90;
	#endif
	#if LEG_DOF > 3
		servoPulse[_leg.id.id][DELTA]   = servoMainProfile.deg90;
		servoPulse[_leg.id.id][EPSILON] = servoMainProfile.deg90;
		servoPulse[_leg.id.id][ZETA]    = servoMainProfile.deg90;
	#endif
}

double limitServoAngle(double angle)
{
  // TODO this is not right solution, we should restart calc to solve the angle instead of just limit if something wrong
  //      also this is nothing about real mechanical limits, so need TODO
  
  // default servo can handle angles from 0 to 180 (PI) degrees only
  if (angle < 0) angle = 0;
  if (angle > PI) angle = PI;

  return angle;
}

double getHALAngle(double angle, double mid, double trimAngle, double gearRatio, bool inverse) {
  angle = (angle - mid) * gearRatio + M_PI_2;
  if (inverse) angle = M_PI - angle;
  angle = angle + trimAngle;
  return angle;
}

bool isHALEnabled()
{
  // This not correct, we are not enbale/disable HAL, but just disable servo position calculations
  return HALEnabled;
}

void enableHAL()
{
	HALEnabled = true;
}

void disableHAL()
{
	HALEnabled = false;
}
