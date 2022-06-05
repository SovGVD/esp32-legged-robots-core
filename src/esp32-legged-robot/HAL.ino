void initHAL()
{
  Serial.print("HAL ");
  initServo();
  initLegs();
  Serial.println();
}

void initLegs() {

}

void updateHAL() {
  updateLegs();
}

void doHAL() {
  servoSet();
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
	Serial.print("HAL: ");
	if (isHALEnabled()) {
		Serial.println(TITLE_ON);
	} else {
		Serial.println(TITLE_OFF);
	}
}
