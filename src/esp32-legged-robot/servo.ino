float testAngle = 0;
float testAngleInc = 0.5;
bool testAngleWay = true;
bool isServoReady = false;

void initServo()
{
	if (xPortGetCoreID() != SERVO_CORE) {
		return;
	}

	Serial.print("Servo on core");
	Serial.print(xPortGetCoreID());
	Serial.print(" ");
	initServoHAL();
	Serial.println();
	isServoReady = true;
}

/**
 * Set servo to init robot calibration position
 */
void setMotorsToCalibratePosition()
{
	for (uint8_t i = 0; i < LEG_NUM; i++) {
		legs[i].angle.alpha = legs[i].calibrate.alpha;
		legs[i].angle.beta  = legs[i].calibrate.beta;
		legs[i].angle.gamma = legs[i].calibrate.gamma;
		#if LEG_DOF == 6
			legs[i].angle.delta    = legs[i].calibrate.delta;
			legs[i].angle.epsilon  = legs[i].calibrate.epsilon;
			legs[i].angle.zeta     = legs[i].calibrate.zeta;
		#endif
		setLegPWM(legs[i]);
		doLeg(legs[i]);
	}
}

void servoSet()
{
	for (uint8_t i = 0; i < LEG_NUM; i++) {
		setLegPWM(legs[i]);
	}
}

void doMotors()
{
  for (uint8_t i = 0; i < LEG_NUM; i++) {
    doLeg(legs[i]);
  }
}

bool servoReady()
{
	return isServoReady;
}

uint16_t angleToPulse(double angleRad) {
  double angleDeg = radToDeg(angleRad);

  if (angleDeg < servoMainProfile.minAngle) angleDeg = servoMainProfile.minAngle;
  if (angleDeg > servoMainProfile.maxAngle) angleDeg = servoMainProfile.maxAngle;

  // TODO how to make it better???

  if (angleDeg < 30) {
    return mapf(angleDeg, servoMainProfile.minAngle, 30, servoMainProfile.degMin, servoMainProfile.deg30);
  }

  if (angleDeg < 50) {
    return mapf(angleDeg, 30, 50, servoMainProfile.deg30, servoMainProfile.deg50);
  }

  if (angleDeg < 70) {
    return mapf(angleDeg, 50, 70, servoMainProfile.deg50, servoMainProfile.deg70);
  }

  if (angleDeg < 90) {
    return mapf(angleDeg, 70, 90, servoMainProfile.deg70, servoMainProfile.deg90);
  }

  if (angleDeg < 110) {
    return mapf(angleDeg, 90, 110, servoMainProfile.deg90, servoMainProfile.deg110);
  }

  if (angleDeg < 130) {
    return mapf(angleDeg, 110, 130, servoMainProfile.deg110, servoMainProfile.deg130);
  }

  if (angleDeg < 150) {
    return mapf(angleDeg, 130, 150, servoMainProfile.deg130, servoMainProfile.deg150);
  }

  if (angleDeg <= servoMainProfile.maxAngle) {
    return mapf(angleDeg, 150, servoMainProfile.maxAngle, servoMainProfile.deg150, servoMainProfile.degMax);
  }


  return 1500;  // TODO actualy we should fail here...
}
