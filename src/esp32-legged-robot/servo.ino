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
 * @TODO this is for robot dog!!!
 * Set servo to init position
 *  it is 90, 45, 90
 */
void setServoToInit()
{
	for (uint8_t i = 0; i < LEG_NUM; i++) {
		legs[i].angle.alpha = M_PI_2;	// @TODO use model config
		legs[i].angle.beta  = M_PI_4;
		legs[i].angle.gamma = M_PI_2;
		#if LEG_DOF == 6
			legs[i].angle.delta    = M_PI_2;	// @TODO use model config
			legs[i].angle.epsilon  = M_PI_4;
			legs[i].angle.zeta     = M_PI_2;
		#endif
		setLegPWM(legs[i]);
	}
}

void servoSet()
{
	for (uint8_t i = 0; i < LEG_NUM; i++) {
		setLegPWM(legs[i]);
	}
}

bool setAngleDeg(leg &_leg, int angleId, double deg)
{
	// TODO limits?
	double rad = degToRad(deg);
	switch (angleId) {
		case ALPHA:
			_leg.angle.alpha = rad;
			break;
		case BETA:
			_leg.angle.beta  = rad;
			break;
		case GAMMA:
			_leg.angle.gamma = rad;
			break;
		#if LEG_DOF == 6
			case DELTA:
				_leg.angle.delta = rad;
				break;
			case EPSILON:
				_leg.angle.epsilon  = rad;
				break;
			case ZETA:
				_leg.angle.zeta = rad;
				break;
		#endif
	}

	return true;
}

double getAngleDeg(leg &_leg, int angleId)
{
	switch (angleId) {
		case ALPHA:
			return radToDeg(_leg.angle.alpha);
			break;
		case BETA:
			return radToDeg(_leg.angle.beta);
			break;
		case GAMMA:
			return radToDeg(_leg.angle.gamma);
			break;
		#if LEG_DOF == 6
			case DELTA:
				return radToDeg(_leg.angle.delta);
				break;
			case EPSILON:
				return radToDeg(_leg.angle.epsilon);
				break;
			case ZETA:
				return radToDeg(_leg.angle.zeta);
				break;
		#endif
	}

	return 0.0;
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
