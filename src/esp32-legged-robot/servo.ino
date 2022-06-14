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
