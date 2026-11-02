#include "robot.h"

float testAngle = 0;
float testAngleInc = 0.5;
bool testAngleWay = true;
bool isServoReady = false;

/**
 * Output index of a joint on the servo bus
 */
static inline uint8_t busIndex(uint8_t legId, uint8_t joint)
{
	return legId * LEG_DOF + joint;
}

/**
 * Sanity check of the pin map, printed at boot: duplicated pins, channels the
 * bus can't drive and pins the bus warns about (ESP32 strapping and input
 * only pins for LEDC).
 */
static uint8_t checkServoPins(const ServoBus &bus, const uint8_t *pins, uint8_t n)
{
	uint8_t problems = 0;
	for (uint8_t i = 0; i < n; i++) {
		for (uint8_t j = i + 1; j < n; j++) {
			if (pins[i] == pins[j]) {
				Serial.printf("\nWARNING: pin %u is used by output %u and %u", pins[i], i, j);
				problems++;
			}
		}
		if (pins[i] >= bus.channelLimit()) {
			Serial.printf("\nERROR: output %u uses channel %u, only %u available", i, pins[i], bus.channelLimit());
			problems++;
		} else if (const char *why = bus.checkChannel(pins[i])) {
			Serial.printf("\nWARNING: output %u uses pin %u: %s", i, pins[i], why);
			problems++;
		}
	}
	if (problems) {
		Serial.println();
	}
	return problems;
}

void initServo()
{
	if (xPortGetCoreID() != MOTOR_CORE) {
		return;
	}

	ServoBus &bus = servoBus();
	uint8_t pins[LEG_NUM * LEG_DOF];
	for (uint8_t i = 0; i < LEG_NUM; i++) {
		for (uint8_t j = 0; j < LEG_DOF; j++) {
			pins[busIndex(i, j)] = lrLegPin(legs[i], j);
		}
	}

	Serial.print("Servo on core");
	Serial.print(xPortGetCoreID());
	Serial.print(" ");
	Serial.print(bus.name());
	checkServoPins(bus, pins, LEG_NUM * LEG_DOF);
	bus.begin(pins, LEG_NUM * LEG_DOF);
	Serial.println();
	isServoReady = true;
}

void disableServos()
{
	servoBus().setEnabled(false);
}

void enableServos()
{
	servoBus().setEnabled(true);
}

/**
 * Set servo to init robot calibration position
 */
void setMotorsToCalibratePosition()
{
	for (uint8_t i = 0; i < LEG_NUM; i++) {
		legs[i].angle.alpha = legs[i].calibrate.alpha;
		legs[i].angle.beta  = legs[i].calibrate.beta;
		#if LEG_DOF > 2
			legs[i].angle.gamma = legs[i].calibrate.gamma;
		#endif
		#if LEG_DOF > 3
			legs[i].angle.delta    = legs[i].calibrate.delta;
			legs[i].angle.epsilon  = legs[i].calibrate.epsilon;
			legs[i].angle.zeta     = legs[i].calibrate.zeta;
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

void doMotors()
{
	ServoBus &bus = servoBus();
	for (uint8_t i = 0; i < LEG_NUM; i++) {
		for (uint8_t j = 0; j < LEG_DOF; j++) {
			bus.write(busIndex(i, j), servoPulse[i][j]);
		}
	}
}

bool servoReady()
{
	return isServoReady;
}

uint16_t angleToPulse(float angleRad)
{
	return lrAngleToPulse(servoMainProfile, angleRad);
}
