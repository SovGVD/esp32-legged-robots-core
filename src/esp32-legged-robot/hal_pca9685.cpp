#include "robot.h"

#if PWM_CONTROLLER_TYPE == PCA9685
#include <Adafruit_PWMServoDriver.h>

/**
 * One or two PCA9685 boards. A servo "pin" is the channel number: 0-15 is the
 * first board (PWM_CONTROLLER_ADDRESS), 16-31 the second one
 * (PWM_CONTROLLER_ADDRESS_2, only when defined).
 */
#ifdef PWM_CONTROLLER_ADDRESS_2
	#define PCA9685_BOARDS 2
	static Adafruit_PWMServoDriver pwmBoards[PCA9685_BOARDS] = {
		Adafruit_PWMServoDriver(PWM_CONTROLLER_ADDRESS, Wire),
		Adafruit_PWMServoDriver(PWM_CONTROLLER_ADDRESS_2, Wire)
	};
#else
	#define PCA9685_BOARDS 1
	static Adafruit_PWMServoDriver pwmBoards[PCA9685_BOARDS] = {
		Adafruit_PWMServoDriver(PWM_CONTROLLER_ADDRESS, Wire)
	};
#endif

static void writeChannel(uint8_t channel, uint16_t us)
{
	uint8_t board = channel / 16;
	if (board >= PCA9685_BOARDS) {
		return;   // reported by checkServoPins() at boot
	}
	pwmBoards[board].writeMicroseconds(channel % 16, us);
}

void initServoHAL() {
	Serial.printf("PCA9685 x%u ", PCA9685_BOARDS);
	for (uint8_t i = 0; i < PCA9685_BOARDS; i++) {
		pwmBoards[i].begin();
		pwmBoards[i].setOscillatorFrequency(27000000);  // The int.osc. is closer to 27MHz
		pwmBoards[i].setPWMFreq(SERVO_FREQ);  // This is the maximum PWM frequency of servo
	}
	enableServos();
}

void doLeg(leg &_leg)
{
	writeChannel(_leg.hal.pin.alpha, servoPulse[_leg.id.id][ALPHA]);
	writeChannel(_leg.hal.pin.beta,  servoPulse[_leg.id.id][BETA]);
	#if LEG_DOF > 2
		writeChannel(_leg.hal.pin.gamma, servoPulse[_leg.id.id][GAMMA]);
	#endif
	#if LEG_DOF > 3
		writeChannel(_leg.hal.pin.delta,   servoPulse[_leg.id.id][DELTA]);
		writeChannel(_leg.hal.pin.epsilon, servoPulse[_leg.id.id][EPSILON]);
		writeChannel(_leg.hal.pin.zeta,    servoPulse[_leg.id.id][ZETA]);
	#endif
}

void disableServos()
{
	for (uint8_t i = 0; i < PCA9685_BOARDS; i++) {
		pwmBoards[i].sleep();
	}
}

void enableServos()
{
	for (uint8_t i = 0; i < PCA9685_BOARDS; i++) {
		pwmBoards[i].wakeup();
	}
}

uint8_t servoChannelLimit()
{
	return PCA9685_BOARDS * 16;
}
#endif
