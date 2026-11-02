#include "robot.h"

#if PWM_CONTROLLER_TYPE == DCMOTOR

#define ESP32_DCMOTOR_FREQ    30000
#define ESP32_DCMOTOR_BIT     8

#if LEG_DOF > 2
	#error "Not supported or never tested"
#endif

void initServoHAL() {
	Serial.println("ESP32 ledc");
	for (uint8_t i = 0; i < LEG_NUM; i++) {
		// Arduino-ESP32 3.x LEDC API: channels are allocated automatically per pin
		ledcAttach(legs[i].hal.pin.alpha, ESP32_DCMOTOR_FREQ, ESP32_DCMOTOR_BIT);
		ledcAttach(legs[i].hal.pin.beta,  ESP32_DCMOTOR_FREQ, ESP32_DCMOTOR_BIT);
	}
}

void doLeg(leg &_leg)
{
	ledcWrite(_leg.hal.pin.alpha, servoPulse[_leg.id.id][ALPHA]);
	ledcWrite(_leg.hal.pin.beta,  servoPulse[_leg.id.id][BETA]);
}

void disableServos()
{
	// @TODO
}

void enableServos()
{
	// @TODO
}
#endif
