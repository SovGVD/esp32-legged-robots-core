/**
 * Foot contact switches on ESP32 GPIO pins (FOOT_SWITCH_PINS in the model's
 * config.h), one per leg in leg id order. Read every motion tick, debounced
 * (FootSwitches in lib/robot_core) and, while in use (`feet on`), handed to the
 * gait as legs[i].sensor.onGround (GaitParams.useContactSensors).
 *
 * Wiring: the switch's NO contact between the pin and GND, the pin's
 * internal pull-up: pressed reads LOW. FOOT_SWITCH_ACTIVE_HIGH flips it (NC
 * contact, or a pull-down). Not on strapping pins 0, 2, 5, 12, 15 (the robot
 * boots standing, switches closed) or input-only 34-39 (no pull-up).
 */
#include "robot.h"

#ifndef FOOT_SWITCH_DEBOUNCE
	#define FOOT_SWITCH_DEBOUNCE 2   // motion ticks in a row (~9 ms at 230 ticks/s)
#endif

static FootSwitches  feet;
static volatile bool feetUse =
#if defined(FOOT_SWITCH_PINS) && defined(FOOT_SWITCH_ON)
	true;
#else
	false;
#endif

#ifdef FOOT_SWITCH_PINS
static const uint8_t footPins[] = FOOT_SWITCH_PINS;
static const uint8_t footCount  = sizeof(footPins) / sizeof(footPins[0]);
static_assert(sizeof(footPins) / sizeof(footPins[0]) == LEG_NUM, "FOOT_SWITCH_PINS needs one pin per leg");
#else
static const uint8_t *const footPins = nullptr;
static const uint8_t footCount = 0;
#endif

static const char *footPinProblem(uint8_t pin)
{
	switch (pin) {
		case 0: case 2: case 5: case 12: case 15:
			return "strapping pin, the board may fail to boot with the foot pressed";
		case 34: case 35: case 36: case 37: case 38: case 39:
			return "input only pin, no internal pull-up: add an external 10k";
	}
	return nullptr;
}

static uint32_t readFeetRaw()
{
	uint32_t raw = 0;
	for (uint8_t i = 0; i < footCount; i++) {
		bool level = digitalRead(footPins[i]) == HIGH;
	#ifdef FOOT_SWITCH_ACTIVE_HIGH
		bool pressed = level;
	#else
		bool pressed = !level;
	#endif
		if (pressed) {
			raw |= 1u << i;
		}
	}
	return raw;
}

void initFeet()
{
	if (footCount == 0) {
		return;
	}
	Serial.println("Init foot switches");
	for (uint8_t i = 0; i < footCount; i++) {
		const char *problem = footPinProblem(footPins[i]);
		if (problem != nullptr) {
			Serial.printf("Foot switch %u on GPIO %u: %s\n", i, footPins[i], problem);
		}
	#ifdef FOOT_SWITCH_ACTIVE_HIGH
		pinMode(footPins[i], INPUT);
	#else
		pinMode(footPins[i], INPUT_PULLUP);
	#endif
	}
	delayMicroseconds(50);   // let the pull-ups charge the wires
	feet.begin(footCount, FOOT_SWITCH_DEBOUNCE, readFeetRaw());
}

bool footSwitchesFitted()
{
	return footCount > 0;
}

bool footSwitchesInUse()
{
	return footCount > 0 && feetUse;
}

void updateFeet(leg *legsOut, uint8_t legCount, bool apply)
{
	if (footCount == 0) {
		return;
	}
	feet.update(readFeetRaw());
	if (apply) {
		feet.apply(legsOut, legCount);
	}
}

/**
 * CLI `feet [on|off|state]`: use the switches in the gait, and what each
 * one reads now. `changes` counts debounced presses + releases since boot:
 * a foot whose count stays 0 while walking has a broken wire or a stuck lever.
 */
void cliFeet()
{
	if (footCount == 0) {
		cliSerial->println("No foot switches: set FOOT_SWITCH_PINS in the model's config.h.");
		return;
	}
	char *mode = CLI_readWord();
	if (strcmp(mode, TITLE_ON) == 0) {
		feetUse = true;
		cliSerial->println("Foot switches on once the robot stands still: a swing ends when its foot touches.");
	} else if (strcmp(mode, TITLE_OFF) == 0) {
		feetUse = false;
		cliSerial->println("Foot switches off once the robot stands still: timed steps.");
	}
	cliSerial->printf("Foot switches: %s, gait uses them: %s, debounce %u ticks\n",
	                  feetUse ? TITLE_ON : TITLE_OFF, motionUsesContacts() ? "yes" : "no",
	                  (unsigned)FOOT_SWITCH_DEBOUNCE);
	uint32_t raw = feet.raw();
	for (uint8_t i = 0; i < footCount; i++) {
		cliSerial->printf("  %-4s GPIO %2u: %s (raw %s), changes %lu\n", legs[i].id.title, footPins[i],
		                  feet.pressed(i) ? "pressed " : "released", ((raw >> i) & 1) ? "pressed" : "released",
		                  (unsigned long)feet.changes(i));
	}
}
