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

#ifdef HIL
	#define FOOT_SWITCHES_FITTED 1   // simulated on every leg (hil.cpp)
#elif defined(FOOT_SWITCH_PINS)
	#define FOOT_SWITCHES_FITTED 1
#endif

static FootSwitches  feet;
static volatile bool feetUse =
#if defined(FOOT_SWITCHES_FITTED) && defined(FOOT_SWITCH_ON)
	true;
#else
	false;
#endif

#ifndef HIL   // the HIL build reads the simulated switches (hil.cpp)

#ifdef FOOT_SWITCH_PINS
static const uint8_t footPins[] = FOOT_SWITCH_PINS;
static_assert(sizeof(footPins) / sizeof(footPins[0]) == LEG_NUM, "FOOT_SWITCH_PINS needs one pin per leg");
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

/**
 * One switch per GPIO pin (FOOT_SWITCH_PINS), none without it.
 */
class GpioFootInput : public FootInput {
	public:
		const char *name() const override { return "GPIO"; }

		uint8_t count() const override
		{
		#ifdef FOOT_SWITCH_PINS
			return sizeof(footPins) / sizeof(footPins[0]);
		#else
			return 0;
		#endif
		}

		int pin(uint8_t i) const override
		{
		#ifdef FOOT_SWITCH_PINS
			return i < count() ? footPins[i] : -1;
		#else
			(void)i;
			return -1;
		#endif
		}

		void begin() override
		{
			for (uint8_t i = 0; i < count(); i++) {
				uint8_t p = (uint8_t)pin(i);
				const char *problem = footPinProblem(p);
				if (problem != nullptr) {
					Serial.printf("Foot switch %u on GPIO %u: %s\n", i, p, problem);
				}
			#ifdef FOOT_SWITCH_ACTIVE_HIGH
				pinMode(p, INPUT);
			#else
				pinMode(p, INPUT_PULLUP);
			#endif
			}
			delayMicroseconds(50);   // let the pull-ups charge the wires
		}

		uint32_t read() override
		{
			uint32_t raw = 0;
			for (uint8_t i = 0; i < count(); i++) {
				bool level = digitalRead((uint8_t)pin(i)) == HIGH;
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
};

FootInput &footInput()
{
	static GpioFootInput dev;
	return dev;
}

#endif   // HIL

static uint8_t footCount()
{
	return footInput().count();
}

void initFeet()
{
	if (footCount() == 0) {
		return;
	}
	Serial.printf("Init foot switches (%s)\n", footInput().name());
	footInput().begin();
	feet.begin(footCount(), FOOT_SWITCH_DEBOUNCE, footInput().read());
}

bool footSwitchesFitted()
{
	return footCount() > 0;
}

bool footSwitchesInUse()
{
	return footCount() > 0 && feetUse;
}

uint32_t footPressedBits()
{
	uint32_t bits = 0;
	for (uint8_t i = 0; i < footCount(); i++) {
		if (feet.pressed(i)) {
			bits |= 1u << i;
		}
	}
	return bits;
}

void updateFeet(leg *legsOut, uint8_t legCount, bool apply)
{
	if (footCount() == 0) {
		return;
	}
	feet.update(footInput().read());
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
	if (footCount() == 0) {
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
	for (uint8_t i = 0; i < footCount(); i++) {
		char where[12];
		if (footInput().pin(i) >= 0) {
			snprintf(where, sizeof(where), "GPIO %2d", footInput().pin(i));
		} else {
			snprintf(where, sizeof(where), "%s", footInput().name());
		}
		cliSerial->printf("  %-4s %s: %s (raw %s), changes %lu\n", legs[i].id.title, where,
		                  feet.pressed(i) ? "pressed " : "released", ((raw >> i) & 1) ? "pressed" : "released",
		                  (unsigned long)feet.changes(i));
	}
}
