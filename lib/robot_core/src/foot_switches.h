/**
 * Foot contact switches: debouncing, Arduino free (the firmware reads the
 * pins, feet.cpp). One bit per leg, bit i = leg i, 1 = the foot is pressed.
 *
 * A switch changes state only after it read the new state on `ticks` motion
 * ticks in a row: a lever microswitch bounces for a few ms, and a foot
 * sliding on the ground can chatter. At ~230 ticks/s, 2 ticks is ~9 ms.
 */
#ifndef LR_FOOT_SWITCHES_H
#define LR_FOOT_SWITCHES_H

#include <stdint.h>
#include "leg.h"

class FootSwitches
{
	public:
		static const uint8_t MAX_SWITCHES = 8;

		/**
		 * count switches (max MAX_SWITCHES), ticks >= 1 (1 = no debouncing).
		 * initial: the state they start in (raw read at boot).
		 */
		void begin(uint8_t count, uint8_t ticks, uint32_t initial = 0);

		/**
		 * One motion tick with the raw switch reads. Returns true if any
		 * debounced state changed.
		 */
		bool update(uint32_t raw);

		uint32_t state() const { return _state; }   // debounced
		uint32_t raw() const { return _raw; }       // last raw read
		bool     pressed(uint8_t i) const { return (_state >> i) & 1; }
		uint32_t changes(uint8_t i) const { return i < _count ? _changes[i] : 0; }   // debounced changes since begin()
		uint8_t  count() const { return _count; }

		/** Debounced states into legs[i].sensor.onGround (GaitParams.useContactSensors). */
		void apply(leg *legs, uint8_t legCount) const;

	private:
		uint8_t  _count = 0;
		uint8_t  _ticks = 1;
		uint32_t _state = 0;
		uint32_t _raw   = 0;
		uint8_t  _pending[MAX_SWITCHES] = {};   // ticks the raw read has differed from _state
		uint32_t _changes[MAX_SWITCHES] = {};
};

#endif
