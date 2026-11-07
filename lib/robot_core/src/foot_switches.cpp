#include "foot_switches.h"

void FootSwitches::begin(uint8_t count, uint8_t ticks, uint32_t initial)
{
	_count = count > MAX_SWITCHES ? MAX_SWITCHES : count;
	_ticks = ticks < 1 ? 1 : ticks;
	uint32_t mask = _count >= 32 ? 0xFFFFFFFFu : ((1u << _count) - 1);
	_state = initial & mask;
	_raw   = _state;
	for (uint8_t i = 0; i < MAX_SWITCHES; i++) {
		_pending[i] = 0;
		_changes[i] = 0;
	}
}

bool FootSwitches::update(uint32_t raw)
{
	bool changed = false;
	_raw = raw;
	for (uint8_t i = 0; i < _count; i++) {
		uint32_t bit = 1u << i;
		if ((raw & bit) == (_state & bit)) {
			_pending[i] = 0;
			continue;
		}
		if (++_pending[i] >= _ticks) {
			_state ^= bit;
			_pending[i] = 0;
			_changes[i]++;
			changed = true;
		}
	}
	return changed;
}

void FootSwitches::apply(leg *legs, uint8_t legCount) const
{
	for (uint8_t i = 0; i < legCount && i < _count; i++) {
		legs[i].sensor.onGround = pressed(i);
	}
}
