#include "battery_monitor.h"

void BatteryMonitor::begin(const BatteryParams &params)
{
	_params = params;
	_state  = BATTERY_UNKNOWN;
	_ready  = false;
	_filtered = _lowFor = _critFor = 0;
}

BatteryState BatteryMonitor::update(float dt, float voltage, bool valid)
{
	// under ~1 V the sensor sees no battery (USB powered bench test)
	if (!valid || voltage < 1.0f) {
		return _state;
	}
	if (!_ready) {
		_filtered = voltage;
		_ready    = true;
	} else {
		float a = _params.filter > 0 ? dt / (_params.filter + dt) : 1.0f;
		_filtered += a * (voltage - _filtered);
	}

	_lowFor  = _filtered < _params.lowVoltage      ? _lowFor  + dt : 0;
	_critFor = _filtered < _params.criticalVoltage ? _critFor + dt : 0;

	if (_state == BATTERY_CRITICAL) {
		return _state;   // latched
	}
	if (_critFor >= _params.holdTime) {
		_state = BATTERY_CRITICAL;
	} else if (_lowFor >= _params.holdTime) {
		_state = BATTERY_LOW;
	} else if (_state == BATTERY_UNKNOWN || _filtered > _params.lowVoltage + 0.1f) {
		_state = BATTERY_OK;   // back from low only with some margin
	}
	return _state;
}
