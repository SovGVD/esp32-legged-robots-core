/**
 * Battery state from the power sensor voltage.
 *
 * Servo current makes the voltage sag for a moment every step, so the
 * voltage is low-pass filtered and has to stay under a threshold for
 * `holdTime` before the state changes. Critical is latched: a battery that
 * recovers a little once the servos rest is still empty.
 */
#ifndef LR_BATTERY_MONITOR_H
#define LR_BATTERY_MONITOR_H

#include <stdint.h>

struct BatteryParams {
	float lowVoltage;       // V, warn
	float criticalVoltage;  // V, stop and sit down
	float filter;           // s, low-pass time constant
	float holdTime;         // s, below a threshold this long before it counts
};

enum BatteryState : uint8_t {
	BATTERY_UNKNOWN  = 0,   // no power sensor or no reading yet
	BATTERY_OK       = 1,
	BATTERY_LOW      = 2,
	BATTERY_CRITICAL = 3,
};

class BatteryMonitor {
	public:
		void begin(const BatteryParams &params);
		/**
		 * @param valid false when there is no reading (no sensor); the state
		 *              then stays as it is
		 */
		BatteryState update(float dt, float voltage, bool valid);
		BatteryState state() const { return _state; }
		float voltage() const { return _filtered; }

	private:
		BatteryParams _params = { 6.8f, 6.4f, 2.0f, 3.0f };
		BatteryState  _state  = BATTERY_UNKNOWN;
		bool  _ready    = false;
		float _filtered = 0;
		float _lowFor   = 0;
		float _critFor  = 0;
};

#endif
