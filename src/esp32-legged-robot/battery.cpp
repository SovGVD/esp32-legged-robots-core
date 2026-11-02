/**
 * Battery protection: the power sensor voltage through BatteryMonitor
 * (lib/robot_core), run at 10 Hz from the motion task. What the robot does
 * about it is in motion.cpp (stop, sit down, servos off).
 */
#include "robot.h"

static BatteryMonitor battery;
static BatteryState   reported = BATTERY_UNKNOWN;

void initBattery()
{
	battery.begin(BATTERY_PARAMS);
}

void updateBattery(float dt)
{
	BatteryState s = battery.update(dt, getPowerSensorVoltage(), powerSensor().ready());
	if (s != reported) {
		reported = s;
		if (s == BATTERY_LOW) {
			Serial.printf("Battery low: %.2f V\n", battery.voltage());
			cliSerial->printf("Battery low: %.2f V, time to charge\n", battery.voltage());
		} else if (s == BATTERY_CRITICAL) {
			Serial.printf("Battery critical: %.2f V, stopping\n", battery.voltage());
			cliSerial->printf("Battery critical: %.2f V. Stopping, sitting down, servos off. Charge and restart.\n", battery.voltage());
		}
	}
}

BatteryState batteryState()
{
	return battery.state();
}
