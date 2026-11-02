/**
 * IMU and power sensor drivers behind the Imu / PowerSensor interfaces
 * (lib/robot_core/src/hardware.h). The model's config.h picks them with
 * IMU_TYPE and POWER_SENSOR; without one the No* stand-in is used.
 */
#include "robot.h"
#include <Wire.h>
#include <Preferences.h>

/**
 * IMU mounting: which sensor axis is body roll / pitch and the signs.
 * Defaults keep the old mapping (roll = sensor X, pitch = sensor Y).
 */
#ifndef IMU_SWAP_ROLL_PITCH
	#define IMU_SWAP_ROLL_PITCH false
#endif
#ifndef IMU_ROLL_SIGN
	#define IMU_ROLL_SIGN 1
#endif
#ifndef IMU_PITCH_SIGN
	#define IMU_PITCH_SIGN 1
#endif

#if defined(IMU_TYPE) && IMU_TYPE == MPU9250
#include <MPU9250_WE.h>

/**
 * MPU9250 accelerometer tilt. Calibration offsets are kept in NVS
 * (namespace "imu"), so `calibrate imu` only has to be run once.
 */
class Mpu9250Imu : public Imu {
	public:
		const char *name() const override { return "MPU9250"; }

		bool begin() override
		{
			if (!_dev.init()) {
				Serial.print(" does not respond");
				return false;
			}
			_dev.setAccRange(MPU9250_ACC_RANGE_2G);
			_dev.enableAccDLPF(true);
			_dev.setAccDLPF(MPU9250_DLPF_6);
			loadOffsets();
			_ready = true;
			return true;
		}

		bool ready() const override { return _ready; }

		bool read(LR_angle &angle) override
		{
			if (!_ready) {
				return false;
			}
			xyzFloat a = _dev.getAngles();   // deg
			float roll  = IMU_SWAP_ROLL_PITCH ? a.y : a.x;
			float pitch = IMU_SWAP_ROLL_PITCH ? a.x : a.y;
			angle.roll  = IMU_ROLL_SIGN  * degToRad(roll);
			angle.pitch = IMU_PITCH_SIGN * degToRad(pitch);
			angle.yaw   = degToRad(a.z);
			return true;
		}

		void calibrate() override
		{
			if (!_ready) {
				return;
			}
			_dev.autoOffsets();
			saveOffsets();
		}

	private:
		void loadOffsets()
		{
			Preferences prefs;
			if (!prefs.begin("imu", true)) {
				return;
			}
			xyzFloat acc, gyr;
			if (prefs.getBytes("acc", &acc, sizeof(acc)) == sizeof(acc)
			 && prefs.getBytes("gyr", &gyr, sizeof(gyr)) == sizeof(gyr)) {
				_dev.setAccOffsets(acc);
				_dev.setGyrOffsets(gyr);
				Serial.print(" offsets loaded");
			} else {
				Serial.print(" not calibrated");
			}
			prefs.end();
		}

		void saveOffsets()
		{
			Preferences prefs;
			if (!prefs.begin("imu", false)) {
				Serial.println("Can't open NVS for IMU offsets");
				return;
			}
			xyzFloat acc = _dev.getAccOffsets();
			xyzFloat gyr = _dev.getGyrOffsets();
			bool ok = prefs.putBytes("acc", &acc, sizeof(acc)) == sizeof(acc)
			       && prefs.putBytes("gyr", &gyr, sizeof(gyr)) == sizeof(gyr);
			prefs.end();
			Serial.println(ok ? "IMU offsets saved" : "Saving IMU offsets failed");
		}

		MPU9250_WE _dev = MPU9250_WE(IMU_ADDRESS);
		bool _ready = false;
};

Imu &imu()
{
	static Mpu9250Imu dev;
	return dev;
}
#else
Imu &imu()
{
	static NoImu dev;
	return dev;
}
#endif

#if defined(POWER_SENSOR) && POWER_SENSOR == INA219
#include <INA219_WE.h>

class Ina219PowerSensor : public PowerSensor {
	public:
		const char *name() const override { return "INA219"; }

		bool begin() override
		{
			if (!_dev.init()) {
				Serial.print(" failed to find INA219 chip");
				return false;
			}
			_dev.setADCMode(INA219_BIT_MODE_9);  // just to make it fast
			_dev.setPGain(INA219_PG_320);
			_dev.setBusRange(INA219_BRNG_16);
			_ready = true;
			return true;
		}

		bool ready() const override { return _ready; }

		bool read(float &voltage, float &current) override
		{
			if (!_ready) {
				return false;
			}
			voltage = _dev.getBusVoltage_V();
			current = _dev.getCurrent_mA() / 1000.0f;
			return true;
		}

	private:
		INA219_WE _dev = INA219_WE(POWER_SENSOR_ADDRESS);
		bool _ready = false;
};

PowerSensor &powerSensor()
{
	static Ina219PowerSensor dev;
	return dev;
}
#else
PowerSensor &powerSensor()
{
	static NoPowerSensor dev;
	return dev;
}
#endif

static float voltage_V = 0.0;
static float current_A = 0.0;

void initIMU()
{
	Serial.print("IMU ");
	Serial.print(imu().name());
	imu().begin();
	Serial.println();
}

void calibrateIMU()
{
	if (!imu().ready()) {
		Serial.println("IMU is not available.");
		return;
	}
	disableServos();
	Serial.println("Calibrating ACC and GYRO in 5 seconds. Put device on flat leveled surface.");
	delay(5000);
	Serial.print("Calibration...");
	imu().calibrate();
	Serial.println("Done.");
	delay(1000);
	enableServos();
}

void updateIMU()
{
	imu().read(IMU_DATA);
}

void initPowerSensor()
{
	Serial.print("Power sensor ");
	Serial.print(powerSensor().name());
	powerSensor().begin();
	Serial.println();
}

void updatePower()
{
	powerSensor().read(voltage_V, current_A);
}

// return V
float getPowerSensorVoltage()
{
	return voltage_V;
}

// return A
float getPowerSensorCurrent()
{
	return current_A;
}
