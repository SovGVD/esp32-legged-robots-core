/**
 * IMU and power sensor drivers behind the Imu / PowerSensor interfaces
 * (lib/robot_core/src/hardware.h). The model's config.h picks them with
 * IMU_TYPE and POWER_SENSOR; without one the No* stand-in is used.
 */
#include "robot.h"
#include <Wire.h>
#include <Preferences.h>

/**
 * IMU mounting. Firmware convention (BodyKinematics, the move command):
 * pitch = lean about the forward axis (sideways), roll = nose up/down.
 * Default: sensor X to the right, Y forward, Z up, like the simulator. Check
 * on the robot with `level state`: lift the left side, then the nose, and
 * fix the axes/signs here if the wrong value moves or has the wrong sign.
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
 * MPU9250 tilt: accelerometer and gyro fused by TiltEstimator (the
 * accelerometer alone is thrown off by the robot's own accelerations).
 * Calibration offsets are kept in NVS (namespace "imu"), so `calib imu`
 * only has to be run once.
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
			_dev.setAccRange(MPU9250_ACC_RANGE_4G);
			_dev.enableAccDLPF(true);
			_dev.setAccDLPF(MPU9250_DLPF_3);
			_dev.setGyrRange(MPU9250_GYRO_RANGE_500);
			_dev.enableGyrDLPF();
			_dev.setGyrDLPF(MPU9250_DLPF_3);
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
			uint32_t now = micros();
			float dt = _last ? (now - _last) / 1000000.0f : 0.0f;
			_last = now;

			xyzFloat g = _dev.getGValues();       // g
			xyzFloat w = _dev.getGyrValues();     // deg/s
			float accel[3] = { g.x, g.y, g.z };
			float gyro[3]  = { degToRad(w.x), degToRad(w.y), degToRad(w.z) };
			_tilt.update(dt, accel, gyro);

			// sensor X right, Y forward: rotation about Y is the firmware pitch
			float pitch = IMU_SWAP_ROLL_PITCH ? _tilt.aboutX() : _tilt.aboutY();
			float roll  = IMU_SWAP_ROLL_PITCH ? _tilt.aboutY() : _tilt.aboutX();
			angle.pitch = IMU_PITCH_SIGN * pitch;
			angle.roll  = IMU_ROLL_SIGN  * roll;
			angle.yaw   = 0;   // no heading estimate (magnetometer unused)
			return true;
		}

		void calibrate() override
		{
			if (!_ready) {
				return;
			}
			_dev.autoOffsets();
			saveOffsets();
			_tilt.reset();
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

		MPU9250_WE    _dev = MPU9250_WE(IMU_ADDRESS);
		TiltEstimator _tilt;
		uint32_t      _last  = 0;
		bool          _ready = false;
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
