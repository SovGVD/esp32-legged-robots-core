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

/**
 * Sensor axes -> body axes (x right, y forward, z up), from the same mounting
 * settings: pitch is the rotation about the forward axis, roll about the
 * right axis. Up follows from the two (the frame stays right handed).
 */
static void imuToBody(const float s[3], float b[3])
{
	if (IMU_SWAP_ROLL_PITCH) {
		b[0] = IMU_ROLL_SIGN  * s[1];
		b[1] = IMU_PITCH_SIGN * s[0];
		b[2] = -IMU_ROLL_SIGN * IMU_PITCH_SIGN * s[2];
	} else {
		b[0] = IMU_ROLL_SIGN  * s[0];
		b[1] = IMU_PITCH_SIGN * s[1];
		b[2] = IMU_ROLL_SIGN * IMU_PITCH_SIGN * s[2];
	}
}

// Heading: how fast it follows the magnetometer, and which readings count as disturbed
#ifndef HEADING_MAG_TAU
	#define HEADING_MAG_TAU 2.0f   // s
#endif
#ifndef HEADING_FIELD_TOLERANCE
	#define HEADING_FIELD_TOLERANCE 0.25f
#endif
#ifndef MAG_READ_EVERY
	#define MAG_READ_EVERY 4       // motion ticks per magnetometer read (AK8963 runs at 100 Hz)
#endif

#if defined(IMU_TYPE) && IMU_TYPE == MPU9250
#include <MPU9250_WE.h>

/**
 * MPU9250 tilt: accelerometer and gyro fused by TiltEstimator (the
 * accelerometer alone is thrown off by the robot's own accelerations).
 * Calibration offsets are kept in NVS (namespace "imu"), so `calib imu`
 * only has to be run once.
 *
 * Heading: gyro Z and the AK8963 magnetometer inside the MPU9250 (GY-91),
 * fused by HeadingEstimator. Many cheap "GY-91" boards carry an MPU6500,
 * which has no magnetometer: then the heading is the gyro alone (relative
 * to the start, drifts). Magnetometer calibration (`heading cal`) is kept in
 * NVS too.
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
			_heading.begin({ HEADING_MAG_TAU, HEADING_FIELD_TOLERANCE });
			_mag = _dev.initMagnetometer();
			if (_mag) {
				_dev.setMagOpMode(AK8963_CONT_MODE_100HZ);
				Serial.print(", magnetometer");
				loadMagCalibration();
			} else {
				Serial.print(", no magnetometer (MPU6500?): heading from the gyro only");
			}
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
			angle.yaw   = 0;   // tilt only; the heading is heading()

			float gyroBody[3];
			imuToBody(gyro, gyroBody);
			const float *mag = nullptr;
			float magBody[3];
			if (_mag && ++_magTick >= MAG_READ_EVERY) {
				_magTick = 0;
				xyzFloat m = _dev.getMagValues();   // uT, AK8963 axes
				// the AK8963 sits in the MPU9250 with X and Y swapped and Z reversed
				float s[3] = { m.y, m.x, -m.z };
				imuToBody(s, magBody);
				if (_calibrating) {
					_cal.add(magBody);
				}
				mag = magBody;
			}
			_yawRate = lrHeadingRate(gyroBody, angle, _heading.gyroBias());   // also with the nose up
			for (int i = 0; i < 3; i++) _gyro[i] = gyroBody[i];
			_heading.update(dt, _yawRate, angle, mag);
			return true;
		}

		bool heading(float &rad) const override
		{
			if (!_ready) {
				return false;
			}
			rad = _heading.heading();
			return true;
		}

		bool hasMagnetometer() const override { return _mag; }
		float yawRate() const override { return _yawRate; }
		bool gyro(float out[3]) const override
		{
			for (int i = 0; i < 3; i++) out[i] = _gyro[i];
			return _ready;
		}
		float gyroBias() const override { return _heading.gyroBias(); }
		const HeadingEstimator *headingEstimator() const override { return &_heading; }
		bool headingAbsolute() const override { return _heading.absolute(); }

		float magField(bool &accepted) const override
		{
			accepted = _heading.magAccepted();
			return _heading.field();
		}

		void magCalibrationStart() override
		{
			_cal.reset();
			_calibrating = _mag;
		}

		bool magCalibrationFinish() override
		{
			_calibrating = false;
			float offset[3], scale[3];
			if (!_cal.result(offset, scale)) {
				return false;
			}
			float field = _cal.fieldStrength(offset, scale);
			_heading.setMagCalibration(offset, scale, field);
			saveMagCalibration(offset, scale, field);
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

		void loadMagCalibration()
		{
			Preferences prefs;
			if (!prefs.begin("imu", true)) {
				return;
			}
			float c[7];
			if (prefs.getBytes("mag", c, sizeof(c)) == sizeof(c)) {
				_heading.setMagCalibration(&c[0], &c[3], c[6]);
				Serial.print(" (calibrated)");
			} else {
				Serial.print(" (not calibrated: `heading cal`)");
			}
			prefs.end();
		}

		void saveMagCalibration(const float offset[3], const float scale[3], float field)
		{
			Preferences prefs;
			if (!prefs.begin("imu", false)) {
				return;
			}
			float c[7] = { offset[0], offset[1], offset[2], scale[0], scale[1], scale[2], field };
			prefs.putBytes("mag", c, sizeof(c));
			prefs.end();
		}

		MPU9250_WE       _dev = MPU9250_WE(IMU_ADDRESS);
		TiltEstimator    _tilt;
		HeadingEstimator _heading;
		MagCalibrator    _cal;
		uint32_t         _last  = 0;
		uint8_t          _magTick = 0;
		float            _yawRate = 0;
		float            _gyro[3] = { 0, 0, 0 };
		bool             _ready = false;
		bool             _mag   = false;
		volatile bool    _calibrating = false;
};

Imu &imu()
{
	static Mpu9250Imu dev;
	return dev;
}
#elif defined(IMU_TYPE) && IMU_TYPE == HIL_SIM

/**
 * Hardware in the loop: the simulator's accelerometer, gyro and magnetometer
 * (body frame, hil.cpp) through the same TiltEstimator and HeadingEstimator
 * as the MPU9250. No mounting settings: the sim sends body axes.
 */
class HilImu : public Imu {
	public:
		const char *name() const override { return "HIL (simulated)"; }

		bool begin() override
		{
			_heading.begin({ HEADING_MAG_TAU, HEADING_FIELD_TOLERANCE });
			_ready = true;
			return true;
		}

		bool ready() const override { return _ready; }

		bool read(LR_angle &angle) override
		{
			HilSensors s;
			bool magFresh = false;
			if (!_ready || !hilSensors(s, magFresh)) {
				return false;
			}
			uint32_t now = micros();
			float dt = _last ? (now - _last) / 1000000.0f : 0.0f;
			_last = now;

			_tilt.update(dt, s.accel, s.gyro);
			angle.pitch = _tilt.aboutY();   // lean about the forward axis
			angle.roll  = _tilt.aboutX();   // nose up / down
			angle.yaw   = 0;

			const float *mag = nullptr;
			if (magFresh) {
				if (_calibrating) {
					_cal.add(s.mag);
				}
				mag = s.mag;
				_mag = true;
			}
			_yawRate = lrHeadingRate(s.gyro, angle, _heading.gyroBias());
			for (int i = 0; i < 3; i++) _gyro[i] = s.gyro[i];
			_heading.update(dt, _yawRate, angle, mag);
			return true;
		}

		bool heading(float &rad) const override
		{
			if (!_ready) {
				return false;
			}
			rad = _heading.heading();
			return true;
		}

		bool hasMagnetometer() const override { return _mag; }
		float yawRate() const override { return _yawRate; }
		bool gyro(float out[3]) const override
		{
			for (int i = 0; i < 3; i++) out[i] = _gyro[i];
			return _ready;
		}
		float gyroBias() const override { return _heading.gyroBias(); }
		const HeadingEstimator *headingEstimator() const override { return &_heading; }
		bool headingAbsolute() const override { return _heading.absolute(); }

		float magField(bool &accepted) const override
		{
			accepted = _heading.magAccepted();
			return _heading.field();
		}

		// kept until the next reboot (no NVS: the simulated field never changes)
		void magCalibrationStart() override
		{
			_cal.reset();
			_calibrating = true;
		}

		bool magCalibrationFinish() override
		{
			_calibrating = false;
			float offset[3], scale[3];
			if (!_cal.result(offset, scale)) {
				return false;
			}
			_heading.setMagCalibration(offset, scale, _cal.fieldStrength(offset, scale));
			return true;
		}

		void calibrate() override { _tilt.reset(); }

	private:
		TiltEstimator    _tilt;
		HeadingEstimator _heading;
		MagCalibrator    _cal;
		uint32_t         _last = 0;
		float            _yawRate = 0;
		float            _gyro[3] = { 0, 0, 0 };
		bool             _ready = false;
		bool             _mag = false;
		volatile bool    _calibrating = false;
};

Imu &imu()
{
	static HilImu dev;
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
#elif defined(POWER_SENSOR) && POWER_SENSOR == HIL_SIM

// Hardware in the loop: the battery the simulator reports (hil.cpp)
class HilPowerSensor : public PowerSensor {
	public:
		const char *name() const override { return "HIL (simulated)"; }
		bool begin() override { return true; }
		bool ready() const override { return true; }
		bool read(float &voltage, float &current) override
		{
			HilSensors s;
			bool magFresh;   // left for the IMU
			if (!hilSensors(s, magFresh, false)) {
				return false;
			}
			voltage = s.voltage;
			current = s.current;
			return true;
		}
};

PowerSensor &powerSensor()
{
	static HilPowerSensor dev;
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
