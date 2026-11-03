/**
 * Hardware interfaces. The motion code and the rest of the firmware only see
 * these; each driver (LEDC, PCA9685, DC motor H-bridge, MPU9250, INA219)
 * implements one of them, chosen at build time by the model's config.h.
 * No Arduino here, so tests and the simulator can provide fakes.
 */
#ifndef LR_HARDWARE_H
#define LR_HARDWARE_H

#include <stdint.h>
#include "geometry.h"
#include "leg.h"

/**
 * A set of outputs addressed by index (leg * dof + joint). For servos the
 * value is a pulse width in microseconds, for a DC motor bus it is the PWM
 * duty of one H-bridge input. 0 means "no pulse / off".
 */
class ServoBus {
	public:
		virtual ~ServoBus() {}
		virtual const char *name() const = 0;
		// map output index -> pin or channel; count <= LEG_NUM * LEG_DOF
		virtual bool begin(const uint8_t *channels, uint8_t count) = 0;
		virtual void write(uint8_t index, uint16_t value) = 0;
		// false: outputs off (servos limp, motors coast) until enabled again
		virtual void setEnabled(bool enabled) = 0;
		// highest channel number + 1 this bus can drive
		virtual uint8_t channelLimit() const = 0;
		// null if the channel is fine, otherwise a short warning
		virtual const char *checkChannel(uint8_t channel) const { (void)channel; return nullptr; }
};

class Imu {
	public:
		virtual ~Imu() {}
		virtual const char *name() const = 0;
		virtual bool begin() = 0;
		virtual bool ready() const = 0;
		// body orientation, rad; false if there is no new reading
		virtual bool read(LR_angle &angle) = 0;
		// robot must stand still on a level surface
		virtual void calibrate() = 0;

		/**
		 * Heading (gyro + magnetometer, HeadingEstimator): compass heading in
		 * rad, clockwise from magnetic north, or from the direction at start
		 * without a magnetometer (then it drifts). false: no heading at all.
		 */
		virtual bool heading(float &rad) const { (void)rad; return false; }
		virtual bool hasMagnetometer() const { return false; }
		// turn rate about the body's up axis, rad/s, counter-clockwise > 0
		virtual float yawRate() const { return 0; }
		// the magnetometer has set the heading (it is a compass heading)
		virtual bool headingAbsolute() const { return false; }
		// last magnetometer field strength, uT, and whether it was trusted
		virtual float magField(bool &accepted) const { accepted = false; return 0; }
		// collect magnetometer readings while the robot turns, then fit and save the calibration
		virtual void magCalibrationStart() {}
		virtual bool magCalibrationFinish() { return false; }
};

class PowerSensor {
	public:
		virtual ~PowerSensor() {}
		virtual const char *name() const = 0;
		virtual bool begin() = 0;
		virtual bool ready() const = 0;
		virtual bool read(float &voltage, float &current) = 0;   // V, A
};

// Used when a model has no IMU or power sensor
class NoImu : public Imu {
	public:
		const char *name() const override { return "none"; }
		bool begin() override { return false; }
		bool ready() const override { return false; }
		bool read(LR_angle &angle) override { (void)angle; return false; }
		void calibrate() override {}
};

class NoPowerSensor : public PowerSensor {
	public:
		const char *name() const override { return "none"; }
		bool begin() override { return false; }
		bool ready() const override { return false; }
		bool read(float &voltage, float &current) override { (void)voltage; (void)current; return false; }
};

// Pin or channel of joint `joint` (0 = alpha .. 5 = zeta)
uint8_t lrLegPin(const leg &l, uint8_t joint);

#endif
