/**
 * Heading (compass direction) of the robot from the gyro and a magnetometer,
 * the magnetometer's calibration, and a heading hold for the move command.
 *
 * Heading here is a compass heading: rad, clockwise seen from above, 0 =
 * magnetic north (or the direction the robot faced at start, without a
 * magnetometer). The body frame is the firmware's: x right, y forward, z up.
 *
 * The gyro gives a smooth heading that drifts (a few deg per minute); the
 * magnetometer gives an absolute one that is noisy and thrown off by the
 * servos' currents and magnets. The estimator integrates the gyro and pulls
 * the result slowly (magTau) toward the magnetometer, ignoring readings whose
 * field strength is far from the calibrated one (a disturbance). It also
 * learns the gyro's bias from the magnetometer (slower still), so a gyro
 * that drifts doesn't leave the heading lagging, and the heading keeps
 * better through a disturbance.
 */
#ifndef LR_HEADING_H
#define LR_HEADING_H

#include "lr_compat.h"
#include "geometry.h"

/** Angle wrapped to [-pi, pi). */
float lrWrapAngle(float a);

/**
 * Compass heading from a magnetometer reading, tilt compensated.
 * @param mag  body frame field (x right, y forward, z up), any unit
 * @param tilt body tilt, firmware convention: pitch = lean about the forward
 *             axis (left side up > 0), roll = nose up > 0
 */
float lrMagHeading(const float mag[3], const LR_angle &tilt);

/**
 * Hard and soft iron calibration from readings taken while the robot turns
 * (at least one full turn on level ground): per axis offset = middle of the
 * range, scale = mean radius / axis radius. Only x and y (horizontal) are
 * calibrated: the z axis barely changes while turning, it keeps offset 0,
 * scale 1 (z only matters for the tilt compensation).
 */
class MagCalibrator {
	public:
		void reset();
		void add(const float mag[3]);
		/** false if the readings don't span a full turn well enough */
		bool result(float offset[3], float scale[3]) const;
		/** field strength the corrected readings had during the turn, uT */
		float fieldStrength(const float offset[3], const float scale[3]) const;
		uint32_t samples() const { return _n; }
	private:
		float    _min[3] = { 0, 0, 0 };
		float    _max[3] = { 0, 0, 0 };
		uint32_t _n = 0;
};

typedef struct {
	float magTau;        // s, how fast the heading follows the magnetometer (gyro drift vs. disturbances)
	float fieldTolerance;  // 0..1, readings whose strength differs more than this from the calibrated field are ignored
} HeadingParams;

/**
 * Heading rate (rad/s, counter-clockwise > 0) from the body frame gyro
 * (x right, y forward, z up) and the body tilt: the yaw rate of
 * Rz(yaw) * Ry(pitch) * Rx(roll), as in BodyKinematics. Gyro z alone reads
 * only rate * cos(tilt) on a tilted body (a ToF scan's nose up), and the
 * gyro's x and y matter too while the tilted body wobbles around a turn: in
 * the simulator gyro z / cos(tilt) lost ~2 deg per room scan at 18 deg nose
 * up, this ~0.4 deg.
 * bias: the gyro z bias (HeadingEstimator::gyroBias): taken off z before and
 * added back after, so it comes out as it is (the estimator takes it off and
 * keeps learning it), not scaled by the tilt.
 */
float lrHeadingRate(const float gyro[3], const LR_angle &tilt, float bias = 0);

class HeadingEstimator {
	public:
		void begin(const HeadingParams &params) { _params = params; reset(); }
		void reset() { _heading = 0; _bias = 0; _ready = false; _absolute = false; }
		/** learned gyro bias, rad/s (subtracted from the yaw rate) */
		float gyroBias() const { return _bias; }
		/** Calibration from MagCalibrator; field = calibrated field strength (0: unknown, no check) */
		void setMagCalibration(const float offset[3], const float scale[3], float field);
		/**
		 * @param yawRate rad/s about the vertical, counter-clockwise > 0 (lrHeadingRate)
		 * @param tilt    body tilt (firmware convention), for the tilt compensation
		 * @param mag     body frame field in uT, raw (calibration applied here); nullptr: none this tick
		 */
		void update(float dt, float yawRate, const LR_angle &tilt, const float *mag);
		/** rad, clockwise from magnetic north, [0, 2 pi) */
		float heading() const;
		/** true once the magnetometer has set the heading (else it is relative to the start) */
		bool absolute() const { return _absolute; }
		/** strength of the last (calibrated) magnetometer reading, uT */
		float field() const { return _lastField; }
		/** the last magnetometer reading was used (strength within tolerance) */
		bool magAccepted() const { return _magOk; }
		/** the last raw (uncalibrated) body frame reading, uT; false before the first one */
		bool magRaw(float out[3]) const
		{
			for (int i = 0; i < 3; i++) out[i] = _raw[i];
			return _haveRaw;
		}
		/** the calibration in use; field 0 = not calibrated */
		void magCalibration(float offset[3], float scale[3], float &field) const
		{
			for (int i = 0; i < 3; i++) {
				offset[i] = _offset[i];
				scale[i]  = _scale[i];
			}
			field = _field;
		}

	private:
		HeadingParams _params = { 2.0f, 0.25f };
		float _offset[3] = { 0, 0, 0 };
		float _scale[3]  = { 1, 1, 1 };
		float _field     = 0;
		float _heading   = 0;
		float _bias      = 0;
		float _lastField = 0;
		float _raw[3]    = { 0, 0, 0 };
		bool  _haveRaw   = false;
		bool  _ready     = false;
		bool  _absolute  = false;
		bool  _magOk     = false;
};

typedef struct {
	float gain;          // 1/s: turn rate = gain * heading error
	float maxRate;       // rad/s, at most this fast toward the target
	float deadband;      // rad: standing still, errors below this don't start stepping
	float stickDeadband; // |stick yaw| below this counts as "not turning"
} HeadingHoldParams;

/**
 * Turns the robot toward a target heading by writing the yaw of the move
 * command. While the operator turns (stick yaw), the target follows the
 * heading, so letting go of the stick holds the heading the robot has then.
 * A target set with setTarget() is turned to and held the same way.
 */
class HeadingHold {
	public:
		void begin(const HeadingHoldParams &params) { _params = params; _hasTarget = false; }
		void setEnabled(bool on) { _enabled = on; _hasTarget = false; }
		bool enabled() const { return _enabled; }
		void setTarget(float heading) { _target = heading; _hasTarget = true; }
		bool hasTarget() const { return _hasTarget; }
		float target() const { return _target; }
		/**
		 * @param stickYaw   operator's yaw command, -1..1, > 0 turns clockwise
		 * @param heading    current heading (HeadingEstimator)
		 * @param maxYawRate rad/s the gait turns at stickYaw = 1
		 * @param moving     the operator asks to walk (move x/y not zero)
		 * @return yaw command for the gait, -1..1
		 */
		float update(float stickYaw, float heading, float maxYawRate, bool moving);
		/** heading error of the last update, rad (target - heading) */
		float error() const { return _error; }

	private:
		HeadingHoldParams _params = { 1.5f, 0.6f, 0.07f, 0.05f };
		bool  _enabled   = false;
		bool  _hasTarget = false;
		bool  _turning   = false;
		float _target    = 0;
		float _error     = 0;
};

#endif
