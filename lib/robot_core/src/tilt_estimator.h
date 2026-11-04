/**
 * Body tilt from an accelerometer and a gyro (complementary filter).
 *
 * The accelerometer alone gives the tilt only while the body is not
 * accelerating; a walking robot accelerates all the time, and leveling on
 * the raw accelerometer angle feeds those accelerations back into the body
 * (in the simulator this rocked the dog by up to 15 deg on flat ground). The
 * gyro is integrated for the fast part and the accelerometer only pulls the
 * estimate back slowly, with less trust while |a| is far from 1 g. The
 * gyro's Z rate turns the tilt around (small angle form): turning in place
 * with the nose up doesn't make up a sideways tilt.
 *
 * Sensor frame, right handed. Outputs the rotation about the sensor X and Y
 * axes; with the sensor mounted X right, Y forward, Z up these are the
 * firmware roll (nose up/down) and pitch (sideways lean).
 */
#ifndef LR_TILT_ESTIMATOR_H
#define LR_TILT_ESTIMATOR_H

#include "lr_compat.h"

class TiltEstimator {
	public:
		explicit TiltEstimator(float timeConstant = 1.0f) : _tau(timeConstant) {}
		void setTimeConstant(float seconds) { _tau = seconds; }
		/**
		 * @param accel in g, reads +1 on Z when level
		 * @param gyro  rad/s
		 */
		void update(float dt, const float accel[3], const float gyro[3]);
		void reset() { _ready = false; _aboutX = _aboutY = 0; }
		float aboutX() const { return _aboutX; }
		float aboutY() const { return _aboutY; }

	private:
		float _tau;
		bool  _ready  = false;
		float _aboutX = 0;
		float _aboutY = 0;
};

#endif
