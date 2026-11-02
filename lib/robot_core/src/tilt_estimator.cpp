#include "tilt_estimator.h"

void TiltEstimator::update(float dt, const float accel[3], const float gyro[3])
{
	float ax = accel[0], ay = accel[1], az = accel[2];
	float norm = sqrtf(ax*ax + ay*ay + az*az);
	if (norm < 0.2f) {
		return;   // free fall or no data
	}
	// accelerometer angles, valid when gravity is the only acceleration
	float accX = atan2f(ay, sqrtf(ax*ax + az*az));
	float accY = atan2f(-ax, sqrtf(ay*ay + az*az));

	if (!_ready || dt > 0.5f) {   // first sample, or not read for a while
		_aboutX = accX;
		_aboutY = accY;
		_ready  = true;
		return;
	}

	_aboutX += gyro[0] * dt;
	_aboutY += gyro[1] * dt;

	// trust the accelerometer less the further |a| is from gravity
	float off   = fabsf(norm - 1.0f);
	float trust = off >= 0.3f ? 0.0f : 1.0f - off / 0.3f;
	float k     = trust * dt / (_tau + dt);
	_aboutX += k * (accX - _aboutX);
	_aboutY += k * (accY - _aboutY);
}
