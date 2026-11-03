#include "heading.h"

static const float PI_F     = 3.14159265f;
static const float TWO_PI_F = 6.28318531f;

float lrWrapAngle(float a)
{
	a = fmodf(a + PI_F, TWO_PI_F);
	if (a < 0) {
		a += TWO_PI_F;
	}
	return a - PI_F;
}

float lrMagHeading(const float mag[3], const LR_angle &tilt)
{
	// level the reading: body -> world is roll about x (nose up) after pitch
	// about y (left side up), the same order as BodyKinematics
	float cp = cosf(tilt.pitch), sp = sinf(tilt.pitch);
	float cr = cosf(tilt.roll),  sr = sinf(tilt.roll);
	float hx = mag[0] * cp + mag[2] * sp;
	float z1 = -mag[0] * sp + mag[2] * cp;
	float hy = mag[1] * cr - z1 * sr;
	// the field points north: robot facing north reads (0, +), facing east (-, 0)
	float h = atan2f(-hx, hy);
	return h < 0 ? h + TWO_PI_F : h;
}

void MagCalibrator::reset()
{
	_n = 0;
}

void MagCalibrator::add(const float mag[3])
{
	for (uint8_t i = 0; i < 3; i++) {
		if (_n == 0 || mag[i] < _min[i]) _min[i] = mag[i];
		if (_n == 0 || mag[i] > _max[i]) _max[i] = mag[i];
	}
	_n++;
}

bool MagCalibrator::result(float offset[3], float scale[3]) const
{
	float rx = (_max[0] - _min[0]) / 2, ry = (_max[1] - _min[1]) / 2;
	// the horizontal field is 15-45 uT on earth: a full turn spans twice that
	if (_n < 50 || rx < 5 || ry < 5) {
		return false;
	}
	float r = (rx + ry) / 2;
	offset[0] = (_max[0] + _min[0]) / 2;
	offset[1] = (_max[1] + _min[1]) / 2;
	offset[2] = 0;
	scale[0] = r / rx;
	scale[1] = r / ry;
	scale[2] = 1;
	return true;
}

float MagCalibrator::fieldStrength(const float offset[3], const float scale[3]) const
{
	// horizontal: the corrected radius; vertical: the middle of the z range (it barely changes while turning)
	float h = ((_max[0] - _min[0]) / 2 * scale[0] + (_max[1] - _min[1]) / 2 * scale[1]) / 2;
	float v = ((_max[2] + _min[2]) / 2 - offset[2]) * scale[2];
	return sqrtf(h * h + v * v);
}

void HeadingEstimator::setMagCalibration(const float offset[3], const float scale[3], float field)
{
	for (uint8_t i = 0; i < 3; i++) {
		_offset[i] = offset[i];
		_scale[i]  = scale[i];
	}
	_field = field;
}

void HeadingEstimator::update(float dt, float yawRate, const LR_angle &tilt, const float *mag)
{
	// gyro: counter-clockwise turn lowers the compass heading
	_heading = lrWrapAngle(_heading - yawRate * dt);
	_ready = true;
	_magOk = false;
	if (mag == nullptr) {
		return;
	}
	float m[3];
	for (uint8_t i = 0; i < 3; i++) {
		m[i] = (mag[i] - _offset[i]) * _scale[i];
	}
	_lastField = sqrtf(m[0] * m[0] + m[1] * m[1] + m[2] * m[2]);
	if (_lastField < 1.0f) {
		return;   // no reading
	}
	if (_field > 0 && fabsf(_lastField - _field) > _params.fieldTolerance * _field) {
		return;   // something magnetic nearby (servo current, steel): keep the gyro
	}
	_magOk = true;
	float h = lrWrapAngle(lrMagHeading(m, tilt));
	if (!_absolute) {
		_heading  = h;   // first good reading: take it
		_absolute = true;
		return;
	}
	float k = _params.magTau > 0 ? dt / _params.magTau : 1.0f;
	if (k > 1) {
		k = 1;
	}
	_heading = lrWrapAngle(_heading + lrWrapAngle(h - _heading) * k);
}

float HeadingEstimator::heading() const
{
	return _heading < 0 ? _heading + TWO_PI_F : _heading;
}

float HeadingHold::update(float stickYaw, float heading, float maxYawRate, bool moving)
{
	_error = 0;
	if (!_enabled) {
		return stickYaw;
	}
	if (fabsf(stickYaw) > _params.stickDeadband) {
		// the operator turns: hold whatever heading the robot has when they let go
		_turning = true;
		_target = heading;
		_hasTarget = true;
		return stickYaw;
	}
	if (_turning || !_hasTarget) {
		_turning = false;
		_target = heading;
		_hasTarget = true;
	}
	_error = lrWrapAngle(_target - heading);
	if (!moving && fabsf(_error) < _params.deadband) {
		return 0;   // standing and close enough: don't step for a degree or two
	}
	float rate = constrain(_params.gain * _error, -_params.maxRate, _params.maxRate);
	if (maxYawRate <= 0) {
		return 0;
	}
	// error > 0: turn clockwise, a positive yaw command
	return constrain(rate / maxYawRate, -1.0f, 1.0f);
}
