#include <math.h>
#include "body_leveler.h"

static float clampf(float v, float lo, float hi)
{
	return v < lo ? lo : (v > hi ? hi : v);
}

void BodyLeveler::begin(const LevelerParams &params)
{
	_params = params;
	reset();
}

void BodyLeveler::reset()
{
	_hasPrevious = false;
	_filtered    = { 0, 0, 0 };
	_integral    = { 0, 0, 0 };
	_correction  = { 0, 0, 0 };
}

void BodyLeveler::setEnabled(bool enabled)
{
	if (enabled && !_enabled) {
		// start from the correction we still have, so turning on doesn't jump
		_integral    = _correction;
		_hasPrevious = false;
	}
	_enabled = enabled;
}

float BodyLeveler::axis(float dt, float measured, float previous, float target, float &integral, float current)
{
	// Tilted by +e, the body has to be turned by -e. BodyKinematics subtracts
	// the correction, so the correction follows the error itself.
	float error = measured - target;
	float slow  = fabsf(error) <= _params.deadband ? 0.0f
	            : error - (error > 0 ? _params.deadband : -_params.deadband);
	integral = clampf(integral + _params.ki * slow * dt, -_params.limit, _params.limit);
	float rate = dt > 0 ? (measured - previous) / dt : 0;
	float out  = clampf(_params.kp * error + integral + _params.kd * rate, -_params.limit, _params.limit);
	float step = _params.rate * dt;
	return current + clampf(out - current, -step, step);
}

const LR_angle &BodyLeveler::update(float dt, const LR_angle &measured, const LR_angle &target, bool hold)
{
	if (!_enabled) {
		float step = _params.rate * dt;
		_correction.pitch -= clampf(_correction.pitch, -step, step);
		_correction.roll  -= clampf(_correction.roll,  -step, step);
		_correction.yaw    = 0;
		_hasPrevious = false;
		return _correction;
	}

	if (hold) {
		_hasPrevious = false;
		return _correction;
	}

	LR_angle previous = _filtered;
	if (!_hasPrevious || _params.filter <= 0) {
		_filtered = measured;
		if (!_hasPrevious) previous = measured;
	} else {
		float a = dt / (_params.filter + dt);
		_filtered.pitch += a * (measured.pitch - _filtered.pitch);
		_filtered.roll  += a * (measured.roll  - _filtered.roll);
	}
	_hasPrevious = true;

	_correction.pitch = axis(dt, _filtered.pitch, previous.pitch, target.pitch, _integral.pitch, _correction.pitch);
	_correction.roll  = axis(dt, _filtered.roll,  previous.roll,  target.roll,  _integral.roll,  _correction.roll);
	_correction.yaw   = 0;   // yaw is the heading, not a tilt
	return _correction;
}
