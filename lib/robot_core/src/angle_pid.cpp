#include "angle_pid.h"

AnglePID::AnglePID(float Kp, float Ki, float Kd, float minIntegral, float maxIntegral)
	: _Kp(Kp), _Ki(Ki), _Kd(Kd), _minIntegral(minIntegral), _maxIntegral(maxIntegral)
{
}

void AnglePID::set(float Kp, float Ki, float Kd)
{
	_Kp = Kp;
	_Ki = Ki;
	_Kd = Kd;
}

void AnglePID::reset()
{
	_hasPrevious = false;
	_integral    = {0, 0, 0};
}

float AnglePID::axis(float current, float target, float previous, float &integral, float dt)
{
	float error = target - current;
	integral = constrain(integral + _Ki * error * dt, _minIntegral, _maxIntegral);
	// derivative on measurement, avoids a kick when the target changes
	float dInput = (_hasPrevious && dt > 0) ? (current - previous) / dt : 0.0f;
	return _Kp * error + integral - _Kd * dInput;
}

LR_angle AnglePID::update(const LR_angle &current, const LR_angle &target, float dt)
{
	LR_angle output;
	output.pitch = axis(current.pitch, target.pitch, _previous.pitch, _integral.pitch, dt);
	output.roll  = axis(current.roll,  target.roll,  _previous.roll,  _integral.roll,  dt);
	output.yaw   = axis(current.yaw,   target.yaw,   _previous.yaw,   _integral.yaw,   dt);
	_previous    = current;
	_hasPrevious = true;
	return output;
}
