#include "diff_drive.h"

static float slew(float current, float target, float maxStep)
{
	float d = target - current;
	if (d >  maxStep) d =  maxStep;
	if (d < -maxStep) d = -maxStep;
	return current + d;
}

void lrDiffDriveMix(float forward, float turn, float &left, float &right)
{
	forward = constrain(forward, -1.0f, 1.0f);
	turn    = constrain(turn,    -1.0f, 1.0f);
	left  = forward + turn;
	right = forward - turn;
	float m = fmaxf(fabsf(left), fabsf(right));
	if (m > 1.0f) {
		left  /= m;
		right /= m;
	}
}

DiffDriveDuty lrDiffDriveDuty(float speed, const DiffDriveParams &params)
{
	DiffDriveDuty d = { 0, 0 };
	float a = fabsf(speed);
	if (a < params.deadband || a == 0) {
		return d;
	}
	if (a > 1) a = 1;
	uint16_t duty = (uint16_t)lroundf(params.minDuty + a * (float)(params.maxDuty - params.minDuty));
	if (speed > 0) {
		d.in1 = duty;
	} else {
		d.in2 = duty;
	}
	return d;
}

void DiffDrive::begin(const DiffDriveParams &params)
{
	_params = params;
	stop();
}

void DiffDrive::update(float dt, float forward, float turn)
{
	float l, r;
	lrDiffDriveMix(forward, turn, l, r);
	float step = _params.maxAccel * dt;
	_left  = slew(_left,  l, step);
	_right = slew(_right, r, step);
}

void DiffDrive::stop()
{
	_left  = 0;
	_right = 0;
}
