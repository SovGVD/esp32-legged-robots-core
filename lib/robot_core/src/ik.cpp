#include "ik.h"

LR_point LegIK::legVector(const leg &l, const LR_figure &body)
{
	// Normalize by body rotation, so IK can ignore body transitions and
	// still use absolute positions for each robot point.
	float s = sinf(-body.orientation.yaw);
	float c = cosf(-body.orientation.yaw);

	LR_point foot = { l.foot.x - body.position.x, l.foot.y - body.position.y, l.foot.z };
	LR_point shoulder = { l.body.x - body.position.x, l.body.y - body.position.y, l.body.z };
	foot     = lrRotateZ(foot, s, c);
	shoulder = lrRotateZ(shoulder, s, c);

	LR_point v = { foot.x - shoulder.x, foot.y - shoulder.y, foot.z - shoulder.z };
	if (l.inverse.x) v.x = -v.x;
	if (l.inverse.y) v.y = -v.y;
	if (l.inverse.z) v.z = -v.z;
	return v;
}

float LegIK::clampUnit(float value)
{
	if (isnan(value)) {
		_clamped = true;
		return 1.0f;
	}
	if (value > 1.0f) {
		_clamped = true;
		return 1.0f;
	}
	if (value < -1.0f) {
		_clamped = true;
		return -1.0f;
	}
	return value;
}
