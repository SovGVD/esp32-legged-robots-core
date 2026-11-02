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

bool lrClampJointLimits(const leg &l, LR_legAngle &a, uint8_t dof)
{
	const float eps = 1e-4f;
	float *v[6]       = { &a.alpha, &a.beta, &a.gamma, &a.delta, &a.epsilon, &a.zeta };
	const float lo[6] = { l.min.alpha, l.min.beta, l.min.gamma, l.min.delta, l.min.epsilon, l.min.zeta };
	const float hi[6] = { l.max.alpha, l.max.beta, l.max.gamma, l.max.delta, l.max.epsilon, l.max.zeta };
	bool clamped = false;
	if (dof > 6) dof = 6;
	for (uint8_t j = 0; j < dof; j++) {
		if (*v[j] < lo[j] - eps) { *v[j] = lo[j]; clamped = true; }
		if (*v[j] > hi[j] + eps) { *v[j] = hi[j]; clamped = true; }
	}
	return clamped;
}
