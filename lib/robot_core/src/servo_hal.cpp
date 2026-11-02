#include "servo_hal.h"
#include "lr_compat.h"

float lrLimitServoAngle(float angle)
{
	// TODO this is not about the real mechanical limits of each joint
	if (angle < 0.0f) angle = 0.0f;
	if (angle > (float)M_PI) angle = M_PI;
	return angle;
}

float lrHalAngle(float angle, float mid, float trimAngle, float gearRatio, bool inverse)
{
	angle = (angle - mid) * gearRatio + (float)M_PI_2;
	if (inverse) angle = (float)M_PI - angle;
	return angle + trimAngle;
}

static float mapRange(float val, float inMin, float inMax, float outMin, float outMax)
{
	return (val - inMin) * (outMax - outMin) / (inMax - inMin) + outMin;
}

uint16_t lrAngleToPulse(const servoProfile &p, float angleRad)
{
	float deg = angleRad * (180.0f / (float)M_PI);
	if (deg < p.minAngle) deg = p.minAngle;
	if (deg > p.maxAngle) deg = p.maxAngle;

	const float    a[9] = { (float)p.minAngle, 30, 50, 70, 90, 110, 130, 150, (float)p.maxAngle };
	const uint16_t u[9] = { p.degMin, p.deg30, p.deg50, p.deg70, p.deg90, p.deg110, p.deg130, p.deg150, p.degMax };
	for (uint8_t i = 1; i < 8; i++) {
		if (deg < a[i]) {
			return (uint16_t)mapRange(deg, a[i - 1], a[i], u[i - 1], u[i]);
		}
	}
	return (uint16_t)mapRange(deg, a[7], a[8], u[7], u[8]);
}

float lrLegAngleFromServo(float servoAngle, float mid, float trimAngle, float gearRatio, bool inverse)
{
	float a = servoAngle - trimAngle;
	if (inverse) a = (float)M_PI - a;
	return (a - (float)M_PI_2) / gearRatio + mid;
}

bool lrClampServoRange(const leg &l, LR_legAngle &a, uint8_t dof, const servoProfile &p)
{
	float *angle[6]      = { &a.alpha, &a.beta, &a.gamma, &a.delta, &a.epsilon, &a.zeta };
	const float mid[6]   = { l.hal.mid.alpha, l.hal.mid.beta, l.hal.mid.gamma, l.hal.mid.delta, l.hal.mid.epsilon, l.hal.mid.zeta };
	const float trim[6]  = { l.hal.trim.alpha, l.hal.trim.beta, l.hal.trim.gamma, l.hal.trim.delta, l.hal.trim.epsilon, l.hal.trim.zeta };
	const float ratio[6] = { l.hal.ratio.alpha, l.hal.ratio.beta, l.hal.ratio.gamma, l.hal.ratio.delta, l.hal.ratio.epsilon, l.hal.ratio.zeta };
	const bool  inv[6]   = { l.inverse.alpha, l.inverse.beta, l.inverse.gamma, l.inverse.delta, l.inverse.epsilon, l.inverse.zeta };
	float lo = p.minAngle * ((float)M_PI / 180.0f);
	float hi = p.maxAngle * ((float)M_PI / 180.0f);
	if (lo < 0) lo = 0;
	if (hi > (float)M_PI) hi = M_PI;
	bool clamped = false;
	if (dof > 6) dof = 6;
	for (uint8_t j = 0; j < dof; j++) {
		if (ratio[j] == 0) continue;
		float s = lrHalAngle(*angle[j], mid[j], trim[j], ratio[j], inv[j]);
		const float eps = 1e-4f;
		if (s < lo - eps || s > hi + eps) {
			float limit = s < lo ? lo : hi;
			*angle[j] = lrLegAngleFromServo(limit, mid[j], trim[j], ratio[j], inv[j]);
			clamped = true;
		}
	}
	return clamped;
}

void lrLegPulses(const leg &l, uint8_t dof, const servoProfile &profile, uint16_t *pulses)
{
	const float angle[6] = { l.angle.alpha, l.angle.beta, l.angle.gamma, l.angle.delta, l.angle.epsilon, l.angle.zeta };
	const float mid[6]   = { l.hal.mid.alpha, l.hal.mid.beta, l.hal.mid.gamma, l.hal.mid.delta, l.hal.mid.epsilon, l.hal.mid.zeta };
	const float trim[6]  = { l.hal.trim.alpha, l.hal.trim.beta, l.hal.trim.gamma, l.hal.trim.delta, l.hal.trim.epsilon, l.hal.trim.zeta };
	const float ratio[6] = { l.hal.ratio.alpha, l.hal.ratio.beta, l.hal.ratio.gamma, l.hal.ratio.delta, l.hal.ratio.epsilon, l.hal.ratio.zeta };
	const bool  inv[6]   = { l.inverse.alpha, l.inverse.beta, l.inverse.gamma, l.inverse.delta, l.inverse.epsilon, l.inverse.zeta };
	if (dof > 6) dof = 6;
	for (uint8_t j = 0; j < dof; j++) {
		pulses[j] = lrAngleToPulse(profile, lrLimitServoAngle(lrHalAngle(angle[j], mid[j], trim[j], ratio[j], inv[j])));
	}
}

uint8_t lrLegPin(const leg &l, uint8_t joint)
{
	switch (joint) {
		case 0:  return l.hal.pin.alpha;
		case 1:  return l.hal.pin.beta;
		case 2:  return l.hal.pin.gamma;
		case 3:  return l.hal.pin.delta;
		case 4:  return l.hal.pin.epsilon;
		default: return l.hal.pin.zeta;
	}
}
