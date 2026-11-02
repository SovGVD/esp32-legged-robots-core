#include "gait_engine.h"

static const float LR_TWO_PI = 6.28318530718f;

static float lrApproach(float current, float target, float maxStep)
{
	if (target > current + maxStep) return current + maxStep;
	if (target < current - maxStep) return current - maxStep;
	return target;
}

static float lrFrac(float x)
{
	return x - floorf(x);
}

LR_point lrSwingPoint(const LR_point &from, const LR_point &to, float height, float s)
{
	s = constrain(s, 0.0f, 1.0f);
	// cycloid: 0 -> 1 with zero slope at both ends
	float k = s - sinf(LR_TWO_PI * s) / LR_TWO_PI;
	LR_point p;
	p.x = from.x + (to.x - from.x) * k;
	p.y = from.y + (to.y - from.y) * k;
	p.z = from.z + (to.z - from.z) * k + height * 0.5f * (1.0f - cosf(LR_TWO_PI * s));
	return p;
}

void GaitEngine::begin(const GaitParams &params, leg *legs, uint8_t legCount)
{
	_params   = params;
	_legs     = legs;
	_legCount = legCount > LR_MAX_LEGS ? LR_MAX_LEGS : legCount;
	reset();
}

void GaitEngine::reset()
{
	_body     = {};
	_velocity = {0, 0, 0};
	_yawRate  = 0;
	_phase    = 0;
	_active   = false;
	for (uint8_t i = 0; i < _legCount; i++) {
		_state[i] = {};
		_legs[i].foot = neutralFoot(i, _body.position, _body.orientation.yaw);
		_legs[i].sensor.onGround = true;
	}
}

LR_point GaitEngine::neutralFoot(uint8_t legId, const LR_point &bodyPosition, float bodyYaw) const
{
	const LR_point &d = _legs[legId].defaultFoot;
	LR_point p = lrRotateZ(d, sinf(bodyYaw), cosf(bodyYaw));
	p.x += bodyPosition.x;
	p.y += bodyPosition.y;
	p.z  = d.z;   // ground level does not follow the body
	return p;
}

bool GaitEngine::isAtNeutral(uint8_t legId) const
{
	LR_point n = neutralFoot(legId, _body.position, _body.orientation.yaw);
	float dx = _legs[legId].foot.x - n.x;
	float dy = _legs[legId].foot.y - n.y;
	float dz = _legs[legId].foot.z - n.z;
	return dx*dx + dy*dy + dz*dz <= _params.idleTolerance * _params.idleTolerance;
}

void GaitEngine::rampVelocity(float dt, const LR_moveVector &command)
{
	float targetX   = constrain(command.move.x,     -1.0f, 1.0f) * _params.maxSpeed;
	float targetY   = constrain(command.move.y,     -1.0f, 1.0f) * _params.maxSpeed;
	// negative to keep the turn direction of the original planner
	float targetYaw = -constrain(command.rotate.yaw, -1.0f, 1.0f) * _params.maxYawRate;

	_velocity.x = lrApproach(_velocity.x, targetX,   _params.maxAccel * dt);
	_velocity.y = lrApproach(_velocity.y, targetY,   _params.maxAccel * dt);
	_yawRate    = lrApproach(_yawRate,    targetYaw, _params.maxYawAccel * dt);
}

/**
 * Where the foot should land: the neutral foot position for the body pose at
 * the middle of the coming stance phase (predicted from current velocity).
 */
LR_point GaitEngine::touchdownTarget(uint8_t legId, float swingProgress) const
{
	float swingTime  = (1.0f - _params.dutyFactor) * _params.cycleTime;
	float stanceTime = _params.dutyFactor * _params.cycleTime;
	float t = (1.0f - swingProgress) * swingTime + 0.5f * stanceTime;

	float yaw = _body.orientation.yaw + _yawRate * t;
	// average heading over the prediction window
	float midYaw = _body.orientation.yaw + _yawRate * t * 0.5f;
	LR_point v = lrRotateZ(_velocity, sinf(midYaw), cosf(midYaw));
	LR_point bodyAt = { _body.position.x + v.x * t, _body.position.y + v.y * t, _body.position.z };

	return neutralFoot(legId, bodyAt, yaw);
}

void GaitEngine::update(float dt, const LR_moveVector &command)
{
	if (!_params.enabled || _legs == nullptr || dt <= 0) {
		return;
	}

	rampVelocity(dt, command);

	const float eps = 1e-3f;
	bool moving = fabsf(_velocity.x) > eps || fabsf(_velocity.y) > eps || fabsf(_yawRate) > eps;

	// body pose
	_body.orientation.yaw += _yawRate * dt;
	LR_point v = lrRotateZ(_velocity, sinf(_body.orientation.yaw), cosf(_body.orientation.yaw));
	_body.position.x += v.x * dt;
	_body.position.y += v.y * dt;

	// keep stepping while moving, or until every foot is back at neutral
	if (!moving && _active) {
		bool settled = true;
		for (uint8_t i = 0; i < _legCount; i++) {
			if (_state[i].swing || !isAtNeutral(i)) {
				settled = false;
				break;
			}
		}
		if (settled) {
			_active = false;
			_phase  = 0;   // next start: legs with offset >= duty swing first
		}
	} else if (moving) {
		_active = true;
	}

	if (!_active) {
		return;
	}

	_phase = lrFrac(_phase + dt / _params.cycleTime);

	float swingDuration = (1.0f - _params.dutyFactor) * _params.cycleTime;
	// a swing may only start at the beginning of its window, otherwise the foot would jump
	float startWindow = dt / swingDuration + 0.05f;

	for (uint8_t i = 0; i < _legCount; i++) {
		legState &st = _state[i];
		leg &l = _legs[i];
		float local = lrFrac(_phase + _params.phaseOffset[i]);
		bool inSwingWindow = local >= _params.dutyFactor;
		float s = inSwingWindow ? (local - _params.dutyFactor) / (1.0f - _params.dutyFactor) : 0.0f;

		if (!inSwingWindow) {
			if (st.swing) {
				// swing time is over: put the foot down on its target
				l.foot = touchdownTarget(i, 1.0f);
				st.swing = false;
			}
			st.skipped = false;
		} else if (!st.swing && !st.skipped) {
			if (s <= startWindow && (moving || !isAtNeutral(i))) {
				st.swing   = true;
				st.liftOff = l.foot;
			} else {
				st.skipped = true;
			}
		}

		if (st.swing) {
			if (_params.useContactSensors && s > 0.5f && l.sensor.onGround) {
				// touched down early: keep the foot where it is
				st.swing   = false;
				st.skipped = true;
				continue;
			}
			LR_point target = touchdownTarget(i, s);
			l.foot = lrSwingPoint(st.liftOff, target, _params.stepHeight, s);
		}

		if (!_params.useContactSensors) {
			// no sensors: the gait is the best estimate of contact
			l.sensor.onGround = !st.swing;
		}
	}
}
