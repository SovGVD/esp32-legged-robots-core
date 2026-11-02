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

static float lrSmoothstep(float x)
{
	x = constrain(x, 0.0f, 1.0f);
	return x * x * (3.0f - 2.0f * x);
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
	computeRamps();
	reset();
}

void GaitEngine::reset()
{
	_body     = {};
	_base     = {0, 0, 0};
	_shift    = {0, 0, 0};
	_velocity = {0, 0, 0};
	_yawRate  = 0;
	_phase    = 0;
	_active   = false;
	for (uint8_t i = 0; i < _legCount; i++) {
		_state[i] = {};
		_legs[i].foot = neutralFoot(i, _base, _body.orientation.yaw);
		_legs[i].sensor.onGround = true;
	}
	updateShift(0);
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
	LR_point n = neutralFoot(legId, _base, _body.orientation.yaw);
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
	LR_point bodyAt = { _base.x + v.x * t, _base.y + v.y * t, _base.z };

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
	_base.x += v.x * dt;
	_base.y += v.y * dt;

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
	} else if (moving && !_active) {
		_active = true;
		if (_params.staticStability) {
			// start so that the first leg lifts only after the body had time to shift
			float first = 1.0f;
			for (uint8_t i = 0; i < _legCount; i++) {
				float lift = lrFrac(_params.dutyFactor - _params.phaseOffset[i]);
				if (lift < first) first = lift;
			}
			_phase = lrFrac(first - _params.stabilityBlend);
			for (uint8_t i = 0; i < _legCount; i++) {
				_state[i].landed = false;
			}
		}
	}

	if (!_active) {
		updateShift(dt);
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
				st.landed = true;
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

	updateShift(dt);
}

/**
 * Support weight of a foot for the static stability shift: 0 in swing, 1 in
 * mid stance, smoothstep ramps after touchdown and before lift-off.
 */
float GaitEngine::supportWeight(uint8_t legId) const
{
	const legState &st = _state[legId];
	if (st.swing) {
		return 0.0f;
	}
	if (!_active) {
		return 1.0f;
	}
	float local = lrFrac(_phase + _params.phaseOffset[legId]);
	if (local >= _params.dutyFactor) {
		return 1.0f;   // swing window skipped, the foot stays down
	}
	float w = 1.0f;
	if (st.landed && _rampUp[legId] > 0.0f && local < _rampUp[legId]) {
		w = lrSmoothstep(local / _rampUp[legId]);
	}
	float untilLift = _params.dutyFactor - local;
	if (_rampDown[legId] > 0.0f && untilLift < _rampDown[legId]) {
		float d = lrSmoothstep(untilLift / _rampDown[legId]);
		if (d < w) w = d;
	}
	return w;
}

/**
 * Support weight ramp lengths (fractions of the cycle) per leg. A foot may
 * only hand over its load while every other foot it relies on is down, so a
 * ramp before lift-off ends no earlier than the previous touchdown of any
 * other leg, and a ramp after touchdown is done before the next lift-off.
 * Then the landing foot takes over exactly what the lifting foot gives away
 * and the centre of mass moves straight from one support triangle to the
 * next during the four-foot phase.
 */
void GaitEngine::computeRamps()
{
	float swing = 1.0f - _params.dutyFactor;
	for (uint8_t i = 0; i < _legCount; i++) {
		float liftI = lrFrac(_params.dutyFactor - _params.phaseOffset[i]);
		float touchI = lrFrac(liftI + swing);
		float down = _params.stabilityBlend;
		float up = _params.stabilityBlend;
		for (uint8_t j = 0; j < _legCount; j++) {
			float liftJ = lrFrac(_params.dutyFactor - _params.phaseOffset[j]);
			float touchJ = lrFrac(liftJ + swing);
			if (j != i && fabsf(liftJ - liftI) > 1e-4f) {
				float gap = lrFrac(liftI - touchJ);   // since j's last touchdown
				if (gap < down) down = gap;
			}
			float next = lrFrac(liftJ - touchI);      // until the next lift-off
			if (next > 1e-4f && next < up) up = next;
		}
		_rampDown[i] = down > 0 ? down : 0;
		_rampUp[i] = up > 0 ? up : 0;
	}
}

LR_point GaitEngine::centerOfMass() const
{
	LR_point c = lrRotateZ(_params.centerOfMass, sinf(_body.orientation.yaw), cosf(_body.orientation.yaw));
	c.x += _body.position.x;
	c.y += _body.position.y;
	c.z  = 0;
	return c;
}

void GaitEngine::updateShift(float dt)
{
	LR_point target = {0, 0, 0};
	if (_params.staticStability) {
		float sum = 0, x = 0, y = 0;
		for (uint8_t i = 0; i < _legCount; i++) {
			float w = supportWeight(i);
			x   += w * _legs[i].foot.x;
			y   += w * _legs[i].foot.y;
			sum += w;
		}
		if (sum > 1e-3f) {
			// the centre of mass goes over the weighted feet centroid
			LR_point com = lrRotateZ(_params.centerOfMass, sinf(_body.orientation.yaw), cosf(_body.orientation.yaw));
			LR_point w = { x / sum - _base.x - com.x, y / sum - _base.y - com.y, 0 };
			// scale per body axis
			float sy = sinf(_body.orientation.yaw), cy = cosf(_body.orientation.yaw);
			LR_point b = lrRotateZ(w, -sy, cy);
			b.x *= constrain(_params.stabilityGain.x, 0.0f, 1.0f);
			b.y *= constrain(_params.stabilityGain.y, 0.0f, 1.0f);
			target = lrRotateZ(b, sy, cy);
		} else {
			target = _shift;
		}
	}

	if (dt <= 0) {
		_shift = target;
	} else {
		// the weights are continuous; this only smooths jumps (gait start/stop, parameter change)
		const float maxShiftSpeed = 1000.0f;   // mm/s
		_shift.x = lrApproach(_shift.x, target.x, maxShiftSpeed * dt);
		_shift.y = lrApproach(_shift.y, target.y, maxShiftSpeed * dt);
	}
	_body.position.x = _base.x + _shift.x;
	_body.position.y = _base.y + _shift.y;
	_body.position.z = _base.z;
}

float GaitEngine::supportMargin() const
{
	LR_point feet[LR_MAX_LEGS];
	uint8_t n = 0;
	for (uint8_t i = 0; i < _legCount; i++) {
		if (!_state[i].swing) {
			feet[n++] = _legs[i].foot;
		}
	}
	return lrSupportMargin(feet, n, centerOfMass());
}

static float cross2(const LR_point &o, const LR_point &a, const LR_point &b)
{
	return (a.x - o.x) * (b.y - o.y) - (a.y - o.y) * (b.x - o.x);
}

static float segmentDistance(const LR_point &a, const LR_point &b, const LR_point &p)
{
	float dx = b.x - a.x, dy = b.y - a.y;
	float len2 = dx * dx + dy * dy;
	float t = len2 > 0 ? ((p.x - a.x) * dx + (p.y - a.y) * dy) / len2 : 0;
	t = constrain(t, 0.0f, 1.0f);
	float ex = a.x + t * dx - p.x, ey = a.y + t * dy - p.y;
	return sqrtf(ex * ex + ey * ey);
}

float lrSupportMargin(const LR_point *points, uint8_t count, const LR_point &p)
{
	if (count == 0) {
		return -1e9f;
	}
	if (count == 1) {
		return -segmentDistance(points[0], points[0], p);
	}

	// convex hull (gift wrapping, n <= LR_MAX_LEGS), counter-clockwise
	LR_point hull[LR_MAX_LEGS];
	uint8_t h = 0;
	uint8_t start = 0;
	for (uint8_t i = 1; i < count; i++) {
		if (points[i].x < points[start].x || (points[i].x == points[start].x && points[i].y < points[start].y)) {
			start = i;
		}
	}
	uint8_t cur = start;
	do {
		hull[h++] = points[cur];
		uint8_t next = (cur + 1) % count;
		for (uint8_t i = 0; i < count; i++) {
			if (cross2(points[cur], points[next], points[i]) < 0) {
				next = i;
			}
		}
		cur = next;
	} while (cur != start && h < count);

	if (h < 3) {
		return -segmentDistance(hull[0], hull[h - 1], p);
	}

	float margin = 1e9f;
	bool inside = true;
	for (uint8_t i = 0; i < h; i++) {
		const LR_point &a = hull[i];
		const LR_point &b = hull[(i + 1) % h];
		float len = sqrtf((b.x - a.x) * (b.x - a.x) + (b.y - a.y) * (b.y - a.y));
		if (len <= 0) {
			continue;
		}
		float d = cross2(a, b, p) / len;   // > 0: left of the edge, inside a CCW hull
		if (d < 0) {
			inside = false;
		}
		if (d < margin) {
			margin = d;
		}
	}
	if (inside) {
		return margin;
	}
	// outside: distance to the nearest hull edge
	float best = 1e9f;
	for (uint8_t i = 0; i < h; i++) {
		float d = segmentDistance(hull[i], hull[(i + 1) % h], p);
		if (d < best) best = d;
	}
	return -best;
}
