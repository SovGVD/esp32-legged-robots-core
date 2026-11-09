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

LR_point lrSwingPointHigh(const LR_point &from, const LR_point &to, float apex, float s)
{
	s = constrain(s, 0.0f, 1.0f);
	const float pi = LR_TWO_PI * 0.5f;
	// horizontal: cycloid between 15 % and 85 % of the swing, after the foot has lifted
	float u = constrain((s - 0.15f) / 0.7f, 0.0f, 1.0f);
	float k = u - sinf(LR_TWO_PI * u) / LR_TWO_PI;
	LR_point p;
	p.x = from.x + (to.x - from.x) * k;
	p.y = from.y + (to.y - from.y) * k;
	// vertical: up to the apex in the first 40 %, down in the last 40 %
	if (s < 0.4f) {
		p.z = from.z + (apex - from.z) * 0.5f * (1.0f - cosf(pi * s / 0.4f));
	} else if (s > 0.6f) {
		p.z = apex + (to.z - apex) * 0.5f * (1.0f - cosf(pi * (s - 0.6f) / 0.4f));
	} else {
		p.z = apex;
	}
	return p;
}

void GaitEngine::begin(const GaitParams &params, leg *legs, uint8_t legCount)
{
	_params   = params;
	_legs     = legs;
	_legCount = legCount > LR_MAX_LEGS ? LR_MAX_LEGS : legCount;
	_sideLever = 0;
	for (uint8_t i = 0; i < _legCount; i++) {
		if (fabsf(_legs[i].defaultFoot.y) > _sideLever) _sideLever = fabsf(_legs[i].defaultFoot.y);
	}
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
	_terrainZ = 0;
	_blocked  = false;
	for (uint8_t i = 0; i < _legCount; i++) {
		_state[i] = {};
		_state[i].ground = _legs[i].defaultFoot.z;
		_legs[i].foot = neutralFoot(i, _base, _body.orientation.yaw);
		_legs[i].sensor.onGround = true;
	}
	updateShift(0);
}

void GaitEngine::setTerrain(const TerrainParams *params, const TerrainMap *map)
{
	_terrain = params;
	_map     = params ? map : nullptr;
	if (!params) {
		_blocked = false;
		for (uint8_t i = 0; i < _legCount; i++) {
			_state[i].ground = _legs[i].defaultFoot.z;
			_state[i].searching = false;
		}
	}
}

bool GaitEngine::holeLevel(float &floorZ, float &depth) const
{
	if (!_terrain || _legCount == 0) {
		return false;
	}
	float sum = 0;
	for (uint8_t i = 0; i < _legCount; i++) {
		sum += _state[i].ground;
	}
	floorZ = sum / _legCount - _terrain->footRadius;
	depth  = _terrain->maxStep + 10.0f;
	return true;
}

/**
 * Where a foot at `at` (world XY) should land: the mapped ground if the map
 * knows it and it is within reach of the leg's own ground, else the ground
 * the leg stands on now (the terrain goes on as it is).
 */
float GaitEngine::plannedGround(uint8_t legId, const LR_point &at) const
{
	float own = _state[legId].ground;
	float z;
	if (_map && !_map->isFace(at.x, at.y) && _map->heightAt(at.x, at.y, z)) {
		z += _terrain->footRadius;
		// within `tolerance` it is the same ground (map noise): following it
		// from step to step would let the feet wander
		if (fabsf(z - own) <= _terrain->tolerance) {
			return own;
		}
		if (fabsf(z - own) <= _terrain->maxStep) {
			return z;
		}
	}
	return own;
}

/**
 * The ground within `gapMargin` of (x, y) along the step direction (dx, dy)
 * drops away: a mapped point lower than the leg's own ground by more than
 * `maxStep`, or ground the sensors couldn't see between known ground (the
 * inside of a narrow gap, the shadow below a cliff edge). A foot there would
 * slide or fall in.
 */
bool GaitEngine::dropsAway(float x, float y, float dx, float dy, float own) const
{
	float cs = _map->params().cellSize;
	float r = _terrain->gapMargin > 0 ? _terrain->gapMargin : _terrain->footRadius + cs;
	float low = own - _terrain->footRadius - _terrain->maxStep;   // map heights are surface heights
	for (float d = -r; d <= r + 1e-3f; d += cs * 0.5f) {
		float px = x + dx * d, py = y + dy * d, z;
		if (_map->heightAt(px, py, z)) {
			if (z < low) {
				return true;
			}
		} else if (isShadow(px, py, dx, dy, own)) {
			return true;
		}
	}
	return false;
}

/**
 * (x, y) is unknown, known ground lies just before it along (dx, dy), and
 * what comes after says it is a hole the sensors couldn't see into: low
 * ground (the bottom of a wide gap, the floor past a cliff), or a low reading
 * with level ground right behind it (the far wall of a narrow gap). An
 * unknown cell before level ground is just a spot no zone hit; one before
 * lower ground that stays low is the shadow of a step down. Deep ground counts
 * even without known ground before the cell; the far wall needs it.
 */
bool GaitEngine::isShadow(float x, float y, float dx, float dy, float own) const
{
	float cs = _map->params().cellSize, z;
	bool before = false;
	for (float d = cs; d <= 8.0f * cs + 1e-3f && !before; d += cs) {
		before = _map->heightAt(x - dx * d, y - dy * d, z);
	}
	// no known ground before it: still a hole if deep ground follows (a cliff
	// edge beside the sensor's view, where the floor up to the edge was never
	// seen: the dog's feet stand wider than its sensor sees near the nose)
	float surface = own - _terrain->footRadius;
	float deep = surface - _terrain->maxStep;
	float low  = surface - _terrain->clearance;
	for (float d = cs; d <= 8.0f * cs + 1e-3f; d += cs) {
		if (!_map->heightAt(x + dx * d, y + dy * d, z)) {
			continue;
		}
		if (z < deep) {
			return true;
		}
		if (z >= low) {
			return false;
		}
		if (!before) {
			return false;
		}
		// lower than the foot's ground: a gap's far wall if level ground follows
		for (float e = d + cs; e <= d + 3.0f * cs + 1e-3f; e += cs) {
			if (_map->heightAt(x + dx * e, y + dy * e, z) && z >= low) {
				return true;
			}
		}
		return false;
	}
	return false;
}

/**
 * A foot can stand at (x, y): the ground doesn't drop away next to it and it
 * isn't on the edge of an obstacle across the step direction (dx, dy).
 */
bool GaitEngine::footholdOk(float x, float y, float dx, float dy, float own) const
{
	if (dropsAway(x, y, dx, dy, own)) {
		return false;
	}
	if (_terrain->edgeLimit > 0) {
		// the foot needs its own radius plus 1.5 cells of flat ground on both sides:
		// the cell holding an edge reads as high as the top (rays hit the face)
		float reach = _terrain->footRadius + 1.5f * _map->params().cellSize;
		if (_map->edge(x, y, dx, dy, reach) > _terrain->edgeLimit) {
			return false;
		}
	}
	return true;
}

/**
 * At lift-off (terrain on): pick the touchdown height, move the foothold off
 * an edge, and set how high the foot has to go to clear what lies between.
 */
void GaitEngine::planSwing(uint8_t legId)
{
	legState &st = _state[legId];
	LR_point target = touchdownTarget(legId, 0.0f);
	st.offset = { 0, 0, 0 };
	st.hold = false;

	if (_map && _terrain->footholdShift > 0) {
		// along the step: an edge or a gap lies across the path
		float dx = target.x - st.liftOff.x, dy = target.y - st.liftOff.y;
		float len = sqrtf(dx * dx + dy * dy);
		bool stepping = len >= 1.0f;   // not just settling in place
		if (!stepping) {
			dx = 0; dy = 1; len = 1;
		}
		dx /= len; dy /= len;
		float own = st.ground;
		if (!footholdOk(target.x, target.y, dx, dy, own)) {
			// nearest good spot along the step, beyond first (onto the obstacle, across the gap)
			bool found = false, beyond = false;
			float cs = _map->params().cellSize;
			for (float d = cs * 0.5f; d <= _terrain->footholdShift + 1e-3f && !found; d += cs * 0.5f) {
				for (int sign = 1; sign >= -1; sign -= 2) {
					float ox = dx * d * sign, oy = dy * d * sign;
					if (footholdOk(target.x + ox, target.y + oy, dx, dy, own)) {
						st.offset = { ox, oy, 0 };
						found = true;
						beyond = sign > 0;
						break;
					}
				}
			}
			if (stepping && !beyond && dropsAway(target.x, target.y, dx, dy, own)) {
				// a gap ahead and this step stays in front of it: can the next one
				// (a step length further) get across?
				bool later = false;
				for (float d = _terrain->footholdShift; d <= _terrain->footholdShift + len + 1e-3f && !later; d += cs * 0.5f) {
					later = footholdOk(target.x + dx * d, target.y + dy * d, dx, dy, own);
				}
				if (!later) {
					// too wide to step over: stop going that way, the foot stays in front of it
					st.hold = !found;
					float yaw = _body.orientation.yaw;
					_blockedDir = lrRotateZ({ dx, dy, 0 }, -sinf(yaw), cosf(yaw));
					_blocked = true;
				}
			}
		}
	}

	LR_point at = { target.x + st.offset.x, target.y + st.offset.y, 0 };
	float ground = plannedGround(legId, at);
	float top = st.liftOff.z > ground ? st.liftOff.z : ground;
	st.apex = top + _params.stepHeight;
	float obstacle;
	if (_map && _map->maxHeightAlong(st.liftOff, at, _map->params().cellSize * 0.5f, obstacle)) {
		obstacle += _terrain->footRadius;
		if (obstacle - top <= _terrain->maxStep && obstacle + _terrain->clearance > st.apex) {
			st.apex = obstacle + _terrain->clearance;
		}
	}
	st.ground = ground;
}

/**
 * Height of a foot that just touched down, in the level gait frame: the leg
 * put it at leg.foot relative to the body, but the body is tilted by _tilt,
 * so the foot really is that much higher or lower.
 */
float GaitEngine::contactHeight(uint8_t legId) const
{
	const LR_point &f = _legs[legId].foot;
	LR_point rel = { f.x - _body.position.x, f.y - _body.position.y, f.z - _body.position.z };
	float yaw = _body.orientation.yaw;
	rel = lrRotateZ(rel, -sinf(yaw), cosf(yaw));
	LR_angle tilt = { _tilt.pitch, _tilt.roll, 0 };
	float z = _body.position.z + lrRotateBody(rel, tilt).z;
	float planned = _state[legId].ground;
	if (fabsf(z - planned) <= _terrain->tolerance) {
		return planned;
	}
	return z;
}

/**
 * Body height follows the average ground under the feet (planned ground for
 * the swinging ones), so it rises when the robot climbs onto something.
 */
void GaitEngine::updateTerrainHeight(float dt)
{
	if (!_terrain || _terrain->bodyRate <= 0 || _legCount == 0) {
		return;
	}
	float sum = 0;
	for (uint8_t i = 0; i < _legCount; i++) {
		sum += _state[i].ground - _legs[i].defaultFoot.z;
	}
	_terrainZ = lrApproach(_terrainZ, sum / _legCount, _terrain->bodyRate * dt);
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
	// on uneven ground a foot stands where the ground is, only XY counts
	float dz = _terrain ? 0.0f : _legs[legId].foot.z - n.z;
	return dx*dx + dy*dy + dz*dz <= _params.idleTolerance * _params.idleTolerance;
}

void GaitEngine::rampVelocity(float dt, const LR_moveVector &command)
{
	float targetX   = constrain(command.move.x,     -1.0f, 1.0f) * _params.maxSpeed;
	float targetY   = constrain(command.move.y,     -1.0f, 1.0f) * _params.maxSpeed;
	// negative to keep the turn direction of the original planner
	float targetYaw = -constrain(command.rotate.yaw, -1.0f, 1.0f) * _params.maxYawRate;

	if (_params.maxSideSpeed > 0) {
		// strafing and turning both swing the feet sideways
		float side = fabsf(targetX) + fabsf(targetYaw) * _sideLever;
		if (side > _params.maxSideSpeed) {
			float k = _params.maxSideSpeed / side;
			targetX   *= k;
			targetYaw *= k;
		}
	}

	if (_blocked) {
		// a gap ahead: no further that way until the command stops pushing that way
		float push = command.move.x * _blockedDir.x + command.move.y * _blockedDir.y;
		if (push <= 0.05f) {
			_blocked = false;
		} else {
			float along = targetX * _blockedDir.x + targetY * _blockedDir.y;
			if (along > 0) {
				targetX -= along * _blockedDir.x;
				targetY -= along * _blockedDir.y;
			}
		}
	}

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
			// at a gap the feet stay where they are: no settling steps next to the edge
			if (_state[i].swing || _state[i].searching || (!isAtNeutral(i) && !_blocked)) {
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
		updateSearch(dt);
		updateTerrainHeight(dt);
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
			if (_terrain && _params.useContactSensors && !st.swing && !st.searching && st.landed) {
				// a standing foot that lost the ground (slid off an edge): reach down again
				// (leaky: a foot resting on an edge makes the switch flicker)
				st.lost = l.sensor.onGround ? (st.lost > dt ? st.lost - dt : 0.0f) : st.lost + dt;
				if (st.lost > _terrain->latency && local < _params.dutyFactor * 0.8f) {
					st.searching = true;
					st.lost = 0;
				}
			}
			if (st.swing) {
				// swing time is over: put the foot down on its target
				l.foot = touchdownTarget(i, 1.0f);
				st.swing = false;
				st.landed = true;
				if (_terrain) {
					if (st.hold) {
						l.foot.x = st.liftOff.x - st.offset.x;
						l.foot.y = st.liftOff.y - st.offset.y;
					}
					l.foot.x += st.offset.x;
					l.foot.y += st.offset.y;
					l.foot.z  = st.ground;
					// not down yet: keep reaching for the ground (updateSearch)
					st.searching = _params.useContactSensors && !l.sensor.onGround && _terrain->searchDepth > 0;
				}
			}
			st.skipped = false;
		} else if (!st.swing && !st.skipped) {
			if (s <= startWindow && (moving || (!isAtNeutral(i) && !_blocked))) {
				st.swing     = true;
				st.searching = false;
				st.released  = false;
				st.liftOff   = l.foot;
				if (_terrain) {
					planSwing(i);
				}
			} else {
				st.skipped = true;
			}
		}

		if (st.swing) {
			if (!l.sensor.onGround) {
				st.released = true;
			}
			// on rough ground only once the foot is over its foothold and coming
			// down (the high swing moves until s = 0.85): earlier it is an edge
			// it brushes on the way, the swing goes on
			float early = _terrain ? 0.8f : 0.5f;
			if (_params.useContactSensors && s > early && l.sensor.onGround && st.released) {
				// touched down early: keep the foot where it is
				st.swing   = false;
				st.skipped = true;
				st.landed  = true;
				if (_terrain) {
					st.ground = contactHeight(i);
				}
				continue;
			}
			LR_point target = touchdownTarget(i, s);
			if (_terrain) {
				if (st.hold) {
					target.x = st.liftOff.x - st.offset.x;
					target.y = st.liftOff.y - st.offset.y;
				}
				target.x += st.offset.x;
				target.y += st.offset.y;
				target.z  = st.ground;
				l.foot = lrSwingPointHigh(st.liftOff, target, st.apex, s);
			} else {
				l.foot = lrSwingPoint(st.liftOff, target, _params.stepHeight, s);
			}
		}

		if (!_params.useContactSensors) {
			// no sensors: the gait is the best estimate of contact
			l.sensor.onGround = !st.swing;
		}
	}

	updateSearch(dt);
	updateTerrainHeight(dt);
	updateShift(dt);
}


/**
 * Feet past their swing time without contact: lower them until the sensor
 * says they are down, or `searchDepth` below where they were meant to land
 * (a hole: stand there anyway).
 */
void GaitEngine::updateSearch(float dt)
{
	if (!_terrain) {
		return;
	}
	for (uint8_t i = 0; i < _legCount; i++) {
		legState &st = _state[i];
		if (!st.searching) {
			continue;
		}
		leg &l = _legs[i];
		float limit = st.ground - _terrain->searchDepth;   // st.ground: the planned touchdown
		if (l.sensor.onGround) {
			// the foot is really where it was commanded `latency` ago:
			// put it back up by what it has been lowered since
			st.searching = false;
			float back = _terrain->searchSpeed * _terrain->latency;
			if (back > st.ground - l.foot.z) {
				back = st.ground - l.foot.z;   // not above where the search started
			}
			l.foot.z += back > 0 ? back : 0;
			st.ground = contactHeight(i);
			continue;
		}
		if (l.foot.z <= limit) {
			st.searching = false;   // nothing there: stand as low as the search goes
			st.ground = l.foot.z;
			continue;
		}
		l.foot.z = lrApproach(l.foot.z, limit, _terrain->searchSpeed * dt);
	}
}

/**
 * Support weight of a foot for the static stability shift: 0 in swing, 1 in
 * mid stance, smoothstep ramps after touchdown and before lift-off.
 */
float GaitEngine::supportWeight(uint8_t legId) const
{
	const legState &st = _state[legId];
	if (st.swing || st.searching) {
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
	float sy = sinf(_body.orientation.yaw), cy = cosf(_body.orientation.yaw);
	// the centre of mass always sits over the middle of the neutral stance: a
	// constant offset, in every gait (an off-centre mass makes a trot rock over
	// the diagonals and drift towards the heavy end)
	LR_point com = lrRotateZ(_params.centerOfMass, sy, cy);
	LR_point target = { -com.x, -com.y, 0 };
	if (_params.staticStability) {
		float sum = 0, x = 0, y = 0;
		for (uint8_t i = 0; i < _legCount; i++) {
			float w = supportWeight(i);
			x   += w * _legs[i].foot.x;
			y   += w * _legs[i].foot.y;
			sum += w;
		}
		if (sum > 1e-3f) {
			// and moves towards the weighted feet centroid, scaled per body axis
			LR_point w = { x / sum - _base.x, y / sum - _base.y, 0 };
			LR_point b = lrRotateZ(w, -sy, cy);
			b.x *= constrain(_params.stabilityGain.x, 0.0f, 1.0f);
			b.y *= constrain(_params.stabilityGain.y, 0.0f, 1.0f);
			b = lrRotateZ(b, sy, cy);
			target.x += b.x;
			target.y += b.y;
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
	_body.position.z = _base.z + _terrainZ;
}

float GaitEngine::supportMargin() const
{
	LR_point feet[LR_MAX_LEGS];
	uint8_t n = 0;
	for (uint8_t i = 0; i < _legCount; i++) {
		if (!_state[i].swing && !_state[i].searching) {
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
