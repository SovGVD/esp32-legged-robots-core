#include "tof_scan.h"
#include "heading.h"
#include <string.h>

/* ------------------------------------------------------------ TofScanBuilder */

void TofScanBuilder::start(const LR_pose2 &odom)
{
	_cx = odom.x;
	_cy = odom.y;
	memset(_r, 0, sizeof(_r));
	memset(_clear, 0, sizeof(_clear));
	_points = 0;
	_readings = 0;
}

/** Zone ray and sensor position in the level body frame (the body's tilt taken out). */
static void levelRay(const TofSensor &s, uint8_t zone, const LR_angle &tilt, LR_point &origin, LR_point &dir)
{
	LR_angle t = { tilt.pitch, tilt.roll, 0 };
	origin = lrRotateBody(s.position, t);
	dir = lrRotateBody(TerrainMap::zoneDirection(s, zone), t);
}

void TofScanBuilder::addFrame(const TofSensor &s, const float *mm, const LR_pose2 &odom, const LR_angle &tilt, float floorZ)
{
	uint8_t zones = s.zones ? s.zones * s.zones : 1;
	float maxDz = sinf(_params.band);
	// a zone reports the closest target in its whole cone: the floor under its
	// lower edge comes back before the wall its centre looks at
	float halfZone = s.fov / (s.zones ? s.zones : 1) * 0.5f;
	for (uint8_t z = 0; z < zones; z++) {
		float d = mm[z];
		bool nothing = d <= 0 || d > _params.maxRange;
		if (!nothing && d < _params.minRange) {
			continue;
		}
		LR_point o, dir;
		levelRay(s, z, tilt, o, dir);
		if (fabsf(dir.z) > maxDz) {
			continue;   // looking at the floor (or the ceiling)
		}
		float lowZ = sinf(asinf(dir.z) - halfZone);
		if (nothing) {
			// empty as far as the sensor reaches, or until the zone's lower edge
			// would come down to the floor (it would have seen that)
			float free = _params.maxRange;
			if (lowZ < 0) {
				float toFloor = (o.z - floorZ - _params.minHeight) / -lowZ;
				free = toFloor < free ? toFloor : free;
			}
			if (free < _params.minRange) {
				continue;
			}
			float wx, wy;
			lrPoseApply(odom, o.x + dir.x * free, o.y + dir.y * free, wx, wy);
			float rx = wx - _cx, ry = wy - _cy;
			float r = sqrtf(rx * rx + ry * ry);
			int bin = (int)floorf((atan2f(ry, rx) + (float)M_PI) * (LR_LIDAR_BINS / (2 * (float)M_PI)));
			if (bin < 0) bin = 0;
			if (bin >= LR_LIDAR_BINS) bin = LR_LIDAR_BINS - 1;
			if (r > _clear[bin]) {
				_clear[bin] = r;
			}
			continue;
		}
		if (o.z + lowZ * d - floorZ < _params.minHeight) {
			continue;   // the floor after all (or it could be)
		}
		float wx, wy;
		lrPoseApply(odom, o.x + dir.x * d, o.y + dir.y * d, wx, wy);
		float rx = wx - _cx, ry = wy - _cy;
		float r = sqrtf(rx * rx + ry * ry);
		if (r < 1) {
			continue;
		}
		float a = atan2f(ry, rx);
		int bin = (int)floorf((a + (float)M_PI) * (LR_LIDAR_BINS / (2 * (float)M_PI)));
		if (bin < 0) bin = 0;
		if (bin >= LR_LIDAR_BINS) bin = LR_LIDAR_BINS - 1;
		_readings++;
		float &cell = _r[bin];
		if (cell == 0) {
			_points++;
			cell = r;
			_x[bin] = wx;
			_y[bin] = wy;
		} else if (r + r * 0.05f < cell) {
			cell = r;   // clearly closer: a nearer object in the same degree
			_x[bin] = wx;
			_y[bin] = wy;
		} else if (r < cell + cell * 0.05f) {
			// the same surface: average (each degree is seen in several frames)
			cell = (cell + r) * 0.5f;
			_x[bin] = (_x[bin] + wx) * 0.5f;
			_y[bin] = (_y[bin] + wy) * 0.5f;
		}
	}
}

uint16_t TofScanBuilder::finish(const LR_pose2 &odomEnd, float *x, float *y) const
{
	LR_pose2 inv = lrPoseInverse(odomEnd);
	uint16_t n = 0;
	for (uint16_t i = 0; i < LR_LIDAR_BINS; i++) {
		if (_r[i] == 0) {
			continue;
		}
		lrPoseApply(inv, _x[i], _y[i], x[n], y[n]);
		n++;
	}
	return n;
}

uint16_t TofScanBuilder::finishClear(const LR_pose2 &odomEnd, float *x, float *y) const
{
	LR_pose2 inv = lrPoseInverse(odomEnd);
	uint16_t n = 0;
	for (uint16_t i = 0; i < LR_LIDAR_BINS; i++) {
		if (_r[i] != 0 || _clear[i] == 0) {
			continue;   // a point there (its beam frees the cells before it), or not looked
		}
		float a = (i + 0.5f) * (2 * (float)M_PI / LR_LIDAR_BINS) - (float)M_PI;
		lrPoseApply(inv, _cx + _clear[i] * cosf(a), _cy + _clear[i] * sinf(a), x[n], y[n]);
		n++;
	}
	return n;
}

bool lrTofObstacleAhead(const TofSensor &s, const float *mm, const LR_angle &tilt, float floorZ,
                        float distance, float halfWidth, float height, LR_point *at)
{
	uint8_t zones = s.zones ? s.zones * s.zones : 1;
	uint8_t hits = 0;
	float best = distance;
	for (uint8_t z = 0; z < zones; z++) {
		float d = mm[z];
		if (d <= 0 || d > distance * 2) {
			continue;
		}
		LR_point o, dir;
		levelRay(s, z, tilt, o, dir);
		float px = o.x + dir.x * d, py = o.y + dir.y * d, pz = o.z + dir.z * d;
		if (py > 0 && py < distance && fabsf(px) < halfWidth && pz - floorZ > height) {
			hits++;
			if (at != nullptr && py < best) {
				best = py;
				at->x = px; at->y = py; at->z = pz;
			}
		}
	}
	return hits >= 2;
}

/* -------------------------------------------------------------- TofNavigator */

void TofNavigator::begin(const TofScanParams &scan, const NavParams &nav)
{
	_params = scan;
	_builder.begin(scan);
	_nav.begin(nav);
	_phase = TOF_SCAN_IDLE;
	_tilt = 0;
	_havePrev = false;
	_blocked = false;
	_sinceGap = 1;
	_haveFull = false;
	_wantFull = true;
	_sweep = 0;
}

void TofNavigator::startScan()
{
	_phase = TOF_SCAN_STOP;
	_time = 0;
	_turned = 0;
}

void TofNavigator::scan()
{
	_wantFull = true;
	if (_phase == TOF_SCAN_IDLE || _phase == TOF_SCAN_LEVEL) {
		startScan();
	}
}

bool TofNavigator::setGoal(float x, float y)
{
	if (!_nav.setGoal(x, y)) {
		return false;
	}
	_walked = 0;
	if (_phase == TOF_SCAN_IDLE || _phase == TOF_SCAN_LEVEL) {
		startScan();   // where am I: a fresh scan before walking
	}
	return true;
}

void TofNavigator::stop()
{
	_nav.stop();
	if (_phase != TOF_SCAN_IDLE) {
		_phase = TOF_SCAN_LEVEL;
		_time = 0;
	}
}

void TofNavigator::addFrame(const TofSensor &s, const float *mm, const LR_pose2 &odom, const LR_angle &tilt, float floorZ)
{
	floorZ += crouch();   // the body is lower: the floor is closer
	if (_phase == TOF_SCAN_TURN) {
		_builder.addFrame(s, mm, odom, tilt, floorZ);
	} else if (_phase == TOF_SCAN_IDLE && _tilt == 0) {
		// walking: the sensors look at the ground ahead
		LR_point at;
		bool b = lrTofObstacleAhead(s, mm, tilt, floorZ, _nav.params().stopDistance,
		                            _nav.params().robotRadius * 0.8f, _params.obstacleHeight, &at);
		if (b) {
			_obstacle = at;
		}
		_blocked = b || (_blocked && _sinceFrame < 0.05f);   // two sensors: either one
		_sinceFrame = 0;
	}
}

LR_moveVector TofNavigator::update(float dt, LidarSlam &slam, const LR_pose2 &odom, float maxYawRate, bool gaitActive)
{
	LR_moveVector cmd = { { 0, 0, 0 }, { 0, 0, 0 } };
	float step = 0, turn = 0;
	if (_havePrev) {
		step = hypotf(odom.x - _prev.x, odom.y - _prev.y);
		turn = lrWrapAngle(odom.theta - _prev.theta);
	}
	_prev = odom;
	_havePrev = true;
	_sinceFrame += dt;
	if (_sinceFrame > 0.5f) {
		_blocked = false;   // no frames: nothing known ahead
	}
	_sinceGap += dt;
	bool gap = _sinceGap < 0.1f;
	_time += dt;

	switch (_phase) {
		case TOF_SCAN_STOP:
			// stand still (the gait finishes its step) before the nose goes up
			if ((!gaitActive && _time >= _params.settleTime) || _time > 3.0f) {
				_phase = TOF_SCAN_TILT;
				_time = 0;
			}
			return cmd;

		case TOF_SCAN_TILT:
			_tilt += _params.tiltRate * dt;
			if (_tilt >= _params.tilt) {
				_tilt = _params.tilt;
				if (_time >= _params.tilt / _params.tiltRate + _params.settleTime) {
					_builder.start(odom);
					_turned = 0;
					bool full = _wantFull || !_haveFull || slam.scans() == 0 || _params.rescanAngle <= 0
					            || _params.rescanAngle >= _params.turnAngle;
					_sweep = full ? 0 : _params.rescanAngle;
					_sweepBack = false;
					_phase = TOF_SCAN_TURN;
					_time = 0;
				}
			}
			return cmd;

		case TOF_SCAN_TURN: {
			_turned += turn;
			// full: once around counter-clockwise. Partial: right to -sweep/2, then left to +sweep/2
			float total = _sweep > 0 ? 1.5f * _sweep : _params.turnAngle;
			if (_sweep > 0 && !_sweepBack && _turned <= -_sweep * 0.5f) {
				_sweepBack = true;
			}
			bool done = _sweep > 0 ? _sweepBack && _turned >= _sweep * 0.5f : fabsf(_turned) >= _params.turnAngle;
			if (done || _time > 4 * total / _params.turnRate) {
				uint16_t n = _builder.finish(odom, _px, _py);
				uint16_t nf = _builder.finishClear(odom, _fx, _fy);
				_lastPoints = n;
				LidarMatch m = slam.addPoints(_px, _py, n, odom, _fx, _fy, nf);
				if (_sweep > 0 && m != LIDAR_MATCH_OK) {
					// the partial sweep didn't fit the map (or saw mostly new area): turn all the
					// way round here and now, before walking on with a doubtful pose
					_builder.start(odom);
					_turned = 0;
					_time = 0;
					_sweep = 0;
					return cmd;
				}
				bool good = m == LIDAR_MATCH_OK || m == LIDAR_MATCH_NEW;
				if (_sweep == 0 && good) {
					_haveFull = true;
				}
				_wantFull = !good;   // lost (or too few points): look all the way round next time
				_phase = TOF_SCAN_LEVEL;
				_time = 0;
				_walked = 0;
				return cmd;
			}
			// command > 0 turns clockwise: counter-clockwise is negative
			float rate = maxYawRate > 0 ? constrain(_params.turnRate / maxYawRate, 0.0f, 1.0f) : 0;
			cmd.rotate.yaw = _sweep > 0 && !_sweepBack ? rate : -rate;
			return cmd;
		}

		case TOF_SCAN_LEVEL:
			_tilt -= _params.tiltRate * dt;
			if (_tilt <= 0) {
				_tilt = 0;
				_phase = TOF_SCAN_IDLE;
				_time = 0;
				_sinceFrame = 1;
				_blocked = false;
			}
			return cmd;

		case TOF_SCAN_IDLE:
			break;
	}

	if (!_nav.active()) {
		_blockedTime = 0;
		return cmd;
	}
	_walked += step;
	bool blocked = _blocked || gap;
	if (gap && !_blocked) {
		_obstacle = _gap;
	}
	_blockedTime = blocked ? _blockedTime + dt : 0;
	if (_blockedTime > 1.0f) {
		// stopped by something the map doesn't have (too low for the scan, or
		// that scan was thrown away): put it in, the planner goes around
		LR_pose2 pose = slam.pose(odom);
		for (int k = -1; k <= 1; k++) {
			float bx = _obstacle.x + k * slam.grid().cellSize(), by = _obstacle.y + slam.grid().cellSize() * 0.5f;
			float mx, my;
			lrPoseApply(pose, bx, by, mx, my);
			slam.markOccupied(mx, my);
		}
		_blockedTime = 0;
		_nav.setGoal(_nav.goalX(), _nav.goalY());   // replan now
	}
	if (_walked >= _params.scanEvery) {
		startScan();
		return cmd;
	}
	cmd = _nav.update(dt, slam, slam.pose(odom), maxYawRate, blocked ? 1 : 0, _obstacle.x);
	if (_nav.state() == NAV_NO_PATH && _walked > 0) {
		// maybe the map was wrong here: look again before giving up
		_nav.setGoal(_nav.goalX(), _nav.goalY());
		_walked = 0;
		startScan();
		LR_moveVector zero = { { 0, 0, 0 }, { 0, 0, 0 } };
		return zero;
	}
	return cmd;
}
