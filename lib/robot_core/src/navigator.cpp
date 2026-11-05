#include "navigator.h"
#include "heading.h"
#include <stdlib.h>
#include <string.h>

#define NAV_UNREACHABLE 0xFFFF
#define NAV_N LR_NAV_GRID

static inline int nmin(int a, int b) { return a < b ? a : b; }

Navigator::~Navigator()
{
	free(_cost);
	free(_dist);
}

bool Navigator::allocate()
{
	if (_cost == nullptr) {
		_cost = (uint8_t *)malloc(NAV_N * NAV_N);
	}
	if (_dist == nullptr) {
		_dist = (uint16_t *)malloc(NAV_N * NAV_N * sizeof(uint16_t));
	}
	return _cost != nullptr && _dist != nullptr;
}

bool Navigator::setGoal(float x, float y)
{
	_goalX = x;
	_goalY = y;
	_replan = true;
	_blocked = false;
	_pathLen = 0;
	if (!allocate()) {
		_state = NAV_NO_MEMORY;
		return false;
	}
	_state = NAV_ACTIVE;
	return true;
}

bool Navigator::cellOf(float x, float y, int &ix, int &iy) const
{
	ix = (int)floorf(x / _cell) + LR_NAV_FULL / 2 - _ox;
	iy = (int)floorf(y / _cell) + LR_NAV_FULL / 2 - _oy;
	return ix >= 0 && iy >= 0 && ix < NAV_N && iy < NAV_N;
}

void Navigator::centre(int ix, int iy, float &x, float &y) const
{
	x = (ix + _ox - LR_NAV_FULL / 2 + 0.5f) * _cell;
	y = (iy + _oy - LR_NAV_FULL / 2 + 0.5f) * _cell;
}

/**
 * The planning window: centred between the robot and the goal (or a point
 * toward a far goal, so the robot stays well inside), inside the map.
 */
void Navigator::placeWindow(const LR_pose2 &pose)
{
	float dx = _goalX - pose.x, dy = _goalY - pose.y;
	float d = sqrtf(dx * dx + dy * dy);
	float reach = NAV_N * _cell * 0.8f;   // the robot ends up 0.4 window from the centre at most
	if (d > reach) {
		dx *= reach / d;
		dy *= reach / d;
	}
	float mx = pose.x + dx * 0.5f, my = pose.y + dy * 0.5f;
	int ox = (int)floorf(mx / _cell) + LR_NAV_FULL / 2 - NAV_N / 2;
	int oy = (int)floorf(my / _cell) + LR_NAV_FULL / 2 - NAV_N / 2;
	_ox = ox < 0 ? 0 : (ox > LR_NAV_FULL - NAV_N ? LR_NAV_FULL - NAV_N : ox);
	_oy = oy < 0 ? 0 : (oy > LR_NAV_FULL - NAV_N ? LR_NAV_FULL - NAV_N : oy);
}

/**
 * Cost of each planning cell from the map: distance to the nearest occupied
 * cell (two pass chamfer, 10 mm units, capped at 2.5 m), then blocked /
 * near a wall / unknown.
 */
void Navigator::buildCost(const OccupancyGrid &grid)
{
	uint8_t *clr = _cost;
	const int8_t *fine = grid.cells();
	for (int y = 0; y < NAV_N; y++) {
		for (int x = 0; x < NAV_N; x++) {
			bool occ = false;
			for (int fy = 0; fy < LR_NAV_FACTOR && !occ; fy++) {
				const int8_t *row = fine + ((y + _oy) * LR_NAV_FACTOR + fy) * LR_LIDAR_GRID + (x + _ox) * LR_NAV_FACTOR;
				for (int fx = 0; fx < LR_NAV_FACTOR; fx++) {
					if (row[fx] > 0) {
						occ = true;
						break;
					}
				}
			}
			clr[y * NAV_N + x] = occ ? 0 : 255;
		}
	}
	int a = (int)(_cell / 10), b = (int)(_cell * 1.4142f / 10);
	for (int y = 0; y < NAV_N; y++) {
		for (int x = 0; x < NAV_N; x++) {
			int c = clr[y * NAV_N + x];
			if (x > 0) c = nmin(c, clr[y * NAV_N + x - 1] + a);
			if (y > 0) {
				c = nmin(c, clr[(y - 1) * NAV_N + x] + a);
				if (x > 0) c = nmin(c, clr[(y - 1) * NAV_N + x - 1] + b);
				if (x < NAV_N - 1) c = nmin(c, clr[(y - 1) * NAV_N + x + 1] + b);
			}
			clr[y * NAV_N + x] = (uint8_t)nmin(c, 255);
		}
	}
	for (int y = NAV_N - 1; y >= 0; y--) {
		for (int x = NAV_N - 1; x >= 0; x--) {
			int c = clr[y * NAV_N + x];
			if (x < NAV_N - 1) c = nmin(c, clr[y * NAV_N + x + 1] + a);
			if (y < NAV_N - 1) {
				c = nmin(c, clr[(y + 1) * NAV_N + x] + a);
				if (x < NAV_N - 1) c = nmin(c, clr[(y + 1) * NAV_N + x + 1] + b);
				if (x > 0) c = nmin(c, clr[(y + 1) * NAV_N + x - 1] + b);
			}
			clr[y * NAV_N + x] = (uint8_t)nmin(c, 255);
		}
	}
	// clearance -> cost, in place
	int r = (int)(_params.robotRadius / 10);
	int m = (int)(_params.margin / 10);
	for (int y = 0; y < NAV_N; y++) {
		for (int x = 0; x < NAV_N; x++) {
			int c = clr[y * NAV_N + x];
			if (c < r) {
				clr[y * NAV_N + x] = 0;
				continue;
			}
			int cost = 10;
			if (m > 0 && c < r + m) {
				// steep enough that a doorway's middle beats its edge by more than
				// the few extra cells of going round
				cost += 80 * (r + m - c) / m;
			}
			bool unknown = true;
			for (int fy = 0; fy < LR_NAV_FACTOR && unknown; fy++) {
				const int8_t *row = fine + ((y + _oy) * LR_NAV_FACTOR + fy) * LR_LIDAR_GRID + (x + _ox) * LR_NAV_FACTOR;
				for (int fx = 0; fx < LR_NAV_FACTOR; fx++) {
					if (row[fx] != 0) {
						unknown = false;
						break;
					}
				}
			}
			if (unknown) {
				cost += 15;
			}
			clr[y * NAV_N + x] = (uint8_t)nmin(cost, 255);
		}
	}
}

/** Cost to the goal from every cell: sweeps down-right and up-left until nothing changes. */
void Navigator::fillDistance(int gx, int gy)
{
	for (int i = 0; i < NAV_N * NAV_N; i++) {
		_dist[i] = NAV_UNREACHABLE;
	}
	_dist[gy * NAV_N + gx] = 0;
	auto relax = [&](int x, int y, int nx, int ny, bool diag) -> uint32_t {
		if (nx < 0 || ny < 0 || nx >= NAV_N || ny >= NAV_N) return NAV_UNREACHABLE;
		uint16_t d = _dist[ny * NAV_N + nx];
		uint8_t  c = _cost[ny * NAV_N + nx];
		if (d == NAV_UNREACHABLE || c == 0) return NAV_UNREACHABLE;
		if (diag && (_cost[y * NAV_N + nx] == 0 || _cost[ny * NAV_N + x] == 0)) return NAV_UNREACHABLE;   // no corner cutting
		uint32_t step = (_cost[y * NAV_N + x] + c) / 2;
		if (diag) step = step * 14 / 10;
		return d + step;
	};
	for (int pass = 0; pass < 60; pass++) {
		bool changed = false;
		for (int y = 0; y < NAV_N; y++) {
			for (int x = 0; x < NAV_N; x++) {
				if (_cost[y * NAV_N + x] == 0) continue;
				uint32_t best = _dist[y * NAV_N + x];
				uint32_t v;
				if ((v = relax(x, y, x - 1, y, false)) < best) best = v;
				if ((v = relax(x, y, x, y - 1, false)) < best) best = v;
				if ((v = relax(x, y, x - 1, y - 1, true)) < best) best = v;
				if ((v = relax(x, y, x + 1, y - 1, true)) < best) best = v;
				if (best < _dist[y * NAV_N + x]) {
					_dist[y * NAV_N + x] = (uint16_t)(best < NAV_UNREACHABLE - 1 ? best : NAV_UNREACHABLE - 1);
					changed = true;
				}
			}
		}
		for (int y = NAV_N - 1; y >= 0; y--) {
			for (int x = NAV_N - 1; x >= 0; x--) {
				if (_cost[y * NAV_N + x] == 0) continue;
				uint32_t best = _dist[y * NAV_N + x];
				uint32_t v;
				if ((v = relax(x, y, x + 1, y, false)) < best) best = v;
				if ((v = relax(x, y, x, y + 1, false)) < best) best = v;
				if ((v = relax(x, y, x + 1, y + 1, true)) < best) best = v;
				if ((v = relax(x, y, x - 1, y + 1, true)) < best) best = v;
				if (best < _dist[y * NAV_N + x]) {
					_dist[y * NAV_N + x] = (uint16_t)(best < NAV_UNREACHABLE - 1 ? best : NAV_UNREACHABLE - 1);
					changed = true;
				}
			}
		}
		if (!changed) {
			break;
		}
	}
}

/**
 * The robot's planning cell. Standing closer to a wall than robotRadius
 * puts it on a blocked cell: then the nearest reachable cell around it.
 */
bool Navigator::startCell(const LR_pose2 &pose, int &ix, int &iy) const
{
	int cx, cy;
	if (!cellOf(pose.x, pose.y, cx, cy)) {
		return false;
	}
	ix = cx;
	iy = cy;
	if (_dist[cy * NAV_N + cx] != NAV_UNREACHABLE) {
		return true;
	}
	int reach = (int)(_params.robotRadius / _cell) + 2;
	uint32_t best = NAV_UNREACHABLE;
	for (int y = cy - reach; y <= cy + reach; y++) {
		for (int x = cx - reach; x <= cx + reach; x++) {
			if (x < 0 || y < 0 || x >= NAV_N || y >= NAV_N) continue;
			uint32_t d = _dist[y * NAV_N + x];
			d += (uint32_t)(abs(x - cx) + abs(y - cy)) * 20;   // prefer the closest
			if (_dist[y * NAV_N + x] != NAV_UNREACHABLE && d < best) {
				best = d;
				ix = x;
				iy = y;
			}
		}
	}
	return best != NAV_UNREACHABLE;
}

bool Navigator::downhill(int &ix, int &iy) const
{
	uint16_t here = _dist[iy * NAV_N + ix];
	uint16_t best = here;
	int bx = ix, by = iy;
	for (int dy = -1; dy <= 1; dy++) {
		for (int dx = -1; dx <= 1; dx++) {
			int x = ix + dx, y = iy + dy;
			if ((dx == 0 && dy == 0) || x < 0 || y < 0 || x >= NAV_N || y >= NAV_N) continue;
			if (dx != 0 && dy != 0 && (_cost[iy * NAV_N + x] == 0 || _cost[y * NAV_N + ix] == 0)) continue;
			uint16_t d = _dist[y * NAV_N + x];
			if (d < best) {
				best = d;
				bx = x;
				by = y;
			}
		}
	}
	if (best >= here) {
		return false;
	}
	ix = bx;
	iy = by;
	return true;
}

void Navigator::tracePath(const LR_pose2 &pose)
{
	_pathLen = 0;
	int x, y;
	if (_dist == nullptr || !startCell(pose, x, y)) {
		return;
	}
	_pathX[0] = pose.x;
	_pathY[0] = pose.y;
	_pathLen = 1;
	for (int step = 0; step < 4 * NAV_N; step++) {
		if (!downhill(x, y)) {
			break;
		}
		if (step % 2 == 1 && _pathLen < LR_NAV_PATH - 1) {
			centre(x, y, _pathX[_pathLen], _pathY[_pathLen]);
			_pathLen++;
		}
	}
	// the goal itself last
	_pathX[_pathLen] = _goalX;
	_pathY[_pathLen] = _goalY;
	_pathLen++;
}

bool Navigator::plan(const OccupancyGrid &grid, const LR_pose2 &pose)
{
	if (!grid.ready() || !allocate()) {
		return false;
	}
	_cell = grid.cellSize() * LR_NAV_FACTOR;
	placeWindow(pose);
	buildCost(grid);
	int gx, gy;
	if (!cellOf(_goalX, _goalY, gx, gy)) {
		// further than the window reaches: head for its edge toward the goal, the
		// window moves along at the next replan
		gx = gx < 0 ? 0 : (gx >= NAV_N ? NAV_N - 1 : gx);
		gy = gy < 0 ? 0 : (gy >= NAV_N ? NAV_N - 1 : gy);
	}
	if (_cost[gy * NAV_N + gx] == 0) {
		// goal too close to a wall: the nearest free cell within 50 cm
		int reach = (int)(500 / _cell);
		int best = 1 << 30, bx = -1, by = -1;
		for (int y = gy - reach; y <= gy + reach; y++) {
			for (int x = gx - reach; x <= gx + reach; x++) {
				if (x < 0 || y < 0 || x >= NAV_N || y >= NAV_N || _cost[y * NAV_N + x] == 0) continue;
				int d = (x - gx) * (x - gx) + (y - gy) * (y - gy);
				if (d < best) { best = d; bx = x; by = y; }
			}
		}
		if (bx < 0) {
			return false;
		}
		gx = bx;
		gy = by;
	}
	fillDistance(gx, gy);
	int sx, sy;
	if (!startCell(pose, sx, sy)) {
		_pathLen = 0;
		return false;
	}
	tracePath(pose);
	return true;
}

LR_moveVector Navigator::update(float dt, const LidarSlam &slam, const LR_pose2 &pose, float maxYawRate,
                                int8_t blockedAhead, float obstacleX)
{
	LR_moveVector cmd = { { 0, 0, 0 }, { 0, 0, 0 } };
	_blocked = false;
	if (_state != NAV_ACTIVE) {
		return cmd;
	}
	_sinceplan += dt;
	_distance = hypotf(_goalX - pose.x, _goalY - pose.y);
	if (_distance < _params.goalTolerance) {
		_state = NAV_ARRIVED;
		_pathLen = 0;
		return cmd;
	}
	if (_replan || _sinceplan >= _params.replanPeriod) {
		_replan = false;
		_sinceplan = 0;
		if (!plan(slam.grid(), pose)) {
			_state = NAV_NO_PATH;
			return cmd;
		}
	} else {
		tracePath(pose);
	}
	if (_pathLen < 2) {
		_state = NAV_NO_PATH;
		return cmd;
	}

	// the point `lookahead` mm down the path, but not past a bend: the straight
	// line to it must stay within 50 mm of the path, or the robot cuts the corner
	// (a door frame) the path goes round
	float tx = _pathX[_pathLen - 1], ty = _pathY[_pathLen - 1];
	float walked = 0;
	for (uint8_t i = 1; i < _pathLen; i++) {
		walked += hypotf(_pathX[i] - _pathX[i - 1], _pathY[i] - _pathY[i - 1]);
		bool bend = false;
		float cx = _pathX[i] - pose.x, cy = _pathY[i] - pose.y;
		float len = hypotf(cx, cy);
		for (uint8_t j = 1; j < i && len > 1; j++) {
			float qx = _pathX[j] - pose.x, qy = _pathY[j] - pose.y;
			if (fabsf(qx * cy - qy * cx) / len > 50) {
				bend = true;
				break;
			}
		}
		if (bend) {
			tx = _pathX[i - 1];
			ty = _pathY[i - 1];
			break;
		}
		if (walked >= _params.lookahead) {
			tx = _pathX[i];
			ty = _pathY[i];
			break;
		}
	}
	float dx = tx - pose.x, dy = ty - pose.y;
	float err = lrWrapAngle(atan2f(-dx, dy) - pose.theta);   // > 0: target is to the left

	// something right in front (people, chair legs the map hasn't caught up with)
	const float *px, *py;
	uint16_t n = blockedAhead < 0 ? slam.points(px, py) : 0;
	_blocked = blockedAhead > 0;
	float sideSum = blockedAhead > 0 ? obstacleX : 0;
	uint16_t sideN = blockedAhead > 0 ? 1 : 0;
	for (uint16_t i = 0; i < n; i++) {
		if (py[i] > 0 && py[i] < _params.stopDistance && fabsf(px[i]) < _params.robotRadius * 0.8f) {
			_blocked = true;
			sideSum += px[i];
			sideN++;
		}
	}

	if (_blocked && _distance < 2 * _params.goalTolerance) {
		// the goal is next to something: this is as close as it gets
		_state = NAV_ARRIVED;
		_pathLen = 0;
		return cmd;
	}

	float rate = constrain(_params.turnGain * err, -maxYawRate, maxYawRate);
	cmd.rotate.yaw = maxYawRate > 0 ? constrain(-rate / maxYawRate, -1.0f, 1.0f) : 0;   // command > 0 turns clockwise
	if (fabsf(err) < _params.turnInPlace && !_blocked) {
		float slow = constrain(_distance / (3 * _params.goalTolerance), 0.3f, 1.0f);
		cmd.move.y = _params.speed * cosf(err) * slow;
	} else if (_blocked && fabsf(err) < _params.turnInPlace) {
		// facing the way to go but something is in front (a door frame the path
		// cuts close to): step sideways away from it, the path is still ahead
		float side = sideN ? sideSum / sideN : 0;
		cmd.move.x = side > 0 ? -0.5f * _params.speed : 0.5f * _params.speed;
	}
	return cmd;
}
