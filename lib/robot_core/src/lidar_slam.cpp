#include "lidar_slam.h"
#include "heading.h"
#include <stdlib.h>
#include <string.h>

/* ------------------------------------------------------------ OccupancyGrid */

OccupancyGrid::~OccupancyGrid()
{
	free(_cells);
}

bool OccupancyGrid::begin(float cellSize)
{
	_cell = cellSize;
	if (_cells == nullptr) {
		_cells = (int8_t *)malloc(LR_LIDAR_GRID * LR_LIDAR_GRID);
	}
	if (_cells != nullptr) {
		clear();
	}
	return _cells != nullptr;
}

void OccupancyGrid::clear()
{
	if (_cells != nullptr) {
		memset(_cells, 0, LR_LIDAR_GRID * LR_LIDAR_GRID);
	}
}

bool OccupancyGrid::cellOf(float x, float y, int &ix, int &iy) const
{
	ix = (int)floorf(x / _cell) + LR_LIDAR_GRID / 2;
	iy = (int)floorf(y / _cell) + LR_LIDAR_GRID / 2;
	return ix >= 0 && iy >= 0 && ix < LR_LIDAR_GRID && iy < LR_LIDAR_GRID;
}

void OccupancyGrid::cellCentre(int ix, int iy, float &x, float &y) const
{
	x = (ix - LR_LIDAR_GRID / 2 + 0.5f) * _cell;
	y = (iy - LR_LIDAR_GRID / 2 + 0.5f) * _cell;
}

void OccupancyGrid::addBeam(float x0, float y0, float x1, float y1, bool hit, int8_t hitStep, int8_t missStep, int8_t limit)
{
	int ax, ay, bx, by;
	if (!cellOf(x0, y0, ax, ay)) {
		return;
	}
	bool endInside = cellOf(x1, y1, bx, by);
	// Bresenham from a to b, the end cell excluded
	int dx = abs(bx - ax), sx = ax < bx ? 1 : -1;
	int dy = -abs(by - ay), sy = ay < by ? 1 : -1;
	int err = dx + dy;
	int x = ax, y = ay;
	while (x != bx || y != by) {
		if (x < 0 || y < 0 || x >= LR_LIDAR_GRID || y >= LR_LIDAR_GRID) {
			return;   // left the map
		}
		int8_t &c = _cells[y * LR_LIDAR_GRID + x];
		int v = c - missStep;
		c = (int8_t)(v < -limit ? -limit : v);
		int e2 = 2 * err;
		if (e2 >= dy) { err += dy; x += sx; }
		if (e2 <= dx) { err += dx; y += sy; }
	}
	if (hit && endInside) {
		int8_t &c = _cells[by * LR_LIDAR_GRID + bx];
		int v = c + hitStep;
		c = (int8_t)(v > limit ? limit : v);
	}
}

float OccupancyGrid::value(int ix, int iy, int level) const
{
	int n = 1 << level;
	int x0 = ix * n, y0 = iy * n;
	if (x0 < 0 || y0 < 0 || x0 + n > LR_LIDAR_GRID || y0 + n > LR_LIDAR_GRID) {
		return 0;
	}
	int best = 0;
	for (int y = y0; y < y0 + n; y++) {
		const int8_t *row = _cells + y * LR_LIDAR_GRID;
		for (int x = x0; x < x0 + n; x++) {
			if (row[x] > best) {
				best = row[x];
			}
		}
	}
	float v = best * _occScale;
	return v > 1 ? 1 : v;
}

float OccupancyGrid::sample(float x, float y, int level, float &gx, float &gy) const
{
	float c = _cell * (1 << level);
	int half = (LR_LIDAR_GRID >> level) / 2;
	float u = x / c + half - 0.5f;
	float v = y / c + half - 0.5f;
	int i = (int)floorf(u), j = (int)floorf(v);
	float fu = u - i, fv = v - j;
	float v00 = value(i, j, level),     v10 = value(i + 1, j, level);
	float v01 = value(i, j + 1, level), v11 = value(i + 1, j + 1, level);
	gx = ((v10 - v00) * (1 - fv) + (v11 - v01) * fv) / c;
	gy = ((v01 - v00) * (1 - fu) + (v11 - v10) * fu) / c;
	return (v00 * (1 - fu) + v10 * fu) * (1 - fv) + (v01 * (1 - fu) + v11 * fu) * fv;
}

uint32_t OccupancyGrid::occupiedCells() const
{
	uint32_t n = 0;
	for (int i = 0; i < LR_LIDAR_GRID * LR_LIDAR_GRID; i++) {
		n += _cells[i] > 0;
	}
	return n;
}

uint32_t OccupancyGrid::knownCells() const
{
	uint32_t n = 0;
	for (int i = 0; i < LR_LIDAR_GRID * LR_LIDAR_GRID; i++) {
		n += _cells[i] != 0;
	}
	return n;
}

/* --------------------------------------------------------------- Odometry2D */

void Odometry2D::update(float dt, const LR_point &v, float yawRate)
{
	float mid = _pose.theta + yawRate * dt * 0.5f;
	LR_point w = lrRotateZ(v, sinf(mid), cosf(mid));
	_pose.x += w.x * dt;
	_pose.y += w.y * dt;
	_pose.theta = lrWrapAngle(_pose.theta + yawRate * dt);
}

/* ---------------------------------------------------------------- LidarSlam */

bool LidarSlam::begin(const LidarParams &lidar, const LidarMapParams &map)
{
	_lidar = lidar;
	_params = map;
	for (uint16_t i = 0; i < LR_LIDAR_BINS; i++) {
		lrLidarBinDirection(lidar, i, _dirX[i], _dirY[i]);
	}
	if (!_grid.begin(map.cellSize)) {
		return false;
	}
	_grid._occScale = 1.0f / (2 * (map.hit > 0 ? map.hit : 20));   // two hits: fully occupied for the matcher
	reset();
	return true;
}

void LidarSlam::reset()
{
	_grid.clear();
	LR_pose2 zero = { 0, 0, 0 };
	_mapFromOdom = zero;
	_pose = zero;
	_lastInsert = zero;
	_empty = true;
	_last = LIDAR_MATCH_NONE;
	_score = 0;
	_jump = 0;
	_turn = 0;
	_scans = 0;
	_rejected = 0;
	_n = 0;
}

/**
 * Scan points in the body frame at the end of the revolution: bin i was
 * measured i/360 of the way through it, at a pose between odomStart and
 * odomEnd.
 */
void LidarSlam::deskew(const LidarScan &scan, const LR_pose2 &odomStart, const LR_pose2 &odomEnd)
{
	LR_pose2 startRel = lrPoseCompose(lrPoseInverse(odomEnd), odomStart);   // start pose seen from the end
	static const LR_pose2 zero = { 0, 0, 0 };
	_n = 0;
	for (uint16_t i = 0; i < LR_LIDAR_BINS; i++) {
		uint16_t d = scan.mm[i];
		if (d == 0 || d > _params.mapRange) {
			continue;
		}
		float bx = _lidar.position.x + d * _dirX[i];
		float by = _lidar.position.y + d * _dirY[i];
		LR_pose2 rel = lrPoseLerp(startRel, zero, (i + 0.5f) / LR_LIDAR_BINS);
		lrPoseApply(rel, bx, by, _px[_n], _py[_n]);
		_n++;
	}
}

/** Gauss-Newton steps moving `pose` so the points land on occupied cells. */
void LidarSlam::match(LR_pose2 &pose, int level, int iterations) const
{
	float maxStep = _grid.cellSize() * (1 << level);
	for (int it = 0; it < iterations; it++) {
		float s = sinf(pose.theta), c = cosf(pose.theta);
		float H[6] = { 0, 0, 0, 0, 0, 0 };   // h00 h01 h02 h11 h12 h22
		float b[3] = { 0, 0, 0 };
		for (uint16_t k = 0; k < _n; k++) {
			float px = _px[k], py = _py[k];
			float wx = pose.x + px * c - py * s;
			float wy = pose.y + px * s + py * c;
			float gx, gy;
			float m = _grid.sample(wx, wy, level, gx, gy);
			float r = 1 - m;
			float j2 = gx * (-px * s - py * c) + gy * (px * c - py * s);
			H[0] += gx * gx; H[1] += gx * gy; H[2] += gx * j2;
			H[3] += gy * gy; H[4] += gy * j2; H[5] += j2 * j2;
			b[0] += gx * r; b[1] += gy * r; b[2] += j2 * r;
		}
		// a little damping: a flat wall leaves one direction without gradient
		H[0] = H[0] * 1.05f + 1e-9f;
		H[3] = H[3] * 1.05f + 1e-9f;
		H[5] = H[5] * 1.05f + 1e-9f;
		float a00 = H[3] * H[5] - H[4] * H[4];
		float a01 = H[2] * H[4] - H[1] * H[5];
		float a02 = H[1] * H[4] - H[2] * H[3];
		float det = H[0] * a00 + H[1] * a01 + H[2] * a02;
		if (fabsf(det) < 1e-30f) {
			return;
		}
		float a11 = H[0] * H[5] - H[2] * H[2];
		float a12 = H[1] * H[2] - H[0] * H[4];
		float a22 = H[0] * H[3] - H[1] * H[1];
		float dx = (a00 * b[0] + a01 * b[1] + a02 * b[2]) / det;
		float dy = (a01 * b[0] + a11 * b[1] + a12 * b[2]) / det;
		float dt = (a02 * b[0] + a12 * b[1] + a22 * b[2]) / det;
		float len = sqrtf(dx * dx + dy * dy);
		if (len > maxStep) {
			dx *= maxStep / len;
			dy *= maxStep / len;
		}
		dt = constrain(dt, -0.1f, 0.1f);
		pose.x += dx;
		pose.y += dy;
		pose.theta = lrWrapAngle(pose.theta + dt);
		if (len < 1.0f && fabsf(dt) < 0.002f) {
			return;
		}
	}
}

/**
 * Mean occupancy (0..1) under the points; the share of points next to an
 * occupied cell (known), and of points on cells the map has seen empty
 * (free: a contradiction, the pose is wrong; points on unknown cells are a
 * new area).
 */
float LidarSlam::score(const LR_pose2 &pose, float &known, float &freeShare) const
{
	float s = sinf(pose.theta), c = cosf(pose.theta);
	float sum = 0;
	uint16_t k = 0, f = 0;
	for (uint16_t i = 0; i < _n; i++) {
		float wx = pose.x + _px[i] * c - _py[i] * s;
		float wy = pose.y + _px[i] * s + _py[i] * c;
		float gx, gy;
		sum += _grid.sample(wx, wy, 0, gx, gy);
		int ix, iy;
		if (_grid.cellOf(wx, wy, ix, iy)) {
			// near a known cell: the 3x3 around it
			bool seen = false;
			for (int y = iy - 1; y <= iy + 1 && !seen; y++) {
				for (int x = ix - 1; x <= ix + 1; x++) {
					if (x >= 0 && y >= 0 && x < LR_LIDAR_GRID && y < LR_LIDAR_GRID && _grid.at(x, y) > 0) {
						seen = true;
						break;
					}
				}
			}
			k += seen;
			f += !seen && _grid.at(ix, iy) < 0;
		}
	}
	known = _n ? (float)k / _n : 0;
	freeShare = _n ? (float)f / _n : 0;
	return _n ? sum / _n : 0;
}

void LidarSlam::insert(const LR_pose2 &pose)
{
	float ox, oy;
	lrPoseApply(pose, _lidar.position.x, _lidar.position.y, ox, oy);
	float s = sinf(pose.theta), c = cosf(pose.theta);
	for (uint16_t i = 0; i < _n; i++) {
		float wx = pose.x + _px[i] * c - _py[i] * s;
		float wy = pose.y + _px[i] * s + _py[i] * c;
		_grid.addBeam(ox, oy, wx, wy, true, _params.hit, _params.miss, _params.maxLogOdds);
	}
	_lastInsert = pose;
}

LidarMatch LidarSlam::addScan(const LidarScan &scan, const LR_pose2 &odomStart, const LR_pose2 &odomEnd)
{
	if (!_grid.ready()) {
		return LIDAR_MATCH_NONE;
	}
	_scans++;
	deskew(scan, odomStart, odomEnd);
	LR_pose2 guess = lrPoseCompose(_mapFromOdom, odomEnd);
	if (_n < 30) {
		_pose = guess;
		_last = LIDAR_MATCH_FEW;
		return _last;
	}
	if (_empty) {
		_pose = guess;
		if (!_frozen) {
			insert(_pose);
			_empty = false;
		}
		_last = LIDAR_MATCH_NEW;
		return _last;
	}

	LR_pose2 p = guess;
	match(p, 2, 4);
	match(p, 1, 5);
	match(p, 0, 6);
	float known, freeShare;
	_score = score(p, known, freeShare);
	float jump = hypotf(p.x - guess.x, p.y - guess.y);
	float turn = fabsf(lrWrapAngle(p.theta - guess.theta));
	bool fits = _score >= _params.minScore && jump <= _params.maxJump && turn <= _params.maxTurn;
	if (fits && known >= 0.4f) {
		_pose = p;
		_jump = jump;
		_turn = turn;
		_last = LIDAR_MATCH_OK;
	} else {
		// a new area: the guess puts most points where the map knows nothing,
		// not on cells it has seen empty (that would be a wrong pose)
		float knownGuess, freeGuess;
		score(guess, knownGuess, freeGuess);
		_pose = guess;
		bool newArea = knownGuess < 0.4f && known < 0.4f && freeGuess < 0.2f;
		_last = newArea ? LIDAR_MATCH_NEW : LIDAR_MATCH_REJECTED;
		if (_last == LIDAR_MATCH_REJECTED) {
			_rejected++;
		}
	}
	_mapFromOdom = lrPoseCompose(_pose, lrPoseInverse(odomEnd));

	if (!_frozen && _last != LIDAR_MATCH_REJECTED) {
		float moved = hypotf(_pose.x - _lastInsert.x, _pose.y - _lastInsert.y);
		float turned = fabsf(lrWrapAngle(_pose.theta - _lastInsert.theta));
		if (moved >= _params.updateDistance || turned >= _params.updateAngle) {
			insert(_pose);
		}
	}
	return _last;
}
