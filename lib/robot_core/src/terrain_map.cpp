#include "terrain_map.h"

LR_point lrRotateBody(const LR_point &p, const LR_angle &o)
{
	float sP = sinf(o.pitch), cP = cosf(o.pitch);
	float sR = sinf(o.roll),  cR = cosf(o.roll);
	float sY = sinf(o.yaw),   cY = cosf(o.yaw);
	LR_point r;
	r.x = p.x * (cY*cP) + p.y * (cY*sP*sR - sY*cR) + p.z * (cY*sP*cR + sY*sR);
	r.y = p.x * (sY*cP) + p.y * (sY*sP*sR + cY*cR) + p.z * (sY*sP*cR - cY*sR);
	r.z = p.x * (-sP)   + p.y * (cP*sR)            + p.z * (cP*cR);
	return r;
}

void TerrainMap::begin(const TerrainMapParams &params)
{
	_params = params;
	if (_params.cellSize <= 0) {
		_params.cellSize = 20.0f;
	}
	clear();
}

void TerrainMap::clear()
{
	for (uint16_t i = 0; i < LR_TERRAIN_GRID * LR_TERRAIN_GRID; i++) {
		_cells[i].time = -1.0f;
	}
	_time = 0;
}

static inline uint16_t wrap(int16_t i)
{
	int m = i % LR_TERRAIN_GRID;
	return (uint16_t)(m < 0 ? m + LR_TERRAIN_GRID : m);
}

static inline uint16_t slot(int16_t ix, int16_t iy)
{
	return (uint16_t)(wrap(iy) * LR_TERRAIN_GRID + wrap(ix));
}

const TerrainMap::cell *TerrainMap::find(int16_t ix, int16_t iy) const
{
	const cell &c = _cells[slot(ix, iy)];
	if (c.time < 0 || c.ix != ix || c.iy != iy || _time - c.time > _params.maxAge) {
		return nullptr;
	}
	return &c;
}

LR_point TerrainMap::zoneDirection(const TofSensor &s, uint8_t zone)
{
	uint8_t n = s.zones ? s.zones : 1;
	uint8_t row = zone / n, col = zone % n;
	float h = ((col + 0.5f) / n - 0.5f) * s.fov;
	float v = (0.5f - (row + 0.5f) / n) * s.fov;
	LR_point d = { tanf(h), 1.0f, tanf(v) };
	float len = sqrtf(d.x * d.x + d.y * d.y + d.z * d.z);
	d.x /= len; d.y /= len; d.z /= len;
	// tilt down about X, then yaw about Z
	float st = sinf(s.tilt), ct = cosf(s.tilt);
	LR_point t = { d.x, d.y * ct + d.z * st, -d.y * st + d.z * ct };
	return lrRotateZ(t, sinf(s.yaw), cosf(s.yaw));
}

void TerrainMap::addReading(const TofSensor &s, uint8_t zone, float distance, const LR_figure &body)
{
	if (distance <= 0 || distance > s.maxRange) {
		return;
	}
	addPoint(readingPoint(s, zone, distance, body));
}

LR_point TerrainMap::readingPoint(const TofSensor &s, uint8_t zone, float distance, const LR_figure &body)
{
	LR_point d = zoneDirection(s, zone);
	LR_point b = { s.position.x + d.x * distance, s.position.y + d.y * distance, s.position.z + d.z * distance };
	LR_point w = lrRotateBody(b, body.orientation);
	w.x += body.position.x;
	w.y += body.position.y;
	w.z += body.position.z;
	return w;
}

float TerrainMap::frameOffset(const TofSensor &s, const float *mm, const LR_figure &body, float deadband) const
{
	const uint8_t zones = s.zones ? s.zones * s.zones : 1;
	float diff[64];
	uint8_t n = 0;
	for (uint8_t z = 0; z < zones && z < 64; z++) {
		if (mm[z] <= 0 || mm[z] > s.maxRange) {
			continue;
		}
		LR_point w = readingPoint(s, z, mm[z], body);
		const cell *c = find(index(w.x), index(w.y));
		if (!c || c->wall || c->time >= _time) {
			continue;   // unknown, a vertical face, or only this frame's
		}
		float d = w.z - c->z;
		if (fabsf(d) < 4 * _params.jump) {
			// insertion sort: the median below
			uint8_t k = n++;
			while (k > 0 && diff[k - 1] > d) {
				diff[k] = diff[k - 1];
				k--;
			}
			diff[k] = d;
		}
	}
	if (n < 8) {
		return 0;
	}
	float median = n % 2 ? diff[n / 2] : 0.5f * (diff[n / 2 - 1] + diff[n / 2]);
	return fabsf(median) < deadband ? 0 : median;
}

void TerrainMap::addPoint(const LR_point &p)
{
	int16_t ix = index(p.x), iy = index(p.y);
	cell &c = _cells[slot(ix, iy)];
	if (c.time < 0 || c.ix != ix || c.iy != iy || _time - c.time > _params.maxAge) {
		c.ix = ix;
		c.iy = iy;
		c.z  = p.z;
		c.lo = INT16_MAX;
		c.wall = false;
	} else if (fmaxf(c.z, p.z) - fminf(c.lo, p.z) > 3 * _params.jump) {
		c.wall = true;                                // far apart heights: a vertical face
		c.z  = fmaxf(c.z, p.z);
	} else if (p.z > c.z + _params.jump) {
		c.z = p.z;                                    // something stands here: take its top
	} else {
		c.z += (p.z - c.z) * _params.smoothing;       // same surface, average the noise
	}
	if (p.z < c.lo) {
		c.lo = (int16_t)fmaxf(p.z, (float)INT16_MIN);
	}
	c.time = _time;
}

bool TerrainMap::heightAt(float x, float y, float &z) const
{
	const cell *c = find(index(x), index(y));
	if (!c) {
		return false;
	}
	z = c->wall ? c->z + WALL_HEIGHT : c->z;
	return true;
}

bool TerrainMap::maxHeightAlong(const LR_point &a, const LR_point &b, float radius, float &z) const
{
	float dx = b.x - a.x, dy = b.y - a.y;
	float len = sqrtf(dx * dx + dy * dy);
	float cs = _params.cellSize;
	int steps = (int)(len / (cs * 0.5f)) + 1;
	int r = (int)ceilf(radius / cs);
	bool found = false;
	int16_t lastX = INT16_MIN, lastY = INT16_MIN;
	for (int k = 0; k <= steps; k++) {
		float t = (float)k / steps;
		int16_t cx = index(a.x + dx * t), cy = index(a.y + dy * t);
		if (cx == lastX && cy == lastY) {
			continue;
		}
		lastX = cx; lastY = cy;
		for (int oy = -r; oy <= r; oy++) {
			for (int ox = -r; ox <= r; ox++) {
				const cell *c = find(cx + ox, cy + oy);
				if (c && (!found || c->z + (c->wall ? WALL_HEIGHT : 0) > z)) {
					z = c->z + (c->wall ? WALL_HEIGHT : 0);
					found = true;
				}
			}
		}
	}
	return found;
}

float TerrainMap::edge(float x, float y, float dx, float dy, float reach) const
{
	float lo = 0, hi = 0, z;
	int n = 0;
	for (int k = -1; k <= 1; k++) {
		if (!heightAt(x + dx * reach * k, y + dy * reach * k, z)) continue;
		if (n == 0 || z < lo) lo = z;
		if (n == 0 || z > hi) hi = z;
		n++;
	}
	return n < 2 ? -1.0f : hi - lo;
}

uint16_t TerrainMap::knownCells() const
{
	uint16_t n = 0;
	for (uint16_t i = 0; i < LR_TERRAIN_GRID * LR_TERRAIN_GRID; i++) {
		const cell &c = _cells[i];
		if (c.time >= 0 && _time - c.time <= _params.maxAge) n++;
	}
	return n;
}

/** Least squares z = a x + b y + c over the used points; false if singular. */
static bool fitPlane(const LR_point *p, const bool *use, uint8_t n, float &a, float &b, float &c)
{
	double sxx = 0, sxy = 0, syy = 0, sx = 0, sy = 0, sxz = 0, syz = 0, sz = 0, k = 0;
	for (uint8_t i = 0; i < n; i++) {
		if (!use[i]) continue;
		double x = p[i].x, y = p[i].y, z = p[i].z;
		sxx += x * x; sxy += x * y; syy += y * y; sx += x; sy += y;
		sxz += x * z; syz += y * z; sz += z; k += 1;
	}
	// | sxx sxy sx | |a|   |sxz|
	// | sxy syy sy | |b| = |syz|
	// | sx  sy  k  | |c|   |sz |
	double det = sxx * (syy * k - sy * sy) - sxy * (sxy * k - sy * sx) + sx * (sxy * sy - syy * sx);
	if (fabs(det) < 1e-6) {
		return false;
	}
	a = (float)((sxz * (syy * k - sy * sy) - sxy * (syz * k - sy * sz) + sx * (syz * sy - syy * sz)) / det);
	b = (float)((sxx * (syz * k - sz * sy) - sxz * (sxy * k - sy * sx) + sx * (sxy * sz - syz * sx)) / det);
	c = (float)((sxx * (syy * sz - syz * sy) - sxy * (sxy * sz - syz * sx) + sxz * (sxy * sy - syy * sx)) / det);
	return true;
}

bool TerrainMap::floorTilt(const TofSensor &s, const float *mm, const LR_angle &prior, LR_angle &tilt)
{
	const uint8_t zones = s.zones ? s.zones * s.zones : 1;
	if (zones < 9 || zones > 64) {
		return false;   // one ray (or a 2x2) can't tell a plane
	}
	LR_point p[64];
	bool use[64];
	uint8_t n = 0;
	for (uint8_t z = 0; z < zones; z++) {
		float d = mm[z];
		use[z] = d > 0 && d <= s.maxRange;
		if (use[z]) {
			LR_point dir = zoneDirection(s, z);
			p[z] = { s.position.x + dir.x * d, s.position.y + dir.y * d, s.position.z + dir.z * d };
			n++;
		}
	}
	if (n < 12) {
		return false;
	}
	// the most common level, seen with the prior tilt (10 mm bins): the floor, or a step
	// top if that is most of the view (level too: the same tilt), not a mix of the two
	LR_angle pt = { prior.pitch, prior.roll, 0 };
	float lz[64];
	uint8_t bestCount = 0;
	float best = 0;
	for (uint8_t z = 0; z < zones; z++) {
		if (use[z]) lz[z] = lrRotateBody(p[z], pt).z;
	}
	for (uint8_t z = 0; z < zones; z++) {
		if (!use[z]) continue;
		uint8_t count = 0;
		for (uint8_t w = 0; w < zones; w++) {
			if (use[w] && fabsf(lz[w] - lz[z]) < 10.0f) count++;
		}
		if (count > bestCount) {
			bestCount = count;
			best = lz[z];
		}
	}
	n = 0;
	for (uint8_t z = 0; z < zones; z++) {
		if (use[z] && fabsf(lz[z] - best) >= 15.0f) use[z] = false;
		if (use[z]) n++;
	}
	float a = 0, b = 0, c = 0;
	// fit, drop the zones off the plane, fit again
	const float limits[2] = { 10.0f, 6.0f };
	for (uint8_t round = 0; round < 2; round++) {
		if (n < 12 || !fitPlane(p, use, zones, a, b, c)) {
			return false;
		}
		n = 0;
		for (uint8_t z = 0; z < zones; z++) {
			if (!use[z]) continue;
			if (fabsf(p[z].z - (a * p[z].x + b * p[z].y + c)) > limits[round]) {
				use[z] = false;
			} else {
				n++;
			}
		}
	}
	// most of the view on one plane (a plain floor); near a step or an obstacle the IMU's tilt does
	if (n * 8 < zones * 5 || !fitPlane(p, use, zones, a, b, c)) {
		return false;
	}
	// the points must spread both ways (forward and sideways), or the plane is a line
	float xmin = 1e9f, xmax = -1e9f, ymin = 1e9f, ymax = -1e9f;
	for (uint8_t z = 0; z < zones; z++) {
		if (!use[z]) continue;
		xmin = fminf(xmin, p[z].x); xmax = fmaxf(xmax, p[z].x);
		ymin = fminf(ymin, p[z].y); ymax = fmaxf(ymax, p[z].y);
	}
	if (xmax - xmin < 40 || ymax - ymin < 40 || c > -20) {
		return false;   // too small a patch, or not below the body
	}
	// floor z = a x + b y + c in the body frame: nose up (roll > 0) makes the floor
	// ahead fall away (b < 0), the left side up (pitch > 0) makes it rise to the right (a > 0)
	tilt.pitch = atanf(a);
	tilt.roll  = -atanf(b);
	tilt.yaw   = 0;
	return true;
}
