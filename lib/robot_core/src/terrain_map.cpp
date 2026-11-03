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
	LR_point d = zoneDirection(s, zone);
	LR_point b = { s.position.x + d.x * distance, s.position.y + d.y * distance, s.position.z + d.z * distance };
	LR_point w = lrRotateBody(b, body.orientation);
	w.x += body.position.x;
	w.y += body.position.y;
	w.z += body.position.z;
	addPoint(w);
}

void TerrainMap::addPoint(const LR_point &p)
{
	int16_t ix = index(p.x), iy = index(p.y);
	cell &c = _cells[slot(ix, iy)];
	if (c.time < 0 || c.ix != ix || c.iy != iy || _time - c.time > _params.maxAge) {
		c.ix = ix;
		c.iy = iy;
		c.z  = p.z;
	} else if (p.z > c.z + _params.jump) {
		c.z = p.z;                                    // something stands here: take its top
	} else {
		c.z += (p.z - c.z) * _params.smoothing;       // same surface, average the noise
	}
	c.time = _time;
}

bool TerrainMap::heightAt(float x, float y, float &z) const
{
	const cell *c = find(index(x), index(y));
	if (!c) {
		return false;
	}
	z = c->z;
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
				if (c && (!found || c->z > z)) {
					z = c->z;
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
