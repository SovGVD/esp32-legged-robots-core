#include "lidar.h"
#include "heading.h"
#include <string.h>

LR_pose2 lrPoseCompose(const LR_pose2 &a, const LR_pose2 &b)
{
	float s = sinf(a.theta), c = cosf(a.theta);
	LR_pose2 r = { a.x + b.x * c - b.y * s, a.y + b.x * s + b.y * c, lrWrapAngle(a.theta + b.theta) };
	return r;
}

LR_pose2 lrPoseInverse(const LR_pose2 &a)
{
	float s = sinf(a.theta), c = cosf(a.theta);
	LR_pose2 r = { -(a.x * c + a.y * s), -(-a.x * s + a.y * c), lrWrapAngle(-a.theta) };
	return r;
}

void lrPoseApply(const LR_pose2 &a, float x, float y, float &ox, float &oy)
{
	float s = sinf(a.theta), c = cosf(a.theta);
	ox = a.x + x * c - y * s;
	oy = a.y + x * s + y * c;
}

LR_pose2 lrPoseLerp(const LR_pose2 &a, const LR_pose2 &b, float f)
{
	LR_pose2 r = { a.x + (b.x - a.x) * f, a.y + (b.y - a.y) * f,
	               lrWrapAngle(a.theta + lrWrapAngle(b.theta - a.theta) * f) };
	return r;
}

/* --------------------------------------------------------------- LD06 */

uint8_t lrLd06Crc8(const uint8_t *data, size_t len)
{
	uint8_t crc = 0;
	for (size_t i = 0; i < len; i++) {
		crc ^= data[i];
		for (uint8_t b = 0; b < 8; b++) {
			crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x4D) : (uint8_t)(crc << 1);
		}
	}
	return crc;
}

bool Ld06Parser::feed(uint8_t b)
{
	if (_len == 0) {
		if (b == LD06_HEADER) {
			_buf[_len++] = b;
		}
		return false;
	}
	if (_len == 1) {
		if (b == LD06_VER_LEN) {
			_buf[_len++] = b;
		} else {
			_len = b == LD06_HEADER ? 1 : 0;
		}
		return false;
	}
	_buf[_len++] = b;
	if (_len < LD06_PACKET_LEN) {
		return false;
	}
	_len = 0;
	if (lrLd06Crc8(_buf, LD06_PACKET_LEN - 1) != _buf[LD06_PACKET_LEN - 1]) {
		_crcErrors++;
		return false;
	}
	_packets++;
	return true;
}

float Ld06Parser::speed() const
{
	return u16(2);
}

float Ld06Parser::angle(uint8_t i) const
{
	uint16_t start = u16(4), end = u16(42);
	uint16_t span = (uint16_t)((end + 36000 - start) % 36000);
	uint32_t a = start + (uint32_t)span * i / (LD06_POINTS - 1);
	return (a % 36000) * 0.01f;
}

uint16_t Ld06Parser::distance(uint8_t i) const
{
	return u16(6 + i * 3);
}

uint8_t Ld06Parser::intensity(uint8_t i) const
{
	return _buf[8 + i * 3];
}

uint16_t Ld06Parser::timestamp() const
{
	return u16(44);
}

void lrLd06Encode(uint8_t out[LD06_PACKET_LEN], uint16_t speed, uint16_t startAngle, uint16_t endAngle,
                  const uint16_t *mm, const uint8_t *intensity, uint16_t timestamp)
{
	out[0] = LD06_HEADER;
	out[1] = LD06_VER_LEN;
	out[2] = speed & 0xFF;      out[3] = speed >> 8;
	out[4] = startAngle & 0xFF; out[5] = startAngle >> 8;
	for (uint8_t i = 0; i < LD06_POINTS; i++) {
		out[6 + i * 3] = mm[i] & 0xFF;
		out[7 + i * 3] = mm[i] >> 8;
		out[8 + i * 3] = intensity[i];
	}
	out[42] = endAngle & 0xFF;  out[43] = endAngle >> 8;
	out[44] = timestamp & 0xFF; out[45] = timestamp >> 8;
	out[46] = lrLd06Crc8(out, LD06_PACKET_LEN - 1);
}

/* --------------------------------------------------------------- scans */

void lrLidarBinDirection(const LidarParams &p, uint16_t bin, float &dx, float &dy)
{
	float a = (bin + 0.5f) * (2 * (float)M_PI / LR_LIDAR_BINS);
	float body = p.yaw + (p.clockwise ? -a : a);
	dx = -sinf(body);
	dy = cosf(body);
}

void LidarScanBuilder::reset()
{
	memset(&_current, 0, sizeof(_current));
	memset(&_done, 0, sizeof(_done));
	_last = -1;
	_scans = 0;
}

bool LidarScanBuilder::add(float angleDeg, uint16_t mm, uint8_t intensity)
{
	bool wrapped = _last >= 0 && angleDeg < _last - 180.0f;
	_last = angleDeg;
	if (wrapped) {
		_done = _current;
		memset(&_current, 0, sizeof(_current));
		_scans++;
	}
	if (mm == 0 || mm < _params.minRange || mm > _params.maxRange || intensity < _params.minIntensity) {
		return wrapped;
	}
	int bin = (int)(angleDeg * (LR_LIDAR_BINS / 360.0f));
	if (bin < 0 || bin >= LR_LIDAR_BINS) {
		return wrapped;
	}
	uint16_t &cell = _current.mm[bin];
	if (cell == 0) {
		_current.points++;
		cell = mm;
	} else if (mm + mm / 20 < cell) {
		cell = mm;   // clearly closer: a nearer object in the same degree
	} else if (mm < cell + cell / 20) {
		cell = (uint16_t)((cell + mm) / 2);   // the same surface: average (the minimum of two noisy readings is short)
	}
	return wrapped;
}
