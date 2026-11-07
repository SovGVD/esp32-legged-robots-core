#include "hil_link.h"
#include <string.h>

uint16_t hilCrc16(const uint8_t *data, size_t len, uint16_t crc)
{
	for (size_t i = 0; i < len; i++) {
		crc ^= (uint16_t)data[i] << 8;
		for (uint8_t b = 0; b < 8; b++) {
			crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021) : (uint16_t)(crc << 1);
		}
	}
	return crc;
}

size_t hilFrame(uint8_t type, const uint8_t *payload, uint8_t len, uint8_t *out)
{
	out[0] = HIL_SYNC0;
	out[1] = HIL_SYNC1;
	out[2] = type;
	out[3] = len;
	if (len > 0 && out + 4 != payload) {
		memmove(out + 4, payload, len);
	}
	uint16_t crc = hilCrc16(out + 2, (size_t)len + 2);
	out[4 + len] = (uint8_t)(crc & 0xFF);
	out[5 + len] = (uint8_t)(crc >> 8);
	return (size_t)len + HIL_OVERHEAD;
}

/**
 * Little endian writer / reader: the ESP32 and every PC are little endian,
 * so memcpy is enough, but keep it in one place.
 */
namespace {

struct Writer {
	uint8_t *p;
	size_t   n = 0;
	void u8(uint8_t v) { p[n++] = v; }
	void u16(uint16_t v) { memcpy(p + n, &v, 2); n += 2; }
	void i16(int16_t v) { memcpy(p + n, &v, 2); n += 2; }
	void u32(uint32_t v) { memcpy(p + n, &v, 4); n += 4; }
	void f32(float v) { memcpy(p + n, &v, 4); n += 4; }
};

struct Reader {
	const uint8_t *p;
	size_t         n = 0;
	uint8_t  u8() { return p[n++]; }
	uint16_t u16() { uint16_t v; memcpy(&v, p + n, 2); n += 2; return v; }
	int16_t  i16() { int16_t v; memcpy(&v, p + n, 2); n += 2; return v; }
	uint32_t u32() { uint32_t v; memcpy(&v, p + n, 4); n += 4; return v; }
	float    f32() { float v; memcpy(&v, p + n, 4); n += 4; return v; }
};

const uint8_t HELLO_LEN   = 9;
const uint8_t SENSORS_LEN = 46;
const uint8_t TOF_LEN     = 1 + 2 * HIL_TOF_ZONES;
const uint8_t MOVE_LEN    = 24;

}

size_t hilEncodeHello(const HilHello &m, uint8_t *out)
{
	Writer w{out + 4};
	w.u8(m.version);
	w.u8(m.model);
	w.u8(m.legs);
	w.u8(m.dof);
	w.u32(m.loopTimeUs);
	w.u8(m.tofCount);
	return hilFrame(HIL_HELLO, out + 4, (uint8_t)w.n, out);
}

size_t hilEncodeServos(const HilServos &m, uint8_t *out)
{
	uint8_t count = m.count > HIL_MAX_SERVOS ? HIL_MAX_SERVOS : m.count;
	Writer w{out + 4};
	w.u32(m.seq);
	w.u32(m.dtUs);
	w.u8(m.enabled);
	w.u8(count);
	for (uint8_t i = 0; i < count; i++) {
		w.u16(m.us[i]);
	}
	return hilFrame(HIL_SERVOS, out + 4, (uint8_t)w.n, out);
}

size_t hilEncodeSensors(const HilSensors &m, uint8_t *out)
{
	Writer w{out + 4};
	for (int i = 0; i < 3; i++) w.f32(m.accel[i]);
	for (int i = 0; i < 3; i++) w.f32(m.gyro[i]);
	for (int i = 0; i < 3; i++) w.f32(m.mag[i]);
	w.u8(m.flags);
	w.u8(m.feet);
	w.f32(m.voltage);
	w.f32(m.current);
	return hilFrame(HIL_SENSORS, out + 4, (uint8_t)w.n, out);
}

size_t hilEncodeTof(const HilTof &m, uint8_t *out)
{
	Writer w{out + 4};
	w.u8(m.sensor);
	for (int i = 0; i < HIL_TOF_ZONES; i++) w.i16(m.mm[i]);
	return hilFrame(HIL_TOF, out + 4, (uint8_t)w.n, out);
}

size_t hilEncodeMove(const HilMove &m, uint8_t *out)
{
	Writer w{out + 4};
	for (int i = 0; i < 6; i++) w.f32(m.v[i]);
	return hilFrame(HIL_MOVE, out + 4, (uint8_t)w.n, out);
}

bool hilDecodeHello(const uint8_t *p, uint8_t len, HilHello &m)
{
	if (len != HELLO_LEN) {
		return false;
	}
	Reader r{p};
	m.version    = r.u8();
	m.model      = r.u8();
	m.legs       = r.u8();
	m.dof        = r.u8();
	m.loopTimeUs = r.u32();
	m.tofCount   = r.u8();
	return true;
}

bool hilDecodeServos(const uint8_t *p, uint8_t len, HilServos &m)
{
	if (len < 10) {
		return false;
	}
	Reader r{p};
	m.seq     = r.u32();
	m.dtUs    = r.u32();
	m.enabled = r.u8();
	m.count   = r.u8();
	if (m.count > HIL_MAX_SERVOS || len != 10 + 2 * m.count) {
		return false;
	}
	for (uint8_t i = 0; i < m.count; i++) {
		m.us[i] = r.u16();
	}
	return true;
}

bool hilDecodeSensors(const uint8_t *p, uint8_t len, HilSensors &m)
{
	if (len != SENSORS_LEN) {
		return false;
	}
	Reader r{p};
	for (int i = 0; i < 3; i++) m.accel[i] = r.f32();
	for (int i = 0; i < 3; i++) m.gyro[i] = r.f32();
	for (int i = 0; i < 3; i++) m.mag[i] = r.f32();
	m.flags   = r.u8();
	m.feet    = r.u8();
	m.voltage = r.f32();
	m.current = r.f32();
	return true;
}

bool hilDecodeTof(const uint8_t *p, uint8_t len, HilTof &m)
{
	if (len != TOF_LEN) {
		return false;
	}
	Reader r{p};
	m.sensor = r.u8();
	for (int i = 0; i < HIL_TOF_ZONES; i++) m.mm[i] = r.i16();
	return true;
}

bool hilDecodeMove(const uint8_t *p, uint8_t len, HilMove &m)
{
	if (len != MOVE_LEN) {
		return false;
	}
	Reader r{p};
	for (int i = 0; i < 6; i++) m.v[i] = r.f32();
	return true;
}

bool HilParser::feed(uint8_t b)
{
	switch (_state) {
		case SYNC0:
			if (b == HIL_SYNC0) {
				_state = SYNC1;
			} else {
				_skipped++;
			}
			return false;
		case SYNC1:
			if (b == HIL_SYNC1) {
				_state = TYPE;
			} else {
				_skipped++;
				_state = b == HIL_SYNC0 ? SYNC1 : SYNC0;
			}
			return false;
		case TYPE:
			_type  = b;
			_state = LEN;
			return false;
		case LEN:
			_len   = b;
			_pos   = 0;
			_state = _len ? DATA : CRC0;
			return false;
		case DATA:
			_buf[_pos++] = b;
			if (_pos >= _len) {
				_state = CRC0;
			}
			return false;
		case CRC0:
			_crc   = b;
			_state = CRC1;
			return false;
		case CRC1: {
			_crc |= (uint16_t)b << 8;
			_state = SYNC0;
			uint8_t head[2] = { _type, _len };
			uint16_t crc = hilCrc16(_buf, _len, hilCrc16(head, 2));
			if (crc != _crc) {
				_crcErrors++;
				return false;
			}
			_frames++;
			return true;
		}
	}
	_state = SYNC0;
	return false;
}
