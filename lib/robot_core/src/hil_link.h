/**
 * Hardware in the loop link: the real ESP32 runs the whole firmware (motion
 * task, WiFi, web UI, CLI) while the servos, IMU, power sensor, ToF sensors
 * and foot switches are simulated in MuJoCo on a PC (sim/examples/hil.py).
 * They talk over the USB serial port. Arduino free, so the simulator and the
 * native tests use the same code (sim/legsim/hil.py is the Python side).
 *
 * Frame: 0xA5 0x5A, type, payload length, payload, CRC-16/CCITT-FALSE
 * (over type, length and payload; little endian). All numbers little endian.
 * The firmware's text output (Serial.print) goes over the same port between
 * frames: it is ASCII, so it never holds the 0xA5 0x5A sync, and the PC
 * prints whatever is not a frame.
 *
 * ESP32 -> PC
 *   'H' hello, once a second until the PC answers: protocol version, robot
 *       model (ROBOT_MODEL), legs, joints per leg, LOOP_TIME (us), ToF sensors
 *   'S' servos, every motion tick: tick number, us since the previous tick,
 *       outputs on, then one pulse width (us, 0 = off) per joint, leg * dof + joint
 * PC -> ESP32
 *   'I' sensors, after every 'S': accelerometer (g), gyro (rad/s), magnetometer
 *       (uT), all in the body frame (x right, y forward, z up); flags (bit 0:
 *       the magnetometer reading is new), foot switches (bit i = leg i pressed),
 *       battery voltage (V) and current (A)
 *   'T' one ToF frame (VL53L5CX rate): sensor, 64 zones mm (0 = no target),
 *       terrain map zone order (TofSensor)
 *   'M' move command from the PC's keyboard / gamepad: x, y, z, pitch, roll,
 *       yaw in -1..1 (feeds the failsafe like a web client)
 */
#ifndef LR_HIL_LINK_H
#define LR_HIL_LINK_H

#include <stdint.h>
#include <stddef.h>

#define HIL_VERSION      1
#define HIL_SYNC0        0xA5
#define HIL_SYNC1        0x5A
#define HIL_OVERHEAD     6     // sync x2, type, length, crc x2
#define HIL_MAX_PAYLOAD  255
#define HIL_MAX_SERVOS   24
#define HIL_TOF_ZONES    64

enum HilType : uint8_t {
	HIL_HELLO   = 'H',
	HIL_SERVOS  = 'S',
	HIL_SENSORS = 'I',
	HIL_TOF     = 'T',
	HIL_MOVE    = 'M',
};

struct HilHello {
	uint8_t  version;
	uint8_t  model;        // ROBOT_MODEL (def/def.h)
	uint8_t  legs;
	uint8_t  dof;
	uint32_t loopTimeUs;
	uint8_t  tofCount;
};

struct HilServos {
	uint32_t seq;
	uint32_t dtUs;
	uint8_t  enabled;
	uint8_t  count;
	uint16_t us[HIL_MAX_SERVOS];
};

#define HIL_FLAG_MAG 0x01

struct HilSensors {
	float   accel[3];   // g
	float   gyro[3];    // rad/s
	float   mag[3];     // uT
	uint8_t flags;
	uint8_t feet;
	float   voltage;    // V
	float   current;    // A
};

struct HilTof {
	uint8_t sensor;
	int16_t mm[HIL_TOF_ZONES];
};

struct HilMove {
	float v[6];   // x, y, z, pitch, roll, yaw
};

uint16_t hilCrc16(const uint8_t *data, size_t len, uint16_t crc = 0xFFFF);

/** One frame into out (room for len + HIL_OVERHEAD bytes). Returns its size. */
size_t hilFrame(uint8_t type, const uint8_t *payload, uint8_t len, uint8_t *out);

// encoders: a whole frame into out, returns its size
size_t hilEncodeHello(const HilHello &m, uint8_t *out);       // out >= 9 + 6
size_t hilEncodeServos(const HilServos &m, uint8_t *out);     // out >= 10 + 2 * count + 6
size_t hilEncodeSensors(const HilSensors &m, uint8_t *out);   // out >= 46 + 6
size_t hilEncodeTof(const HilTof &m, uint8_t *out);           // out >= 129 + 6
size_t hilEncodeMove(const HilMove &m, uint8_t *out);         // out >= 24 + 6

// decoders: a frame's payload, false if the length is wrong for the type
bool hilDecodeHello(const uint8_t *p, uint8_t len, HilHello &m);
bool hilDecodeServos(const uint8_t *p, uint8_t len, HilServos &m);
bool hilDecodeSensors(const uint8_t *p, uint8_t len, HilSensors &m);
bool hilDecodeTof(const uint8_t *p, uint8_t len, HilTof &m);
bool hilDecodeMove(const uint8_t *p, uint8_t len, HilMove &m);

/**
 * Byte by byte frame parser. feed() returns true when a whole frame with a
 * good CRC has arrived: type(), payload(), length() hold it until the next
 * feed(). Bytes outside frames are skipped (counted in skipped()).
 */
class HilParser {
	public:
		bool feed(uint8_t b);
		uint8_t        type() const { return _type; }
		const uint8_t *payload() const { return _buf; }
		uint8_t        length() const { return _len; }
		uint32_t crcErrors() const { return _crcErrors; }
		uint32_t skipped() const { return _skipped; }
		uint32_t frames() const { return _frames; }

	private:
		enum State : uint8_t { SYNC0, SYNC1, TYPE, LEN, DATA, CRC0, CRC1 };
		State    _state = SYNC0;
		uint8_t  _type = 0;
		uint8_t  _len = 0;
		uint16_t _pos = 0;
		uint16_t _crc = 0;
		uint8_t  _buf[HIL_MAX_PAYLOAD];
		uint32_t _crcErrors = 0;
		uint32_t _skipped = 0;
		uint32_t _frames = 0;
};

#endif
