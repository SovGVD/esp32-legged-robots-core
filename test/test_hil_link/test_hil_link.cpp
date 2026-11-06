#include <unity.h>
#include <string.h>
#include <robot_core.h>

void setUp() {}
void tearDown() {}

// feed bytes until a frame comes out; false if none
static bool parse(HilParser &p, const uint8_t *data, size_t n)
{
	for (size_t i = 0; i < n; i++) {
		if (p.feed(data[i])) {
			return true;
		}
	}
	return false;
}

void test_crc_check_value()
{
	const char *s = "123456789";
	TEST_ASSERT_EQUAL_HEX16(0x29B1, hilCrc16((const uint8_t *)s, 9));
}

void test_servos_round_trip()
{
	HilServos s = {};
	s.seq = 123456;
	s.dtUs = 4300;
	s.enabled = 1;
	s.count = 18;
	for (uint8_t i = 0; i < s.count; i++) s.us[i] = 1000 + 50 * i;
	uint8_t buf[64];
	size_t n = hilEncodeServos(s, buf);
	TEST_ASSERT_EQUAL(10 + 36 + HIL_OVERHEAD, n);
	HilParser p;
	TEST_ASSERT_TRUE(parse(p, buf, n));
	TEST_ASSERT_EQUAL(HIL_SERVOS, p.type());
	HilServos d;
	TEST_ASSERT_TRUE(hilDecodeServos(p.payload(), p.length(), d));
	TEST_ASSERT_EQUAL_UINT32(123456, d.seq);
	TEST_ASSERT_EQUAL_UINT32(4300, d.dtUs);
	TEST_ASSERT_EQUAL(18, d.count);
	TEST_ASSERT_EQUAL_UINT16_ARRAY(s.us, d.us, 18);
}

void test_sensors_tof_move_round_trip()
{
	HilSensors s = { { 0.1f, -0.2f, 1.0f }, { 0.01f, 0.0f, -0.5f }, { 10, 20, -40 }, HIL_FLAG_MAG, 0x2D, 7.6f, 1.5f };
	HilTof t;
	t.sensor = 1;
	for (int i = 0; i < HIL_TOF_ZONES; i++) t.mm[i] = (int16_t)(i * 31);
	HilMove m = { { 0, 1, 0, 0, 0, -0.5f } };
	uint8_t buf[400];
	size_t n = hilEncodeSensors(s, buf);
	n += hilEncodeTof(t, buf + n);
	n += hilEncodeMove(m, buf + n);

	HilParser p;
	int got = 0;
	for (size_t i = 0; i < n; i++) {
		if (!p.feed(buf[i])) continue;
		got++;
		if (p.type() == HIL_SENSORS) {
			HilSensors d;
			TEST_ASSERT_TRUE(hilDecodeSensors(p.payload(), p.length(), d));
			TEST_ASSERT_EQUAL_FLOAT(-0.2f, d.accel[1]);
			TEST_ASSERT_EQUAL_FLOAT(-0.5f, d.gyro[2]);
			TEST_ASSERT_EQUAL_FLOAT(-40, d.mag[2]);
			TEST_ASSERT_EQUAL(HIL_FLAG_MAG, d.flags);
			TEST_ASSERT_EQUAL(0x2D, d.feet);
			TEST_ASSERT_EQUAL_FLOAT(7.6f, d.voltage);
		} else if (p.type() == HIL_TOF) {
			HilTof d;
			TEST_ASSERT_TRUE(hilDecodeTof(p.payload(), p.length(), d));
			TEST_ASSERT_EQUAL(1, d.sensor);
			TEST_ASSERT_EQUAL_INT16_ARRAY(t.mm, d.mm, HIL_TOF_ZONES);
		} else if (p.type() == HIL_MOVE) {
			HilMove d;
			TEST_ASSERT_TRUE(hilDecodeMove(p.payload(), p.length(), d));
			TEST_ASSERT_EQUAL_FLOAT(-0.5f, d.v[5]);
		}
	}
	TEST_ASSERT_EQUAL(3, got);
	TEST_ASSERT_EQUAL_UINT32(0, p.crcErrors());
}

// the firmware's text output between frames, and a damaged frame, are skipped
void test_text_and_damaged_frames_are_skipped()
{
	HilMove m = { { 0, 1, 0, 0, 0, 0 } };
	uint8_t frame[32];
	size_t n = hilEncodeMove(m, frame);
	uint8_t buf[128];
	const char *text = "Failsafe: no commands\r\n";
	size_t len = strlen(text);
	memcpy(buf, text, len);
	memcpy(buf + len, frame, n);
	buf[len + 8] ^= 0x40;   // damaged copy
	memcpy(buf + len + n, frame, n);

	HilParser p;
	int got = 0;
	for (size_t i = 0; i < len + 2 * n; i++) {
		if (p.feed(buf[i])) got++;
	}
	TEST_ASSERT_EQUAL(1, got);
	TEST_ASSERT_EQUAL_UINT32(1, p.crcErrors());
	TEST_ASSERT_EQUAL_UINT32(len, p.skipped());
}

void test_wrong_lengths_are_refused()
{
	uint8_t payload[80] = {};
	HilSensors s;
	HilTof t;
	HilMove m;
	HilServos v;
	TEST_ASSERT_FALSE(hilDecodeSensors(payload, 45, s));
	TEST_ASSERT_FALSE(hilDecodeTof(payload, 60, t));
	TEST_ASSERT_FALSE(hilDecodeMove(payload, 23, m));
	payload[9] = 30;   // more servos than HIL_MAX_SERVOS
	TEST_ASSERT_FALSE(hilDecodeServos(payload, 70, v));
	payload[9] = 4;    // 4 servos but the wrong length
	TEST_ASSERT_FALSE(hilDecodeServos(payload, 10 + 6, v));
}

int main()
{
	UNITY_BEGIN();
	RUN_TEST(test_crc_check_value);
	RUN_TEST(test_servos_round_trip);
	RUN_TEST(test_sensors_tof_move_round_trip);
	RUN_TEST(test_text_and_damaged_frames_are_skipped);
	RUN_TEST(test_wrong_lengths_are_refused);
	return UNITY_END();
}
