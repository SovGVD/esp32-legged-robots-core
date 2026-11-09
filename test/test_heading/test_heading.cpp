#include <unity.h>
#include <robot_core.h>

void setUp() {}
void tearDown() {}

static const float DEG = 3.14159265f / 180.0f;

// Earth field seen by a robot with compass heading h (clockwise from north) and
// tilt t: 20 uT north, 40 uT down (mid northern latitudes)
static void bodyField(float h, const LR_angle &t, float out[3])
{
	float w[3] = { -sinf(h) * 20.0f, cosf(h) * 20.0f, -40.0f };   // level frame: x right, y forward, z up
	// body = Ry(-pitch) * Rx(-roll) * world (inverse of lrMagHeading's leveling)
	float cr = cosf(t.roll), sr = sinf(t.roll);
	float y1 = w[1] * cr + w[2] * sr;
	float z1 = -w[1] * sr + w[2] * cr;
	float cp = cosf(t.pitch), sp = sinf(t.pitch);
	out[0] = w[0] * cp - z1 * sp;
	out[1] = y1;
	out[2] = w[0] * sp + z1 * cp;
}

void test_wrap_angle()
{
	TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.1f, lrWrapAngle(0.1f + 6.28318531f));
	TEST_ASSERT_FLOAT_WITHIN(1e-4f, -0.1f, lrWrapAngle(-0.1f - 2 * 6.28318531f));
	TEST_ASSERT_FLOAT_WITHIN(1e-4f, -3.0f, lrWrapAngle(3.28318531f));
}

void test_mag_heading_level_and_tilted()
{
	const LR_angle tilts[] = { { 0, 0, 0 }, { 10 * DEG, 0, 0 }, { 0, -12 * DEG, 0 }, { -8 * DEG, 15 * DEG, 0 } };
	for (const LR_angle &t : tilts) {
		for (float h = 0; h < 360; h += 30) {
			float m[3];
			bodyField(h * DEG, t, m);
			float got = lrMagHeading(m, t) / DEG;
			TEST_ASSERT_FLOAT_WITHIN(0.2f, 0, lrWrapAngle((got - h) * DEG) / DEG);
		}
	}
	// facing east: north is on the left
	float east[3] = { -20, 0, -40 };
	TEST_ASSERT_FLOAT_WITHIN(0.1f, 90, lrMagHeading(east, { 0, 0, 0 }) / DEG);
}

void test_tilt_ignored_would_be_wrong()
{
	// 10 deg nose up facing east, uncompensated, reads tens of degrees off: the compensation matters
	LR_angle t = { 0, 10 * DEG, 0 };
	float m[3];
	bodyField(90 * DEG, t, m);
	TEST_ASSERT_FLOAT_WITHIN(0.2f, 90, lrMagHeading(m, t) / DEG);
	TEST_ASSERT_TRUE(fabsf(lrMagHeading(m, { 0, 0, 0 }) / DEG - 90) > 10);
}

void test_calibrator_finds_hard_and_soft_iron()
{
	MagCalibrator cal;
	cal.reset();
	const float off[3] = { 30, -12, 5 }, sx = 1.2f, sy = 0.9f;
	for (int i = 0; i < 400; i++) {
		float m[3];
		bodyField(i * 2 * DEG, { 0, 0, 0 }, m);
		float raw[3] = { m[0] * sx + off[0], m[1] * sy + off[1], m[2] + off[2] };
		cal.add(raw);
	}
	float o[3], s[3];
	TEST_ASSERT_TRUE(cal.result(o, s));
	TEST_ASSERT_FLOAT_WITHIN(0.1f, 30, o[0]);
	TEST_ASSERT_FLOAT_WITHIN(0.1f, -12, o[1]);
	// corrected x and y radius are equal
	TEST_ASSERT_FLOAT_WITHIN(0.01f, 1.0f, (s[0] * sx) / (s[1] * sy));
	// field strength of the corrected readings (z keeps its hard iron offset, a turn can't
	// show it; the estimator compares against the same uncorrected z)
	// horizontal radius 24 and 18 uT raw, corrected to their mean 21
	TEST_ASSERT_FLOAT_WITHIN(0.5f, sqrtf(21 * 21 + 35 * 35), cal.fieldStrength(o, s));

	MagCalibrator few;
	few.reset();
	for (int i = 0; i < 100; i++) {
		float m[3];
		bodyField(i * 0.2f * DEG, { 0, 0, 0 }, m);   // 20 deg of turn: not enough
		few.add(m);
	}
	TEST_ASSERT_FALSE(few.result(o, s));
}

void test_estimator_gyro_then_magnetometer()
{
	HeadingEstimator est;
	est.begin({ 2.0f, 0.25f });
	const LR_angle level = { 0, 0, 0 };
	// no magnetometer: the gyro alone, relative to the start
	for (int i = 0; i < 250; i++) est.update(0.004f, -0.5f, level, nullptr);   // clockwise 0.5 rad/s for 1 s
	TEST_ASSERT_FALSE(est.absolute());
	TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.5f, est.heading());

	// first magnetometer reading sets the heading
	float m[3];
	bodyField(200 * DEG, level, m);
	est.update(0.004f, 0, level, m);
	TEST_ASSERT_TRUE(est.absolute());
	TEST_ASSERT_FLOAT_WITHIN(0.5f, 200, est.heading() / DEG);

	// a gyro that drifts 2 deg/s is held at the magnetometer heading: its bias is learned
	for (int i = 0; i < 15000; i++) est.update(0.004f, 2 * DEG, level, m);   // 60 s
	TEST_ASSERT_FLOAT_WITHIN(0.3f, 200, est.heading() / DEG);
	TEST_ASSERT_FLOAT_WITHIN(0.1f, 2, est.gyroBias() / DEG);
}

void test_estimator_ignores_disturbed_field()
{
	HeadingEstimator est;
	est.begin({ 0.5f, 0.25f });
	const float off[3] = { 0, 0, 0 }, scale[3] = { 1, 1, 1 };
	est.setMagCalibration(off, scale, sqrtf(20 * 20 + 40 * 40));
	const LR_angle level = { 0, 0, 0 };
	float m[3];
	bodyField(10 * DEG, level, m);
	est.update(0.004f, 0, level, m);
	TEST_ASSERT_FLOAT_WITHIN(0.5f, 10, est.heading() / DEG);
	// a servo next to the sensor adds 60 uT and points it elsewhere: ignored
	float bad[3] = { m[0] + 60, m[1] - 30, m[2] };
	for (int i = 0; i < 500; i++) est.update(0.004f, 0, level, bad);
	TEST_ASSERT_FALSE(est.magAccepted());
	TEST_ASSERT_FLOAT_WITHIN(0.5f, 10, est.heading() / DEG);
}

// the web client's compass view plots the raw readings and the calibration
void test_estimator_reports_raw_reading_and_calibration()
{
	HeadingEstimator est;
	est.begin({ 0.5f, 0.25f });
	float raw[3];
	TEST_ASSERT_FALSE(est.magRaw(raw));
	const float off[3] = { 5, -3, 1 }, scale[3] = { 1.1f, 0.9f, 1 };
	est.setMagCalibration(off, scale, 44);
	const LR_angle level = { 0, 0, 0 };
	const float m[3] = { 12, 30, -40 };
	est.update(0.004f, 0, level, m);
	TEST_ASSERT_TRUE(est.magRaw(raw));
	TEST_ASSERT_EQUAL_FLOAT(12, raw[0]);
	TEST_ASSERT_EQUAL_FLOAT(30, raw[1]);
	TEST_ASSERT_EQUAL_FLOAT(-40, raw[2]);
	float o[3], s[3], field;
	est.magCalibration(o, s, field);
	TEST_ASSERT_EQUAL_FLOAT(-3, o[1]);
	TEST_ASSERT_EQUAL_FLOAT(1.1f, s[0]);
	TEST_ASSERT_EQUAL_FLOAT(44, field);
}

void test_hold_keeps_the_heading_after_the_stick()
{
	HeadingHold hold;
	hold.begin({ 1.5f, 0.6f, 0.07f, 0.05f });
	TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.3f, hold.update(0.3f, 1.0f, 1.0f, true));   // off: passes the stick
	hold.setEnabled(true);
	TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.5f, hold.update(0.5f, 1.0f, 1.0f, true));   // operator turns
	hold.update(0, 1.2f, 1.0f, true);                                             // lets go at 1.2 rad
	TEST_ASSERT_FLOAT_WITHIN(1e-6f, 1.2f, hold.target());
	// walking, drifted 0.1 rad counter-clockwise: turns back clockwise (positive)
	float yaw = hold.update(0, 1.1f, 1.0f, true);
	TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.15f, yaw);
	// standing, 2 deg off: inside the deadband, no stepping
	TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0, hold.update(0, 1.2f - 2 * DEG, 1.0f, false));
}

void test_hold_turns_to_a_target_the_short_way()
{
	HeadingHold hold;
	hold.begin({ 1.5f, 0.6f, 0.07f, 0.05f });
	hold.setEnabled(true);
	hold.setTarget(350 * DEG);
	// at 10 deg the short way to 350 is counter-clockwise: negative, at most maxRate
	float yaw = hold.update(0, 10 * DEG, 1.2f, false);
	TEST_ASSERT_FLOAT_WITHIN(1e-4f, -1.5f * 20 * DEG / 1.2f, yaw);   // 0.52 rad/s, under maxRate
	TEST_ASSERT_TRUE(yaw < 0);
	TEST_ASSERT_FLOAT_WITHIN(1e-4f, -20 * DEG, hold.error());
}

void test_terrain_tilt_modes_and_aging()
{
	TerrainMap map;
	map.begin({ 10, 1.0f, 0.3f, 8 });
	LR_point p = { 0, 0, -80 };
	map.addPoint(p);
	TerrainTilt tt;
	const LR_angle bias = { 0.03f, -0.02f, 0 };   // the IMU reads 1-2 deg off while walking
	const LR_angle none = { 0, 0, 0 };
	tt.begin(TERRAIN_TILT_FILTERED);
	for (int i = 0; i < 2000; i++) tt.update(0.005f, bias, none, true, &map);   // 10 s walking
	TEST_ASSERT_FLOAT_WITHIN(0.002f, 0, tt.tilt().pitch);
	TEST_ASSERT_FLOAT_WITHIN(0.002f, 0, tt.tilt().roll);
	float z;
	TEST_ASSERT_FALSE(map.heightAt(0, 0, z));   // aged out while walking

	map.addPoint(p);
	for (int i = 0; i < 2000; i++) tt.update(0.005f, bias, none, false, &map);  // standing: kept
	TEST_ASSERT_TRUE(map.heightAt(0, 0, z));

	tt.setMode(TERRAIN_TILT_RAW);
	tt.update(0.005f, bias, none, false, nullptr);
	TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.03f, tt.tilt().pitch);
	tt.setMode(TERRAIN_TILT_COMMANDED);
	LR_angle corr = { 0.05f, 0, 0 };
	tt.update(0.005f, bias, corr, false, nullptr);
	TEST_ASSERT_FLOAT_WITHIN(1e-6f, -0.05f, tt.tilt().pitch);
}

/** A turn about the vertical with the nose 18 deg up (a ToF scan): the full heading rate, also with the tilt estimate 1.5 deg off. */
void test_vertical_rate_on_a_tilted_body()
{
	LR_angle tilt = { 0.03f, 0.314f, 0 };
	float cp = cosf(tilt.pitch), sp = sinf(tilt.pitch), cr = cosf(tilt.roll), sr = sinf(tilt.roll);
	float gyro[3] = { -sp, cp * sr, cp * cr };   // 1 rad/s about world up, in the body frame (Rz Ry Rx)
	TEST_ASSERT_FLOAT_WITHIN(1e-5f, 1.0f, lrHeadingRate(gyro, tilt));
	LR_angle off = { tilt.pitch, tilt.roll - 0.026f, 0 };
	TEST_ASSERT_FLOAT_WITHIN(0.001f, 1.0f, lrHeadingRate(gyro, off));
	// gyro z alone would read 5 % short
	TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.95f, gyro[2]);
}

int main()
{
	UNITY_BEGIN();
	RUN_TEST(test_vertical_rate_on_a_tilted_body);
	RUN_TEST(test_wrap_angle);
	RUN_TEST(test_mag_heading_level_and_tilted);
	RUN_TEST(test_tilt_ignored_would_be_wrong);
	RUN_TEST(test_calibrator_finds_hard_and_soft_iron);
	RUN_TEST(test_estimator_gyro_then_magnetometer);
	RUN_TEST(test_estimator_ignores_disturbed_field);
	RUN_TEST(test_estimator_reports_raw_reading_and_calibration);
	RUN_TEST(test_hold_keeps_the_heading_after_the_stick);
	RUN_TEST(test_hold_turns_to_a_target_the_short_way);
	RUN_TEST(test_terrain_tilt_modes_and_aging);
	return UNITY_END();
}
