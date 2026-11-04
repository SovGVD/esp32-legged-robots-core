#include <unity.h>
#include <robot_core.h>

void setUp() {}
void tearDown() {}

static const LevelerParams P = { 0.0f, 1.0f, 0.0f, 0.25f, 0.5f, 0.0f, 0.0f };
static const LR_angle ZERO = { 0, 0, 0 };

// A body on a slope: what the IMU reads is the slope plus the correction's effect (body tilt = slope - correction)
static LR_angle settle(BodyLeveler &lv, float slopePitch, float slopeRoll, float seconds, bool hold = false)
{
	const float dt = 0.0044f;
	for (float t = 0; t < seconds; t += dt) {
		LR_angle c = lv.correction();
		LR_angle measured = { slopePitch - c.pitch, slopeRoll - c.roll, 0 };
		lv.update(dt, measured, ZERO, hold);
	}
	LR_angle c = lv.correction();
	LR_angle tilt = { slopePitch - c.pitch, slopeRoll - c.roll, 0 };
	return tilt;
}

void test_leveler_removes_steady_slope()
{
	BodyLeveler lv;
	lv.begin(P);
	lv.setEnabled(true);
	LR_angle tilt = settle(lv, 0.12f, -0.08f, 8.0f);
	TEST_ASSERT_FLOAT_WITHIN(0.005f, 0, tilt.pitch);
	TEST_ASSERT_FLOAT_WITHIN(0.005f, 0, tilt.roll);
	TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0, lv.correction().yaw);
}

void test_leveler_limits_correction()
{
	BodyLeveler lv;
	lv.begin(P);
	lv.setEnabled(true);
	settle(lv, 0.6f, 0, 10.0f);
	TEST_ASSERT_FLOAT_WITHIN(1e-5f, 0.25f, lv.correction().pitch);
}

void test_leveler_rate_limit_and_hold()
{
	BodyLeveler lv;
	lv.begin({ 10.0f, 0, 0, 0.25f, 0.5f, 0, 0 });   // big kp: only the rate limit holds it back
	lv.setEnabled(true);
	LR_angle m = { 0.2f, 0, 0 };
	lv.update(0.1f, m, ZERO);
	TEST_ASSERT_FLOAT_WITHIN(1e-5f, 0.05f, lv.correction().pitch);   // 0.5 rad/s * 0.1 s
	lv.update(0.1f, m, ZERO, true);
	TEST_ASSERT_FLOAT_WITHIN(1e-5f, 0.05f, lv.correction().pitch);   // held
}

void test_leveler_deadband_ignores_small_error()
{
	BodyLeveler lv;
	lv.begin({ 0, 1.0f, 0, 0.25f, 0.5f, 0, 0.03f });
	lv.setEnabled(true);
	LR_angle tilt = settle(lv, 0.02f, 0, 5.0f);
	TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.02f, tilt.pitch);   // inside the deadband: untouched
	tilt = settle(lv, 0.1f, 0, 10.0f);
	TEST_ASSERT_FLOAT_WITHIN(0.005f, 0.03f, tilt.pitch);  // leaves the deadband as residual
}

void test_leveler_disabled_ramps_back()
{
	BodyLeveler lv;
	lv.begin(P);
	lv.setEnabled(true);
	settle(lv, 0.1f, 0, 8.0f);
	lv.setEnabled(false);
	lv.update(0.1f, ZERO, ZERO);
	TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.05f, lv.correction().pitch);   // 0.1 - 0.5 * 0.1
	settle(lv, 0.1f, 0, 1.0f);
	TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0, lv.correction().pitch);
}

void test_tilt_estimator_static_angles()
{
	TiltEstimator t;
	float a = 0.2f;
	float accel[3] = { -sinf(a), 0, cosf(a) };   // rotated +0.2 rad about Y
	float gyro[3]  = { 0, 0, 0 };
	t.update(0.004f, accel, gyro);
	TEST_ASSERT_FLOAT_WITHIN(1e-4f, a, t.aboutY());
	TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0, t.aboutX());
}

void test_tilt_estimator_ignores_acceleration_and_follows_gyro()
{
	TiltEstimator t(1.0f);
	float level[3] = { 0, 0, 1 };
	float still[3] = { 0, 0, 0 };
	t.update(0.004f, level, still);
	// 0.5 g forward push for 0.2 s: the accelerometer angle says 0.46 rad, gyro says 0
	float push[3] = { 0, 0.5f, 1 };
	for (int i = 0; i < 50; i++) t.update(0.004f, push, still);
	TEST_ASSERT_FLOAT_WITHIN(0.06f, 0, t.aboutX());
	t.reset();
	t.update(0.004f, level, still);
	// a real rotation at 0.5 rad/s for 0.2 s shows up at once
	float spin[3] = { 0.5f, 0, 0 };
	for (int i = 0; i < 50; i++) t.update(0.004f, level, spin);
	TEST_ASSERT_FLOAT_WITHIN(0.02f, 0.1f, t.aboutX());
}

void test_tilt_estimator_turning_while_tilted()
{
	// nose up 0.15 rad (about X), turning about the vertical at 0.5 rad/s for 10 s:
	// the tilt stays, the turn doesn't make up any
	TiltEstimator t(1.0f);
	float a = 0.15f;
	float up[3] = { 0, sinf(a), cosf(a) };   // gravity reading, sensor frame
	float w = 0.5f;
	float gyro[3] = { up[0] * w, up[1] * w, up[2] * w };   // a turn about the world vertical, seen by the tilted sensor
	for (int i = 0; i < 2500; i++) t.update(0.004f, up, gyro);
	TEST_ASSERT_FLOAT_WITHIN(0.005f, a, t.aboutX());
	TEST_ASSERT_FLOAT_WITHIN(0.005f, 0, t.aboutY());
}

int main()
{
	UNITY_BEGIN();
	RUN_TEST(test_leveler_removes_steady_slope);
	RUN_TEST(test_leveler_limits_correction);
	RUN_TEST(test_leveler_rate_limit_and_hold);
	RUN_TEST(test_leveler_deadband_ignores_small_error);
	RUN_TEST(test_leveler_disabled_ramps_back);
	RUN_TEST(test_tilt_estimator_static_angles);
	RUN_TEST(test_tilt_estimator_ignores_acceleration_and_follows_gyro);
	RUN_TEST(test_tilt_estimator_turning_while_tilted);
	return UNITY_END();
}
