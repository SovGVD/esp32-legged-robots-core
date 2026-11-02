#include <unity.h>
#include <robot_core.h>

void setUp() {}
void tearDown() {}

static const DiffDriveParams P = { 2.0f, 0.05f, 60, 255 };

void test_mix_forward_and_spin()
{
	float l, r;
	lrDiffDriveMix(1, 0, l, r);
	TEST_ASSERT_FLOAT_WITHIN(1e-6f, 1, l);
	TEST_ASSERT_FLOAT_WITHIN(1e-6f, 1, r);
	lrDiffDriveMix(0, 1, l, r);
	TEST_ASSERT_FLOAT_WITHIN(1e-6f,  1, l);
	TEST_ASSERT_FLOAT_WITHIN(1e-6f, -1, r);
}

void test_mix_keeps_turn_ratio_when_saturated()
{
	float l, r;
	lrDiffDriveMix(1, 0.5f, l, r);   // 1.5 / 0.5 -> 1 / 0.333
	TEST_ASSERT_FLOAT_WITHIN(1e-6f, 1, l);
	TEST_ASSERT_FLOAT_WITHIN(1e-5f, 1.0f / 3, r);
}

void test_duty_direction_deadband_and_min()
{
	DiffDriveDuty d = lrDiffDriveDuty(0.02f, P);
	TEST_ASSERT_EQUAL_UINT16(0, d.in1);
	TEST_ASSERT_EQUAL_UINT16(0, d.in2);

	d = lrDiffDriveDuty(1, P);
	TEST_ASSERT_EQUAL_UINT16(255, d.in1);
	TEST_ASSERT_EQUAL_UINT16(0, d.in2);

	d = lrDiffDriveDuty(-0.1f, P);
	TEST_ASSERT_EQUAL_UINT16(0, d.in1);
	TEST_ASSERT_EQUAL_UINT16(80, d.in2);   // 60 + 0.1 * 195 = 79.5
}

void test_slew_limits_reversal()
{
	DiffDrive drive;
	drive.begin(P);
	for (int i = 0; i < 100; i++) drive.update(0.01f, 1, 0);   // 1 s at 2/s: full speed
	TEST_ASSERT_FLOAT_WITHIN(1e-5f, 1, drive.left());

	drive.update(0.1f, -1, 0);   // can only move 0.2 per 0.1 s
	TEST_ASSERT_FLOAT_WITHIN(1e-5f, 0.8f, drive.left());
	TEST_ASSERT_FLOAT_WITHIN(1e-5f, 0.8f, drive.right());

	drive.stop();
	TEST_ASSERT_EQUAL_UINT16(0, drive.leftDuty().in1);
	TEST_ASSERT_EQUAL_UINT16(0, drive.leftDuty().in2);
}

int main()
{
	UNITY_BEGIN();
	RUN_TEST(test_mix_forward_and_spin);
	RUN_TEST(test_mix_keeps_turn_ratio_when_saturated);
	RUN_TEST(test_duty_direction_deadband_and_min);
	RUN_TEST(test_slew_limits_reversal);
	return UNITY_END();
}
