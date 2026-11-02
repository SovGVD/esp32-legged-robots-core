/**
 * Checks run against a real model description (model/<name>/model_data.h).
 * Include after the model's config.h + model_data.h and after defining
 * `static <Solver> modelIK;`.
 */
#ifndef TEST_MODEL_CHECKS_H
#define TEST_MODEL_CHECKS_H

#include <unity.h>
#include <string.h>
#include <stdlib.h>

static const float DT = LOOP_TIME / 1000000.0f;

alignas(leg) static unsigned char pristine[sizeof(legs)];
static bool saved = false;

void setUp()
{
	// every test starts from the model's own data
	if (!saved) {
		memcpy((void *)pristine, (const void *)legs, sizeof(legs));
		saved = true;
	} else {
		memcpy((void *)legs, (const void *)pristine, sizeof(legs));
	}
}
void tearDown() {}

static LR_moveVector cmd(float x, float y, float yaw)
{
	LR_moveVector c = { { x, y, 0 }, { 0, 0, yaw } };
	return c;
}

void test_neutral_stance_is_solved_and_in_range()
{
	MotionController motion;
	motion.begin(GAIT_PARAMS, legs, LEG_NUM, modelIK, LEG_DOF, &servoMainProfile);
	LR_angle none = {};
	TEST_ASSERT_EQUAL(0, motion.update(0.0f, cmd(0, 0, 0), none));
	TEST_ASSERT_EQUAL(0, motion.jointLimitCount());
	TEST_ASSERT_EQUAL(0, motion.servoRangeCount());
}

void test_default_gait_stays_reachable()
{
	MotionController motion;
	motion.begin(GAIT_PARAMS, legs, LEG_NUM, modelIK, LEG_DOF, &servoMainProfile);
	LR_angle none = {};
	const LR_moveVector cmds[] = { cmd(0, 1, 0), cmd(0, -1, 0), cmd(1, 0, 0), cmd(-1, 0, 0), cmd(0, 0, 1), cmd(0, 0, -1) };
	for (const LR_moveVector &c : cmds) {
		for (int t = 0; t < (int)(3 * GAIT_PARAMS.cycleTime / DT); t++) {
			motion.update(DT, c, none);
		}
		for (int t = 0; t < (int)(3 * GAIT_PARAMS.cycleTime / DT); t++) {
			motion.update(DT, cmd(0, 0, 0), none);
		}
	}
	TEST_ASSERT_EQUAL_MESSAGE(0, motion.unreachableCount(), "foot target out of reach");
	TEST_ASSERT_EQUAL_MESSAGE(0, motion.jointLimitCount(), "joint limit hit");
	TEST_ASSERT_EQUAL_MESSAGE(0, motion.servoRangeCount(), "servo range hit");
}

void test_sit_down_stays_reachable()
{
	// battery critical: the body is lowered by SIT_HEIGHT (negative) while standing
	MotionController motion;
	motion.begin(GAIT_PARAMS, legs, LEG_NUM, modelIK, LEG_DOF, &servoMainProfile);
	LR_angle none = {};
	motion.setHeightOffset(SIT_HEIGHT, 20.0f);
	for (int t = 0; t < (int)(5.0f / DT); t++) {
		motion.update(DT, cmd(0, 0, 0), none);
	}
	TEST_ASSERT_TRUE(motion.heightSettled());
	TEST_ASSERT_FLOAT_WITHIN(1e-3f, SIT_HEIGHT, motion.heightOffset());
	TEST_ASSERT_LESS_THAN(-10.0f, SIT_HEIGHT);
	TEST_ASSERT_EQUAL_MESSAGE(0, motion.unreachableCount(), "foot target out of reach");
	TEST_ASSERT_EQUAL_MESSAGE(0, motion.jointLimitCount(), "joint limit hit");
	TEST_ASSERT_EQUAL_MESSAGE(0, motion.servoRangeCount(), "servo range hit");
}

void test_servo_angle_round_trip()
{
	for (uint8_t i = 0; i < LEG_NUM; i++) {
		const leg &l = legs[i];
		for (float a = 0.8f; a < 2.4f; a += 0.1f) {
			float s = lrHalAngle(a, l.hal.mid.alpha, 0.05f, l.hal.ratio.alpha, l.inverse.alpha);
			TEST_ASSERT_FLOAT_WITHIN(1e-4f, a, lrLegAngleFromServo(s, l.hal.mid.alpha, 0.05f, l.hal.ratio.alpha, l.inverse.alpha));
		}
	}
}

void test_clamped_pose_maps_to_profile_limit()
{
	leg &l = legs[0];
	LR_legAngle a = l.angle;
	a.alpha = 100.0f;   // far outside anything a servo can do
	TEST_ASSERT_TRUE(lrClampServoRange(l, a, LEG_DOF, servoMainProfile));
	float s = lrHalAngle(a.alpha, l.hal.mid.alpha, l.hal.trim.alpha, l.hal.ratio.alpha, l.inverse.alpha);
	float deg = s * 180.0f / (float)M_PI;
	TEST_ASSERT_TRUE(fabsf(deg - servoMainProfile.minAngle) < 0.01f || fabsf(deg - servoMainProfile.maxAngle) < 0.01f);
	// and the pulse is then exactly the profile end, not something clamped later
	uint16_t pulses[6];
	l.angle = a;
	lrLegPulses(l, LEG_DOF, servoMainProfile, pulses);
	// (float -> uint16 truncation may land 1 us below)
	TEST_ASSERT_TRUE(abs((int)pulses[0] - servoMainProfile.degMin) <= 1 || abs((int)pulses[0] - servoMainProfile.degMax) <= 1);
}

static int runModelChecks()
{
	UNITY_BEGIN();
	RUN_TEST(test_neutral_stance_is_solved_and_in_range);
	RUN_TEST(test_default_gait_stays_reachable);
	RUN_TEST(test_sit_down_stays_reachable);
	RUN_TEST(test_servo_angle_round_trip);
	RUN_TEST(test_clamped_pose_maps_to_profile_limit);
	return UNITY_END();
}

#endif
