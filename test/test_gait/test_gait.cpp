#include <unity.h>
#include <robot_core.h>
#include "../common/fixtures.h"

static const float DT = 0.0044f;   // dog LOOP_TIME

void setUp() {}
void tearDown() {}

static LR_moveVector command(float x, float y, float yaw)
{
	LR_moveVector c = { { x, y, 0 }, { 0, 0, yaw } };
	return c;
}

static float dist(const LR_point &a, const LR_point &b)
{
	return sqrtf((a.x-b.x)*(a.x-b.x) + (a.y-b.y)*(a.y-b.y) + (a.z-b.z)*(a.z-b.z));
}

void test_swing_curve_endpoints_apex_and_soft_contact()
{
	LR_point from = { 0, 0, -80 };
	LR_point to   = { 0, 40, -80 };
	LR_point p0 = lrSwingPoint(from, to, 30, 0);
	LR_point p1 = lrSwingPoint(from, to, 30, 1);
	LR_point mid = lrSwingPoint(from, to, 30, 0.5f);
	TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0,   dist(p0, from));
	TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0,   dist(p1, to));
	TEST_ASSERT_FLOAT_WITHIN(1e-3f, -50, mid.z);
	TEST_ASSERT_FLOAT_WITHIN(1e-3f, 20,  mid.y);

	// zero velocity at lift off and touch down: a tiny step barely moves the foot
	float h = 1e-3f;
	TEST_ASSERT_LESS_THAN_FLOAT(0.01f, dist(lrSwingPoint(from, to, 30, h), from));
	TEST_ASSERT_LESS_THAN_FLOAT(0.01f, dist(lrSwingPoint(from, to, 30, 1 - h), to));
}

void test_idle_robot_does_not_step()
{
	leg legs[4] = DOG_LEGS;
	GaitEngine gait;
	gait.begin(dogTrot(), legs, 4);
	LR_point start[4];
	for (int i = 0; i < 4; i++) start[i] = legs[i].foot;

	for (int t = 0; t < 1000; t++) {
		gait.update(DT, command(0, 0, 0));
	}
	TEST_ASSERT_FALSE(gait.isActive());
	for (int i = 0; i < 4; i++) {
		TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0, dist(start[i], legs[i].foot));
	}
}

void test_walk_forward_trot_pattern()
{
	leg legs[4] = DOG_LEGS;
	GaitParams p = dogTrot();
	GaitEngine gait;
	gait.begin(p, legs, 4);

	LR_point prev[4];
	bool wasSwinging[4] = {};
	float maxLift = 0;
	int steps = 0;
	const int ticks = (int)(3.0f / DT);
	for (int t = 0; t < ticks; t++) {
		for (int i = 0; i < 4; i++) prev[i] = legs[i].foot;
		gait.update(DT, command(0, 1, 0));

		int swinging = 0;
		for (int i = 0; i < 4; i++) {
			if (gait.isSwinging(i)) {
				swinging++;
				maxLift = fmaxf(maxLift, legs[i].foot.z + DOG_FOOT_Z);
			} else if (!wasSwinging[i]) {
				// stance: the foot must not slide on the ground
				TEST_ASSERT_FLOAT_WITHIN(1e-3f, 0, dist(prev[i], legs[i].foot));
			}
			if (gait.isSwinging(i) && !wasSwinging[i]) steps++;
			wasSwinging[i] = gait.isSwinging(i);
		}
		// trot: never more than one diagonal pair in the air
		TEST_ASSERT_LESS_OR_EQUAL(2, swinging);
		TEST_ASSERT_FALSE(gait.isSwinging(0) && gait.isSwinging(1));
		TEST_ASSERT_FALSE(gait.isSwinging(2) && gait.isSwinging(3));
	}

	// 3 s at up to 150 mm/s with a 0.25 s ramp
	float expected = p.maxSpeed * (3.0f - 0.5f * p.maxSpeed / p.maxAccel);
	TEST_ASSERT_FLOAT_WITHIN(5.0f, expected, gait.body().position.y);
	TEST_ASSERT_FLOAT_WITHIN(1e-3f, 0, gait.body().position.x);
	TEST_ASSERT_FLOAT_WITHIN(1.0f, p.stepHeight, maxLift);
	// 3 s / 0.4 s cycle = 7.5 cycles, every leg steps once per cycle
	TEST_ASSERT_INT_WITHIN(4, 30, steps);

	// feet stay near the body: no leg is left behind
	for (int i = 0; i < 4; i++) {
		LR_point n = gait.neutralFoot(i, gait.body().position, gait.body().orientation.yaw);
		TEST_ASSERT_LESS_THAN_FLOAT(40.0f, dist(n, legs[i].foot));
	}
}

void test_stops_and_settles_at_neutral()
{
	leg legs[4] = DOG_LEGS;
	GaitParams p = dogTrot();
	GaitEngine gait;
	gait.begin(p, legs, 4);
	for (int t = 0; t < (int)(1.0f / DT); t++) gait.update(DT, command(0.5f, 1, 0.3f));
	TEST_ASSERT_TRUE(gait.isActive());

	for (int t = 0; t < (int)(3.0f / DT); t++) gait.update(DT, command(0, 0, 0));
	TEST_ASSERT_FALSE(gait.isActive());
	for (int i = 0; i < 4; i++) {
		LR_point n = gait.neutralFoot(i, gait.body().position, gait.body().orientation.yaw);
		TEST_ASSERT_LESS_OR_EQUAL_FLOAT(p.idleTolerance, dist(n, legs[i].foot));
		TEST_ASSERT_FALSE(gait.isSwinging(i));
	}
}

void test_turning_in_place()
{
	leg legs[4] = DOG_LEGS;
	GaitParams p = dogTrot();
	GaitEngine gait;
	gait.begin(p, legs, 4);
	for (int t = 0; t < (int)(2.0f / DT); t++) gait.update(DT, command(0, 0, 1));
	// positive yaw command turns clockwise (negative yaw), as in the original planner
	TEST_ASSERT_LESS_THAN_FLOAT(-2.0f, gait.body().orientation.yaw);
	TEST_ASSERT_FLOAT_WITHIN(1e-3f, 0, gait.body().position.x);
	TEST_ASSERT_FLOAT_WITHIN(1e-3f, 0, gait.body().position.y);
}

void test_full_speed_walk_is_always_reachable()
{
	// gait + body kinematics + IK together: every foot target must be solvable
	leg legs[4] = DOG_LEGS;
	GaitEngine gait;
	BodyKinematics bodyKinematics;
	QuadrupedLegIK ik;
	gait.begin(dogTrot(), legs, 4);
	LR_angle noCorrection = {};

	LR_moveVector cmds[] = { command(0, 1, 0), command(1, 0, 0), command(-0.7f, -0.7f, 0), command(0.5f, 1, -1) };
	for (const LR_moveVector &c : cmds) {
		for (int t = 0; t < (int)(2.0f / DT); t++) {
			gait.update(DT, c);
			bodyKinematics.update(legs, 4, gait.body(), noCorrection, c);
			for (int i = 0; i < 4; i++) {
				iksolver s = ik.solve(legs[i], gait.body(), c);
				TEST_ASSERT_TRUE_MESSAGE(s.isSolved, "foot target out of reach");
			}
		}
	}
}

void test_contact_sensor_ends_swing_early()
{
	leg legs[4] = DOG_LEGS;
	GaitParams p = dogTrot();
	p.useContactSensors = true;
	GaitEngine gait;
	gait.begin(p, legs, 4);

	// sensors report "in the air" until a leg is past the middle of its swing
	float swingTime = (1 - p.dutyFactor) * p.cycleTime;
	int ticksInSwing[4] = {};
	bool endedEarly = false;
	for (int t = 0; t < (int)(1.0f / DT); t++) {
		for (int i = 0; i < 4; i++) {
			legs[i].sensor.onGround = !(gait.isSwinging(i) && ticksInSwing[i] * DT < 0.7f * swingTime);
		}
		gait.update(DT, command(0, 1, 0));
		for (int i = 0; i < 4; i++) {
			if (gait.isSwinging(i)) {
				ticksInSwing[i]++;
			} else {
				if (ticksInSwing[i] > 0 && ticksInSwing[i] * DT < 0.9f * swingTime) endedEarly = true;
				ticksInSwing[i] = 0;
			}
		}
	}
	TEST_ASSERT_TRUE(endedEarly);
}

void test_disabled_gait_never_moves()
{
	leg legs[4] = DOG_LEGS;
	GaitParams p = dogTrot();
	p.enabled = false;
	GaitEngine gait;
	gait.begin(p, legs, 4);
	LR_point start = legs[0].foot;
	for (int t = 0; t < 500; t++) gait.update(DT, command(1, 1, 1));
	TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0, dist(start, legs[0].foot));
	TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0, gait.body().position.y);
}

int main()
{
	UNITY_BEGIN();
	RUN_TEST(test_swing_curve_endpoints_apex_and_soft_contact);
	RUN_TEST(test_idle_robot_does_not_step);
	RUN_TEST(test_walk_forward_trot_pattern);
	RUN_TEST(test_stops_and_settles_at_neutral);
	RUN_TEST(test_turning_in_place);
	RUN_TEST(test_full_speed_walk_is_always_reachable);
	RUN_TEST(test_contact_sensor_ends_swing_early);
	RUN_TEST(test_disabled_gait_never_moves);
	return UNITY_END();
}
