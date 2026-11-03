#include <stdio.h>
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
	// trot at full speed, and the static walk (its body shift needs reach too)
	const GaitParams gaits[2] = { dogTrot(), dogWalk() };
	for (const GaitParams &p : gaits) {
	leg legs[4] = DOG_LEGS;
	GaitEngine gait;
	BodyKinematics bodyKinematics;
	QuadrupedLegIK ik;
	gait.begin(p, legs, 4);
	LR_angle noCorrection = {};

	LR_moveVector cmds[] = { command(0, 1, 0), command(1, 0, 0), command(-0.7f, -0.7f, 0), command(0.5f, 1, -1), command(0, 0, 1) };
	for (const LR_moveVector &c : cmds) {
		for (int t = 0; t < (int)(2.0f * p.cycleTime / 0.4f / DT); t++) {
			gait.update(DT, c);
			bodyKinematics.update(legs, 4, gait.body(), noCorrection, c);
			for (int i = 0; i < 4; i++) {
				iksolver s = ik.solve(legs[i], gait.body(), c);
				if (!s.isSolved) {
					char msg[96];
					snprintf(msg, sizeof(msg), "out of reach: gait %d cmd %d t %.2f leg %d", (int)(&p - gaits), (int)(&c - cmds), t * DT, i);
					TEST_FAIL_MESSAGE(msg);
				}
			}
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

void test_support_margin_geometry()
{
	LR_point tri[3] = { { 0, 0, 0 }, { 100, 0, 0 }, { 0, 100, 0 } };
	LR_point in = { 10, 20, 0 };
	LR_point out = { -5, 50, 0 };
	TEST_ASSERT_FLOAT_WITHIN(1e-3f, 10, lrSupportMargin(tri, 3, in));      // nearest edge x = 0
	TEST_ASSERT_FLOAT_WITHIN(1e-3f, -5, lrSupportMargin(tri, 3, out));
	// point order does not matter
	LR_point cw[3] = { tri[0], tri[2], tri[1] };
	TEST_ASSERT_FLOAT_WITHIN(1e-3f, 10, lrSupportMargin(cw, 3, in));
	// a quadrilateral with an inner point that is not part of the hull
	LR_point quad[5] = { { 0, 0, 0 }, { 100, 0, 0 }, { 100, 100, 0 }, { 0, 100, 0 }, { 50, 50, 0 } };
	LR_point c = { 50, 40, 0 };
	TEST_ASSERT_FLOAT_WITHIN(1e-3f, 40, lrSupportMargin(quad, 5, c));
	// two feet: never stable, minus the distance to the line between them
	LR_point two[2] = { { 0, 0, 0 }, { 100, 0, 0 } };
	TEST_ASSERT_FLOAT_WITHIN(1e-3f, -7, lrSupportMargin(two, 2, (LR_point){ 30, 7, 0 }));
}

static float minThreeLegMargin(const GaitParams &p, const LR_moveVector &cmd, float seconds, float *maxStepMm)
{
	leg legs[4] = DOG_LEGS;
	GaitEngine gait;
	gait.begin(p, legs, 4);
	float worst = 1e9f;
	float maxStep = 0;
	LR_point prev = gait.body().position;
	LR_point prevBase = gait.basePosition();
	for (int t = 0; t < (int)(seconds / DT); t++) {
		gait.update(DT, cmd);
		int air = 0;
		for (int i = 0; i < 4; i++) air += gait.isSwinging(i) ? 1 : 0;
		if (air > 0) {
			float m = gait.supportMargin();
			if (m < worst) worst = m;
		}
		// body motion on top of the odometry: the shift itself
		LR_point b = gait.body().position, base = gait.basePosition();
		float dx = (b.x - prev.x) - (base.x - prevBase.x);
		float dy = (b.y - prev.y) - (base.y - prevBase.y);
		float step = sqrtf(dx * dx + dy * dy);
		if (step > maxStep) maxStep = step;
		prev = b;
		prevBase = base;
	}
	if (maxStepMm) *maxStepMm = maxStep;
	return worst;
}

void test_static_walk_keeps_center_of_mass_over_three_feet()
{
	const LR_moveVector cmds[4] = { command(0, 1, 0), command(0, -1, 0), command(1, 0, 0), command(0, 0, 1) };
	for (int c = 0; c < 4; c++) {
		float maxStep = 0;
		float margin = minThreeLegMargin(dogWalk(), cmds[c], 8.0f, &maxStep);
		// about 11 mm in steady state; the first step and turning less
		TEST_ASSERT_GREATER_THAN_FLOAT(3.0f, margin);
		// the shift is smooth: well under 2 mm per tick (450 mm/s)
		TEST_ASSERT_LESS_THAN_FLOAT(2.0f, maxStep);
	}

	// the same walk without the shift tips over a diagonal (negative margin)
	GaitParams off = dogWalk();
	off.staticStability = false;
	TEST_ASSERT_LESS_THAN_FLOAT(0.0f, minThreeLegMargin(off, command(0, 1, 0), 8.0f, nullptr));
}

void test_static_walk_forward_steady_margin()
{
	leg legs[4] = DOG_LEGS;
	GaitEngine gait;
	gait.begin(dogWalk(), legs, 4);
	float worst = 1e9f;
	for (int t = 0; t < (int)(8.0f / DT); t++) {
		gait.update(DT, command(0, 1, 0));
		if (t * DT > 1.2f && (gait.isSwinging(0) || gait.isSwinging(1) || gait.isSwinging(2) || gait.isSwinging(3))) {
			float m = gait.supportMargin();
			if (m < worst) worst = m;
		}
	}
	TEST_ASSERT_GREATER_THAN_FLOAT(10.0f, worst);
	// odometry still walks at the commanded 20 mm/s (after the ramp)
	TEST_ASSERT_FLOAT_WITHIN(10.0f, 20.0f * 8.0f, gait.basePosition().y);
}

void test_no_shift_without_static_stability()
{
	leg legs[4] = DOG_LEGS;
	GaitEngine gait;
	gait.begin(dogTrot(), legs, 4);
	for (int t = 0; t < 1000; t++) {
		gait.update(DT, command(0.5f, 1, 0.3f));
		TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0, gait.shift().x);
		TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0, gait.shift().y);
		TEST_ASSERT_FLOAT_WITHIN(1e-4f, gait.basePosition().x, gait.body().position.x);
	}
}

void test_trot_keeps_center_of_mass_over_stance_centre()
{
	// centre of mass 8 mm behind the body origin: the body sits 8 mm forward,
	// also while turning (the offset turns with the body)
	leg legs[4] = DOG_LEGS;
	GaitParams p = dogTrot();
	p.centerOfMass = { 0, -8, 0 };
	GaitEngine gait;
	gait.begin(p, legs, 4);
	TEST_ASSERT_FLOAT_WITHIN(1e-4f, 8.0f, gait.body().position.y - gait.basePosition().y);
	for (int t = 0; t < 1000; t++) {
		gait.update(DT, command(0.5f, 1, 0.3f));
		float yaw = gait.body().orientation.yaw;
		TEST_ASSERT_FLOAT_WITHIN(1e-3f, -8.0f * sinf(yaw), gait.shift().x);
		TEST_ASSERT_FLOAT_WITHIN(1e-3f,  8.0f * cosf(yaw), gait.shift().y);
	}
}

void test_side_speed_limit_scales_strafe_and_turn()
{
	leg legs[4] = DOG_LEGS;
	GaitParams p = dogTrot();
	p.maxSideSpeed = 50;
	GaitEngine gait;
	gait.begin(p, legs, 4);
	float lever = fabsf(legs[0].defaultFoot.y);
	// full strafe and turn together: both scaled so their sideways foot speeds add up to 50 mm/s
	for (int t = 0; t < 1000; t++) {
		gait.update(DT, command(1, 1, 1));
	}
	float side = fabsf(gait.velocity().x) + fabsf(gait.yawRate()) * lever;
	TEST_ASSERT_FLOAT_WITHIN(0.5f, 50.0f, side);
	TEST_ASSERT_FLOAT_WITHIN(0.01f, fabsf(gait.velocity().x) / p.maxSpeed, fabsf(gait.yawRate()) / p.maxYawRate);
	// walking straight ahead is not limited
	gait.reset();
	for (int t = 0; t < 1000; t++) {
		gait.update(DT, command(0, 1, 0));
	}
	TEST_ASSERT_FLOAT_WITHIN(0.1f, p.maxSpeed, gait.velocity().y);
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
	RUN_TEST(test_support_margin_geometry);
	RUN_TEST(test_static_walk_keeps_center_of_mass_over_three_feet);
	RUN_TEST(test_static_walk_forward_steady_margin);
	RUN_TEST(test_no_shift_without_static_stability);
	RUN_TEST(test_trot_keeps_center_of_mass_over_stance_centre);
	RUN_TEST(test_side_speed_limit_scales_strafe_and_turn);
	return UNITY_END();
}
