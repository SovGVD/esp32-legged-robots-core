#include <unity.h>
#include <robot_core.h>
#include "../common/fixtures.h"

static const LR_figure     ORIGIN  = {};
static const LR_moveVector NO_MOVE = {};

void setUp() {}
void tearDown() {}

/**
 * Forward kinematics of the quadruped leg (inverse of QuadrupedLegIK),
 * returns the foot relative to the shoulder in the IK leg frame.
 */
static LR_point quadrupedFK(const leg &l, const LR_legAngle &a)
{
	float l2 = l.size.l2, l3 = l.size.l3;
	float d   = sqrtf(l2*l2 + l3*l3 - 2*l2*l3*cosf(a.gamma));
	float phi = (float)M_PI_2 - a.beta - acosf((d*d + l2*l2 - l3*l3) / (2*d*l2));
	float r   = d * cosf(phi);
	LR_point p = { r * cosf(a.alpha), d * sinf(phi), -r * sinf(a.alpha) };
	return p;
}

static bool finiteAngles(const LR_legAngle &a)
{
	return isfinite(a.alpha) && isfinite(a.beta) && isfinite(a.gamma)
	    && isfinite(a.delta) && isfinite(a.epsilon) && isfinite(a.zeta);
}

void test_quadruped_ik_round_trip()
{
	QuadrupedLegIK ik;
	leg l = makeDogLeg(0, 1, 1, { false, false, false, false, false, false, false, false, false });
	int checked = 0;
	for (float x = -30; x <= 30; x += 10) {
		for (float y = -40; y <= 40; y += 10) {
			for (float z = -95; z <= -40; z += 5) {
				float dist = sqrtf(x*x + y*y + z*z);
				if (dist > l.size.l2 + l.size.l3 - 1 || dist < fabsf(l.size.l2 - l.size.l3) + 5) continue;
				l.body = { 0, 0, 0 };
				l.foot = { x, y, z };
				iksolver s = ik.solve(l, ORIGIN, NO_MOVE);
				TEST_ASSERT_TRUE(s.isSolved);
				LR_point p = quadrupedFK(l, s.angle);
				TEST_ASSERT_FLOAT_WITHIN(0.1f, x, p.x);
				TEST_ASSERT_FLOAT_WITHIN(0.1f, y, p.y);
				TEST_ASSERT_FLOAT_WITHIN(0.1f, z, p.z);
				checked++;
			}
		}
	}
	TEST_ASSERT_GREATER_THAN(100, checked);
}

void test_quadruped_ik_neutral_stance_is_solved()
{
	QuadrupedLegIK ik;
	leg legs[4] = DOG_LEGS;
	for (int i = 0; i < 4; i++) {
		iksolver s = ik.solve(legs[i], ORIGIN, NO_MOVE);
		TEST_ASSERT_TRUE(s.isSolved);
		TEST_ASSERT_TRUE(finiteAngles(s.angle));
	}
}

void test_quadruped_ik_out_of_reach_is_finite_and_not_solved()
{
	QuadrupedLegIK ik;
	leg l = makeDogLeg(0, 1, 1, {});
	l.body = { 0, 0, 0 };
	l.foot = { 0, 0, -500 };
	iksolver s = ik.solve(l, ORIGIN, NO_MOVE);
	TEST_ASSERT_FALSE(s.isSolved);
	TEST_ASSERT_TRUE(finiteAngles(s.angle));
}

void test_quadruped_ik_foot_on_shoulder_is_finite()
{
	QuadrupedLegIK ik;
	leg l = makeDogLeg(0, 1, 1, {});
	l.body = { 0, 0, 0 };
	l.foot = { 0, 0, 0 };
	iksolver s = ik.solve(l, ORIGIN, NO_MOVE);
	TEST_ASSERT_FALSE(s.isSolved);
	TEST_ASSERT_TRUE(finiteAngles(s.angle));
}

void test_ik_ignores_body_yaw_and_position()
{
	// Rotating and moving the whole robot (body, shoulder and foot) must not change joint angles
	QuadrupedLegIK ik;
	leg l = makeDogLeg(0, 1, 1, {});
	l.body = { 10, 20, 0 };
	l.foot = { 25, 30, -80 };
	iksolver a = ik.solve(l, ORIGIN, NO_MOVE);

	float yaw = 0.7f;
	LR_figure body = { { 100, -50, 0 }, { 0, 0, yaw } };
	LR_point shoulder = lrRotateZ(l.body, sinf(yaw), cosf(yaw));
	LR_point foot     = lrRotateZ(l.foot, sinf(yaw), cosf(yaw));
	l.body = { shoulder.x + 100, shoulder.y - 50, shoulder.z };
	l.foot = { foot.x + 100, foot.y - 50, foot.z };
	iksolver b = ik.solve(l, body, NO_MOVE);

	TEST_ASSERT_FLOAT_WITHIN(1e-3f, a.angle.alpha, b.angle.alpha);
	TEST_ASSERT_FLOAT_WITHIN(1e-3f, a.angle.beta,  b.angle.beta);
	TEST_ASSERT_FLOAT_WITHIN(1e-3f, a.angle.gamma, b.angle.gamma);
}

void test_hexapod_knee_angle_matches_foot_distance()
{
	// Law of cosines: the tibia angle must reproduce the femur-joint-to-foot distance.
	// (Catches the old `2 * l2 + l3` typo.)
	HexapodLegIK ik;
	leg l = { { 0, "" }, {}, {}, { 35, 55, 110, 0, 0, 0 } };
	l.body = { 0, 0, 0 };
	l.foot = { 150, 40, -70 };
	iksolver s = ik.solve(l, ORIGIN, NO_MOVE);
	TEST_ASSERT_TRUE(s.isSolved);

	float horizontal = sqrtf(150*150 + 40*40) - l.size.l1;
	float expected   = sqrtf(horizontal*horizontal + 70*70);
	float f = l.size.l2, t = l.size.l3;
	float fromKnee = sqrtf(f*f + t*t - 2*f*t*cosf(s.angle.gamma));
	TEST_ASSERT_FLOAT_WITHIN(0.1f, expected, fromKnee);
	TEST_ASSERT_FLOAT_WITHIN(1e-4f, atan2f(40, 150), s.angle.alpha);
}

void test_tank_drive_forward_and_turn()
{
	TankDriveIK ik;
	leg left  = { { 0, "LEFT" },  {}, {}, {}, {}, { (float)M_PI, (float)M_PI, 0, 0, 0, 0 } };
	leg right = { { 1, "RIGHT" }, {}, {}, {}, {}, { (float)M_PI, (float)M_PI, 0, 0, 0, 0 } };

	LR_moveVector forward = { { 0, -1, 0 }, { 0, 0, 0 } };   // forward is -1 on the web client
	TEST_ASSERT_FLOAT_WITHIN(1e-4f, M_PI, ik.solve(left, ORIGIN, forward).angle.alpha);
	TEST_ASSERT_FLOAT_WITHIN(1e-4f, M_PI, ik.solve(right, ORIGIN, forward).angle.alpha);
	TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0, ik.solve(left, ORIGIN, forward).angle.beta);

	LR_moveVector turn = { { 0, 0, 0 }, { 0, 0, 1 } };
	TEST_ASSERT_FLOAT_WITHIN(1e-4f, M_PI, ik.solve(left, ORIGIN, turn).angle.beta);
	TEST_ASSERT_FLOAT_WITHIN(1e-4f, M_PI, ik.solve(right, ORIGIN, turn).angle.alpha);
}

int main()
{
	UNITY_BEGIN();
	RUN_TEST(test_quadruped_ik_round_trip);
	RUN_TEST(test_quadruped_ik_neutral_stance_is_solved);
	RUN_TEST(test_quadruped_ik_out_of_reach_is_finite_and_not_solved);
	RUN_TEST(test_quadruped_ik_foot_on_shoulder_is_finite);
	RUN_TEST(test_ik_ignores_body_yaw_and_position);
	RUN_TEST(test_hexapod_knee_angle_matches_foot_distance);
	RUN_TEST(test_tank_drive_forward_and_turn);
	return UNITY_END();
}
