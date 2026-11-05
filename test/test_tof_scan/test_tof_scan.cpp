#include <unity.h>
#include <robot_core.h>
#include <math.h>

void setUp() {}
void tearDown() {}

static const float DEG = 3.14159265f / 180.0f;
static const float ROOM = 2000;     // walls at x, y = +-ROOM (map frame), mm
static const float FLOOR = -100;    // floor below the body origin, mm
static const float WALL_H = 300;    // walls this high above the floor

// VL53L5CX on the nose, 25 deg down (the flatter mount)
static const TofSensor SENSOR = { { 0, 90, 0 }, 0.0f, 25 * DEG, 45 * DEG, 8, 400 };
static const TofScanParams SCAN = {
	8.1f * DEG,   // nose up: the top zone row (19.7 +- 2.8 deg above the axis) looks level at its lower edge
	0.5f, 0.5f, 6.7f, 0.07f, 120, 2500, 40, 1000, 0.2f, 40, 0
};
static const LidarMapParams MAP = { 100, 2500, 20, 5, 100, 50, 0.1f, 0.25f, 250, 0.35f };
static const NavParams NAV = { 130, 100, 350, 120, 0.8f, 1.5f, 0.6f, 180, 1.0f };
static const LidarParams ORIGIN = { { 0, 0, 0 }, 0, false, 0, 0, 0 };

/** Distance along (o, d) to the floor or a wall of the square room, mm (world frame, z up from the body origin). */
static float cast(const LR_point &o, const LR_point &d)
{
	float best = 1e9f;
	if (d.z < -1e-6f) {
		best = (FLOOR - o.z) / d.z;
	}
	const float walls[4][3] = { { 1, 0, ROOM }, { -1, 0, ROOM }, { 0, 1, ROOM }, { 0, -1, ROOM } };
	for (const auto &w : walls) {
		float dn = d.x * w[0] + d.y * w[1];
		if (dn <= 1e-6f) continue;
		float t = (w[2] - (o.x * w[0] + o.y * w[1])) / dn;
		float z = o.z + d.z * t;
		if (t > 0 && t < best && z >= FLOOR && z <= FLOOR + WALL_H) best = t;
	}
	return best;
}

/** One frame of SENSOR at odometry pose `p` (= the true pose) with body tilt `tilt`. */
static void frame(const LR_pose2 &p, const LR_angle &tilt, float *mm, const TofSensor &s = SENSOR)
{
	LR_angle t = { tilt.pitch, tilt.roll, 0 };
	LR_point o = lrRotateBody(s.position, t);
	float sn = sinf(p.theta), cs = cosf(p.theta);
	LR_point ow = { p.x + o.x * cs - o.y * sn, p.y + o.x * sn + o.y * cs, o.z };
	for (uint8_t z = 0; z < 64; z++) {
		LR_point d = lrRotateBody(TerrainMap::zoneDirection(s, z), t);
		LR_point dw = { d.x * cs - d.y * sn, d.x * sn + d.y * cs, d.z };
		float r = cast(ow, dw);
		mm[z] = r < 4000 ? r : 0;
	}
}

void test_scan_points_lie_on_the_walls()
{
	TofScanBuilder b;
	b.begin(SCAN);
	LR_pose2 p = { 300, -200, 0 };
	b.start(p);
	LR_angle tilt = { 0, SCAN.tilt, 0 };
	float mm[64];
	for (int k = 0; k < 400; k++) {   // a full turn in 1 deg steps (and a bit)
		p.theta = lrWrapAngle(k * DEG);
		frame(p, tilt, mm);
		b.addFrame(SENSOR, mm, p, tilt, FLOOR);
	}
	float x[LR_LIDAR_BINS], y[LR_LIDAR_BINS];
	uint16_t n = b.finish(p, x, y);
	TEST_ASSERT_GREATER_THAN(250, n);
	for (uint16_t i = 0; i < n; i++) {
		float wx, wy;
		lrPoseApply(p, x[i], y[i], wx, wy);
		float wall = fmaxf(fabsf(wx), fabsf(wy));
		TEST_ASSERT_FLOAT_WITHIN(60, ROOM, wall);   // the zone's width and the closest-target rule: within a few cm
	}
}

void test_floor_is_not_a_wall()
{
	// no nose up: every zone row looks down at the floor, nothing may come out
	TofScanBuilder b;
	b.begin(SCAN);
	LR_pose2 p = { 0, 0, 0 };
	b.start(p);
	LR_angle level = { 0, 0, 0 };
	float mm[64];
	for (int k = 0; k < 360; k += 3) {
		p.theta = lrWrapAngle(k * DEG);
		frame(p, level, mm);
		b.addFrame(SENSOR, mm, p, level, FLOOR);
	}
	TEST_ASSERT_EQUAL(0, b.points());
}

void test_nothing_in_range_is_free_space()
{
	// a hall bigger than the sensor reaches: no points, but every degree is
	// seen empty up to maxRange, and those beams make the map free
	TofScanBuilder b;
	b.begin(SCAN);
	LR_pose2 p = { 0, 0, 0 };
	b.start(p);
	LR_angle tilt = { 0, SCAN.tilt, 0 };
	float mm[64];
	for (int k = 0; k < 400; k++) {
		p.theta = lrWrapAngle(k * DEG);
		for (uint8_t z = 0; z < 64; z++) mm[z] = 0;   // no target
		b.addFrame(SENSOR, mm, p, tilt, FLOOR);
	}
	float x[LR_LIDAR_BINS], y[LR_LIDAR_BINS];
	TEST_ASSERT_EQUAL(0, b.finish(p, x, y));
	uint16_t n = b.finishClear(p, x, y);
	TEST_ASSERT_GREATER_THAN(300, n);
	for (uint16_t i = 0; i < n; i++) {
		float r = hypotf(x[i], y[i]);
		TEST_ASSERT_TRUE(r > SCAN.minRange && r <= SCAN.maxRange + 100);
	}

	LidarSlam slam;
	TEST_ASSERT_TRUE(slam.begin(ORIGIN, MAP));
	slam.addPoints(x, y, 0, p, x, y, n);
	int ix, iy;
	TEST_ASSERT_TRUE(slam.grid().cellOf(1500, 300, ix, iy));
	TEST_ASSERT_TRUE(slam.grid().at(ix, iy) < 0);      // free, not unknown
	TEST_ASSERT_TRUE(slam.grid().cellOf(-300, -1800, ix, iy));
	TEST_ASSERT_TRUE(slam.grid().at(ix, iy) < 0);
}

void test_obstacle_ahead()
{
	LR_angle level = { 0, 0, 0 };
	float mm[64];
	// the floor only: nothing ahead
	for (uint8_t z = 0; z < 64; z++) {
		LR_point d = TerrainMap::zoneDirection(SENSOR, z);
		mm[z] = d.z < 0 ? (FLOOR - SENSOR.position.z) / d.z : 0;
	}
	TEST_ASSERT_FALSE(lrTofObstacleAhead(SENSOR, mm, level, FLOOR, 180, 100, 40));
	// a box 150 mm ahead of the body, 100 mm high: the zones that reach it before the floor
	int hits = 0;
	for (uint8_t z = 0; z < 64; z++) {
		LR_point d = TerrainMap::zoneDirection(SENSOR, z);
		float t = (150 - SENSOR.position.y) / d.y;
		float zz = SENSOR.position.z + d.z * t;
		if (t > 0 && t < mm[z] && zz < FLOOR + 100 && fabsf(SENSOR.position.x + d.x * t) < 150) {
			mm[z] = t;
			hits++;
		}
	}
	TEST_ASSERT_GREATER_THAN(1, hits);
	LR_point at;
	TEST_ASSERT_TRUE(lrTofObstacleAhead(SENSOR, mm, level, FLOOR, 180, 100, 40, &at));
	TEST_ASSERT_FLOAT_WITHIN(5, 150, at.y);
}

void test_odometry_learns_gyro_bias_standing()
{
	Odometry2D o;
	const float bias = 1.0f * DEG;   // 1 deg/s
	LR_point still = { 0, 0, 0 };
	for (int i = 0; i < 1000; i++) {   // 4 s standing at 250 Hz
		o.update(0.004f, still, bias, true);
	}
	TEST_ASSERT_FLOAT_WITHIN(0.02f * DEG, bias, o.bias());
	float before = o.pose().theta;
	for (int i = 0; i < 2500; i++) {   // 10 s turning at 30 deg/s, stepping (not learning)
		o.update(0.004f, still, 30 * DEG + bias, false);
	}
	TEST_ASSERT_FLOAT_WITHIN(0.5f * DEG, lrWrapAngle(300 * DEG), lrWrapAngle(o.pose().theta - before));
}

/**
 * A scan through TofNavigator: stop, nose up, turn (the "robot" here turns
 * exactly as commanded), the scan into the map, nose down.
 */
void test_navigator_scan_cycle()
{
	LidarSlam slam;
	TEST_ASSERT_TRUE(slam.begin(ORIGIN, MAP));
	TofNavigator nav;
	nav.begin(SCAN, NAV);
	const float dt = 0.02f, maxYawRate = 1.0f;
	LR_pose2 p = { 0, 0, 0 };
	float mm[64];
	nav.scan();
	TEST_ASSERT_TRUE(nav.active());
	bool sawTurn = false;
	float maxTilt = 0;
	int k = 0;
	for (; k < 3000 && nav.active(); k++) {
		LR_moveVector cmd = nav.update(dt, slam, p, maxYawRate, false);
		TEST_ASSERT_EQUAL_FLOAT(0, cmd.move.y);
		if (nav.phase() == TOF_SCAN_TURN) {
			sawTurn = true;
			TEST_ASSERT_TRUE(cmd.rotate.yaw <= 0);   // counter-clockwise (0 on the tick the turn starts)
		}
		maxTilt = fmaxf(maxTilt, nav.tilt());
		p.theta = lrWrapAngle(p.theta - cmd.rotate.yaw * maxYawRate * dt);
		LR_angle tilt = { 0, nav.tilt(), 0 };
		if (k % 3 == 0) {   // ~15 Hz frames
			frame(p, tilt, mm);
			nav.addFrame(SENSOR, mm, p, tilt, FLOOR);
		}
	}
	TEST_ASSERT_FALSE(nav.active());
	TEST_ASSERT_TRUE(sawTurn);
	TEST_ASSERT_FLOAT_WITHIN(0.001f, SCAN.tilt, maxTilt);
	TEST_ASSERT_EQUAL_FLOAT(0, nav.tilt());
	TEST_ASSERT_EQUAL(1, slam.scans());
	TEST_ASSERT_GREATER_THAN(250, nav.lastPoints());
	// the walls are in the map: a cell at the +x wall is occupied, the middle free
	int ix, iy, jx, jy;
	TEST_ASSERT_TRUE(slam.grid().cellOf(ROOM - 30, 500, ix, iy));
	TEST_ASSERT_TRUE(slam.grid().cellOf(ROOM + 30, 500, jx, jy));
	TEST_ASSERT_TRUE(slam.grid().at(ix, iy) > 0 || slam.grid().at(jx, jy) > 0);
	TEST_ASSERT_TRUE(slam.grid().cellOf(800, 300, ix, iy));
	TEST_ASSERT_TRUE(slam.grid().at(ix, iy) < 0);
}

void test_goal_scans_then_walks()
{
	LidarSlam slam;
	TEST_ASSERT_TRUE(slam.begin(ORIGIN, MAP));
	TofNavigator nav;
	nav.begin(SCAN, NAV);
	const float dt = 0.02f, maxYawRate = 1.0f, maxSpeed = 200;
	LR_pose2 p = { 0, 0, 0 };
	float mm[64];
	TEST_ASSERT_TRUE(nav.setGoal(0, 1200));   // straight ahead, 1.2 m
	int scans = 0, k = 0;
	bool walked = false;
	for (; k < 20000 && nav.active(); k++) {
		LR_moveVector cmd = nav.update(dt, slam, p, maxYawRate, false);
		if (cmd.move.y > 0) walked = true;
		float sn = sinf(p.theta), cs = cosf(p.theta);
		p.x += (cmd.move.x * cs - cmd.move.y * sn) * maxSpeed * dt;
		p.y += (cmd.move.x * sn + cmd.move.y * cs) * maxSpeed * dt;
		p.theta = lrWrapAngle(p.theta - cmd.rotate.yaw * maxYawRate * dt);
		LR_angle tilt = { 0, nav.tilt(), 0 };
		if (k % 3 == 0) {
			frame(p, tilt, mm);
			nav.addFrame(SENSOR, mm, p, tilt, FLOOR);
		}
		scans = slam.scans();
	}
	TEST_ASSERT_TRUE(walked);
	TEST_ASSERT_GREATER_OR_EQUAL(2, scans);   // one before walking, one after 1 m
	TEST_ASSERT_EQUAL(NAV_ARRIVED, nav.nav().state());
	// stops within goalTolerance of where the map says it is (a few cm off the truth)
	TEST_ASSERT_FLOAT_WITHIN(NAV.goalTolerance + 60, 0, hypotf(p.x - 0, p.y - 1200));
}

/** After the first full scan, rescans on the way sweep only rescanAngle: right first, then left, and still match. */
void test_rescans_on_the_way_are_partial()
{
	LidarSlam slam;
	TEST_ASSERT_TRUE(slam.begin(ORIGIN, MAP));
	TofNavigator nav;
	TofScanParams scan = SCAN;
	scan.rescanAngle = 150 * DEG;
	nav.begin(scan, NAV);
	const float dt = 0.02f, maxYawRate = 1.0f, maxSpeed = 200;
	LR_pose2 p = { 0, 0, 0 };
	float mm[64];
	TEST_ASSERT_TRUE(nav.setGoal(0, 1200));
	int k = 0, partials = 0, fulls = 0;
	bool wasTurning = false, partial = false, wentRight = false;
	float turned = 0, minTurn = 0, maxTurn = 0;
	for (; k < 20000 && nav.active(); k++) {
		LR_moveVector cmd = nav.update(dt, slam, p, maxYawRate, false);
		bool turning = nav.phase() == TOF_SCAN_TURN;
		if (turning && !wasTurning) {
			turned = minTurn = maxTurn = 0;
			partial = nav.partialScan();
			wentRight = false;
		}
		if (!turning && wasTurning) {
			if (partial) {
				partials++;
				TEST_ASSERT_TRUE(wentRight);
				TEST_ASSERT_FLOAT_WITHIN(0.15f, -75 * DEG, minTurn);
				TEST_ASSERT_FLOAT_WITHIN(0.15f, 75 * DEG, maxTurn);
			} else {
				fulls++;
				TEST_ASSERT_TRUE(maxTurn > 6);
			}
		}
		wasTurning = turning;
		if (turning && cmd.rotate.yaw > 0 && turned > -0.01f) {
			wentRight = true;   // clockwise first
		}
		float sn = sinf(p.theta), cs = cosf(p.theta);
		p.x += (cmd.move.x * cs - cmd.move.y * sn) * maxSpeed * dt;
		p.y += (cmd.move.x * sn + cmd.move.y * cs) * maxSpeed * dt;
		float d = -cmd.rotate.yaw * maxYawRate * dt;
		p.theta = lrWrapAngle(p.theta + d);
		if (turning) {
			turned += d;
			minTurn = fminf(minTurn, turned);
			maxTurn = fmaxf(maxTurn, turned);
		}
		LR_angle tilt = { 0, nav.tilt(), 0 };
		if (k % 3 == 0) {
			frame(p, tilt, mm);
			nav.addFrame(SENSOR, mm, p, tilt, FLOOR);
		}
	}
	TEST_ASSERT_EQUAL(1, fulls);
	TEST_ASSERT_GREATER_OR_EQUAL(1, partials);
	TEST_ASSERT_EQUAL(1 + partials, slam.scans());   // every partial sweep matched and went in
	TEST_ASSERT_EQUAL(NAV_ARRIVED, nav.nav().state());
	TEST_ASSERT_FLOAT_WITHIN(NAV.goalTolerance + 60, 0, hypotf(p.x - 0, p.y - 1200));
}

/** A gap the terrain code stops at (gapAhead): no walking forward, after a second it goes into the map. */
void test_gap_ahead_blocks_and_marks_the_map()
{
	LidarSlam slam;
	TEST_ASSERT_TRUE(slam.begin(ORIGIN, MAP));
	TofNavigator nav;
	nav.begin(SCAN, NAV);
	const float dt = 0.02f, maxYawRate = 1.0f;
	LR_pose2 p = { 0, 0, 0 };
	float mm[64];
	TEST_ASSERT_TRUE(nav.setGoal(0, 1500));
	// the first scan
	for (int k = 0; k < 3000; k++) {
		LR_moveVector cmd = nav.update(dt, slam, p, maxYawRate, false);
		p.theta = lrWrapAngle(p.theta - cmd.rotate.yaw * maxYawRate * dt);
		LR_angle tilt = { 0, nav.tilt(), 0 };
		if (k % 3 == 0) {
			frame(p, tilt, mm);
			nav.addFrame(SENSOR, mm, p, tilt, FLOOR);
		}
		if (k > 10 && nav.phase() == TOF_SCAN_IDLE) break;
	}
	TEST_ASSERT_EQUAL(1, slam.scans());
	// facing the goal, a gap 180 mm ahead
	p.theta = 0;
	int cx, cy;
	TEST_ASSERT_TRUE(slam.grid().cellOf(0, 230, cx, cy));
	TEST_ASSERT_TRUE(slam.grid().at(cx, cy) <= 0);
	bool marked = false;
	for (int k = 0; k < 100; k++) {   // 2 s
		nav.gapAhead(NAV.stopDistance);
		LR_moveVector cmd = nav.update(dt, slam, p, maxYawRate, false);
		TEST_ASSERT_TRUE(nav.blocked());
		TEST_ASSERT_TRUE(cmd.move.y <= 0.001f);   // no walking into it
		marked = marked || slam.grid().at(cx, cy) > 0;
	}
	TEST_ASSERT_TRUE(marked);
	// gone (the robot turned away): not blocked any more
	for (int k = 0; k < 10; k++) {
		nav.update(dt, slam, p, maxYawRate, false);
	}
	TEST_ASSERT_FALSE(nav.blocked());
}

int main(int, char **)
{
	UNITY_BEGIN();
	RUN_TEST(test_scan_points_lie_on_the_walls);
	RUN_TEST(test_floor_is_not_a_wall);
	RUN_TEST(test_nothing_in_range_is_free_space);
	RUN_TEST(test_obstacle_ahead);
	RUN_TEST(test_odometry_learns_gyro_bias_standing);
	RUN_TEST(test_navigator_scan_cycle);
	RUN_TEST(test_goal_scans_then_walks);
	RUN_TEST(test_rescans_on_the_way_are_partial);
	RUN_TEST(test_gap_ahead_blocks_and_marks_the_map);
	return UNITY_END();
}
