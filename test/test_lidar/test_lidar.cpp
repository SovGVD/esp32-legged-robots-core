#include <unity.h>
#include <robot_core.h>
#include <string.h>

void setUp() {}
void tearDown() {}

static const float DEG = 3.14159265f / 180.0f;

static const LidarParams LIDAR = { { 0, 0, 40 }, 0.0f, true, 100, 8000, 10 };
static const LidarMapParams MAP = { 50, 6000, 20, 5, 100, 50, 0.1f, 0.25f, 150, 0.25f };

/* ------------------------------------------------------------------ LD06 */

void test_crc_known_value()
{
	// LDROBOT CRC-8 table: crc(0x01) = 0x4D, crc(0x02) = 0x9A
	uint8_t a = 0x01, b = 0x02;
	TEST_ASSERT_EQUAL_HEX8(0x4D, lrLd06Crc8(&a, 1));
	TEST_ASSERT_EQUAL_HEX8(0x9A, lrLd06Crc8(&b, 1));
}

void test_packet_roundtrip_and_resync()
{
	uint16_t mm[LD06_POINTS];
	uint8_t in[LD06_POINTS];
	for (int i = 0; i < LD06_POINTS; i++) { mm[i] = 1000 + i; in[i] = 200; }
	uint8_t pkt[LD06_PACKET_LEN];
	lrLd06Encode(pkt, 3600, 35500, 400, mm, in, 1234);   // wraps through 0 deg

	Ld06Parser p;
	const uint8_t junk[] = { 0x54, 0x00, 0x2C, 0x12, 0x54 };
	int done = 0;
	for (uint8_t b : junk) done += p.feed(b);
	for (uint8_t b : pkt) done += p.feed(b);
	TEST_ASSERT_EQUAL(1, done);
	TEST_ASSERT_EQUAL_FLOAT(3600, p.speed());
	TEST_ASSERT_EQUAL(1000, p.distance(0));
	TEST_ASSERT_EQUAL(1011, p.distance(11));
	TEST_ASSERT_EQUAL(200, p.intensity(5));
	TEST_ASSERT_FLOAT_WITHIN(0.01f, 355.0f, p.angle(0));
	TEST_ASSERT_FLOAT_WITHIN(0.01f, 4.0f, p.angle(11));
	TEST_ASSERT_EQUAL(1234, p.timestamp());

	pkt[10] ^= 0xFF;   // corrupt: dropped and counted
	for (uint8_t b : pkt) done += p.feed(b);
	TEST_ASSERT_EQUAL(1, done);
	TEST_ASSERT_EQUAL(1, p.crcErrors());
}

void test_scan_builder_revolutions_and_filters()
{
	LidarScanBuilder b;
	b.begin(LIDAR);
	bool wrapped = false;
	for (int i = 0; i < 450; i++) {
		wrapped |= b.add(i * 0.8f, 2000, 200);
	}
	TEST_ASSERT_FALSE(wrapped);
	TEST_ASSERT_TRUE(b.add(0.1f, 1500, 200));   // next revolution: the first one is done
	TEST_ASSERT_EQUAL(1, b.scans());
	TEST_ASSERT_EQUAL(2000, b.scan().mm[100]);
	TEST_ASSERT_EQUAL(360, b.scan().points);
	b.add(10.0f, 50, 200);    // closer than minRange: the robot itself
	b.add(11.0f, 3000, 2);    // too weak
	b.add(12.0f, 0, 0);       // no target
	for (int i = 0; i < 10; i++) b.add(300 + i, 1000, 200);
	b.add(0.5f, 1000, 200);
	TEST_ASSERT_EQUAL(0, b.scan().mm[10]);
	TEST_ASSERT_EQUAL(0, b.scan().mm[11]);
	TEST_ASSERT_EQUAL(1500, b.scan().mm[0]);
}

void test_bin_direction()
{
	float dx, dy;
	lrLidarBinDirection(LIDAR, 0, dx, dy);          // ~0 deg: forward
	TEST_ASSERT_FLOAT_WITHIN(0.02f, 0.0f, dx);
	TEST_ASSERT_FLOAT_WITHIN(0.02f, 1.0f, dy);
	lrLidarBinDirection(LIDAR, 90, dx, dy);         // clockwise 90 deg: right
	TEST_ASSERT_FLOAT_WITHIN(0.02f, 1.0f, dx);
	TEST_ASSERT_FLOAT_WITHIN(0.02f, 0.0f, dy);
}

void test_pose_compose_inverse()
{
	LR_pose2 a = { 100, -50, 0.7f };
	LR_pose2 b = { 30, 200, -0.2f };
	LR_pose2 c = lrPoseCompose(a, b);
	LR_pose2 back = lrPoseCompose(lrPoseInverse(a), c);
	TEST_ASSERT_FLOAT_WITHIN(0.01f, b.x, back.x);
	TEST_ASSERT_FLOAT_WITHIN(0.01f, b.y, back.y);
	TEST_ASSERT_FLOAT_WITHIN(1e-4f, b.theta, back.theta);
	// facing theta, forward (0, 1) points to (-sin, cos)
	float x, y;
	lrPoseApply({ 0, 0, 90 * DEG }, 0, 100, x, y);
	TEST_ASSERT_FLOAT_WITHIN(0.01f, -100, x);
	TEST_ASSERT_FLOAT_WITHIN(0.01f, 0, y);
}

/* ------------------------------------------------------------- mapping */

// A room ~4 x 3 m with a box. The walls are not on map cell edges: the
// matcher pulls points toward cell centres, so a wall right on an edge
// biases it by up to half a cell (25 mm); real walls are anywhere.
#define RX0 -1987.0f
#define RX1  2011.0f
#define RY0 -1013.0f
#define RY1  1994.0f
static float rayRect(float ox, float oy, float dx, float dy, float x0, float x1, float y0, float y1, bool inside)
{
	float best = 1e9f;
	const float xs[2] = { x0, x1 }, ys[2] = { y0, y1 };
	for (float X : xs) {
		if (fabsf(dx) < 1e-9f) continue;
		float t = (X - ox) / dx, y = oy + t * dy;
		if (t > 1 && y >= y0 && y <= y1 && t < best) best = t;
	}
	for (float Y : ys) {
		if (fabsf(dy) < 1e-9f) continue;
		float t = (Y - oy) / dy, x = ox + t * dx;
		if (t > 1 && x >= x0 && x <= x1 && t < best) best = t;
	}
	(void)inside;
	return best;
}

static float roomDistance(float ox, float oy, float dx, float dy)
{
	float d = rayRect(ox, oy, dx, dy, RX0, RX1, RY0, RY1, true);
	float b = rayRect(ox, oy, dx, dy, 512, 903, 317, 781, false);
	return b < d ? b : d;
}

/** Scan of the room; the pose moves from `start` to `end` during the revolution. */
static void makeScan(LidarScan &s, const LR_pose2 &start, const LR_pose2 &end)
{
	memset(&s, 0, sizeof(s));
	for (int i = 0; i < LR_LIDAR_BINS; i++) {
		LR_pose2 p = lrPoseLerp(start, end, (i + 0.5f) / LR_LIDAR_BINS);
		float bx, by;
		lrLidarBinDirection(LIDAR, i, bx, by);
		float ox, oy, ex, ey;
		lrPoseApply(p, LIDAR.position.x, LIDAR.position.y, ox, oy);
		lrPoseApply(p, LIDAR.position.x + bx, LIDAR.position.y + by, ex, ey);
		float d = roomDistance(ox, oy, ex - ox, ey - oy);
		if (d < 8000) {
			s.mm[i] = (uint16_t)d;
			s.points++;
		}
	}
}

static LidarSlam slam;   // 40 KB: not on the stack

/** The whole room outline into the map (parts the scans didn't see, behind the box). */
static void drawWalls(OccupancyGrid &g)
{
	for (float t = 0; t <= 1; t += 0.005f) {
		float x = RX0 + (RX1 - RX0) * t, y = RY0 + (RY1 - RY0) * t;
		g.addBeam(x, RY0 + 60, x, RY0, true, 40, 0, 100);
		g.addBeam(x, RY1 - 60, x, RY1, true, 40, 0, 100);
		g.addBeam(RX0 + 60, y, RX0, y, true, 40, 0, 100);
		g.addBeam(RX1 - 60, y, RX1, y, true, 40, 0, 100);
	}
}

static void buildMap()
{
	TEST_ASSERT_TRUE(slam.begin(LIDAR, MAP));
	LidarScan s;
	LR_pose2 zero = { 0, 0, 0 };
	makeScan(s, zero, zero);
	TEST_ASSERT_EQUAL(LIDAR_MATCH_NEW, slam.addScan(s, zero, zero));
	// a few more scans from the start so the walls are confirmed
	for (int i = 0; i < 3; i++) {
		LR_pose2 p = { 60.0f * (i + 1), 0, 0 };
		makeScan(s, p, p);
		slam.addScan(s, p, p);
	}
}

void test_slam_corrects_odometry()
{
	buildMap();
	TEST_ASSERT_TRUE(slam.grid().occupiedCells() > 200);
	LidarScan s;
	LR_pose2 truth = { 250, 350, 8 * DEG };
	LR_pose2 odom  = { 330, 300, 11 * DEG };   // 94 mm and 3 deg off
	makeScan(s, truth, truth);
	TEST_ASSERT_EQUAL(LIDAR_MATCH_OK, slam.addScan(s, odom, odom));
	LR_pose2 p = slam.lastPose();
	TEST_ASSERT_FLOAT_WITHIN(20, truth.x, p.x);
	TEST_ASSERT_FLOAT_WITHIN(20, truth.y, p.y);
	TEST_ASSERT_FLOAT_WITHIN(0.8f * DEG, truth.theta, p.theta);
	// later poses follow the odometry from the corrected one
	LR_pose2 later = slam.pose({ odom.x, odom.y + 100, odom.theta });
	TEST_ASSERT_FLOAT_WITHIN(20, truth.y + 100 * cosf(odom.theta - truth.theta), later.y);
}

void test_slam_deskews_a_turning_scan()
{
	buildMap();
	// turning 0.1 rad during the revolution while moving 30 mm: exact odometry
	LR_pose2 a = { 200, 100, 0 }, b = { 200, 130, 0.1f };
	LidarScan s;
	makeScan(s, a, b);
	TEST_ASSERT_EQUAL(LIDAR_MATCH_OK, slam.addScan(s, a, b));
	LR_pose2 p = slam.lastPose();
	TEST_ASSERT_FLOAT_WITHIN(20, b.x, p.x);
	TEST_ASSERT_FLOAT_WITHIN(20, b.y, p.y);
	TEST_ASSERT_FLOAT_WITHIN(0.5f * DEG, b.theta, p.theta);
}

void test_slam_rejects_a_bad_fit()
{
	buildMap();
	LidarScan s;
	LR_pose2 truth = { 0, 0, 0 };
	LR_pose2 odom  = { 0, 0, 120 * DEG };   // nothing like the map
	makeScan(s, truth, truth);
	LidarMatch m = slam.addScan(s, odom, odom);
	TEST_ASSERT_EQUAL(LIDAR_MATCH_REJECTED, m);
	TEST_ASSERT_TRUE(slam.rejected() >= 1);
}

/* ---------------------------------------------------------------- planner */

static const NavParams NAV = { 130, 100, 350, 120, 0.8f, 1.5f, 0.6f, 180, 1.0f };
static Navigator nav;

void test_planner_goes_through_the_door()
{
	buildMap();
	// a wall across the room at y = 1200 with a door at x = 1000..1600 (put in like scans would)
	OccupancyGrid &g = const_cast<OccupancyGrid &>(slam.grid());
	drawWalls(g);
	for (float x = -2000; x < 2000; x += 25) {
		if (x > 1000 && x < 1600) continue;
		for (int k = 0; k < 3; k++) g.addBeam(x, 1100, x, 1200, true, 20, 0, 100);
	}
	nav.begin(NAV);
	TEST_ASSERT_TRUE(nav.setGoal(-1200, 1700));
	LR_pose2 start = { -1200, 0, 0 };
	TEST_ASSERT_TRUE(nav.plan(slam.grid(), start));
	const float *x, *y;
	uint8_t n = nav.path(x, y);
	TEST_ASSERT_TRUE(n > 5);
	bool throughDoor = false;
	for (uint8_t i = 0; i < n; i++) {
		if (y[i] > 1050 && y[i] < 1250) {
			throughDoor |= x[i] > 1000 && x[i] < 1600;
			TEST_ASSERT_TRUE(x[i] > 1000 && x[i] < 1600);   // never through the wall
		}
	}
	TEST_ASSERT_TRUE(throughDoor);

	// the follower turns toward the path first (it starts facing +y, the door is to the right)
	LR_moveVector cmd = nav.update(0.1f, slam, start, 1.5f);
	TEST_ASSERT_TRUE(cmd.rotate.yaw > 0);   // clockwise
}

void test_planner_no_path_when_closed()
{
	buildMap();
	OccupancyGrid &g = const_cast<OccupancyGrid &>(slam.grid());
	drawWalls(g);
	for (float x = -2000; x < 2000; x += 25) {
		for (int k = 0; k < 3; k++) g.addBeam(x, 1100, x, 1200, true, 20, 0, 100);
	}
	nav.begin(NAV);
	nav.setGoal(-1200, 1700);
	TEST_ASSERT_FALSE(nav.plan(slam.grid(), { -1200, 0, 0 }));
}

int main(int argc, char **argv)
{
	UNITY_BEGIN();
	RUN_TEST(test_crc_known_value);
	RUN_TEST(test_packet_roundtrip_and_resync);
	RUN_TEST(test_scan_builder_revolutions_and_filters);
	RUN_TEST(test_bin_direction);
	RUN_TEST(test_pose_compose_inverse);
	RUN_TEST(test_slam_corrects_odometry);
	RUN_TEST(test_slam_deskews_a_turning_scan);
	RUN_TEST(test_slam_rejects_a_bad_fit);
	RUN_TEST(test_planner_goes_through_the_door);
	RUN_TEST(test_planner_no_path_when_closed);
	return UNITY_END();
}
