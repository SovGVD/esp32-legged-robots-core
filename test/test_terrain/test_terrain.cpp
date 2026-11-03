/**
 * Rough ground: TerrainMap (ToF height map) and the gait's terrain mode
 * (foot switches, mapped footholds).
 */
#include <stdio.h>
#include <unity.h>
#include <robot_core.h>
#include "../common/fixtures.h"

static const float DT = 0.0044f;

void setUp() {}
void tearDown() {}

static LR_moveVector forward(float y)
{
	LR_moveVector c = { { 0, y, 0 }, { 0, 0, 0 } };
	return c;
}

static const TerrainMapParams MAP = { 10, 3.0f, 0.3f, 8 };

static const TerrainParams TERRAIN = {
	20,    // search depth
	150,   // search speed
	10,    // clearance
	30,    // max step
	40,    // body rate
	6,     // edge limit
	30,    // foothold shift
	0,     // foot radius
	5,     // tolerance
	0.05f  // latency
};

void test_map_cells_store_heights_and_forget_them()
{
	TerrainMap map;
	map.begin(MAP);
	float z;
	TEST_ASSERT_FALSE(map.heightAt(0, 0, z));
	LR_point p = { 3, 4, -80 };
	map.addPoint(p);
	TEST_ASSERT_TRUE(map.heightAt(5, 9, z));          // same 10 mm cell
	TEST_ASSERT_FLOAT_WITHIN(1e-4f, -80, z);
	TEST_ASSERT_FALSE(map.heightAt(15, 9, z));        // next cell
	// a lower reading of the same surface only moves it part of the way (noise)
	LR_point lower = { 3, 4, -90 };
	map.addPoint(lower);
	map.heightAt(3, 4, z);
	TEST_ASSERT_FLOAT_WITHIN(1e-3f, -83, z);
	// something on top replaces it at once
	LR_point top = { 3, 4, -60 };
	map.addPoint(top);
	map.heightAt(3, 4, z);
	TEST_ASSERT_FLOAT_WITHIN(1e-4f, -60, z);
	// old cells are forgotten
	map.tick(3.1f);
	TEST_ASSERT_FALSE(map.heightAt(3, 4, z));
	TEST_ASSERT_EQUAL(0, map.knownCells());
}

void test_map_scrolls_with_negative_and_far_coordinates()
{
	TerrainMap map;
	map.begin(MAP);
	LR_point a = { -5, -5, -70 };
	LR_point b = { -5 + 10 * LR_TERRAIN_GRID, -5, -50 };   // same slot, another world cell
	map.addPoint(a);
	float z;
	TEST_ASSERT_TRUE(map.heightAt(-1, -1, z));
	TEST_ASSERT_FLOAT_WITHIN(1e-4f, -70, z);
	map.addPoint(b);
	TEST_ASSERT_FALSE(map.heightAt(-1, -1, z));
	TEST_ASSERT_TRUE(map.heightAt(b.x, b.y, z));
	TEST_ASSERT_FLOAT_WITHIN(1e-4f, -50, z);
}

void test_zone_rays_follow_tilt_and_yaw()
{
	TofSensor s = { { 0, 50, 0 }, 0.0f, (float)M_PI_4, 0.0f, 1, 400 };
	LR_point d = TerrainMap::zoneDirection(s, 0);
	TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0, d.x);
	TEST_ASSERT_FLOAT_WITHIN(1e-4f, sqrtf(0.5f), d.y);
	TEST_ASSERT_FLOAT_WITHIN(1e-4f, -sqrtf(0.5f), d.z);
	// yaw left: the ray turns towards -X
	s.yaw = (float)M_PI_2;
	d = TerrainMap::zoneDirection(s, 0);
	TEST_ASSERT_FLOAT_WITHIN(1e-4f, -sqrtf(0.5f), d.x);
	TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0, d.y);
	// 4x4 zones: row 0 looks higher than row 3, column 0 is left of column 3
	TofSensor m = { { 0, 0, 0 }, 0.0f, 0.5f, 0.785f, 4, 400 };
	TEST_ASSERT_GREATER_THAN_FLOAT(TerrainMap::zoneDirection(m, 12).z, TerrainMap::zoneDirection(m, 0).z);
	TEST_ASSERT_LESS_THAN_FLOAT(TerrainMap::zoneDirection(m, 3).x, TerrainMap::zoneDirection(m, 0).x);
}

void test_readings_land_on_the_ground_even_when_tilted()
{
	// sensor 80 mm above flat ground at z = -80 (body origin at 0)
	TofSensor s = { { 0, 80, 0 }, 0.0f, 0.7f, 0.785f, 8, 400 };
	for (int k = 0; k < 2; k++) {
		LR_figure body = {};
		if (k == 1) {
			body.orientation.roll = 0.1f;     // nose up 5.7 deg (firmware convention: roll about X)
			body.orientation.pitch = -0.05f;  // leaning
		}
		TerrainMap map;
		map.begin(MAP);
		int hits = 0;
		for (uint8_t z = 0; z < 64; z++) {
			LR_point d = lrRotateBody(TerrainMap::zoneDirection(s, z), body.orientation);
			LR_point o = lrRotateBody(s.position, body.orientation);
			if (d.z >= -0.05f) continue;
			float dist = (-80 - o.z) / d.z;     // exact distance to the ground along this ray
			if (dist > s.maxRange) continue;
			map.addReading(s, z, dist, body);
			LR_point at = { o.x + d.x * dist, o.y + d.y * dist, 0 };
			float h;
			TEST_ASSERT_TRUE(map.heightAt(at.x, at.y, h));
			TEST_ASSERT_FLOAT_WITHIN(0.01f, -80, h);
			hits++;
		}
		TEST_ASSERT_GREATER_THAN(20, hits);
	}
}

void test_edges_and_highest_point_along_a_path()
{
	TerrainMap map;
	map.begin(MAP);
	// flat at -80 up to y = 100, a 20 mm step after
	for (float x = -50; x <= 50; x += 10) {
		for (float y = -50; y <= 200; y += 10) {
			LR_point p = { x, y, y < 100 ? -80.0f : -60.0f };
			map.addPoint(p);
		}
	}
	TEST_ASSERT_FLOAT_WITHIN(1e-3f, 0, map.edge(0, 50, 0, 1, 15));
	TEST_ASSERT_FLOAT_WITHIN(1e-3f, 20, map.edge(0, 95, 0, 1, 15));
	TEST_ASSERT_FLOAT_WITHIN(1e-3f, 0, map.edge(0, 150, 0, 1, 15));
	float top;
	LR_point a = { 0, 40, -80 }, b = { 0, 80, -80 }, c = { 0, 140, -80 };
	TEST_ASSERT_TRUE(map.maxHeightAlong(a, b, 5, top));
	TEST_ASSERT_FLOAT_WITHIN(1e-3f, -80, top);
	TEST_ASSERT_TRUE(map.maxHeightAlong(a, c, 5, top));
	TEST_ASSERT_FLOAT_WITHIN(1e-3f, -60, top);
}

void test_terrain_off_changes_nothing()
{
	leg a[4] = DOG_LEGS, b[4] = DOG_LEGS;
	GaitEngine ga, gb;
	ga.begin(dogTrot(), a, 4);
	gb.begin(dogTrot(), b, 4);
	TerrainMap map;
	map.begin(MAP);
	// terrain on without sensors or map knowledge: every leg keeps its neutral ground,
	// only the swing path shape differs, footholds and the body stay the same
	gb.setTerrain(&TERRAIN, &map);
	for (int t = 0; t < (int)(2.0f / DT); t++) {
		ga.update(DT, forward(1));
		gb.update(DT, forward(1));
		for (int i = 0; i < 4; i++) {
			TEST_ASSERT_FLOAT_WITHIN(1e-4f, -DOG_FOOT_Z, gb.groundHeight(i));
			if (!ga.isSwinging(i)) {
				TEST_ASSERT_FLOAT_WITHIN(1e-3f, a[i].foot.z, b[i].foot.z);
				TEST_ASSERT_FLOAT_WITHIN(1e-3f, a[i].foot.y, b[i].foot.y);
			}
		}
		TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0, gb.terrainHeight());
	}
}

/**
 * Simulated ground for the switch: flat at `floorZ`, the switch closes when
 * the commanded foot reaches it.
 */
static void feelGround(leg *legs, GaitEngine &gait, float floorZ, int n)
{
	for (int i = 0; i < n; i++) {
		legs[i].sensor.onGround = legs[i].foot.z <= floorZ + 0.01f;
	}
}

void test_foot_reaches_down_into_a_hole_and_remembers_it()
{
	leg legs[4] = DOG_LEGS;
	GaitParams p = dogTrot();
	p.useContactSensors = true;
	GaitEngine gait;
	gait.begin(p, legs, 4);
	// this switch closes exactly at the commanded height: no servo lag to make up for
	TerrainParams tp = TERRAIN;
	tp.latency = 0;
	gait.setTerrain(&tp, nullptr);
	// the ground is 12 mm lower than the neutral stance everywhere ahead
	const float floorZ = -DOG_FOOT_Z - 12;
	bool searched = false;
	for (int t = 0; t < (int)(2.0f / DT); t++) {
		feelGround(legs, gait, t < 10 ? -DOG_FOOT_Z : floorZ, 4);
		gait.update(DT, forward(0.5f));
		for (int i = 0; i < 4; i++) {
			if (gait.isSearching(i)) searched = true;
			// a foot never goes deeper than the search allows
			TEST_ASSERT_GREATER_THAN_FLOAT(-DOG_FOOT_Z - TERRAIN.searchDepth - 0.1f, legs[i].foot.z);
		}
	}
	TEST_ASSERT_TRUE(searched);
	for (int i = 0; i < 4; i++) {
		// found the floor, and the next swings aim there directly
		TEST_ASSERT_FLOAT_WITHIN(1.0f, floorZ, gait.groundHeight(i));   // one tick of lowering
	}
	// the body followed the ground down
	TEST_ASSERT_FLOAT_WITHIN(1.0f, -12, gait.terrainHeight());
}

void test_early_touchdown_on_something_higher()
{
	leg legs[4] = DOG_LEGS;
	GaitParams p = dogTrot();
	p.useContactSensors = true;
	GaitEngine gait;
	gait.begin(p, legs, 4);
	gait.setTerrain(&TERRAIN, nullptr);
	// a 10 mm plate everywhere: swings hit it on the way down
	const float plate = -DOG_FOOT_Z + 10;
	for (int t = 0; t < (int)(2.0f / DT); t++) {
		for (int i = 0; i < 4; i++) {
			legs[i].sensor.onGround = gait.isSwinging(i) ? legs[i].foot.z <= plate + 0.01f
			                                            : true;
			if (t < 10) legs[i].sensor.onGround = true;
		}
		// opens once the swing is clearly off the ground
		for (int i = 0; i < 4; i++) {
			if (gait.isSwinging(i) && legs[i].foot.z > plate + 1) legs[i].sensor.onGround = false;
		}
		gait.update(DT, forward(0.5f));
	}
	// accepted once the foot is over its foothold (80 % of the swing), so the
	// estimate can be a few mm below the plate (the commanded foot kept going)
	for (int i = 0; i < 4; i++) {
		TEST_ASSERT_FLOAT_WITHIN(4.0f, plate, gait.groundHeight(i));
	}
	TEST_ASSERT_FLOAT_WITHIN(4.0f, 10, gait.terrainHeight());
}

void test_mapped_obstacle_raises_the_swing_and_the_foothold()
{
	leg legs[4] = DOG_LEGS;
	GaitParams p = dogTrot();
	GaitEngine gait;
	gait.begin(p, legs, 4);
	TerrainMap map;
	map.begin(MAP);
	gait.setTerrain(&TERRAIN, &map);
	// flat ground, then a 20 mm platform from y = 110 on (the front feet start at 70.5)
	for (float x = -150; x <= 150; x += 5) {
		for (float y = -150; y <= 400; y += 5) {
			LR_point q = { x, y, y < 110 ? -DOG_FOOT_Z : -DOG_FOOT_Z + 20 };
			map.addPoint(q);
		}
	}
	float maxApex = -1000;
	bool onTop = false;
	for (int t = 0; t < (int)(1.5f / DT); t++) {
		gait.update(DT, forward(0.5f));
		map.tick(0);   // keep the map fresh for the test
		if (gait.isSwinging(0)) {
			if (legs[0].foot.z > maxApex) maxApex = legs[0].foot.z;
		} else if (legs[0].foot.y > 110) {
			onTop = true;
			// standing on the platform, clear of its edge
			TEST_ASSERT_FLOAT_WITHIN(0.01f, -DOG_FOOT_Z + 20, legs[0].foot.z);
			TEST_ASSERT_GREATER_THAN_FLOAT(110 + TERRAIN.footRadius + 5, legs[0].foot.y);
		}
		TEST_ASSERT_FALSE(!gait.isSwinging(0) && legs[0].foot.y > 100 && legs[0].foot.y < 115);
	}
	TEST_ASSERT_TRUE(onTop);
	// over the edge: at least the platform plus the clearance
	TEST_ASSERT_GREATER_THAN_FLOAT(-DOG_FOOT_Z + 20 + TERRAIN.clearance - 0.5f, maxApex);
	TEST_ASSERT_GREATER_THAN_FLOAT(2, gait.terrainHeight());
}

/**
 * Map: flat ground at the neutral stance, a gap from y0 to y1 with its bottom
 * `depth` mm lower (the 40-cell map holds 400 mm along y).
 */
static void mapGap(TerrainMap &map, float y0, float y1, float depth)
{
	for (float x = -150; x <= 150; x += 5) {
		for (float y = -60; y < 340; y += 5) {
			bool in = y >= y0 && y < y1;
			LR_point q = { x, y, in ? -DOG_FOOT_Z - depth : -DOG_FOOT_Z };
			map.addPoint(q);
		}
	}
}

void test_narrow_gap_is_stepped_over()
{
	leg legs[4] = DOG_LEGS;
	GaitEngine gait;
	gait.begin(dogTrot(), legs, 4);
	TerrainMap map;
	map.begin(MAP);
	gait.setTerrain(&TERRAIN, &map);
	mapGap(map, 150, 170, 60);
	float margin = TERRAIN.footRadius + MAP.cellSize;   // gapMargin 0: foot radius + one cell
	for (int t = 0; t < (int)(5.0f / DT); t++) {
		gait.update(DT, forward(0.5f));
		for (int i = 0; i < 4; i++) {
			// no foot stands in the gap or on its brink
			if (!gait.isSwinging(i)) {
				TEST_ASSERT_FALSE(legs[i].foot.y > 150 - margin + 1 && legs[i].foot.y < 170 + margin - 1);
				TEST_ASSERT_FLOAT_WITHIN(0.01f, -DOG_FOOT_Z, legs[i].foot.z);
			}
		}
		TEST_ASSERT_FALSE(gait.isBlocked());
	}
	// all four feet got across
	for (int i = 0; i < 4; i++) {
		TEST_ASSERT_GREATER_THAN_FLOAT(170, legs[i].foot.y);
	}
}

void test_wide_gap_stops_the_robot_until_the_command_lets_go()
{
	leg legs[4] = DOG_LEGS;
	GaitEngine gait;
	gait.begin(dogTrot(), legs, 4);
	TerrainMap map;
	map.begin(MAP);
	gait.setTerrain(&TERRAIN, &map);
	mapGap(map, 150, 340, 60);
	for (int t = 0; t < (int)(4.0f / DT); t++) {
		gait.update(DT, forward(0.5f));
		for (int i = 0; i < 4; i++) {
			// nothing ever steps into the gap
			if (!gait.isSwinging(i)) {
				TEST_ASSERT_LESS_THAN_FLOAT(150, legs[i].foot.y);
			}
			TEST_ASSERT_LESS_THAN_FLOAT(150, legs[i].foot.y);
		}
	}
	TEST_ASSERT_TRUE(gait.isBlocked());
	TEST_ASSERT_FALSE(gait.isActive() && gait.velocity().y > 1);
	// still pushing forward: stays put; the block holds only along its direction
	float y = gait.body().position.y;
	for (int t = 0; t < (int)(1.0f / DT); t++) {
		gait.update(DT, forward(0.5f));
	}
	TEST_ASSERT_FLOAT_WITHIN(1.0f, y, gait.body().position.y);
	// stick released: unblocked, and it can walk back
	gait.update(DT, forward(0));
	TEST_ASSERT_FALSE(gait.isBlocked());
	for (int t = 0; t < (int)(1.0f / DT); t++) {
		gait.update(DT, forward(-0.5f));
	}
	TEST_ASSERT_LESS_THAN_FLOAT(y - 20, gait.body().position.y);
}

void test_cliff_the_sensors_cannot_see_down_also_stops_it()
{
	leg legs[4] = DOG_LEGS;
	GaitEngine gait;
	gait.begin(dogTrot(), legs, 4);
	TerrainMap map;
	map.begin(MAP);
	gait.setTerrain(&TERRAIN, &map);
	// the floor ends at y = 150; right past the edge is unseen (the shadow), the
	// ground far below shows up from y = 220 on
	for (float x = -150; x <= 150; x += 5) {
		for (float y = -60; y < 340; y += 5) {
			if (y >= 150 && y < 220) continue;
			LR_point q = { x, y, y < 150 ? -DOG_FOOT_Z : -DOG_FOOT_Z - 100 };
			map.addPoint(q);
		}
	}
	for (int t = 0; t < (int)(4.0f / DT); t++) {
		gait.update(DT, forward(0.5f));
		for (int i = 0; i < 4; i++) {
			TEST_ASSERT_LESS_THAN_FLOAT(150, legs[i].foot.y);
		}
	}
	TEST_ASSERT_TRUE(gait.isBlocked());
}

int main()
{
	UNITY_BEGIN();
	RUN_TEST(test_map_cells_store_heights_and_forget_them);
	RUN_TEST(test_map_scrolls_with_negative_and_far_coordinates);
	RUN_TEST(test_zone_rays_follow_tilt_and_yaw);
	RUN_TEST(test_readings_land_on_the_ground_even_when_tilted);
	RUN_TEST(test_edges_and_highest_point_along_a_path);
	RUN_TEST(test_terrain_off_changes_nothing);
	RUN_TEST(test_foot_reaches_down_into_a_hole_and_remembers_it);
	RUN_TEST(test_early_touchdown_on_something_higher);
	RUN_TEST(test_mapped_obstacle_raises_the_swing_and_the_foothold);
	RUN_TEST(test_narrow_gap_is_stepped_over);
	RUN_TEST(test_wide_gap_stops_the_robot_until_the_command_lets_go);
	RUN_TEST(test_cliff_the_sensors_cannot_see_down_also_stops_it);
	return UNITY_END();
}
