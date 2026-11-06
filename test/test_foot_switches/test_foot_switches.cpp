#include <unity.h>
#include <robot_core.h>

void setUp() {}
void tearDown() {}

void test_change_needs_ticks_in_a_row()
{
	FootSwitches f;
	f.begin(6, 2);
	TEST_ASSERT_FALSE(f.update(0b000001));   // 1st tick pressed
	TEST_ASSERT_FALSE(f.pressed(0));
	TEST_ASSERT_TRUE(f.update(0b000001));    // 2nd tick: now pressed
	TEST_ASSERT_TRUE(f.pressed(0));
	TEST_ASSERT_EQUAL_UINT32(1, f.changes(0));
}

void test_bounce_is_ignored()
{
	FootSwitches f;
	f.begin(6, 3);
	for (int i = 0; i < 10; i++) {
		f.update((i % 2) ? 0b100000 : 0);   // chatter every tick
	}
	TEST_ASSERT_FALSE(f.pressed(5));
	TEST_ASSERT_EQUAL_UINT32(0, f.changes(5));
	f.update(0);   // released again: the count starts over
	f.update(0b100000);
	f.update(0b100000);
	TEST_ASSERT_FALSE(f.pressed(5));
	f.update(0b100000);
	TEST_ASSERT_TRUE(f.pressed(5));
}

void test_release_debounced_too()
{
	FootSwitches f;
	f.begin(2, 2, 0b11);   // standing at boot: both pressed
	TEST_ASSERT_TRUE(f.pressed(0));
	TEST_ASSERT_TRUE(f.pressed(1));
	f.update(0b01);
	TEST_ASSERT_TRUE(f.pressed(1));
	f.update(0b01);
	TEST_ASSERT_FALSE(f.pressed(1));
	TEST_ASSERT_TRUE(f.pressed(0));
}

void test_one_tick_means_no_debounce()
{
	FootSwitches f;
	f.begin(1, 0);   // 0 is treated as 1
	TEST_ASSERT_TRUE(f.update(1));
	TEST_ASSERT_TRUE(f.pressed(0));
}

void test_bits_past_count_ignored()
{
	FootSwitches f;
	f.begin(2, 1, 0xFF);
	TEST_ASSERT_EQUAL_UINT32(0b11, f.state());
	f.update(0xFC);
	TEST_ASSERT_EQUAL_UINT32(0, f.state());
}

void test_apply_to_legs()
{
	leg legs[2] = {};
	FootSwitches f;
	f.begin(2, 1);
	f.update(0b10);
	f.apply(legs, 2);
	TEST_ASSERT_FALSE(legs[0].sensor.onGround);
	TEST_ASSERT_TRUE(legs[1].sensor.onGround);
}

int main()
{
	UNITY_BEGIN();
	RUN_TEST(test_change_needs_ticks_in_a_row);
	RUN_TEST(test_bounce_is_ignored);
	RUN_TEST(test_release_debounced_too);
	RUN_TEST(test_one_tick_means_no_debounce);
	RUN_TEST(test_bits_past_count_ignored);
	RUN_TEST(test_apply_to_legs);
	return UNITY_END();
}
