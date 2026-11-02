#include <unity.h>
#include <robot_core.h>

void setUp() {}
void tearDown() {}

static const BatteryParams P = { 6.8f, 6.4f, 2.0f, 3.0f };
static const float DT = 0.1f;

static BatteryState run(BatteryMonitor &b, float volts, float seconds)
{
	BatteryState s = b.state();
	for (float t = 0; t < seconds; t += DT) s = b.update(DT, volts, true);
	return s;
}

void test_unknown_without_reading()
{
	BatteryMonitor b;
	b.begin(P);
	TEST_ASSERT_EQUAL(BATTERY_UNKNOWN, b.update(DT, 0, false));
	TEST_ASSERT_EQUAL(BATTERY_UNKNOWN, b.update(DT, 0.2f, true));   // USB powered, no battery
	TEST_ASSERT_EQUAL(BATTERY_OK, b.update(DT, 7.6f, true));
}

void test_short_sag_is_ignored()
{
	BatteryMonitor b;
	b.begin(P);
	run(b, 7.4f, 5);
	// servos pull it to 6.0 V for 0.3 s every step
	for (int i = 0; i < 30; i++) {
		run(b, 6.0f, 0.3f);
		run(b, 7.4f, 0.7f);
	}
	TEST_ASSERT_EQUAL(BATTERY_OK, b.state());
}

void test_low_then_critical_latched()
{
	BatteryMonitor b;
	b.begin(P);
	run(b, 7.4f, 5);
	TEST_ASSERT_EQUAL(BATTERY_LOW, run(b, 6.6f, 15));
	TEST_ASSERT_EQUAL(BATTERY_LOW, run(b, 6.85f, 10));   // inside the hysteresis
	TEST_ASSERT_EQUAL(BATTERY_OK, run(b, 7.2f, 10));
	TEST_ASSERT_EQUAL(BATTERY_CRITICAL, run(b, 6.2f, 15));
	TEST_ASSERT_EQUAL(BATTERY_CRITICAL, run(b, 7.4f, 15));   // servos off, voltage recovers: still empty
}

int main()
{
	UNITY_BEGIN();
	RUN_TEST(test_unknown_without_reading);
	RUN_TEST(test_short_sag_is_ignored);
	RUN_TEST(test_low_then_critical_latched);
	return UNITY_END();
}
