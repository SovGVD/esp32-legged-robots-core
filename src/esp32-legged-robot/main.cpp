/**
 * ESP32 legged robot firmware entry point.
 *
 * Core 1: motion task, woken every LOOP_TIME by a hardware timer (esp_timer):
 *         failsafe, sensors, motion pipeline, servo output.
 * Core 0: services loop (WiFi, web server, CLI, camera).
 */
#include "robot.h"
#include <Wire.h>
#include <esp_timer.h>

#ifdef IGNORE_BROWNOUT_CONDITION
	#include "soc/soc.h"           // Disable brownout problems
	#include "soc/rtc_cntl_reg.h"  // Disable brownout problems
#endif

#ifdef ESP32CAMERA
	#define CAMERAENABLED
	#include <esp32camera.h>
	extern esp32camera mainCamera;
	extern bool mainCameraStream;
	extern bool clientOnline;
#endif

/**
 * Shared state, see robot.h
 */
LR_angle      IMU_DATA      = {0, 0, 0};
LR_angle      imuCorrection = {0.0, 0.0, 0.0};
uint16_t      servoPulse[LEG_NUM][LEG_DOF];
bool          isHalDoReady  = false;
unsigned long loopTime      = 0;
modelSettings settings;

// CLI over websocket
static cliStream cliStreamObj;
cliStream *cliSerial = &cliStreamObj;

static TaskHandle_t core0tasks;
static TaskHandle_t motionTaskHandle = nullptr;
static esp_timer_handle_t motionTimer = nullptr;
static volatile uint32_t motionOverruns = 0;   // ticks that came before the previous one was done

static unsigned long servicePreviousTime     = 0;
static unsigned long serviceFastPreviousTime = 0;

static bool robotLoopReady   = false;
static bool serviceLoopReady = false;

static void core0loop(void *pvParameters);

#ifndef I2C_TIMEOUT_MS
	#define I2C_TIMEOUT_MS 10
#endif

static void robotSetup()
{
	if (!robotLoopReady) {
		Wire.begin(I2C_SDA, I2C_SCL);
		Wire.setClock(400000);
		// A sensor read is well under 1 ms at 400 kHz. The default 50 ms
		// timeout would let one stuck transfer stall the motion loop for
		// eleven ticks; fail fast instead (the next tick reads again).
		Wire.setTimeOut(I2C_TIMEOUT_MS);
		vTaskDelay(100);

		initServo();

		initHAL();
		vTaskDelay(100);

		initMotion();
		vTaskDelay(100);

		initIMU();
		vTaskDelay(100);

		initPowerSensor();
		initBattery();
		vTaskDelay(100);

		robotLoopReady = true;
		Serial.println("MainSetup complete");
	}
}

static void servicesSetup()
{
	if (!serviceLoopReady) {
		initServo();
		vTaskDelay(100);

		initCLI();
		vTaskDelay(100);

		initWiFi();
		vTaskDelay(100);

		initWebServer();
		vTaskDelay(100);

		#ifdef ESP32CAMERA
			initCameraStream();
		#endif

		Serial.println("ServicesSetup complete");
		serviceLoopReady = true;
	}
}

/**
 * Motion loop profiling. The motion task fills `tick` and keeps the slowest
 * tick and the sums of the current one second window; loop() swaps the window
 * out once a second (under the lock) for the serial report and CLI `loop`.
 */
struct LoopWindow {
	uint32_t ticks;
	uint32_t missed;
	uint64_t sum[LOOP_STAGE_COUNT];
	uint64_t sumTotal;
	uint32_t maxPeriod;                 // longest time between two tick starts, us
	uint32_t worstTotal;                // slowest tick, us
	uint32_t worst[LOOP_STAGE_COUNT];   // longest time of each stage, us (not always the same tick)
};

static portMUX_TYPE profileLock = portMUX_INITIALIZER_UNLOCKED;
static uint32_t     tickStage[LOOP_STAGE_COUNT];   // motion task only
static LoopWindow   window     = {};               // being filled
static LoopWindow   lastWindow = {};               // previous full second

static const char *const loopStageNames[LOOP_STAGE_COUNT] = {
	"failsafe", "power", "imu", "motion", "hal"
};

void loopProfileMark(LoopStage stage, uint32_t us)
{
	tickStage[stage] += us;
}

static void loopProfileTick(uint32_t total, uint32_t period, uint32_t missed)
{
	portENTER_CRITICAL(&profileLock);
	window.ticks++;
	window.missed   += missed;
	window.sumTotal += total;
	for (uint8_t i = 0; i < LOOP_STAGE_COUNT; i++) {
		window.sum[i] += tickStage[i];
	}
	if (period > window.maxPeriod) {
		window.maxPeriod = period;
	}
	if (total > window.worstTotal) {
		window.worstTotal = total;
	}
	for (uint8_t i = 0; i < LOOP_STAGE_COUNT; i++) {
		if (tickStage[i] > window.worst[i]) {
			window.worst[i] = tickStage[i];
		}
	}
	portEXIT_CRITICAL(&profileLock);
}

static LoopWindow loopProfileRoll()
{
	portENTER_CRITICAL(&profileLock);
	lastWindow = window;
	window     = LoopWindow{};
	LoopWindow w = lastWindow;
	portEXIT_CRITICAL(&profileLock);
	return w;
}

static void printLoopWindow(Print &out, const LoopWindow &w)
{
	out.printf("Motion loop, last second: %u ticks (expected %u), %u missed, longest gap %u us\n",
	           (unsigned)w.ticks, (unsigned)(1000000 / LOOP_TIME), (unsigned)w.missed, (unsigned)w.maxPeriod);
	if (w.ticks == 0) {
		return;
	}
	out.printf("  %-9s %8s %8s\n", "stage", "avg us", "max us");
	for (uint8_t i = 0; i < LOOP_STAGE_COUNT; i++) {
		out.printf("  %-9s %8u %8u\n", loopStageNames[i],
		           (unsigned)(w.sum[i] / w.ticks), (unsigned)w.worst[i]);
	}
	out.printf("  %-9s %8u %8u (LOOP_TIME %u)\n", "total",
	           (unsigned)(w.sumTotal / w.ticks), (unsigned)w.worstTotal, (unsigned)LOOP_TIME);
}

void cliLoop()
{
	portENTER_CRITICAL(&profileLock);
	LoopWindow w = lastWindow;
	portEXIT_CRITICAL(&profileLock);
	printLoopWindow(*cliSerial, w);
	cliSerial->printf("Missed since boot: %u\n", (unsigned)motionOverruns);
}

// Longest dt the motion pipeline is given after a stall: a late tick moves the
// gait on by the real elapsed time, but never jumps further than this.
#define MOTION_MAX_DT 0.05f

/**
 * One motion tick: everything related to robot-only code (i2c, servos, imu).
 * The pipeline gets the real time since the previous tick, so the gait keeps
 * its speed even if ticks are missed (it used to advance LOOP_DT per tick, so
 * a loop running at a ninth of its rate walked nine times slower).
 */
static void robotStep(uint32_t missed)
{
	static uint32_t previousStart = 0;
	uint32_t currentTime = micros();
	uint32_t period = previousStart ? currentTime - previousStart : LOOP_TIME;
	previousStart = currentTime;
	float dt = period / 1000000.0f;
	if (dt > MOTION_MAX_DT) {
		dt = MOTION_MAX_DT;
	}
	memset(tickStage, 0, sizeof(tickStage));

	uint32_t t = micros();
	updateFailsafe();
	loopProfileMark(LOOP_STAGE_FAILSAFE, micros() - t);

	// power sensor and battery at 10 Hz, not every tick (I2C time)
	static float powerDt = 0;
	powerDt += dt;
	if (powerDt >= 0.1f) {
		t = micros();
		updatePower();
		updateBattery(powerDt);
		loopProfileMark(LOOP_STAGE_POWER, micros() - t);
		powerDt = 0;
	}

	if (!isHalDoReady) {
		t = micros();
		updateMotion(dt);
		// the IMU read is inside updateMotion and counted on its own
		loopProfileMark(LOOP_STAGE_MOTION, micros() - t);
		isHalDoReady = true;
	}
	#if DO_HAL_CORE == 1
		t = micros();
		doHAL();
		loopProfileMark(LOOP_STAGE_HAL, micros() - t);
	#endif

	loopTime = micros() - currentTime;
	loopProfileTick(loopTime, period, missed);
}

static void motionTimerTick(void *)
{
	xTaskNotifyGive(motionTaskHandle);
}

/**
 * Fixed rate motion task. A periodic esp_timer (microsecond resolution;
 * the FreeRTOS tick is 1 ms, too coarse for the 4.4 ms LOOP_TIME) wakes it
 * every LOOP_TIME, instead of loop() spinning on micros().
 */
static void motionTask(void *)
{
	for (;;) {
		uint32_t pending = ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
		uint32_t missed  = pending > 1 ? pending - 1 : 0;
		motionOverruns += missed;
		if (robotLoopReady && serviceLoopReady) {
			robotStep(missed);
		}
	}
}

static void startMotionTask()
{
	xTaskCreatePinnedToCore(motionTask, "motion", 8192, NULL, 5, &motionTaskHandle, MOTOR_CORE);

	const esp_timer_create_args_t args = {
		.callback = motionTimerTick,
		.arg = nullptr,
		.dispatch_method = ESP_TIMER_TASK,
		.name = "motion",
		.skip_unhandled_events = true,
	};
	ESP_ERROR_CHECK(esp_timer_create(&args, &motionTimer));
	ESP_ERROR_CHECK(esp_timer_start_periodic(motionTimer, LOOP_TIME));
}

uint32_t motionOverrunCount()
{
	return motionOverruns;
}

/**
 * WiFi, WebServer, Cli and other bells and whistles
 */
static void servicesLoop()
{
	unsigned long serviceCurrentTime = micros();

	#ifdef CAMERAENABLED
		if (clientOnline && mainCameraStream) {
			mainCamera.processFrame();
		}
	#endif

	if (serviceCurrentTime - serviceFastPreviousTime >= LOOP_TIME) {
		serviceFastPreviousTime = serviceCurrentTime;

		#if DO_HAL_CORE == 0
			doHAL();
		#endif

		unsigned long serviceFastLoopTime = micros() - serviceCurrentTime;
		if (serviceFastLoopTime > LOOP_TIME) {
			Serial.print("WARNING! Increase SERVICE_FAST_LOOP_TIME: ");
			Serial.println(serviceFastLoopTime);
		}
	}

	if (serviceCurrentTime - servicePreviousTime >= SERVICE_LOOP_TIME) {
		servicePreviousTime = serviceCurrentTime;

		updateCLI();
		CLI_doCommand();
		updateWebServer();

		unsigned long serviceLoopTime = micros() - serviceCurrentTime;
		if (serviceLoopTime > SERVICE_LOOP_TIME) {
			Serial.print("WARNING! Increase SERVICE_LOOP_TIME: ");
			Serial.println(serviceLoopTime);
		}
	}
}

void setup()
{
	#ifdef IGNORE_BROWNOUT_CONDITION
		WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); //disable brownout detector
	#endif
	Serial.begin(SERIAL_BAUD);
	cliStreamObj.setStreamPrefix(P_CLI);

	initSettings();
	vTaskDelay(100);

	robotSetup();
	vTaskDelay(10);

	xTaskCreatePinnedToCore(
		core0loop,      /* Task function. */
		"core0loop",    /* name of task. */
		16384,          /* Stack size of task, bytes */
		NULL,           /* parameter of the task */
		0,              /* priority of the task */
		&core0tasks,    /* Task handle to keep track of created task */
		0);             /* pin task to core 0 */

	startMotionTask();
}

/**
 * Core 1: the motion task does the work. Once a second the profiling window
 * is rolled over; a second that missed ticks is printed with the time of each
 * stage, so a USB serial monitor shows what holds the loop up.
 */
void loop()
{
	vTaskDelay(pdMS_TO_TICKS(1000));
	LoopWindow w = loopProfileRoll();
	if (w.missed > 0 || w.worstTotal > LOOP_TIME) {
		Serial.print("WARNING! ");
		printLoopWindow(Serial, w);
	}
}

/**
 * Core 0
 */
static void core0loop(void *pvParameters)
{
	if (robotLoopReady) {
		servicesSetup();
	}

	while (robotLoopReady && serviceLoopReady) {
		servicesLoop();
		vTaskDelay(1);
	}

	// A FreeRTOS task must never return from its function.
	vTaskDelete(NULL);
}
