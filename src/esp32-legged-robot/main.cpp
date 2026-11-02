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

static void robotSetup()
{
	if (!robotLoopReady) {
		Wire.begin(I2C_SDA, I2C_SCL);
		Wire.setClock(400000);
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
 * One motion tick: everything related to robot-only code (i2c, servos, imu).
 */
static void robotStep()
{
	unsigned long currentTime = micros();

	updateFailsafe();

	// power sensor and battery at 10 Hz, not every tick (I2C time)
	static uint32_t powerTicks = 0;
	const uint32_t  powerEvery = 100000 / LOOP_TIME;
	if (++powerTicks >= powerEvery) {
		powerTicks = 0;
		updatePower();
		updateBattery(powerEvery * LOOP_DT);
	}

	if (!isHalDoReady) {
		updateMotion();
		isHalDoReady = true;
	}
	#if DO_HAL_CORE == 1
		doHAL();
	#endif

	loopTime = micros() - currentTime;
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
		if (pending > 1) {
			motionOverruns += pending - 1;
		}
		if (robotLoopReady && serviceLoopReady) {
			robotStep();
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
 * Core 1: the motion task does the work; report overruns once a second.
 */
void loop()
{
	static uint32_t reported = 0;
	uint32_t overruns = motionOverruns;
	if (overruns != reported) {
		Serial.printf("WARNING! Motion loop missed %u ticks (last loop %lu us, LOOP_TIME %u us)\n",
		              (unsigned)(overruns - reported), loopTime, (unsigned)LOOP_TIME);
		reported = overruns;
	}
	vTaskDelay(pdMS_TO_TICKS(1000));
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
