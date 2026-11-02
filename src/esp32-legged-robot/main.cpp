/**
 * ESP32 legged robot firmware entry point.
 *
 * Core 1: robot loop (motion pipeline, sensors, failsafe, servo output).
 * Core 0: services loop (WiFi, web server, CLI, camera).
 */
#include "robot.h"
#include <Wire.h>

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

static unsigned long previousTime            = 0;
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
			mainCamera.setup(
				ESP32CAMERA_FRAMESIZE,
				ESP32CAMERA_FPS,
				ESP32CAMERA_JPEG_QUALITY
			);
		#endif

		Serial.println("ServicesSetup complete");
		serviceLoopReady = true;
	}
}

/**
 * Everything related to robot-only code: i2c, servos, imu, etc
 */
static void robotLoop()
{
	unsigned long currentTime = micros();
	if (currentTime - previousTime >= LOOP_TIME) {
		previousTime = currentTime;

		updateFailsafe();
		updatePower();  // TODO not so often!

		if (!isHalDoReady) {
			updateMotion();
			isHalDoReady = true;
		}
		#if DO_HAL_CORE == 1
			doHAL();
		#endif

		loopTime = micros() - currentTime;
		if (loopTime > LOOP_TIME) {
			Serial.print("WARNING! Increase LOOP_TIME: ");
			Serial.println(loopTime);
		}
	}
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
}

/**
 * Core 1
 */
void loop()
{
	if (robotLoopReady && serviceLoopReady) {
		robotLoop();
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
