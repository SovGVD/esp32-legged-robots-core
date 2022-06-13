#include <math.h>

#include "libraries/geometry/geometry.h"
#include "libraries/leg/leg.h"
#include "libraries/transition/transition.h"
#ifdef HORIZON_LEVEL
	#include "libraries/PID/AnglePID.h"
#endif
#include "def/def.h"
#include "def/cli.h"
#include "def/subscription.h"
#include "config.h"
#include "config_model.h"
#include "model/model.h"
#include "config_wifi.h"
#include "libraries/gait/gait.h"
#include "model/gait.h"
#include "model/settings.h"
#include "libraries/planner/planner.h"
#include "libraries/balance/balance.h"
#include "libraries/HAL_body/HAL_body.h"

#include <EEPROM.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <WiFiAP.h>
#include <DNSServer.h>
#include "ESPAsyncWebServer.h"
#include "web/index.html.gz.h"

#include <MPU9250_WE.h>
#include <Wire.h>

#include "libraries/PID/AnglePID.cpp"
#include "libraries/transition/transition.cpp"
#include "libraries/planner/planner.cpp"
#include "libraries/balance/balance.cpp"
#include "libraries/gait/gait.cpp"
#include "libraries/HAL_body/HAL_body.cpp"

/**
 * Hardware libraries
 */
#if PWM_CONTROLLER_TYPE == PCA9685
  #include <Adafruit_PWMServoDriver.h>
#endif

#if PWM_CONTROLLER_TYPE == ESP32PWM
  #define USE_ESP32_TIMER_NO 0
  #include "ESP32_ISR_Servo.h"
#endif

#ifdef POWER_SENSOR
  float voltage_V = 0.0;
  float current_A = 0.0;

  #if POWER_SENSOR == INA219
    #include <INA219_WE.h>
    INA219_WE ina219;
  #endif
#endif

LR_angle IMU_DATA = {0, 0, 0};
MPU9250_WE IMU;

TaskHandle_t core0tasks;

#if PWM_CONTROLLER_TYPE == PCA9685
  Adafruit_PWMServoDriver pwm;
#endif

unsigned long currentTime;
unsigned long previousTime;
unsigned long loopTime;

unsigned long serviceCurrentTime;

unsigned long servicePreviousTime;
unsigned long serviceLoopTime;

unsigned long serviceFastPreviousTime;
unsigned long serviceFastLoopTime;

bool HALEnabled = true;

// Gait
uint16_t ticksPerGaitItem    = 0;
uint16_t ticksToNextGaitItem = 0;
uint8_t  currentGait         = 0;
uint8_t  nextGait            = 0;
double   gaitItemProgress    = 0;
double   gaitProgress[]      = {0, 0, 0, 0};

transition bodyTransition;
transitionParameters bodyTransitionParams = {{0,0,0}, {0,0,0}, 0};

//Move
LR_moveVector vector = {
  {0, 0, 0},
  {0, 0, 0}
};

//Failsafe
bool FS_FAIL = false;
uint8_t FS_WS_count = 0;

// HAL
LR_figure body = {
  {  0,  0,  0},
  {  0,  0,  0}
};


IK ikLeg(
  body, 
  legs
);
gait gaitLeg(
  GAIT_CONFIG, 
  legs
);


/* We need predict future position of legs/body */
planner walkPlanner(
  vector,
  body,
  legs
);

/* and balance it someway */
balance bodyBalance(
  balanceOffset,
  body,
  legs
);

// IMU+PID
LR_angle imuCorrection = {0.0, 0.0, 0.0};
LR_angle imuTarget     = {0.0, 0.0, 0.0};

#ifdef HORIZON_LEVEL
	AnglePID imuCorrectionPID(IMU_DATA, imuTarget, imuCorrection, PID_LEVEL_P, PID_LEVEL_I, PID_LEVEL_D, -0.2, 0.2);
#endif

HAL_body bodyUpdate(vector, imuCorrection, body, legs);

// WebServer
IPAddress apIP(192,168,1,1);
IPAddress apGateway(192,168,1,1);
IPAddress apSubnet(255,255,255,0);
bool clientOnline = false;
AsyncWebSocketClient * wsclient;
int WiFiMode = AP_MODE;
IPAddress WiFiIP;
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");
uint8_t telemetryPackage[P_TELEMETRY_LEN];

// DNS server
const byte DNS_PORT = 53;
DNSServer dnsServer;

#ifdef ESP32CAMERA
	#define CAMERAENABLED
	#include "libraries/camera/esp32camera.h"
	#include "libraries/camera/esp32camera.cpp"
	esp32camera mainCamera(wsclient, ESP32CAMERA_FRAMESIZE, ESP32CAMERA_FPS, ESP32CAMERA_JPEG_QUALITY);
	bool mainCameraStream = false
#endif

// CLI
Stream *cliSerial;

// Settings
modelSettings settings;

bool mainLoopReady = false;
bool serviceLoopReady = false;

void mainSetup()
{
	if (!mainLoopReady) {
		initSettings();
		vTaskDelay(100);

		Wire.begin();
		Wire.setClock(400000);
		vTaskDelay(100);

		initWiFi();
		vTaskDelay(100);

		initHAL();
		vTaskDelay(100);

		initGait();
		vTaskDelay(100);

		initIMU();
		vTaskDelay(100);

		initPowerSensor();
		vTaskDelay(100);

		mainLoopReady = true;
		Serial.println("MainSetup complete");
	}
}

void servicesSetup()
{
	if (!serviceLoopReady) {
		initCLI();
		vTaskDelay(100);

		initWebServer();
		vTaskDelay(100);

		Serial.println("ServicesSetup complete");
		serviceLoopReady = true;
	}
}

void mainLoop()
{
	currentTime = micros();
	if (currentTime - previousTime >= LOOP_TIME) {
		previousTime = currentTime;

		updateFailsafe();
		updatePower();  // TODO not so often!

		updateIMU();
		#ifdef HORIZON_LEVEL
			imuCorrectionPID.update();
		#endif
		updateGait();

		updateHAL();
		doHAL();

		FS_WS_count++;

		loopTime = micros() - currentTime;
		if (loopTime > LOOP_TIME) {
			Serial.print("WARNING! Increase LOOP_TIME: ");
			Serial.println(loopTime);
		}
	}
}


void servicesLoop() {
    serviceCurrentTime = micros();

    #ifdef CAMERAENABLED
      if (clientOnline && mainCameraStream) {
        mainCamera.cameraHandleStream();
      }
    #endif

    if (serviceCurrentTime - servicePreviousTime >= SERVICE_LOOP_TIME) {
      servicePreviousTime = serviceCurrentTime;

      updateCLI();
      
      serviceLoopTime = micros() - serviceCurrentTime;
      if (serviceLoopTime > SERVICE_LOOP_TIME) {
        Serial.print("WARNING! Increase SERVICE_LOOP_TIME: ");
        Serial.println(serviceLoopTime);
      }
    }
}

/**
   TODO
    - calculate center of mass and use it for balance
*/

void setup()
{
	Serial.begin(SERIAL_BAUD);
	cliSerial = &Serial;

	mainSetup();
	vTaskDelay(10);

	xTaskCreatePinnedToCore(
		core0loop,      /* Task function. */
		"core0loop",    /* name of task. */
		100000,         /* Stack size of task */
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
	if(mainLoopReady && serviceLoopReady) {
		dnsServer.processNextRequest();
		mainLoop();
	}
}

/**
 * Core 0
 */
void core0loop(void * pvParameters) {
	if (mainLoopReady) {
		servicesSetup();
	}
	while(mainLoopReady && serviceLoopReady) {
		servicesLoop();
		vTaskDelay(10);
	}
	vTaskDelay(100);
}
