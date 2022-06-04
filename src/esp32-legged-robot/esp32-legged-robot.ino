#include <math.h>

#include "libraries/geometry/geometry.h"
#include "libraries/leg/leg.h"
#include "libraries/transition/transition.h"
#include "libraries/PID/AnglePID.h"
#include "def/def.h"
#include "def/cli.h"
#include "def/subscription.h"
#include "config.h"
#include "config_model.h"
#include "model/model.h"
#include "config_wifi.h"
#include "libraries/gait/gait.h"
#include "model/gait.h"
#include "libraries/planner/planner.h"
#include "libraries/balance/balance.h"
#include "libraries/HAL_body/HAL_body.h"

#include <EEPROM.h>
#include "WiFi.h"
#include "ESPAsyncWebServer.h"
#include "web/index.html.gz.h"

#include <MPU9250_WE.h> // v 1.1.3
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
  #define USE_ESP32_TIMER_NO 3
  #include "ESP32_ISR_Servo.h"  // v1.1.0
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

// run commands on diferent cores (FAST for main, SLOW for services)
bool runCommandFASTCore = false;
bool runCommandSLOWCore = false;
cliFunction cliFunctionFAST;
cliFunction cliFunctionSLOW;
double cliFunctionFASTVar = 0.0;
double cliFunctionSLOWVar = 0.0;

TaskHandle_t ServicesTask;

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

AnglePID imuCorrectionPID(IMU_DATA, imuTarget, imuCorrection, PID_LEVEL_P, PID_LEVEL_I, PID_LEVEL_D, -0.2, 0.2);

HAL_body bodyUpdate(vector, imuCorrection, body, legs);

// WebServer
bool clientOnline = false;
AsyncWebSocketClient * wsclient;
int WiFiMode = AP_MODE;
IPAddress WiFiIP;
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");
uint8_t telemetryPackage[P_TELEMETRY_LEN];

#ifdef ESP32CAMERA
  #include "libraries/camera/esp32camera.h"
  #include "libraries/camera/esp32camera.cpp"
  esp32camera mainCamera(wsclient, ESP32CAMERA_FRAMESIZE, ESP32CAMERA_FPS, ESP32CAMERA_JPEG_QUALITY);
#endif


// CLI
Stream *cliSerial;

// Subscriptions
bool subscriptionEnabled = false;
bool subscriptionBinary = false;

bool mainLoopReady = false;
bool serviceLoopReady = false;


void setup()
{
  Serial.begin(SERIAL_BAUD);
  delay(100);

  initSettings();
  delay(100);

  Wire.begin();
  Wire.setClock(400000);
  delay(100);
  
  initHAL();
  delay(100);
  
  initGait();
  delay(100);

  initIMU();
  delay(100);

  initPowerSensor();
  delay(100);

  
  xTaskCreatePinnedToCore(
    servicesLoop,   /* Task function. */
    "Services",     /* name of task. */
    100000,         /* Stack size of task */
    NULL,           /* parameter of the task */
    1,              /* priority of the task */
    &ServicesTask,  /* Task handle to keep track of created task */
    0);             /* pin task to core 0 */

  mainLoopReady = true;
}

/**
   Main loop for all major things
   Core 1
*/
void loop()
{
  currentTime = micros();
  if (mainLoopReady && serviceLoopReady && currentTime - previousTime >= LOOP_TIME) {
    previousTime = currentTime;

    updateFailsafe();
    updateCLI();  // hmm...
    updatePower();  // TODO not so often!

    updateIMU();
    imuCorrectionPID.update();
    updateGait();
    updateHAL();
    doHAL();

    FS_WS_count++;

    loopTime = micros() - currentTime;
    if (loopTime > LOOP_TIME) {
      //Serial.print("WARNING! Increase LOOP_TIME: ");
      //Serial.println(loopTime);
    }
  }
}

/**
   Loop for service things, like CLI
   Core 0
*/
void servicesSetup() {
  cliSerial = &Serial;
  initCLI();
  initSubscription();

  initWiFi();
  delay(100);
  
  initWebServer();
  delay(100);

  serviceLoopReady = true;
}

void servicesLoop(void * pvParameters) {
  servicesSetup();

  while(mainLoopReady && serviceLoopReady) {
    serviceCurrentTime = micros();

    #ifdef ESP32CAMERA
      if (clientOnline) {
        mainCamera.cameraHandleStream();  // @TODO send by request only!!!
      }
    #endif

    if (serviceCurrentTime - serviceFastPreviousTime >= SERVICE_FAST_LOOP_TIME) {
      serviceFastPreviousTime = serviceCurrentTime;
      
      runFASTCommand();
      doFASTSubscription();

      serviceFastLoopTime = micros() - serviceCurrentTime;
      if (serviceFastLoopTime > LOOP_TIME) {
        Serial.print("WARNING! Increase SERVICE_FAST_LOOP_TIME: ");
        Serial.println(serviceFastLoopTime);
      }      
    }

    if (serviceCurrentTime - servicePreviousTime >= SERVICE_LOOP_TIME) {
      servicePreviousTime = serviceCurrentTime;

      updateWiFi();
      runSLOWCommand();
      
      doSLOWSubscription();

      serviceLoopTime = micros() - serviceCurrentTime;  // this loop + service fast loop
      if (serviceLoopTime > LOOP_TIME) {
        Serial.print("WARNING! Increase SERVICE_LOOP_TIME: ");
        Serial.println(serviceLoopTime);
      }

    }
    vTaskDelay(1);  // https://github.com/espressif/arduino-esp32/issues/595
  }
}

void runFASTCommand()
{
  if (runCommandFASTCore) {
    runCommandFASTCore = false;
    cliFunctionFAST(cliFunctionFASTVar);
  }
}

void runSLOWCommand()
{
  if (runCommandSLOWCore) {
    runCommandSLOWCore = false;
    cliFunctionSLOW(cliFunctionSLOWVar);
  }
}

/**
   TODO
    - calculate center of mass and use it for balance
    - make the queue of tasks by core, e.g. not just   cliFunctionFAST = runI2CScanFAST; but array/list with commands
*/
