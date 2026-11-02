#include "robot.h"
#include <Wire.h>
#include <MPU9250_WE.h>

#ifdef POWER_SENSOR
  #if POWER_SENSOR == INA219
    #include <INA219_WE.h>
    static INA219_WE ina219(POWER_SENSOR_ADDRESS);
  #endif
#endif

static MPU9250_WE IMU(IMU_ADDRESS);
static float voltage_V = 0.0;
static float current_A = 0.0;

static void setupIMU();
static void setupPowerSensor();

static bool imuSensorReady = false;
void initIMU()
{
  Serial.print("IMU ");
  setupIMU();
  Serial.println();
}

static void setupIMU()
{
  if(!IMU.init()){
    Serial.println("IMU does not respond");
    return;
  }

  IMU.setAccRange(MPU9250_ACC_RANGE_2G);
  IMU.enableAccDLPF(true);
  IMU.setAccDLPF(MPU9250_DLPF_6);
  //IMU.setSampleRateDivider(255);
  imuSensorReady = true;
}

void calibrateIMU()
{
	if (!imuSensorReady) {
		Serial.println("IMU is not available.");
		return;
	}
	// TODO save offset to EEPROM and restore on boot!!!
	disableServos();
	Serial.println("Calibrating ACC and GYRO in 5 seconds. Put device on flat leveled surface.");
	delay(5000);
	Serial.print("Calibration...");
	IMU.autoOffsets();
	Serial.println("Done.");
	delay(1000);
	enableServos();
}

void updateIMU()
{
	if (!imuSensorReady) {
		return;
	}
	xyzFloat angle = IMU.getAngles();

	// TODO not sure about correct mapping!!!

	IMU_DATA.pitch = degToRad(angle.y);
	IMU_DATA.roll  = degToRad(angle.x);
	IMU_DATA.yaw   = degToRad(angle.z);
}
static bool powerSensorReady = false;
void initPowerSensor() {
  Serial.print("Power sensor ");
  #if POWER_SENSOR == INA219
    Serial.print("INA219");
  #else
    Serial.print("<NO>");
  #endif
  setupPowerSensor();
  Serial.println();
}

void updatePower()
{
	if (!powerSensorReady) {
		return;
	}
	#if POWER_SENSOR == INA219
		voltage_V = ina219.getBusVoltage_V();
		current_A = ina219.getCurrent_mA() / 1000;
	#endif
}

static void setupPowerSensor()
{
  #if POWER_SENSOR == INA219
    if (!ina219.init()) {
      Serial.println(" Failed to find INA219 chip");
      return;
    }

    ina219.setADCMode(INA219_BIT_MODE_9);  // just to make it fast
    ina219.setPGain(INA219_PG_320);
    ina219.setBusRange(INA219_BRNG_16);
    powerSensorReady = true;
  #endif
}

// return V
float getPowerSensorVoltage() {
  return voltage_V;
}


// return A
float getPowerSensorCurrent() {
  return current_A;
}
