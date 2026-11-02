bool powerSensorReady = false;
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

void setupPowerSensor()
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
double getPowerSensorVoltage() {
  return voltage_V;
}


// return A
double getPowerSensorCurrent() {
  return current_A;
}
