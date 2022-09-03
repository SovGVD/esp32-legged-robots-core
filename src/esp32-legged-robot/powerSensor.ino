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
	voltage_V = ina219.getBusVoltage_V();
	current_A = ina219.getCurrent_mA() / 1000;
}

void setupPowerSensor()
{
  #if POWER_SENSOR == INA219
    ina219 = INA219_WE(POWER_SENSOR_ADDRESS);

    if (ina219.init()) {
      powerSensorReady = true;
    } else {
      Serial.println(" Failed to find INA219 chip");
    }

    ina219.setADCMode(BIT_MODE_9);  // just to make it fast
    ina219.setPGain(PG_320);
    ina219.setBusRange(BRNG_16);
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
