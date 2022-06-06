/**
 * Save and restore settings using EEPROM
 */
#define EEPROM_OFFSET 0
#include "model/settings.cpp"
uint8_t eepromSize = 0;

void initSettings()
{
	eepromSize = sizeof(settings);
	Serial.printf("Settings size %u\n", eepromSize);

	EEPROM.begin(eepromSize);

	settings = EEPROM.get(EEPROM_OFFSET, settings);
	settingsCheck();
	settingsLoad();

	settingsPrint();
}

void settingsCheck()
{
  if (settings.modelKey == MODEL_SETTINGS_KEY && settings.structSize == eepromSize) {
	  Serial.println("Settings valid.");
	  return;
  }
  Serial.print("Invalid settings. Expected [");
  Serial.print(MODEL_SETTINGS_KEY, HEX);
  Serial.print("but key is [");
  Serial.print(settings.modelKey, HEX);
  Serial.print(", size expected [");
  Serial.print(eepromSize, DEC);
  Serial.print("] but size is [");
  Serial.print(settings.structSize, DEC);
  Serial.println("]\n");
  
  settingsInitEEPROM();
}

void settingsInitEEPROM()
{
	Serial.println("Re-init EEPROM");
	for (int i = 0; i < eepromSize; i++) {
		EEPROM.write(i,0);  // Zeroes
	}
	settings = EEPROM.get(EEPROM_OFFSET, settings);
	settings.modelKey   = MODEL_SETTINGS_KEY;
	settings.structSize = eepromSize;

	settingsCommit(false);
}

void settingsCommit(bool printResult)
{
	EEPROM.put(EEPROM_OFFSET, settings);
	if (!EEPROM.commit()) {
		Serial.println("Error commit EEPROM.");
		return;
	}
	settings = EEPROM.get(EEPROM_OFFSET, settings);
	if (printResult) {
		Serial.println("Settings saved.");
		settingsPrint();
	}
}

void settingsPrint()
{
	settingsPrintTrim();
	modelSettingsPrint();
}

void settingsLoad() {
	settingsLoadTrim();
	modelSettingsLoad();
}

void settingsSave()
{
	settingsSaveTrim();
	modelSettingsSave();
	settingsCommit(true);
}
