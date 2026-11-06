#include "robot.h"
#include <Preferences.h>
#include <EEPROM.h>

/**
 * Settings in NVS (Preferences), namespace "robot":
 *
 *   ver    uint16  SETTINGS_VERSION of the layout below
 *   model  uint16  MODEL_SETTINGS_KEY, settings of another model are ignored
 *   trim   blob    float[LEG_NUM][LEG_DOF] servo trims, rad
 *
 * Every field is read on its own and falls back to its default when it is
 * missing or has the wrong size, so adding a field never wipes the others
 * (the old EEPROM struct was zeroed whenever its size changed). Bump
 * SETTINGS_VERSION and add a step to settingsMigrate() when the meaning of a
 * stored field changes.
 *
 * The first boot after the update imports the trims from the old EEPROM
 * layout once, if they are valid for this model.
 */
#ifdef HIL
	// hardware in the loop: the simulated robot needs no trims, so keep the real
	// robot's settings apart (in case this is the robot's own ESP32)
	#define SETTINGS_NAMESPACE "robot_hil"
#else
	#define SETTINGS_NAMESPACE "robot"
#endif
#define SETTINGS_VERSION   1

static Preferences prefs;

static void settingsMigrate(uint16_t from);
static bool settingsImportEEPROM();
static void settingsLoad();

typedef float trimTable[LEG_NUM][LEG_DOF];

static void trimsFromSettings(trimTable &t)
{
	for (uint8_t i = 0; i < LEG_NUM; i++) {
		t[i][0] = settings.trim[i].alpha;
		t[i][1] = settings.trim[i].beta;
		#if LEG_DOF > 2
			t[i][2] = settings.trim[i].gamma;
		#endif
		#if LEG_DOF > 3
			t[i][3] = settings.trim[i].delta;
			t[i][4] = settings.trim[i].epsilon;
			t[i][5] = settings.trim[i].zeta;
		#endif
	}
}

static void trimsToSettings(const trimTable &t)
{
	for (uint8_t i = 0; i < LEG_NUM; i++) {
		settings.trim[i].alpha = t[i][0];
		settings.trim[i].beta  = t[i][1];
		#if LEG_DOF > 2
			settings.trim[i].gamma = t[i][2];
		#endif
		#if LEG_DOF > 3
			settings.trim[i].delta   = t[i][3];
			settings.trim[i].epsilon = t[i][4];
			settings.trim[i].zeta    = t[i][5];
		#endif
	}
}

void initSettings()
{
	memset(&settings, 0, sizeof(settings));
	settings.modelKey   = MODEL_SETTINGS_KEY;
	settings.structSize = sizeof(settings);

	if (!prefs.begin(SETTINGS_NAMESPACE, false)) {
		Serial.println("Settings: NVS not available, using defaults");
		settingsLoad();
		return;
	}

	uint16_t model   = prefs.getUShort("model", 0);
	uint16_t version = prefs.getUShort("ver", 0);

	if (model == 0 && version == 0) {
		// nothing stored yet
		bool imported = false;
		#ifndef HIL
			imported = settingsImportEEPROM();
		#endif
		if (imported) {
			Serial.println("Settings: imported trims from the old EEPROM settings");
			settingsSave();
		} else {
			Serial.println("Settings: none stored, using defaults");
		}
	} else if (model != MODEL_SETTINGS_KEY) {
		Serial.printf("Settings: stored for another model (%04X), using defaults\n", model);
	} else {
		if (version != SETTINGS_VERSION) {
			settingsMigrate(version);
		}
		trimTable t;
		if (prefs.getBytesLength("trim") == sizeof(t) && prefs.getBytes("trim", t, sizeof(t)) == sizeof(t)) {
			trimsToSettings(t);
		} else {
			Serial.println("Settings: no valid trims stored, using zero trims");
		}
		Serial.println("Settings: loaded");
	}

	settingsLoad();
	settingsPrint();
}

/**
 * Bring stored fields from an older layout version to SETTINGS_VERSION.
 */
static void settingsMigrate(uint16_t from)
{
	Serial.printf("Settings: version %u, current %u\n", from, SETTINGS_VERSION);
	// version 1 is the first NVS layout, nothing to migrate yet
	prefs.putUShort("ver", SETTINGS_VERSION);
}

/**
 * Old layout: the modelSettings struct at EEPROM offset 0, valid when the
 * model key and the struct size match.
 */
static bool settingsImportEEPROM()
{
	modelSettings old;
	if (!EEPROM.begin(sizeof(old))) {
		return false;
	}
	EEPROM.get(0, old);
	EEPROM.end();
	if (old.modelKey != MODEL_SETTINGS_KEY || old.structSize != sizeof(old)) {
		return false;
	}
	settings = old;
	return true;
}

void settingsPrint()
{
	settingsPrintTrim();
	modelSettingsPrint();
}

static void settingsLoad() {
	settingsLoadTrim();
	modelSettingsLoad();
}

void settingsSave()
{
	settingsSaveTrim();
	modelSettingsSave();

	trimTable t;
	trimsFromSettings(t);
	bool ok = prefs.putUShort("model", MODEL_SETTINGS_KEY) == sizeof(uint16_t)
	       && prefs.putUShort("ver", SETTINGS_VERSION) == sizeof(uint16_t)
	       && prefs.putBytes("trim", t, sizeof(t)) == sizeof(t);
	if (!ok) {
		Serial.println("Settings: saving to NVS failed");
		cliSerial->println("Settings: saving to NVS failed");
		return;
	}
	Serial.println("Settings saved.");
	settingsPrint();
}
