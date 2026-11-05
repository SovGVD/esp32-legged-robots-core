#ifndef MODEL_HEXAPOD2_SETTINGS_H
#define MODEL_HEXAPOD2_SETTINGS_H

#define MODEL_SETTINGS_KEY 0x690E   // Some magic UNQIE key to verify EEPROM (not the old hexapod's: other trims)
#define MAX_CPU_CORES      2

typedef struct {
	double alpha;	// Coxa
	double beta;	// Femur
	double gamma;	// Tibia
} modelSettingsAngleTrim;

typedef struct {
	uint16_t               modelKey;
	uint8_t                structSize;
	modelSettingsAngleTrim trim[LEG_NUM];
} modelSettings;

#endif
