#define MODEL_SETTINGS_KEY 0xB1bE   // Some magic UNQIE key to verify EEPROM

typedef struct {
	double alpha;
	double beta;
	double gamma;
	double delta;
	double epsilon;
	double zeta;
} modelSettingsAngleTrim;

typedef struct {
	uint16_t               modelKey;
	uint8_t                structSize;
	modelSettingsAngleTrim trim[LEG_NUM];
} modelSettings;
