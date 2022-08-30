#define MODEL_SETTINGS_KEY 0xD060   // Some magic UNQIE key to verify EEPROM


typedef struct {
	double alpha;
	double beta;
	double gamma;
} modelSettingsAngleTrim;

typedef struct {
	uint16_t               modelKey;
	uint8_t                structSize;
	modelSettingsAngleTrim trim[LEG_NUM];
} modelSettings;
