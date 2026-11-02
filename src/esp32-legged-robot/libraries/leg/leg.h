#ifndef leg_h
#define leg_h

#include "../geometry/geometry.h"

// see IK.h
typedef struct {
	float alpha;
	float beta;
	float gamma;
	float delta;
	float epsilon;
	float zeta;
} LR_legAngle;

//Leg gears ratio
typedef struct {
	float alpha;
	float beta;
	float gamma;
	float delta;
	float epsilon;
	float zeta;
} LR_legGear;

// see IK.h
typedef struct {
	float l1;
	float l2;
	float l3;
	float l4;
	float l5;
	float l6;
} LR_legSize;

// Hardware pin numbers
typedef struct {
	uint8_t alpha;
	uint8_t beta;
	uint8_t gamma;
	uint8_t delta;
	uint8_t epsilon;
	uint8_t zeta;
} LR_legPin;

// Leg sensor
typedef struct {
	bool    onGround;      // State
	uint8_t pin;        // Settings
	uint8_t threshold;
} LR_legSensor;

// Leg IK helpers (this is moslty need to set where is leg: left or right side, to inverse some values)
typedef struct {
	bool x;   // position inverse
	bool y;
	bool z;
	bool alpha; // angles inverse
	bool beta;
	bool gamma;
	bool delta;
	bool epsilon;
	bool zeta;
} LR_legInverse;

// Leg HAL values
typedef struct {
	LR_legAngle mid;    // Hardware middle
	LR_legPin   pin;
	LR_legAngle trim;
	LR_legGear  ratio;
} LR_legHal;

typedef struct {
	uint8_t  id;
	char     title[16];
} LR_legId;

// Leg structure
typedef struct {
	const LR_legId     id;
	const LR_point     defaultBody;
	const LR_point     defaultFoot;
	const LR_legSize   size;
	const LR_legAngle  min;	// setup limits of angles
	const LR_legAngle  max;	// setup limits of angles
	const LR_legAngle  calibrate;	// calibrate position
	LR_legHal          hal;
	LR_point           body;
	LR_point           foot;
	LR_legAngle        angle;
	LR_legInverse      inverse;
	LR_legSensor       sensor;
} leg;

#endif
