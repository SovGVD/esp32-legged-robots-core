#ifndef gait_h
#define gait_h

#include "../leg/leg.h"
#include "../transition/transition.h"

/* Gait actions */
#define IGNORE 0 // Leg ignored
#define STANCE 1 // Leg on ground
#define SWING  2 // Leg off ground


typedef struct gaitSequence_t {
	const uint8_t leg[LEG_NUM];
} gaitSequence;

typedef struct gaitConfig_t {
	const gaitSequence sequence[16];
	const uint8_t      sequenceLength;
	const double       loopTime;       // loop time in microseconds to correclty calculate number of sub moves per gate item, milliseconds
	double             offTheGround;   // in mm
	double             swingDuration;  // duration of swing in microseconds (1sec = 1000 millisec, 1sec = 1000000microsec)
	double             duration;       // duration of gaitSequence item, in microseconds, should be equal or longer than swingDuration
	double             transitionProgressStep[2]; // @see transition.h
} gaitConfig;

typedef struct gaitTick_t
{
	uint16_t tickCurrent;	// this is from `tickMax` to 0, tickCurrent = tickMax -> 0% of progress, tickCurrent = 0 -> 100%
	uint16_t tickMax;
	double   tickProgress;	// 0.0-1.0 (0 to 100%)
	bool     isInProgress;	// tickCurrent != 0
	bool     isDone;
} gaitTick;


typedef struct gaitState_t
{
	gaitTick progress;		// gait progress
	gaitTick beforeSwing;	// progress before SWING action
	gaitTick swing;			// progress of SWING action
	gaitTick afterSwing;	// progress after SWING action
} gaitState;


class gait
{
	public:
		gait(gaitConfig &config, leg (&legs)[LEG_NUM]);
		void start(uint8_t legId, LR_point from, LR_point to);
		void next();
		gaitState* getState();
	private:
		gaitConfig *_config;
		leg        *_legs[LEG_NUM];
		
		transition           _transition[LEG_NUM];
		transitionParameters _transitionParams;
		
		bool shouldProcess(gaitTick &progress);
		void markedAsFinished(uint8_t i);
		gaitState _progress[LEG_NUM];
};
#endif
