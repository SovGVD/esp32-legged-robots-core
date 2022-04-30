#ifndef gait_h
#define gait_h

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
} gaitConfig;

class gait
{
	public:
		gait(gaitConfig &config, leg legs[LEG_NUM]);
		void start(uint8_t legId, LR_point from, LR_point to);
		double next(uint8_t legId);
	private:
		gaitConfig *_config;
		leg        *_legs[LEG_NUM];
		
		transition           _transition[LEG_NUM];
		transitionParameters tParams;
		
		double   progress[LEG_NUM];
		uint16_t ticksToStop[LEG_NUM];
		uint16_t ticksMax[LEG_NUM];
		uint8_t  _currentGait[LEG_NUM];
};
#endif
