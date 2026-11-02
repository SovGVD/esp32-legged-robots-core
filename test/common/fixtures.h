/**
 * Robot geometry used by the native tests (copied from model/dog and
 * model/hexapod, without pins/HAL data).
 */
#ifndef TEST_FIXTURES_H
#define TEST_FIXTURES_H

#include <robot_core.h>

#define DOG_BODY_X   39.5f
#define DOG_BODY_Y   70.5f
#define DOG_FOOT_X   56.0f
#define DOG_FOOT_Y   70.5f
#define DOG_FOOT_Z   80.0f
#define DOG_L2       51.9f
#define DOG_L3       50.0f

static inline leg makeDogLeg(uint8_t id, float sx, float sy, LR_legInverse inverse)
{
	leg l = {
		{ id, "" },
		{ sx * DOG_BODY_X, sy * DOG_BODY_Y, 0 },
		{ sx * DOG_FOOT_X, sy * DOG_FOOT_Y, -DOG_FOOT_Z },
		{ 0, DOG_L2, DOG_L3, 0, 0, 0 },
		{ 0, 0, 0, 0, 0, 0 },
		{ (float)M_PI, (float)M_PI, (float)M_PI, 0, 0, 0 },
		{ (float)M_PI_2, (float)M_PI_4, (float)M_PI_2, 0, 0, 0 },
		{},
		{ sx * DOG_BODY_X, sy * DOG_BODY_Y, 0 },
		{ sx * DOG_FOOT_X, sy * DOG_FOOT_Y, -DOG_FOOT_Z },
		{},
		inverse,
		{ true, 0, 0 }
	};
	return l;
}

/**
 * Same order and inverse flags as model/dog/model.cpp: LF, RF, LH, RH.
 * Usage: leg legs[4] = DOG_LEGS;
 */
#define DOG_LEGS { \
	makeDogLeg(0, -1,  1, { true,  true, false, false, true,  true,  false, false, false }), \
	makeDogLeg(1,  1,  1, { false, true, false, true,  false, false, false, false, false }), \
	makeDogLeg(2, -1, -1, { true,  true, false, true,  true,  true,  false, false, false }), \
	makeDogLeg(3,  1, -1, { false, true, false, false, false, false, false, false, false })  \
}

static inline GaitParams dogTrot()
{
	GaitParams p = {
		true, 0.4f, 0.75f, { 0.5f, 0.0f, 0.0f, 0.5f, 0, 0 },
		30, 150, 1.5f, 600, 6, 3, false
	};
	return p;
}

/** Same as GAIT_PARAMS_WALK in model/dog/model_data.h */
static inline GaitParams dogWalk()
{
	GaitParams p = {
		true, 1.2f, 0.9f, { 0.65f, 0.15f, 0.9f, 0.4f, 0, 0 },
		25, 20, 0.25f, 300, 2, 3, false,
		true, 0.15f, { 0.7f, 0, 0 }, { 0, 0, 0 }
	};
	return p;
}

#endif
