/**
 * Inverse Kinematics
 *
 * Developed by Gleb Devyatkin (SovGVD) in 2022
 */

#ifndef IK_h
#define IK_h

#include "../compat/compat.h"
#include "../leg/leg.h"

typedef struct {
	bool isSolved;
	LR_legAngle angle;
} iksolver;

class IK
{
	public:
		IK(LR_moveVector &vector, LR_figure &bodyObj, LR_point &bodyBalanceOffset, leg (&legs)[LEG_NUM]);
		iksolver solve(uint8_t legId);
		float normalizeAngleRad(float angleRad);
	private:
		LR_moveVector *_vector;
		LR_figure     *_body;
		LR_point      *_bodyOffset;
		leg           *_legs[LEG_NUM];
		float        ikAtan2(float x, float y);
		float        ikAcos(float angleRad);
		float        ikAsin(float angleRad);
		float        ikClampUnit(float value);
		bool          _clamped = false;   // set by ikAcos/ikAsin when the target is out of reach
};
#endif
