/**
 * Inverse Kinematics
 *
 * Developed by Gleb Devyatkin (SovGVD) in 2022
 */

#ifndef IK_h
#define IK_h

#include "math.h"
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
		double normalizeAngleRad(double angleRad);
	private:
		LR_moveVector *_vector;
		LR_figure     *_body;
		LR_point      *_bodyOffset;
		leg           *_legs[LEG_NUM];
		double        ikAtan2(double x, double y);
		double        ikAcos(double angleRad);
		double        ikAsin(double angleRad);
};
#endif
