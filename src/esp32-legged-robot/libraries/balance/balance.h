/**
 * Balance helper methods
 *
 * Developed by Gleb Devyatkin (SovGVD) in 2022
 */

#ifndef balance_h
#define balance_h

#include "../leg/leg.h"
#include "../gait/gait.h"

class balance
{
	public:
		balance(LR_figure &bodyObj, leg (&legs)[LEG_NUM]);
		LR_point getOffset(gaitState* state);
	private:
		LR_point   _offset;
		LR_figure  *_body;
		leg        *_legs[LEG_NUM];
		uint8_t    _legsOnGround;
		void       _legToBalance(leg *_leg);
};

#endif
