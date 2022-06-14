/**
 * Balance helper methods
 *
 * Developed by Gleb Devyatkin (SovGVD) in 2022
 */

#ifndef balance_h
#define balance_h

class balance
{
	public:
		balance(LR_point &offset, LR_figure &bodyObj, leg (&legs)[LEG_NUM]);
		LR_point getCenter();
		void setBody(LR_point CoM);
	private:
		LR_point   *_offset;
		LR_figure  *_body;
		leg        *_legs[LEG_NUM];
		uint8_t    _legsOnGround;
		LR_point   _CoM;
		void       _legToBalance(leg *_leg);
};

#endif
