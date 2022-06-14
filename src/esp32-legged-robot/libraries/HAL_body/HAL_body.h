/**
 * Body state recalculate
 *
 * Developed by Gleb Devyatkin (SovGVD) in 2022
 */
 
#ifndef HAL_BODY_h
#define HAL_BODY_h

class HAL_body
{
	public:
		HAL_body(LR_moveVector &vector, LR_angle &correction, LR_figure &bodyObj, leg (&legs)[LEG_NUM]);
		void update();
	private:
		LR_moveVector *_vector;
		LR_angle      *_correction;
		LR_figure     *_body;
		leg           *_legs[LEG_NUM];
};

#endif
