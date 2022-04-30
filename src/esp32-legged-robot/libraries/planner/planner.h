/**
 * Planner
 * Almost, it is just move body and legs to the new point
 * Also it looks like a good place to check if new possition possible
 *
 * Developed by Gleb Devyatkin (SovGVD) in 2022
 */

#ifndef planner_h
#define planner_h

class planner
{
	public:
		planner(LR_moveVector &vector, LR_figure &bodyObj, leg legs[LEG_NUM]);
		void predictPosition();
		LR_figure getBodyPosition();
		LR_point getLegPosition(uint8_t legId);
	private:
		LR_moveVector *_vector;
		LR_figure     *_body;
		leg           *_legs[LEG_NUM];
		LR_figure     _predictedBody;
		LR_point      _predictedLegFoot[LEG_NUM];
		double        moveInc = 30;
		double        rotateInc = -0.3;
};

#endif
