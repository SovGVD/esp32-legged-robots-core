/**
 * Planner
 *
 * Developed by Gleb Devyatkin (SovGVD) in 2022
 */
 // TODO use gait data to avoid unnecessary legs update
 // TODO do this with matrix and for 3D

#include "planner.h"

planner::planner(LR_moveVector &vector, LR_figure &bodyObj, leg (&legs)[LEG_NUM])
{
	_vector = &vector;
	_body   = &bodyObj;
	for (uint8_t i = 0; i < LEG_NUM; i++) {
		_legs[i] = &legs[i];
	}
}

void planner::predictPosition()
{
	// TODO normalize angle !!! very important !!!
	_predictedBody.orientation.pitch = _body->orientation.pitch;
	_predictedBody.orientation.roll  = _body->orientation.roll;
	_predictedBody.orientation.yaw   = _body->orientation.yaw + rotateInc * _vector->rotate.yaw;
	
	
	double tmpSin = sin(_predictedBody.orientation.yaw);
	double tmpCos = cos(_predictedBody.orientation.yaw);
	
	// TODO use matrix
	_predictedBody.position.x = _body->position.x + moveInc * (_vector->move.x * tmpCos - _vector->move.y * tmpSin);
	_predictedBody.position.y = _body->position.y + moveInc * (_vector->move.x * tmpSin + _vector->move.y * tmpCos);
	_predictedBody.position.z = _body->position.z;
	
	
	// Im trying to get new position of legs based on rotation of default position for XY-plane
	// this is OK for first time, but terrible for anything else
	for (uint8_t i = 0; i < LEG_NUM; i++) {
		_predictedLegFoot[i].x = _predictedBody.position.x + _legs[i]->defaultFoot.x * tmpCos - _legs[i]->defaultFoot.y * tmpSin;
		_predictedLegFoot[i].y = _predictedBody.position.y + _legs[i]->defaultFoot.x * tmpSin + _legs[i]->defaultFoot.y * tmpCos;
		_predictedLegFoot[i].z = _legs[i]->defaultFoot.z;
	}
}

LR_figure planner::getBodyPosition()
{
	return _predictedBody;
}

LR_point planner::getLegPosition(uint8_t legId)
{
	return _predictedLegFoot[legId];
}
