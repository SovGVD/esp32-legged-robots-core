void initGait() {
	Serial.println("Init gait ");
	ticksPerGaitItem    = GAIT_CONFIG.duration/GAIT_CONFIG.loopTime;
	ticksToNextGaitItem = ticksPerGaitItem;
}

uint8_t getNextGait()
{
  nextGait = currentGait + 1;
  if (nextGait >= GAIT_CONFIG.sequenceLength) nextGait = 0;
  return nextGait;
}

void updateGait() {
  ticksToNextGaitItem--;

  gaitItemProgress = 1 - (float)ticksToNextGaitItem/(float)ticksPerGaitItem;

  LR_figure transitionProgress = bodyTransition.linear(gaitItemProgress);
  body.position                = transitionProgress.position;
  body.orientation             = transitionProgress.orientation;

  bodyUpdate.update();

  for (uint8_t i = 0; i < LEG_NUM; i++) {
    gaitProgress[i] = gaitLeg.next(i);
  }


  // TODO we need predict next position of robot to begin move of CoM at 0.8 progress of current and until 0.2 progress of future gait
  //bodyBalance.setBody(bodyBalance.getCenter());

  if (ticksToNextGaitItem <= 0) {
    ticksToNextGaitItem = ticksPerGaitItem;
    currentGait++;
    if (currentGait >= GAIT_CONFIG.sequenceLength) currentGait = 0;

    // set future position - this needs to be done on 0.8 progress for CoM and balance transition, that is also should include body linear transition
    walkPlanner.predictPosition();

    // body linear transition (TODO, include balance here)
    bodyTransitionParams.initialValue = body;
    bodyTransitionParams.targetValue  = walkPlanner.getBodyPosition();
    bodyTransition.set(bodyTransitionParams);

    // run gait workers
    for (uint8_t i = 0; i < LEG_NUM; i++) {
      if (GAIT_CONFIG.sequence[currentGait].leg[i] == SWING) {
          gaitLeg.start(i, legs[i].foot, walkPlanner.getLegPosition(i));
      }
    }
  }
}
