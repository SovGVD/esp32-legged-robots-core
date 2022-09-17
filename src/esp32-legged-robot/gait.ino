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

  gaitLeg.next();

  //@TODO This is too hard!!! Apply some smooth transaction based on minmal `beforeSwing` progress or something
  //bodyBalanceOffset = bodyBalance.getOffset(gaitLeg.getState());

  if (ticksToNextGaitItem <= 0) {
    ticksToNextGaitItem = ticksPerGaitItem;
    currentGait++;
    if (currentGait >= GAIT_CONFIG.sequenceLength) currentGait = 0;

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
