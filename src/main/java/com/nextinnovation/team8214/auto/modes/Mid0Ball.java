package com.nextinnovation.team8214.auto.modes;

import com.nextinnovation.lib.auto.AutoModeEndedException;
import com.nextinnovation.lib.auto.modes.BaseAutoMode;
import com.nextinnovation.lib.geometry.Rotation2d;
import com.nextinnovation.team8214.Field;
import com.nextinnovation.team8214.auto.TrajectorySet;
import com.nextinnovation.team8214.auto.actions.*;

public class Mid0Ball extends BaseAutoMode {
  private final TrajectorySet trajectorySet = TrajectorySet.getInstance();

  @Override
  protected void routine() throws AutoModeEndedException {
    // Init
    runAction(new ResetPoseAction(Field.Waypoints.MID_START_POSITION));
    runAction(new ResetHeadingAction(new Rotation2d(180.0)));
    runAction(new UpdateSensorRbOffsetAction());
    runAction(new SetVoEnabledAction(false));
    runAction(new SetEnableColorSortingAction(true));

    runAction(new SetInjectAction(true));
    runAction(new SetTrajectoryAction(trajectorySet.midStartToMidEnd, 180.0));
    runAction(new WaitAction(3.0));
    runAction(new SetInjectAction(false));

    runAction(new SetVoEnabledAction(true));
  }
}
