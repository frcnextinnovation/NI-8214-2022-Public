package com.nextinnovation.team8214.auto.modes;

import com.nextinnovation.lib.auto.AutoModeEndedException;
import com.nextinnovation.lib.auto.modes.BaseAutoMode;
import com.nextinnovation.lib.geometry.Rotation2d;
import com.nextinnovation.team8214.Field;
import com.nextinnovation.team8214.auto.TrajectorySet;
import com.nextinnovation.team8214.auto.actions.*;

public class Left2Balls extends BaseAutoMode {
  private final TrajectorySet trajectorySet = TrajectorySet.getInstance();

  @Override
  protected void routine() throws AutoModeEndedException {
    // Init
    runAction(new ResetPoseAction(Field.Waypoints.LEFT_START_POSITION));
    runAction(new ResetHeadingAction(new Rotation2d(135.0)));
    runAction(new UpdateSensorRbOffsetAction());
    runAction(new SetVoEnabledAction(false));
    runAction(new SetEnableColorSortingAction(false));

    // Top start to top ball, shoot 2 balls
    runAction(new SetFlywheelUnlockAction(true));
    runAction(new SetInjectAction(true));
    runAction(new SetTrajectoryAction(trajectorySet.leftStartToLeftBall, 135.0));
    runAction(new WaitAction(1.0));
    runAction(new SetShootAction(true));
    runAction(new SetInjectAction(false));
    runAction(new WaitAction(2.5));

    // Top ball to top reject, reject 1 ball
    runAction(new SetEnableColorSortingAction(true));
    runAction(new SetFlywheelUnlockAction(false));
    runAction(new SetInjectAction(true));
    runAction(new SetTrajectoryAction(trajectorySet.leftBallToLeftEnd, 80.0));
    runAction(new WaitAction(2.5));
    runAction(new SetInjectAction(false));

    runAction(new SetVoEnabledAction(true));
  }
}
