package com.nextinnovation.team8214.auto.modes;

import com.nextinnovation.lib.auto.AutoModeEndedException;
import com.nextinnovation.lib.auto.actions.SeriesAction;
import com.nextinnovation.lib.auto.modes.BaseAutoMode;
import com.nextinnovation.lib.geometry.Rotation2d;
import com.nextinnovation.team8214.Field;
import com.nextinnovation.team8214.auto.TrajectorySet;
import com.nextinnovation.team8214.auto.actions.*;

public class Right5Balls extends BaseAutoMode {
  private final TrajectorySet trajectorySet = TrajectorySet.getInstance();

  @Override
  protected void routine() throws AutoModeEndedException {
    // Init
    runAction(new ResetPoseAction(Field.Waypoints.RIGHT_START_POSITION));
    runAction(new ResetHeadingAction(new Rotation2d(270.0)));
    runAction(new UpdateSensorRbOffsetAction());
    runAction(new SetEnableColorSortingAction(false));
    runAction(new SetVoEnabledAction(false));

    // Right start to right ball
    runAction(new SetInjectAction(true));
    runAction(new SetFlywheelUnlockAction(true));
    runAction(new SetTrajectoryAction(trajectorySet.rightStartToRightBall, -90.0));
    runAction(new SetShootAction(true));
    runAction(new WaitAction(1.4)); // Org 1.5
    runAction(new SetShootAction(false));

    // Right start to mid ball
    runAction(new SetTrajectoryAction(trajectorySet.rightBallToMidBall, -180.0));
    runAction(new SetInjectAction(false));
    runAction(new SetShootAction(true));
    runAction(new WaitAction(1.3)); // Org 2.5
    runAction(new SetFlywheelUnlockAction(false));
    runAction(new SetShootAction(false));

    // Mid start to human station
    runAction(
        new ParallelAction(
            new SeriesAction(new WaitAction(1.2), new SetInjectAction(true)),
            new SetTrajectoryAction(trajectorySet.midBallToHumanStationBall, -135.0)));
    runAction(new SetTrajectoryAction(trajectorySet.humanStationBallToHumanStationWait, -135.0));
    runAction(new WaitAction(0.9));

    runAction(new SetInjectAction(false));
    runAction(new SetFlywheelUnlockAction(true));
    runAction(new SetTrajectoryAction(trajectorySet.humanStationWaitToMidShoot, -180.0));
    runAction(new SetShootAction(true));

    runAction(new SetVoEnabledAction(true));
    runAction(new SetEnableColorSortingAction(true));
  }
}
