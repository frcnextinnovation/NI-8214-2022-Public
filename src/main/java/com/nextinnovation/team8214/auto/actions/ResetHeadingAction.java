package com.nextinnovation.team8214.auto.actions;

import com.nextinnovation.lib.auto.actions.BaseRunOnceAction;
import com.nextinnovation.lib.geometry.Rotation2d;
import com.nextinnovation.team8214.devices.ahrs.AhrsPigeon2;
import com.nextinnovation.team8214.devices.ahrs.BaseAhrs;
import com.nextinnovation.team8214.subsystems.swerve.Swerve;

public class ResetHeadingAction extends BaseRunOnceAction {
  private final Rotation2d newHeading;

  private final BaseAhrs ahrs;
  private final Swerve swerve;

  public ResetHeadingAction(Rotation2d new_heading) {
    newHeading = new_heading;

    ahrs = AhrsPigeon2.getInstance();
    swerve = Swerve.getInstance();
  }

  @Override
  public void runOnce() {
    swerve.disableHeadingController();
    ahrs.setRobotHeading(newHeading);
    swerve.setHeading(newHeading);
  }
}
