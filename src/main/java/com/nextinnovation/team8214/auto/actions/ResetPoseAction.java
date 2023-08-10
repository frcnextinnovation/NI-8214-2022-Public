package com.nextinnovation.team8214.auto.actions;

import com.nextinnovation.lib.auto.actions.BaseRunOnceAction;
import com.nextinnovation.lib.geometry.Pose2d;
import com.nextinnovation.team8214.Odometry;

public class ResetPoseAction extends BaseRunOnceAction {
  private final Pose2d newPose;

  public ResetPoseAction(Pose2d new_pose) {
    this.newPose = new_pose;
  }

  @Override
  public void runOnce() {
    Odometry.getInstance().setPose(newPose);
    System.out.println("Swerve pose reset to " + newPose + " !");
  }
}
