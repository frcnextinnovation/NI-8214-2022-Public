package com.nextinnovation.team8214.dashboard;

import com.nextinnovation.lib.geometry.Pose2d;
import com.nextinnovation.lib.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class FieldViewer {
  private final Field2d robotPose;

  private final double poseXOffset;
  private final double poseYOffset;

  public FieldViewer() {
    this(0.0, 0.0);
  }

  public FieldViewer(double x_start_position, double y_start_position) {
    robotPose = new Field2d();

    poseXOffset = x_start_position;
    poseYOffset = y_start_position;

    SmartDashboard.putData(robotPose);
  }

  public void setRobotPose(Pose2d robot_pose) {
    var translation = robot_pose.getTranslation();
    translation = translation.translateBy(new Translation2d(-poseXOffset, -poseYOffset));
    var rotation = robot_pose.getRotation();
    robotPose.setRobotPose(new Pose2d(translation, rotation).toWpilibPose2d());
  }

  public void setVoPose(Pose2d vo_pose) {
    var translation = vo_pose.getTranslation();
    translation = translation.translateBy(new Translation2d(-poseXOffset, -poseYOffset));
    var rotation = vo_pose.getRotation();
    robotPose.getObject("Vo").setPose(new Pose2d(translation, rotation).toWpilibPose2d());
  }
}
