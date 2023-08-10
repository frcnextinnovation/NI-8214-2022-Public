package com.nextinnovation.team8214.managers;

import com.nextinnovation.lib.geometry.Pose2d;
import com.nextinnovation.lib.geometry.Rotation2d;
import com.nextinnovation.team8214.Config;

public class CoordinateFrameManager {
  /**
   * CoordinateFrameManager helps with transform coordinate frames throughout the match. A
   * coordinate frame is simply a point and direction in space that defines an (x,y) coordinate
   * system. Transforms (or poses) keep track of the spatial relationship between different frames.
   *
   * <p>Robot frames of interest (from parent to child):
   *
   * <p>1. Field frame: origin is decided by Robot-Automation.
   *
   * <p>2. Hub frame: origin is the center of the hub.
   *
   * <p>3. Robot frame: origin is the center of the robot wheelbase, facing forwards
   *
   * <p>4. Turret frame: origin is the center of the turret.
   *
   * <p>5. Camera frame: origin is the center of the camera.
   */
  public static Pose2d getRobotToTurret(
      Pose2d robot_pose, Rotation2d robot_centric_turret_heading) {
    return robot_pose.transformBy(
        new Pose2d(Config.ROBOT_TO_TURRET_TRANSLATION, robot_centric_turret_heading));
  }

  public static Pose2d getTurretToRobot(
      Pose2d turret_pose, Rotation2d robot_centric_turret_heading) {
    return turret_pose.transformBy(
        new Pose2d(Config.ROBOT_TO_TURRET_TRANSLATION, robot_centric_turret_heading).inverse());
  }

  public static Pose2d getRobotToCamera(
      Pose2d robot_pose, Rotation2d robot_centric_turret_heading) {
    return getRobotToTurret(robot_pose, robot_centric_turret_heading)
        .transformBy(Config.TURRET_TO_CAMERA);
  }

  public static Pose2d getCameraToRobot(
      Pose2d camera_pose, Rotation2d robot_centric_turret_heading) {
    return getTurretToRobot(
        camera_pose.transformBy(Config.TURRET_TO_CAMERA.inverse()), robot_centric_turret_heading);
  }
}
