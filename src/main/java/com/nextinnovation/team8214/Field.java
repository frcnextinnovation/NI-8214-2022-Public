package com.nextinnovation.team8214;

import com.nextinnovation.lib.geometry.Pose2d;
import com.nextinnovation.lib.geometry.Rotation2d;
import com.nextinnovation.lib.geometry.Translation2d;

public final class Field {
  // Field Edge
  public static final double Y_MAX = 8.2296;
  public static final double Y_MIN = 0.0;
  public static final double X_MAX = 16.4592;
  public static final double X_MIN = 0.0;

  public static final double X_MID = X_MIN + (X_MAX - X_MIN) / 2.0;
  public static final double Y_MID = Y_MIN + (Y_MAX - Y_MIN) / 2.0;

  // Physical Dimensions of Field Objects
  public static final double VISUAL_TARGET_VISUAL_CENTER_HEIGHT = 2.516632;
  public static final double UPPER_HUB_RADIANS = 0.6096;

  public static final class Waypoints {
    public static final Translation2d HUB_POSITION = new Translation2d(X_MID, Y_MID);
    public static final Translation2d FRIEND_HANGER_POSITION = new Translation2d(0.0, 6.66);
    public static final Translation2d ENEMY_HANGER_POSITION = new Translation2d(X_MAX, 1.45);

    // Left waypoints
    public static final Pose2d LEFT_START_POSITION =
        new Pose2d(new Translation2d(6.17, 5.20), new Rotation2d(135.0));
    public static final Pose2d LEFT_BALL_IN_POSITION =
        new Pose2d(new Translation2d(5.35, 5.85), new Rotation2d(135.0));
    public static final Pose2d LEFT_BALL_OUT_POSITION =
        new Pose2d(new Translation2d(5.35, 5.85), new Rotation2d(0.0));
    public static final Pose2d LEFT_END_POSITION =
        new Pose2d(new Translation2d(6.07, 6.81), new Rotation2d(90.0));

    // Mid waypoints
    public static final Pose2d MID_START_POSITION =
        new Pose2d(new Translation2d(6.01, 4.0), new Rotation2d(-180));
    public static final Pose2d MID_START_OUT_POSITION =
        new Pose2d(new Translation2d(6.01, 4.0), new Rotation2d(-90));
    public static final Pose2d MID_END_POSITION =
        new Pose2d(new Translation2d(5.00, 3.29), new Rotation2d(-180.0));

    // Right waypoints
    public static final Pose2d RIGHT_START_POSITION =
        new Pose2d(new Translation2d(7.66, 1.87), new Rotation2d(270.0));
    public static final Pose2d RIGHT_BALL_IN_POSITION =
        new Pose2d(new Translation2d(7.66, 0.92), new Rotation2d(270.0));
    public static final Pose2d RIGHT_BALL_OUT_POSITION =
        new Pose2d(new Translation2d(7.66, 0.92), new Rotation2d(90.0));
    public static final Pose2d MID_BALL_POSITION =
        new Pose2d(new Translation2d(5.60, 1.98), new Rotation2d(-180.0));
    public static final Pose2d HUMAN_STATION_BALL_IN_POSITION =
        new Pose2d(new Translation2d(1.71, 1.74), new Rotation2d(-135.0));
    public static final Pose2d HUMAN_STATION_BALL_OUT_POSITION =
        new Pose2d(new Translation2d(1.71, 1.74), new Rotation2d(45.0));
    public static final Pose2d HUMAN_STATION_WAIT_IN_POSITION =
        new Pose2d(new Translation2d(1.86, 1.89), new Rotation2d(45.0));
    public static final Pose2d HUMAN_STATION_WAIT_OUT_POSITION =
        new Pose2d(new Translation2d(1.86, 1.89), new Rotation2d(0.0));
    public static final Pose2d RIGHT_END_SHOOT_POSITION =
        new Pose2d(new Translation2d(5.00, 1.89), new Rotation2d(0.0));
  }
}
