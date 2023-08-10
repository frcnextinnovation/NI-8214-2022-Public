package com.nextinnovation.team8214.subsystems.swerve;

import com.nextinnovation.lib.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

import java.util.Arrays;
import java.util.List;

public final class SwerveConfig {
  // Swerve
  public static final double WHEELBASE_HALF_LENGTH_METER = 0.284734;
  public static final double WHEELBASE_HALF_WIDTH_METER = 0.284734;
  public static final int MODULE_COUNT = 4;

  public static final Translation2d FRONT_LEFT_MODULE_POSITION_RELATIVE_TO_DRIVE_CENTER =
      new Translation2d(WHEELBASE_HALF_LENGTH_METER, WHEELBASE_HALF_WIDTH_METER);
  public static final Translation2d REAR_LEFT_MODULE_POSITION_RELATIVE_TO_DRIVE_CENTER =
      new Translation2d(-WHEELBASE_HALF_LENGTH_METER, WHEELBASE_HALF_WIDTH_METER);
  public static final Translation2d REAR_RIGHT_MODULE_POSITION_RELATIVE_TO_DRIVE_CENTER =
      new Translation2d(-WHEELBASE_HALF_LENGTH_METER, -WHEELBASE_HALF_WIDTH_METER);
  public static final Translation2d FRONT_RIGHT_MODULE_POSITION_RELATIVE_TO_DRIVE_CENTER =
      new Translation2d(WHEELBASE_HALF_LENGTH_METER, -WHEELBASE_HALF_WIDTH_METER);
  public static final List<Translation2d> POSITIONS_RELATIVE_TO_DRIVE_CENTER =
      Arrays.asList(
          FRONT_LEFT_MODULE_POSITION_RELATIVE_TO_DRIVE_CENTER,
          REAR_LEFT_MODULE_POSITION_RELATIVE_TO_DRIVE_CENTER,
          REAR_RIGHT_MODULE_POSITION_RELATIVE_TO_DRIVE_CENTER,
          FRONT_RIGHT_MODULE_POSITION_RELATIVE_TO_DRIVE_CENTER);

  public static final SwerveDriveKinematics WPILIB_SWERVE_KINEMATICS =
      new SwerveDriveKinematics(
          FRONT_LEFT_MODULE_POSITION_RELATIVE_TO_DRIVE_CENTER.toWpilibTranslation2d(),
          REAR_LEFT_MODULE_POSITION_RELATIVE_TO_DRIVE_CENTER.toWpilibTranslation2d(),
          REAR_RIGHT_MODULE_POSITION_RELATIVE_TO_DRIVE_CENTER.toWpilibTranslation2d(),
          FRONT_RIGHT_MODULE_POSITION_RELATIVE_TO_DRIVE_CENTER.toWpilibTranslation2d());

  // Calibration Offsets (calibration encoder values when the wheels are facing 0 degrees)
  public static final int FRONT_LEFT_CALIBRATION_OFFSET = 7031;
  public static final int REAR_LEFT_CALIBRATION_OFFSET = 21231;
  public static final int REAR_RIGHT_CALIBRATION_OFFSET = 19866;
  public static final int FRONT_RIGHT_CALIBRATION_OFFSET = 15019;

  // Speed Config
  public static final double MAX_SPEED_METERS_PER_SECOND = 4.572;
}
