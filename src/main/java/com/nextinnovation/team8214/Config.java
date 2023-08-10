package com.nextinnovation.team8214;

import com.nextinnovation.lib.geometry.Pose2d;
import com.nextinnovation.lib.geometry.Rotation2d;
import com.nextinnovation.lib.geometry.Translation2d;
import com.nextinnovation.lib.utils.Util;

/** Final class to store all constants except ID. */
public final class Config {
  // Mechanical offset
  public static final Translation2d ROBOT_TO_TURRET_TRANSLATION = new Translation2d(0.0, 0.0);
  public static final Pose2d TURRET_TO_CAMERA = new Pose2d(0.2347628, 0.0, new Rotation2d(0.0));

  // AHRS
  public static final double INIT_HEADING = 180.0;

  // Lopper
  public static final double LOOPER_CONTROL_PERIOD_SEC = 0.01;
  public static final int LOOPER_CONTROL_DELTA_TIME_MS =
      Util.roundToInt(LOOPER_CONTROL_PERIOD_SEC * 1000.0);

  // CAN
  public static final int CAN_TIMEOUT_MS = 110;
  public static final int CAN_INSTANT_TIMEOUT_MS = 30;

  // Log
  public static final boolean ENABLE_DEBUG_OUTPUT = false;
}
