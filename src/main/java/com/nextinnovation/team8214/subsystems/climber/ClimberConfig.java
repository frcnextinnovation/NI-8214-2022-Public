package com.nextinnovation.team8214.subsystems.climber;

public class ClimberConfig {
  // Control Inversion
  public static final boolean IS_INVERT = true;

  // Position Constants
  public static final double EXTEND_HEIGHT_METER = 0.65;
  public static final double CONTRACT_HEIGHT_METER = 0.04;
  public static final double CONTRACT_HEIGHT_LOWER_METER = 0.04;
  public static final double CONTRACT_WAIT_HEIGHT_METER = 0.3;
  public static final double LAND_HEIGHT_METER = 0.32;

  // Mechanical Config
  public static final int ENCODER_RESOLUTION = 2048;
  public static final double ENCODER_TO_MOTOR_BASE_RATIO = 7.0;
  public static final double ENCODER_UNITS_PER_MODULE_BASE_REVOLUTION =
      ENCODER_RESOLUTION * ENCODER_TO_MOTOR_BASE_RATIO;
  public static final double CLIMBER_SHAFT_DIAMETER_METER = 0.0127;
  public static final double CLIMBER_SHAFT_GIRTH = Math.PI * CLIMBER_SHAFT_DIAMETER_METER;
  public static final double ENCODER_UNITS_PER_METER =
      ENCODER_UNITS_PER_MODULE_BASE_REVOLUTION / CLIMBER_SHAFT_GIRTH;

  // Vel Config
  public static final double NORMALIZED_DOWN_VEL = -0.75;
  public static final double NORMALIZED_UP_VEL = 0.95;

  // PID Parameters
  // Slot 0 is for motion magic
  public static final double PID0_KP = 1.0;
  public static final double PID0_KI = 0.0;
  public static final double PID0_KD = 1.0;
  public static final double PID_TOLERANCE = 0.015;

  // Communication Parameters
  public static final int STATUS_FRAME_PERIOD = 20;
}
