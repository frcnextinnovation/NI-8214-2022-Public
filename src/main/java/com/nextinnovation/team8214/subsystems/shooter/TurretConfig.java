package com.nextinnovation.team8214.subsystems.shooter;

import com.nextinnovation.team8214.Config;

public class TurretConfig {
  public static final class Yaw {
    // Control Inversion
    public static final boolean IS_INVERT = false;

    // Speed Constants
    public static final double MAX_SPEED = 14400.0; // in encoder

    // Communication Parameters
    public static final int STATUS_FRAME_PERIOD = Config.LOOPER_CONTROL_DELTA_TIME_MS;

    public static final int ENCODER_RESOLUTION = 2048;
    // the number of rotations the encoder undergoes for every (yaw) rotation of the turret
    public static final double ENCODER_TO_TURRET_RATIO = 39.0;
    public static final double ENCODER_UNITS_PER_TURRET_REVOLUTION =
        ENCODER_RESOLUTION * ENCODER_TO_TURRET_RATIO;
    public static final double ENCODER_UNITS_PER_DEGREE =
        ENCODER_UNITS_PER_TURRET_REVOLUTION / 360.0;

    // Mechanical Constants
    public static final double FLOOR_ANGLE = 0.0;
    public static final double CEILING_ANGLE = 360.0;
    public static final double COMPENSATE_ANGLE = 0.0;
    public static final double DEAD_ANGLE = 18.0;
    public static final double AT_HOME_DEGREE = 180.0;
    public static final double AT_HOME_ENCODER_POSITION = AT_HOME_DEGREE * ENCODER_UNITS_PER_DEGREE;
    public static final int LEFT_ENCODER_POSITION_FORWARD_THRESHOLD =
        (int)
            (AT_HOME_ENCODER_POSITION
                + (ENCODER_UNITS_PER_TURRET_REVOLUTION * 0.5
                    + COMPENSATE_ANGLE * ENCODER_UNITS_PER_DEGREE));
    public static final int RIGHT_ENCODER_POSITION_REVERSE_THRESHOLD =
        (int)
            (AT_HOME_ENCODER_POSITION
                - (ENCODER_UNITS_PER_TURRET_REVOLUTION * 0.5
                    + COMPENSATE_ANGLE * ENCODER_UNITS_PER_DEGREE));

    // PID Parameters
    // Slot 0 is for position
    public static final double PID0_KP = 0.1;
    public static final double PID0_KI = 0.0;
    public static final double PID0_KD = 0.1;

    public static final double COMPENSATE_CHASSIS_ROTATION_KF = 1.0;
  }

  public static final class Pitch {
    // Control Inversion
    public static final boolean IS_INVERT = true;

    // Communication Parameters
    public static final int STATUS_FRAME_PERIOD = Config.LOOPER_CONTROL_DELTA_TIME_MS;

    // Mechanical Constants
    public static final int ENCODER_POSITION_LOW = 0;
    public static final int ENCODER_POSITION_HIGH = 13570;
    public static final double SHOOT_ANGLE_MIN = 35.0; // Shoot Angle
    public static final double SHOOT_ANGLE_MAX = 70.0;
    public static final double ENCODER_UNITS_PER_DEGREE =
        (ENCODER_POSITION_HIGH - ENCODER_POSITION_LOW) / (SHOOT_ANGLE_MAX - SHOOT_ANGLE_MIN);

    public static final int HIGH_ENCODER_POSITION_FORWARD_THRESHOLD =
        (int) (ENCODER_POSITION_HIGH - 3.0 * ENCODER_UNITS_PER_DEGREE);
    public static final int LOW_ENCODER_POSITION_REVERSE_THRESHOLD =
        (int) (ENCODER_POSITION_LOW + 3.0 * ENCODER_UNITS_PER_DEGREE);

    // PID Parameters
    // Slot 0 is for position
    public static final double PID0_KP = 0.16;
    public static final double PID0_KI = 0.0;
    public static final double PID0_KD = 0.05;
    public static final double PID0_KF = 0.0;
  }
}
