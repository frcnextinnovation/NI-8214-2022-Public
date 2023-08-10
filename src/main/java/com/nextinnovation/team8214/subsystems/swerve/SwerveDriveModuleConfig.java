package com.nextinnovation.team8214.subsystems.swerve;

import com.nextinnovation.team8214.Config;

public final class SwerveDriveModuleConfig {
  public static final class Translation {
    // Control Inversion
    public static final boolean IS_INVERT = false;

    // Speed Config
    public static final double MAX_SPEED = 18200.0;
    public static final double SPEED_RESERVE_FACTOR = 1 / 16.0;
    public static final double MAX_CRUISE_SPEED =
        MAX_SPEED * (1.0 - SPEED_RESERVE_FACTOR); // in encoder units / 0.1s

    // PID Parameters
    // Slot 0 is for teleop
    public static final double PID0_KP = 0.0625;
    public static final double PID0_KI = 0.0;
    public static final double PID0_KD = 0.00;
    public static final double PID0_KF = 1023.0 / MAX_SPEED;

    // Mechanical Config
    public static final double WHEEL_DIAMETER_METER = 0.0958;
    public static final int ENCODER_RESOLUTION = 2048;
    // the number of rotations the encoder undergoes for every rotation of the wheel
    public static final double ENCODER_TO_WHEEL_RATIO = 570.0 / 91.0; // WCP 8214
    // public static final double ENCODER_TO_WHEEL_RATIO = 38250.0 / 5670.0; // MK4i 8583
    public static final double ENCODER_UNITS_PER_WHEEL_RESOLUTION =
        ENCODER_RESOLUTION * ENCODER_TO_WHEEL_RATIO;
    public static final double ENCODER_UNITS_PER_METER =
        ENCODER_UNITS_PER_WHEEL_RESOLUTION / (Math.PI * WHEEL_DIAMETER_METER);

    // Communication Parameters
    public static final int STATUS_FRAME_PERIOD = Config.LOOPER_CONTROL_DELTA_TIME_MS;
  }

  public static final class Rotation {
    // Control Inversion
    public static final boolean IS_INVERT = true;
    public static final boolean IS_EXTERNAL_SENSOR_INVERT = true;

    // PID Parameters
    // Slot 0 is for teleop
    public static final double PID0_KP = 0.8;
    public static final double PID0_KI = 0.0;
    public static final double PID0_KD = 0.1;

    // Mechanical Config
    public static final int ENCODER_RESOLUTION = 2048;
    public static final double CALIBRATION_ENCODER_RESOLUTION =
        360.0; // 360 for CANCoder, 4096 for SRX
    // the number of rotations the encoder undergoes for every rotation of the base of the module
    public static final double ENCODER_TO_MODULE_BASE_RATIO = 12.0; // WCP 8214
    // public static final double ENCODER_TO_MODULE_BASE_RATIO = 3000.0 / 140.0; // MK4i 8583
    public static final double ENCODER_TO_EXTERNAL_ENCODER_RATIO =
        CALIBRATION_ENCODER_RESOLUTION
            / ENCODER_RESOLUTION
            / ENCODER_TO_MODULE_BASE_RATIO; // the number of encoder units the encoder undergoes
    // for every encoder unit the calibration encoder
    // undergoes
    public static final double ENCODER_UNITS_PER_MODULE_BASE_REVOLUTION =
        ENCODER_RESOLUTION * ENCODER_TO_MODULE_BASE_RATIO;
    public static final double ENCODER_UNITS_PER_DEGREE =
        ENCODER_UNITS_PER_MODULE_BASE_REVOLUTION / 360.0;

    public static final double CALIBRATION_DELAY = 1.8;

    // Communication Parameters
    public static final int STATUS_FRAME_PERIOD = Config.LOOPER_CONTROL_DELTA_TIME_MS;
  }
}
