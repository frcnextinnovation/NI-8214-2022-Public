package com.nextinnovation.team8214.subsystems.shooter;

import com.nextinnovation.lib.geometry.Rotation2d;
import com.nextinnovation.lib.geometry.Translation2d;
import com.nextinnovation.lib.utils.InterpolatingDouble;
import com.nextinnovation.lib.utils.InterpolatingTreeMap;

public class ShooterConfig {
  // Delay Count
  public static final int MAX_ALLOWABLE_ODOMETRY_AIM_COUNT = 10;

  // Shooting parameters
  public static final InterpolatingTreeMap<InterpolatingDouble, Translation2d> shootingParaTree =
      new InterpolatingTreeMap<>(30);

  static {
    shootingParaTree.put(
        new InterpolatingDouble(0.0), Translation2d.fromPolar(new Rotation2d(75.0), 1627.0));
    shootingParaTree.put(
        new InterpolatingDouble(1.0), Translation2d.fromPolar(new Rotation2d(75.0), 1627.0));
    shootingParaTree.put(
        new InterpolatingDouble(1.25), Translation2d.fromPolar(new Rotation2d(74.0), 1627.0));
    shootingParaTree.put(
        new InterpolatingDouble(1.50), Translation2d.fromPolar(new Rotation2d(73.0), 1726.0));
    shootingParaTree.put(
        new InterpolatingDouble(1.75), Translation2d.fromPolar(new Rotation2d(71.0), 2086.0));
    shootingParaTree.put(
        new InterpolatingDouble(2.0), Translation2d.fromPolar(new Rotation2d(65.0), 2096.0));
    shootingParaTree.put(
        new InterpolatingDouble(2.25), Translation2d.fromPolar(new Rotation2d(63.5), 2110.0));
    shootingParaTree.put(
        new InterpolatingDouble(2.5), Translation2d.fromPolar(new Rotation2d(62.0), 2200.0));
    shootingParaTree.put(
        new InterpolatingDouble(2.75), Translation2d.fromPolar(new Rotation2d(61.0), 2263.0));
    shootingParaTree.put(
        new InterpolatingDouble(3.0), Translation2d.fromPolar(new Rotation2d(59.5), 2346.0));
    shootingParaTree.put(
        new InterpolatingDouble(3.25), Translation2d.fromPolar(new Rotation2d(58.0), 2386.0));
    shootingParaTree.put(
        new InterpolatingDouble(3.5), Translation2d.fromPolar(new Rotation2d(56.5), 2410.0));
    shootingParaTree.put(
        new InterpolatingDouble(3.75), Translation2d.fromPolar(new Rotation2d(55.0), 2466.0));
    shootingParaTree.put(
        new InterpolatingDouble(4.0), Translation2d.fromPolar(new Rotation2d(55.0), 2503.0));
    shootingParaTree.put(
        new InterpolatingDouble(4.25), Translation2d.fromPolar(new Rotation2d(53.0), 2523.0));
    shootingParaTree.put(
        new InterpolatingDouble(4.5), Translation2d.fromPolar(new Rotation2d(51.0), 2593.0));
    shootingParaTree.put(
        new InterpolatingDouble(4.75), Translation2d.fromPolar(new Rotation2d(51.0), 2650.0));
    shootingParaTree.put(
        new InterpolatingDouble(5.0), Translation2d.fromPolar(new Rotation2d(51.0), 2710.0));
    shootingParaTree.put(
        new InterpolatingDouble(5.25), Translation2d.fromPolar(new Rotation2d(50.0), 2773.0));
    shootingParaTree.put(
        new InterpolatingDouble(5.5), Translation2d.fromPolar(new Rotation2d(49.0), 2866.0));
    shootingParaTree.put(
        new InterpolatingDouble(5.75), Translation2d.fromPolar(new Rotation2d(48.0), 2900.0));
    shootingParaTree.put(
        new InterpolatingDouble(6.0), Translation2d.fromPolar(new Rotation2d(46.5), 2996.0));
    shootingParaTree.put(
        new InterpolatingDouble(6.25), Translation2d.fromPolar(new Rotation2d(46.5), 3060.0));
    shootingParaTree.put(
        new InterpolatingDouble(6.50), Translation2d.fromPolar(new Rotation2d(46.5), 3093.0));
    shootingParaTree.put(
        new InterpolatingDouble(6.75), Translation2d.fromPolar(new Rotation2d(46.5), 3116.0));
    shootingParaTree.put(
        new InterpolatingDouble(7.0), Translation2d.fromPolar(new Rotation2d(46.5), 3116.0));
    shootingParaTree.put(
        new InterpolatingDouble(7.25), Translation2d.fromPolar(new Rotation2d(46.5), 3116.0));
    shootingParaTree.put(
        new InterpolatingDouble(7.5), Translation2d.fromPolar(new Rotation2d(46.5), 3163.0));
  }

  public static final InterpolatingTreeMap<InterpolatingDouble, Translation2d> rejectParaTree =
      new InterpolatingTreeMap<>(6);

  static {
    rejectParaTree.put(
        new InterpolatingDouble(0.0), Translation2d.fromPolar(new Rotation2d(37.0), 900.0));
    rejectParaTree.put(
        new InterpolatingDouble(2.0), Translation2d.fromPolar(new Rotation2d(37.0), 900.0));
    rejectParaTree.put(
        new InterpolatingDouble(4.5), Translation2d.fromPolar(new Rotation2d(37.0), 950.0));
    rejectParaTree.put(
        new InterpolatingDouble(6.0), Translation2d.fromPolar(new Rotation2d(37.0), 1050.0));
    rejectParaTree.put(
        new InterpolatingDouble(7.5), Translation2d.fromPolar(new Rotation2d(37.0), 1100.0));
    rejectParaTree.put(
        new InterpolatingDouble(20.0), Translation2d.fromPolar(new Rotation2d(37.0), 1100.0));
  }

  public static final class Flywheel {
    // Control Inversion
    public static final boolean IS_INVERT = false;

    // Speed Config
    public static final double MAX_SPEED = 17200.0;
    public static final double COAST_SPEED_OPEN_LOOP = 0.3;

    // Communication Parameters
    public static final int STATUS_FRAME_PERIOD = 20;

    // Voltage Config
    public static final double MAX_COMPENSATED_VOLTAGE = 11.5;

    // PID Parameters
    // Slot 0 is for velocity
    public static final double MASTER_PID0_KP = 0.00115818;
    public static final double MASTER_PID0_KI = 0.0;
    public static final double MASTER_PID0_KD = 0.0;
    public static final double MASTER_PID0_KF = 0.0;
    public static final double MASTER_FF_KS = 0.50788;
    public static final double MASTER_FF_KV = 0.0155;

    // PID Parameters
    // Slot 0 is for velocity
    public static final double SLAVE_PID0_KP = 0.00117545;
    public static final double SLAVE_PID0_KI = 0.0;
    public static final double SLAVE_PID0_KD = 0.0;
    public static final double SLAVE_PID0_KF = 0.0;
    public static final double SLAVE_FF_KS = 0.65372;
    public static final double SLAVE_FF_KV = 0.016715;

    // Tolerance Config
    public static final double MIN_ALLOWABLE_FLYWHEEL_VEL_RATIO = 0.96;

    // Mechanical Config
    public static final int ENCODER_RESOLUTION = 2048;
    public static final double ENCODER_TO_FLYWHEEL_RATIO =
        1.0; // the number of rotations the encoder undergoes for every rotation of the
    // flywheel
    public static final double ENCODER_UNITS_PER_FLYWHEEL_REVOLUTION =
        ENCODER_RESOLUTION * ENCODER_TO_FLYWHEEL_RATIO;

    public static final double ENCODER_UNITS_PER_DEGREE =
        ENCODER_UNITS_PER_FLYWHEEL_REVOLUTION / 360.0;

    public static final double ENCODER_UNITS_PER_RADIANS =
        ENCODER_UNITS_PER_FLYWHEEL_REVOLUTION / (2.0 * Math.PI);
  }
}
