package com.nextinnovation.team8214.subsystems.shooter;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.nextinnovation.lib.drivers.LazyTalonFX;
import com.nextinnovation.lib.drivers.TalonUtil;
import com.nextinnovation.lib.geometry.Rotation2d;
import com.nextinnovation.lib.subsystems.BaseSubsystem;
import com.nextinnovation.lib.utils.*;
import com.nextinnovation.team8214.Config;
import com.nextinnovation.team8214.Ports;
import com.nextinnovation.team8214.subsystems.swerve.Swerve;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;

public class Turret extends BaseSubsystem {
  /***********************************************************************************************
   * Periodic IO *
   ***********************************************************************************************/
  private static class PeriodicInput {
    public double yawEncoderPosition = 0.0;
    public double yawEncoderVelocity = 0.0;
    public double pitchEncoderPosition = 0.0;
  }

  @Override
  public void readPeriodicInputs() {
    periodicInput.yawEncoderPosition = yawMotor.getSelectedSensorPosition();
    periodicInput.yawEncoderVelocity = yawMotor.getSelectedSensorVelocity();
    periodicInput.pitchEncoderPosition = pitchMotor.getSelectedSensorPosition();
  }

  private static class PeriodicOutput {
    public ControlMode yawControlMode = ControlMode.PercentOutput;
    public double yawOutput = 0.0;
    public DemandType yawDemandType = DemandType.ArbitraryFeedForward;
    public double yawFeedForward = 0.0;

    public ControlMode pitchControlMode = ControlMode.PercentOutput;
    public double pitchOutput = 0.0;
  }

  @Override
  public void writePeriodicOutputs() {
    yawMotor.set(
        periodicOutput.yawControlMode,
        periodicOutput.yawOutput,
        periodicOutput.yawDemandType,
        periodicOutput.yawFeedForward);
    pitchMotor.set(periodicOutput.pitchControlMode, periodicOutput.pitchOutput);
  }

  /***********************************************************************************************
   * Singleton *
   ***********************************************************************************************/
  private static Turret instance = null;

  public static synchronized Turret getInstance() {
    if (instance == null) {
      instance = new Turret();
    }

    return instance;
  }

  /***********************************************************************************************
   * Init & Config *
   ***********************************************************************************************/
  private final LazyTalonFX yawMotor = new LazyTalonFX(Ports.Can.TURRET_YAW_MOTOR);

  private final LazyTalonFX pitchMotor = new LazyTalonFX(Ports.Can.TURRET_PITCH_MOTOR);

  private final PeriodicInput periodicInput = new PeriodicInput();
  private final PeriodicOutput periodicOutput = new PeriodicOutput();

  private Turret() {
    configSmartDashboard();
    configMotors();
  }

  private synchronized void configMotors() {
    // Yaw
    yawMotor.configStatusFramePeriod(
        TurretConfig.Yaw.STATUS_FRAME_PERIOD, true, false, Config.CAN_TIMEOUT_MS);
    yawMotor.setInverted(TurretConfig.Yaw.IS_INVERT);
    yawMotor.setNeutralMode(NeutralMode.Brake);
    yawMotor.enableVoltageCompensation(true);
    // Soft limit
    TalonUtil.checkError(
        yawMotor.configForwardSoftLimitEnable(true, Config.CAN_TIMEOUT_MS),
        "Yaw: Can't enable forward soft limit!");
    TalonUtil.checkError(
        yawMotor.configForwardSoftLimitThreshold(
            TurretConfig.Yaw.LEFT_ENCODER_POSITION_FORWARD_THRESHOLD, Config.CAN_TIMEOUT_MS),
        "Yaw: Can't set forward soft limit threshold!");
    TalonUtil.checkError(
        yawMotor.configReverseSoftLimitEnable(true, Config.CAN_TIMEOUT_MS),
        "Yaw: Can't enable forward soft limit!");
    TalonUtil.checkError(
        yawMotor.configReverseSoftLimitThreshold(
            TurretConfig.Yaw.RIGHT_ENCODER_POSITION_REVERSE_THRESHOLD, Config.CAN_TIMEOUT_MS),
        "Yaw: Can't set forward soft limit threshold!");
    // Voltage
    TalonUtil.checkError(
        yawMotor.configVoltageCompSaturation(11.0, Config.CAN_TIMEOUT_MS),
        "Yaw: Can't set voltage comp saturation!");
    TalonUtil.checkError(
        yawMotor.configSupplyCurrentLimit(
            new SupplyCurrentLimitConfiguration(true, 38.0, 38.0, 0.0), Config.CAN_TIMEOUT_MS),
        "Yaw: Can't set supply current limit!");
    // Closed loop
    TalonUtil.checkError(
        yawMotor.configClosedLoopPeakOutput(0, 1.0, Config.CAN_TIMEOUT_MS),
        "Yaw: Can't set closed loop peak output!");
    TalonUtil.checkError(
        yawMotor.configAllowableClosedloopError(0, 0, Config.CAN_TIMEOUT_MS),
        "Yaw: Can't set allowable closed loop error!");
    // PID 0 is for position
    TalonUtil.checkError(
        yawMotor.config_kP(0, TurretConfig.Yaw.PID0_KP, Config.CAN_TIMEOUT_MS),
        "Yaw: Can't set kp!");
    TalonUtil.checkError(
        yawMotor.config_kI(0, TurretConfig.Yaw.PID0_KI, Config.CAN_TIMEOUT_MS),
        "Yaw: Can't set ki!");
    TalonUtil.checkError(
        yawMotor.config_kD(0, TurretConfig.Yaw.PID0_KD, Config.CAN_TIMEOUT_MS),
        "Yaw: Can't set kd!");
    // At home
    TalonUtil.checkError(
        yawMotor.setSelectedSensorPosition(
            TurretConfig.Yaw.AT_HOME_ENCODER_POSITION, 0, Config.CAN_INSTANT_TIMEOUT_MS),
        "Yaw: Can't set at home encoder position!");
    Timer.delay(0.1);

    // Pitch
    pitchMotor.configStatusFramePeriod(
        TurretConfig.Pitch.STATUS_FRAME_PERIOD, true, false, Config.CAN_TIMEOUT_MS);
    pitchMotor.setInverted(TurretConfig.Pitch.IS_INVERT);
    pitchMotor.setNeutralMode(NeutralMode.Brake);
    pitchMotor.enableVoltageCompensation(true);
    // Soft limit
    TalonUtil.checkError(
        pitchMotor.configForwardSoftLimitEnable(true, Config.CAN_TIMEOUT_MS),
        "Pitch: Can't enable forward soft limit!");
    TalonUtil.checkError(
        pitchMotor.configForwardSoftLimitThreshold(
            TurretConfig.Pitch.HIGH_ENCODER_POSITION_FORWARD_THRESHOLD, Config.CAN_TIMEOUT_MS),
        "Pitch: Can't set forward soft limit threshold!");
    TalonUtil.checkError(
        pitchMotor.configReverseSoftLimitEnable(true, Config.CAN_TIMEOUT_MS),
        "Pitch: Can't enable forward soft limit!");
    TalonUtil.checkError(
        pitchMotor.configReverseSoftLimitThreshold(
            TurretConfig.Pitch.LOW_ENCODER_POSITION_REVERSE_THRESHOLD, Config.CAN_TIMEOUT_MS),
        "Pitch: Can't set forward soft limit threshold!");
    // Voltage
    TalonUtil.checkError(
        pitchMotor.configVoltageCompSaturation(9.0, Config.CAN_TIMEOUT_MS),
        "Pitch: Can't set voltage comp saturation!");
    TalonUtil.checkError(
        pitchMotor.configSupplyCurrentLimit(
            new SupplyCurrentLimitConfiguration(true, 38.0, 38.0, 0.0), Config.CAN_TIMEOUT_MS),
        "Pitch: Can't set supply current limit!");
    // Closed loop
    TalonUtil.checkError(
        pitchMotor.configClosedLoopPeakOutput(0, 7.0 / 9.0, Config.CAN_TIMEOUT_MS),
        "Pitch: Can't set closed loop peak output!");
    TalonUtil.checkError(
        pitchMotor.configAllowableClosedloopError(0, 0, Config.CAN_TIMEOUT_MS),
        "Pitch: Can't set allowable closed loop error!");
    // PID 0 is for position
    TalonUtil.checkError(
        pitchMotor.config_kP(0, TurretConfig.Pitch.PID0_KP, Config.CAN_TIMEOUT_MS),
        "Pitch: Can't set kp!");
    TalonUtil.checkError(
        pitchMotor.config_kI(0, TurretConfig.Pitch.PID0_KI, Config.CAN_TIMEOUT_MS),
        "Pitch: Can't set ki!");
    TalonUtil.checkError(
        pitchMotor.config_kD(0, TurretConfig.Pitch.PID0_KD, Config.CAN_TIMEOUT_MS),
        "Pitch: Can't set kd!");
    TalonUtil.checkError(
        pitchMotor.config_kF(0, TurretConfig.Pitch.PID0_KF, Config.CAN_TIMEOUT_MS),
        "Pitch: Can't set kf!");
    // At home
    TalonUtil.checkError(
        pitchMotor.setSelectedSensorPosition(0.0, 0, Config.CAN_INSTANT_TIMEOUT_MS),
        "Pitch: Can't set at home encoder position!");
    Timer.delay(0.1);
  }

  /************************************************************************************************
   * Getter & Setter *
   ************************************************************************************************/
  public synchronized void setYawOpenLoop(double output) {
    periodicOutput.yawControlMode = ControlMode.PercentOutput;
    periodicOutput.yawOutput = output;
  }

  public synchronized void setPitchOpenLoop(double output) {
    periodicOutput.pitchControlMode = ControlMode.PercentOutput;
    periodicOutput.pitchOutput = output;
  }

  public synchronized void setForceRobotCentricYawDegrees(double degrees) {
    periodicOutput.yawControlMode = ControlMode.Position;
    periodicOutput.yawOutput = yawDegreesToEncoderUnitsWithoutBound(degrees);
    periodicOutput.yawDemandType = DemandType.ArbitraryFeedForward;
    periodicOutput.yawFeedForward = 0.0;
  }

  public synchronized void setRobotCentricYawDegrees(Rotation2d degrees, double feed_forward) {
    var compensatedFloorAngle = TurretConfig.Yaw.FLOOR_ANGLE - TurretConfig.Yaw.COMPENSATE_ANGLE;
    var compensatedCeilAngle = TurretConfig.Yaw.CEILING_ANGLE + TurretConfig.Yaw.COMPENSATE_ANGLE;

    var modifiedAngle = Util.boundAngleTo0To360Degrees(degrees.getUnboundedDegrees());

    if (!Util.epsilonEquals(modifiedAngle, compensatedFloorAngle, TurretConfig.Yaw.DEAD_ANGLE)
        && !Util.epsilonEquals(modifiedAngle, compensatedCeilAngle, TurretConfig.Yaw.DEAD_ANGLE)) {
      periodicOutput.yawControlMode = ControlMode.Position;
      periodicOutput.yawOutput = yawDegreesToEncoderUnits(modifiedAngle);
      periodicOutput.yawDemandType = DemandType.ArbitraryFeedForward;
      periodicOutput.yawFeedForward = feed_forward;
    }
  }

  public synchronized void setFieldCentricYawDegrees(Rotation2d degrees) {
    double swerveAngularVelocity = Swerve.getInstance().getRawAngularVelocity();
    double feedForwardVelocity =
        -yawDegreesToEncoderUnitsWithoutBound(swerveAngularVelocity) / 10.0;

    double chassisRotationFeedforward =
        feedForwardVelocity
            / TurretConfig.Yaw.MAX_SPEED
            * TurretConfig.Yaw.COMPENSATE_CHASSIS_ROTATION_KF;

    setRobotCentricYawDegrees(
        degrees.rotateBy(Swerve.getInstance().getFieldCentricHeading().inverse()),
        chassisRotationFeedforward);
  }

  public synchronized void setPitchDegrees(Rotation2d degrees) {
    periodicOutput.pitchControlMode = ControlMode.Position;
    periodicOutput.pitchOutput =
        pitchShootDegreesToEncoderUnits(
            Util.limit(
                Util.boundAngleTo0To360Degrees(degrees.getUnboundedDegrees()),
                TurretConfig.Pitch.SHOOT_ANGLE_MIN,
                TurretConfig.Pitch.SHOOT_ANGLE_MAX));
  }

  public Rotation2d getRobotCentricYawHeading() {
    return Rotation2d.fromDegrees(yawEncoderUnitsToDegrees(periodicInput.yawEncoderPosition));
  }

  public Rotation2d getFieldCentricYawHeading() {
    return Rotation2d.fromDegrees(yawEncoderUnitsToDegrees(periodicInput.yawEncoderPosition))
        .rotateBy(Swerve.getInstance().getFieldCentricHeading());
  }

  public boolean isYawOnTargets(boolean is_strict) {
    return Util.epsilonEquals(
        periodicOutput.yawOutput,
        periodicInput.yawEncoderPosition,
        is_strict ? yawDegreesToEncoderUnits(3.0) : yawDegreesToEncoderUnits(8.0));
  }

  public boolean isPitchOnTargets() {
    return Util.epsilonEquals(
        periodicOutput.pitchOutput,
        periodicInput.pitchEncoderPosition,
        pitchDegreesToEncoderUnits(2.0));
  }

  public boolean isTurretOnTargets(boolean is_strict) {
    return isYawOnTargets(is_strict) && isPitchOnTargets();
  }
  /************************************************************************************************
   * Util *
   ************************************************************************************************/
  public double yawEncoderUnitsToDegrees(double encoder_units) {
    return Util.boundAngleTo0To360Degrees(
        encoder_units / TurretConfig.Yaw.ENCODER_UNITS_PER_DEGREE);
  }

  public double yawDegreesToEncoderUnits(double degrees) {
    return Util.boundAngleToScopeWithCompensate(
            degrees,
            TurretConfig.Yaw.FLOOR_ANGLE,
            TurretConfig.Yaw.CEILING_ANGLE,
            TurretConfig.Yaw.COMPENSATE_ANGLE)
        * TurretConfig.Yaw.ENCODER_UNITS_PER_DEGREE;
  }

  public double yawDegreesToEncoderUnitsWithoutBound(double degrees) {
    return degrees * TurretConfig.Yaw.ENCODER_UNITS_PER_DEGREE;
  }

  public double yawDegreesPerSecondToEncoderUnitsPer100ms(double degrees_per_second) {
    return yawDegreesToEncoderUnits(degrees_per_second) / 10.0;
  }

  public double pitchDegreesToEncoderUnits(double degrees) {
    return degrees * TurretConfig.Pitch.ENCODER_UNITS_PER_DEGREE;
  }

  public double pitchEncoderUnitsToDegrees(double encoder_units) {
    return encoder_units / TurretConfig.Pitch.ENCODER_UNITS_PER_DEGREE;
  }

  public double pitchShootDegreesToEncoderUnits(double degrees) {
    return (TurretConfig.Pitch.SHOOT_ANGLE_MAX - degrees)
        * TurretConfig.Pitch.ENCODER_UNITS_PER_DEGREE;
  }

  /************************************************************************************************
   * Stop & Disable Actions *
   ************************************************************************************************/
  @Override
  public void disable() {
    setYawOpenLoop(0.0);
    setPitchOpenLoop(0.0);
  }

  /************************************************************************************************
   * Log & self-test *
   ************************************************************************************************/
  private NetworkTableEntry pitchErrorEntry;

  private NetworkTableEntry yawErrorEntry;

  public void configSmartDashboard() {
    var tab = Shuffleboard.getTab("Shooter");

    pitchErrorEntry = tab.add("Pitch Error", 999.99).getEntry();
    yawErrorEntry = tab.add("Yaw Error", 999.99).getEntry();
  }

  @Override
  public void logToSmartDashboard() {
    if (Config.ENABLE_DEBUG_OUTPUT) {
      pitchErrorEntry.setNumber(
          pitchEncoderUnitsToDegrees(
              periodicOutput.pitchOutput - periodicInput.pitchEncoderPosition));
      yawErrorEntry.setNumber(
          yawEncoderUnitsToDegrees(periodicOutput.yawOutput - periodicInput.yawEncoderPosition));
    }
  }
}
