package com.nextinnovation.team8214.subsystems.swerve;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.SensorVelocityMeasPeriod;
import com.nextinnovation.lib.drivers.CanDeviceId;
import com.nextinnovation.lib.drivers.LazyTalonFX;
import com.nextinnovation.lib.drivers.TalonUtil;
import com.nextinnovation.lib.geometry.Rotation2d;
import com.nextinnovation.lib.subsystems.BaseSubsystem;
import com.nextinnovation.lib.utils.Util;
import com.nextinnovation.team8214.Config;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SwerveDriveModule extends BaseSubsystem {
  /************************************************************************************************
   * Periodic IO *
   ************************************************************************************************/
  private static class PeriodicInput {
    public int rotationMotorEncoderPosition = 0;
    public int rotationMotorEncoderVelocity = 0;

    public int translationMotorEncoderVel = 0;
  }

  @Override
  public void readPeriodicInputs() {
    periodicInput.rotationMotorEncoderPosition =
        Util.roundToInt(rotationMotor.getSelectedSensorPosition());
    periodicInput.rotationMotorEncoderVelocity =
        Util.roundToInt(rotationMotor.getSelectedSensorVelocity());

    periodicInput.translationMotorEncoderVel =
        Util.roundToInt(translationMotor.getSelectedSensorVelocity());
  }

  private static class PeriodicOutput {
    public ControlMode rotationMotorControlMode = ControlMode.PercentOutput;
    public ControlMode translationMotorControlMode = ControlMode.PercentOutput;
    public double rotationMotorSetpoint = 0.0;
    public double translationMotorSetpoint = 0.0;
  }

  @Override
  public void writePeriodicOutputs() {
    translationMotor.set(
        periodicOutput.translationMotorControlMode, periodicOutput.translationMotorSetpoint);
    rotationMotor.set(
        periodicOutput.rotationMotorControlMode, periodicOutput.rotationMotorSetpoint);
  }

  /************************************************************************************************
   * Init & Config *
   ************************************************************************************************/
  private final LazyTalonFX translationMotor, rotationMotor;

  private final String moduleName;
  private final int rotationCalibrationOffset;
  private final PeriodicInput periodicInput = new PeriodicInput();
  private final PeriodicOutput periodicOutput = new PeriodicOutput();
  private final CANCoder externalRotationEncoder;
  /**
   * Constructor
   *
   * @param module_id ID of module:[1, 4], usually FL -> 1, RL -> 2, RR -> 3, FR -> 4
   * @param translation_motor_id CAN ID of translation motor
   * @param rotation_motor_id CAN ID of translation motor
   * @param external_rotation_encoder CAN ID of CANCoder
   * @param rotation_calibration_offset Calibration offset by CANCoder
   */
  public SwerveDriveModule(
      int module_id,
      CanDeviceId translation_motor_id,
      CanDeviceId rotation_motor_id,
      CANCoder external_rotation_encoder,
      int rotation_calibration_offset) {
    moduleName = "Swerve Module " + module_id + " ";
    translationMotor = new LazyTalonFX(translation_motor_id);
    rotationMotor = new LazyTalonFX(rotation_motor_id);
    externalRotationEncoder = external_rotation_encoder;
    rotationCalibrationOffset = rotation_calibration_offset;
    configTranslationMotor();
    configRotationMotor();
    calibrateRotationEncoder();
    readPeriodicInputs();
    resetSensors();
  }

  private synchronized void configTranslationMotor() {
    translationMotor.configStatusFramePeriod(
        SwerveDriveModuleConfig.Translation.STATUS_FRAME_PERIOD,
        true,
        false,
        Config.CAN_TIMEOUT_MS);

    TalonUtil.checkError(
        translationMotor.configNeutralDeadband(0.015, Config.CAN_TIMEOUT_MS),
        moduleName + " Translation: Can't set neutral deadband!");
    TalonUtil.checkError(
        translationMotor.configSelectedFeedbackSensor(
            FeedbackDevice.IntegratedSensor, 0, Config.CAN_TIMEOUT_MS),
        moduleName + " Translation: Can't set feedback sensor!");
    translationMotor.setInverted(SwerveDriveModuleConfig.Translation.IS_INVERT);
    translationMotor.setNeutralMode(NeutralMode.Brake);
    TalonUtil.checkError(
        translationMotor.configVelocityMeasurementPeriod(
            SensorVelocityMeasPeriod.Period_1Ms, Config.CAN_TIMEOUT_MS),
        moduleName + " Translation: Can't set velocity measurement period!");
    TalonUtil.checkError(
        translationMotor.configVelocityMeasurementWindow(1, Config.CAN_TIMEOUT_MS),
        moduleName + " Translation: Can't set velocity measurement window!");
    TalonUtil.checkError(
        translationMotor.configVoltageCompSaturation(10.0, Config.CAN_TIMEOUT_MS),
        moduleName + " Translation: Can't set voltage comp saturation!");
    TalonUtil.checkError(
        translationMotor.configVoltageMeasurementFilter(1, Config.CAN_TIMEOUT_MS),
        moduleName + " Translation: Can't set voltage measurement filter!");
    translationMotor.enableVoltageCompensation(true);
    TalonUtil.checkError(
        translationMotor.configSupplyCurrentLimit(
            new SupplyCurrentLimitConfiguration(true, 38.0, 38.0, 0.0), Config.CAN_TIMEOUT_MS),
        moduleName + " Translation: Can't set supply current limit!");
    TalonUtil.checkError(
        translationMotor.configAllowableClosedloopError(0, 0.0, Config.CAN_TIMEOUT_MS),
        moduleName + " Translation: Can't set allowable closed loop error!");
    // PID 0 is for velocity
    TalonUtil.checkError(
        translationMotor.config_kP(
            0, SwerveDriveModuleConfig.Translation.PID0_KP, Config.CAN_TIMEOUT_MS),
        moduleName + " Translation: Can't set kp!");
    TalonUtil.checkError(
        translationMotor.config_kI(
            0, SwerveDriveModuleConfig.Translation.PID0_KI, Config.CAN_TIMEOUT_MS),
        moduleName + " Translation: Can't set ki!");
    TalonUtil.checkError(
        translationMotor.config_kD(
            0, SwerveDriveModuleConfig.Translation.PID0_KD, Config.CAN_TIMEOUT_MS),
        moduleName + " Translation: Can't set kd!");
    TalonUtil.checkError(
        translationMotor.config_kF(
            0, SwerveDriveModuleConfig.Translation.PID0_KF, Config.CAN_TIMEOUT_MS),
        moduleName + " Translation: Can't set kf!");
    TalonUtil.checkError(
        translationMotor.setSelectedSensorPosition(0, 0, Config.CAN_INSTANT_TIMEOUT_MS),
        moduleName + " Translation: Can't set selected sensor position!");

    Timer.delay(0.05);
  }

  private synchronized void configRotationMotor() {
    rotationMotor.configStatusFramePeriod(
        SwerveDriveModuleConfig.Rotation.STATUS_FRAME_PERIOD, true, false, Config.CAN_TIMEOUT_MS);
    TalonUtil.checkError(
        rotationMotor.configSelectedFeedbackSensor(
            FeedbackDevice.IntegratedSensor, 0, Config.CAN_TIMEOUT_MS),
        moduleName + " Rotation: Can't set feedback sensor!");
    rotationMotor.setInverted(SwerveDriveModuleConfig.Rotation.IS_INVERT);
    rotationMotor.setNeutralMode(NeutralMode.Brake);
    TalonUtil.checkError(
        rotationMotor.configVelocityMeasurementPeriod(
            SensorVelocityMeasPeriod.Period_1Ms, Config.CAN_TIMEOUT_MS),
        moduleName + " Rotation: Can't set velocity measurement period!");
    TalonUtil.checkError(
        rotationMotor.configVelocityMeasurementWindow(1, Config.CAN_TIMEOUT_MS),
        moduleName + " Rotation: Can't set velocity measurement window!");
    TalonUtil.checkError(
        rotationMotor.configVoltageCompSaturation(8.0, Config.CAN_TIMEOUT_MS),
        moduleName + " Rotation: Can't set voltage comp saturation!");
    TalonUtil.checkError(
        rotationMotor.configVoltageMeasurementFilter(1, Config.CAN_TIMEOUT_MS),
        moduleName + " Rotation: Can't set voltage measurement filter!");
    rotationMotor.enableVoltageCompensation(true);
    TalonUtil.checkError(
        rotationMotor.configSupplyCurrentLimit(
            new SupplyCurrentLimitConfiguration(true, 38.0, 38.0, 0.0), Config.CAN_TIMEOUT_MS),
        moduleName + " Rotation: Can't set supply current limit!");
    TalonUtil.checkError(
        rotationMotor.configAllowableClosedloopError(0, 0, Config.CAN_TIMEOUT_MS),
        moduleName + " Rotation: Can't set allowable closed loop error!");
    // PID 0 is for position
    TalonUtil.checkError(
        rotationMotor.config_kP(0, SwerveDriveModuleConfig.Rotation.PID0_KP, Config.CAN_TIMEOUT_MS),
        moduleName + " Rotation: Can't set kp!");
    TalonUtil.checkError(
        rotationMotor.config_kI(0, SwerveDriveModuleConfig.Rotation.PID0_KI, Config.CAN_TIMEOUT_MS),
        moduleName + " Rotation: Can't set ki!");
    TalonUtil.checkError(
        rotationMotor.config_kD(0, SwerveDriveModuleConfig.Rotation.PID0_KD, Config.CAN_TIMEOUT_MS),
        moduleName + " Rotation: Can't set kd!");

    Timer.delay(0.05);
  }

  private synchronized void calibrateRotationEncoder() {
    Timer.delay(SwerveDriveModuleConfig.Rotation.CALIBRATION_DELAY);
    rotationMotor.setSelectedSensorPosition(getRotationEncoderCalibrationTarget());
  }

  /************************************************************************************************
   * Reset *
   ************************************************************************************************/
  @Override
  public synchronized void resetSensors() {
    translationMotor.setSelectedSensorPosition(0, 0, Config.CAN_TIMEOUT_MS);
  }

  /************************************************************************************************
   * Function Enabler *
   ************************************************************************************************/
  public synchronized void enableTranslationInverted(boolean invert) {
    translationMotor.setInverted(SwerveDriveModuleConfig.Translation.IS_INVERT ^ invert);
  }

  public synchronized void enableRotationMotorInverted(boolean invert) {
    rotationMotor.setInverted(SwerveDriveModuleConfig.Rotation.IS_INVERT ^ invert);
  }

  /************************************************************************************************
   * Getter & Setter *
   ************************************************************************************************/
  private void setTranslationVelocityTarget(double velocity_encoder_units_per_second) {
    periodicOutput.translationMotorControlMode = ControlMode.Velocity;
    periodicOutput.translationMotorSetpoint = velocity_encoder_units_per_second;
  }

  private double getTranslationVelocityMeter() {
    return translationEncoderUnitsToMeters(periodicInput.translationMotorEncoderVel) * 10.0;
  }

  public synchronized void setNormalizedTranslationVelocityTarget(double normalized_velocity) {
    setTranslationVelocityTarget(
        SwerveDriveModuleConfig.Translation.MAX_CRUISE_SPEED * normalized_velocity);
  }

  public synchronized void setTranslationOpenLoop(double normalized_output) {
    periodicOutput.translationMotorControlMode = ControlMode.PercentOutput;
    periodicOutput.translationMotorSetpoint = normalized_output;
  }

  private double getRawRotationHeading() {
    return rotationEncoderUnitsToDegrees(periodicInput.rotationMotorEncoderPosition);
  }

  public Rotation2d getRobotCentricRotationHeading() {
    return Rotation2d.fromDegrees(
        getRawRotationHeading() - rotationEncoderUnitsToDegrees(rotationCalibrationOffset));
  }

  private void setRotationHeadingTarget(double heading_degrees) {
    periodicOutput.rotationMotorControlMode = ControlMode.Position;
    periodicOutput.rotationMotorSetpoint =
        degreesToRotationEncoderUnits(
            Util.boundAngleToClosestScope(
                heading_degrees + rotationEncoderUnitsToDegrees(rotationCalibrationOffset),
                getRawRotationHeading()));
  }

  public synchronized void setRotationHeadingTarget(Rotation2d heading) {
    setRotationHeadingTarget(heading.getUnboundedDegrees());
  }

  public synchronized void setRotationOpenLoop(double normalized_output) {
    periodicOutput.rotationMotorControlMode = ControlMode.PercentOutput;
    periodicOutput.rotationMotorSetpoint = normalized_output;
  }

  public int getExternalRotationEncoderPosition() {
    return Util.conditionalInvert(
        (int) externalRotationEncoder.getPosition(),
        SwerveDriveModuleConfig.Rotation.IS_EXTERNAL_SENSOR_INVERT);
  }

  public int getRotationEncoderCalibrationTarget() {
    return Util.roundToInt(
        Util.boundToNonnegativeScope(
            Util.conditionalInvert(
                getExternalRotationEncoderPosition()
                    / SwerveDriveModuleConfig.Rotation.ENCODER_TO_EXTERNAL_ENCODER_RATIO,
                rotationMotor.getInverted()),
            SwerveDriveModuleConfig.Rotation.ENCODER_UNITS_PER_MODULE_BASE_REVOLUTION));
  }

  // This should return velocity
  public SwerveModuleState getWpilibModuleState() {
    return new SwerveModuleState(
        getTranslationVelocityMeter(), getRobotCentricRotationHeading().toWpilibRotation2d());
  }

  /************************************************************************************************
   * Util *
   ************************************************************************************************/
  private double translationEncoderUnitsToMeters(double encoder_units) {
    return encoder_units / SwerveDriveModuleConfig.Translation.ENCODER_UNITS_PER_METER;
  }

  private int degreesToRotationEncoderUnits(double degrees) {
    return Util.roundToInt(degrees * SwerveDriveModuleConfig.Rotation.ENCODER_UNITS_PER_DEGREE);
  }

  private double rotationEncoderUnitsToDegrees(double encoder_units) {
    return encoder_units / SwerveDriveModuleConfig.Rotation.ENCODER_UNITS_PER_DEGREE;
  }

  /************************************************************************************************
   * Stop & Disable Actions *
   ************************************************************************************************/
  @Override
  public synchronized void disable() {
    setTranslationOpenLoop(0.0);
    setRotationOpenLoop(0.0);
  }

  /************************************************************************************************
   * Log & self-test *
   ************************************************************************************************/
  @Override
  public void logToSmartDashboard() {
    SmartDashboard.putNumber(moduleName + " cali", getRotationEncoderCalibrationTarget());
    //    System.out.println(moduleName + " cali" + getRotationEncoderCalibrationTarget());
  }
}
