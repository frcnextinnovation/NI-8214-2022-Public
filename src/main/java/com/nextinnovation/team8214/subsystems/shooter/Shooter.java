package com.nextinnovation.team8214.subsystems.shooter;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.sensors.SensorVelocityMeasPeriod;
import com.nextinnovation.lib.drivers.LazyTalonFX;
import com.nextinnovation.lib.drivers.TalonUtil;
import com.nextinnovation.lib.geometry.Pose2d;
import com.nextinnovation.lib.geometry.Rotation2d;
import com.nextinnovation.lib.geometry.Translation2d;
import com.nextinnovation.lib.loops.ILoop;
import com.nextinnovation.lib.loops.ILooper;
import com.nextinnovation.lib.subsystems.BaseSubsystem;
import com.nextinnovation.lib.utils.InterpolatingDouble;
import com.nextinnovation.lib.utils.Util;
import com.nextinnovation.team8214.Config;
import com.nextinnovation.team8214.Field;
import com.nextinnovation.team8214.Odometry;
import com.nextinnovation.team8214.Ports;
import com.nextinnovation.team8214.managers.ControlSignalManager;
import com.nextinnovation.team8214.managers.CoordinateFrameManager;
import com.nextinnovation.team8214.subsystems.tunnel.Tunnel;
import com.nextinnovation.team8214.subsystems.vision.Vision;
import com.nextinnovation.team8214.subsystems.vision.VisionState;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Shooter extends BaseSubsystem {
  /***********************************************************************************************
   * Control Loop *
   ***********************************************************************************************/
  @Override
  public void registerEnabledLoops(ILooper enabledLooper) {
    ILoop loop =
        new ILoop() {
          @Override
          public void onStart(double timestamp) {
            setState(ShooterState.MANUAL);
          }

          @Override
          public void onLoop(double timestamp) {
            var vision = Vision.getInstance();
            var controlSignalManager = ControlSignalManager.getInstance();
            var tunnel = Tunnel.getInstance();
            var odometry = Odometry.getInstance();

            var isFlywheelUnlock = controlSignalManager.isFlyWheelUnlock;

            switch (shooterState) {
              case AUTO:
              case MANUAL:
                if (controlSignalManager.isInClimbMode) {
                  vision.setState(VisionState.OFF);
                  turret.setForceRobotCentricYawDegrees(0.0);
                  turret.setPitchOpenLoop(0.0);
                  setFlywheelOpenLoop(0.0);

                  odometryAimTimeoutCount = 0;
                } else if (tunnel.isShooterNeedReject()) {
                  var fieldToRobot = odometry.getLatestFieldCentricRobotPose();
                  var targetRejectPosition =
                      odometry.isInSelfZone()
                          ? Field.Waypoints.FRIEND_HANGER_POSITION
                          : Field.Waypoints.ENEMY_HANGER_POSITION;
                  var fieldCentricTargetTranslation =
                      new Translation2d(fieldToRobot.getTranslation(), targetRejectPosition);

                  var rejectPara =
                      ShooterConfig.rejectParaTree.getInterpolated(
                          new InterpolatingDouble(fieldCentricTargetTranslation.norm()));

                  turret.setFieldCentricYawDegrees(fieldCentricTargetTranslation.direction());
                  turret.setPitchDegrees(rejectPara.direction());
                  setFlywheelRpm(rejectPara.norm());

                  odometryAimTimeoutCount = 0;

                } else {
                  if (vision.isEnabled() && vision.hasTarget()) {
                    odometryAimTimeoutCount = 0;

                    var visionTargetInfo = vision.getVisionTargetInfo();
                    var yawFieldCentricHeading = turret.getFieldCentricYawHeading();
                    var yawRobotCentricHeading = turret.getRobotCentricYawHeading();

                    var realTargetDistance =
                        visionTargetInfo.targetDistance + Field.UPPER_HUB_RADIANS;

                    var realTargetVector =
                        Translation2d.fromPolar(visionTargetInfo.targetHeading, realTargetDistance);

                    var targetHeading =
                        yawFieldCentricHeading.rotateBy(realTargetVector.direction());
                    turret.setFieldCentricYawDegrees(targetHeading);
                    lastFieldCentricTargetHeading = targetHeading;

                    var shootingParameters =
                        ShooterConfig.shootingParaTree.getInterpolated(
                            new InterpolatingDouble(realTargetVector.norm()));

                    if (!turret.isTurretOnTargets(true) || !controlSignalManager.isRobotShoot) {
                      turret.setPitchDegrees(shootingParameters.direction());
                    }

                    if (isFlywheelUnlock) {
                      if (!turret.isTurretOnTargets(true)
                          || !isFlyWheelVelocityOnTarget()
                          || !controlSignalManager.isRobotShoot) {
                        setFlywheelRpm(shootingParameters.norm());
                      }
                    }

                    // Update VO
                    if (odometry.getIsVoEnable()
                        && vision.isUpdated()
                        && visionTargetInfo.isGoodTarget) {
                      var hubToCameraTranslation =
                          Translation2d.fromPolar(
                              visionTargetInfo
                                  .targetHeading
                                  .rotateBy(yawFieldCentricHeading)
                                  .rotateBy(Rotation2d.fromDegrees(180.0)),
                              visionTargetInfo.targetDistance + Field.UPPER_HUB_RADIANS);
                      var fieldToCameraTranslation =
                          Field.Waypoints.HUB_POSITION.translateBy(hubToCameraTranslation);
                      var fieldToCameraPose =
                          new Pose2d(fieldToCameraTranslation, yawFieldCentricHeading);
                      var fieldToRobotPose =
                          CoordinateFrameManager.getCameraToRobot(
                              fieldToCameraPose, yawRobotCentricHeading);

                      odometry.updateVo(fieldToRobotPose, visionTargetInfo.timestamp);
                    }
                  } else {
                    if (odometryAimTimeoutCount < 10) {
                      odometryAimTimeoutCount++;
                    } else if (isOdometryAimEnable) {
                      // Aim by odometer
                      var fieldToRobot = odometry.getLatestFieldCentricRobotPose();
                      var fieldToTurret =
                          CoordinateFrameManager.getRobotToTurret(
                              fieldToRobot, turret.getRobotCentricYawHeading());

                      var fieldCentricTargetTranslation =
                          new Translation2d(
                              fieldToTurret.getTranslation(), Field.Waypoints.HUB_POSITION);

                      var fieldCentricYawHeading = fieldCentricTargetTranslation.direction();
                      var shootingParameters =
                          ShooterConfig.shootingParaTree.getInterpolated(
                              new InterpolatingDouble(fieldCentricTargetTranslation.norm()));

                      // Set turret & flywheel para
                      turret.setFieldCentricYawDegrees(fieldCentricYawHeading);
                      lastFieldCentricTargetHeading = fieldCentricYawHeading;
                      turret.setPitchDegrees(shootingParameters.direction());
                      setFlywheelRpm(shootingParameters.norm());
                    } else {
                      turret.setFieldCentricYawDegrees(lastFieldCentricTargetHeading);
                    }
                  }

                  if (controlSignalManager.isTurretSetToSolidAngle) {
                    turret.setFieldCentricYawDegrees(
                        lastFieldCentricTargetHeading.rotateBy(new Rotation2d(180.0)));
                  }

                  if (!isFlywheelUnlock) {
                    setFlywheelCoast();
                  }
                }
                break;

              case DISABLE:
                disable();
                break;
            }
          }

          @Override
          public void onStop(double timestamp) {
            disable();
          }
        };
    enabledLooper.register(loop);
  }

  /***********************************************************************************************
   * Periodic IO *
   ***********************************************************************************************/
  private static class PeriodicInput {
    public double flywheelMasterEncoderVelocity = 0.0;
    public double flywheelSlaveEncoderVelocity = 0.0;
  }

  @Override
  public void readPeriodicInputs() {
    turret.readPeriodicInputs();
    periodicInput.flywheelMasterEncoderVelocity = flywheelMotorLeft.getSelectedSensorVelocity();
    periodicInput.flywheelSlaveEncoderVelocity = flywheelMotorRight.getSelectedSensorVelocity();
  }

  private static class PeriodicOutput {
    public ControlMode flywheelMotorControlMode = ControlMode.PercentOutput;
    public double flywheelMotorOutput = 0.0;
    public double flywheelMasterFFOutput = 0.0;
    public double flywheelSlaveFFOutput = 0.0;
  }

  @Override
  public void writePeriodicOutputs() {
    turret.writePeriodicOutputs();
    flywheelMotorLeft.set(
        periodicOutput.flywheelMotorControlMode,
        periodicOutput.flywheelMotorOutput,
        DemandType.ArbitraryFeedForward,
        periodicOutput.flywheelMasterFFOutput);
    flywheelMotorRight.set(
        periodicOutput.flywheelMotorControlMode,
        periodicOutput.flywheelMotorOutput,
        DemandType.ArbitraryFeedForward,
        periodicOutput.flywheelSlaveFFOutput);
  }

  /***********************************************************************************************
   * Subsystem States *
   ***********************************************************************************************/
  private ShooterState shooterState;

  public synchronized void setState(ShooterState new_state) {
    shooterState = new_state;
    switch (shooterState) {
      case AUTO:
      case MANUAL:
        enableOdometryAim();
        break;

      case DISABLE:
        turret.disable();
        setFlywheelOpenLoop(0.0);
        enableOdometryAim();
        break;
    }
  }

  /***********************************************************************************************
   * Singleton *
   ***********************************************************************************************/
  private static Shooter instance = null;

  public static synchronized Shooter getInstance() {
    if (instance == null) {
      instance = new Shooter();
    }
    return instance;
  }

  /***********************************************************************************************
   * Init & Config *
   ***********************************************************************************************/
  private final Turret turret = Turret.getInstance();

  private final LazyTalonFX flywheelMotorLeft = new LazyTalonFX(Ports.Can.FLY_WHEEL_MOTOR_LEFT);
  private final SimpleMotorFeedforward flywheelMotorLeftFFModel =
      new SimpleMotorFeedforward(
          ShooterConfig.Flywheel.MASTER_FF_KS, ShooterConfig.Flywheel.MASTER_FF_KV);
  private final LazyTalonFX flywheelMotorRight = new LazyTalonFX(Ports.Can.FLY_WHEEL_MOTOR_RIGHT);
  private final SimpleMotorFeedforward flywheelMotorRightFFModel =
      new SimpleMotorFeedforward(
          ShooterConfig.Flywheel.SLAVE_FF_KS, ShooterConfig.Flywheel.SLAVE_FF_KV);

  private final PeriodicInput periodicInput = new PeriodicInput();
  private final PeriodicOutput periodicOutput = new PeriodicOutput();

  private Rotation2d lastFieldCentricTargetHeading = new Rotation2d();
  private boolean isOdometryAimEnable = true;
  private int odometryAimTimeoutCount = 0;

  private Shooter() {
    configSmartDashboard();
    configMotors();
  }

  private synchronized void configMotors() {
    // Flywheel
    flywheelMotorLeft.configStatusFramePeriod(
        ShooterConfig.Flywheel.STATUS_FRAME_PERIOD, true, false, Config.CAN_TIMEOUT_MS);
    flywheelMotorLeft.setInverted(ShooterConfig.Flywheel.IS_INVERT);
    flywheelMotorLeft.setNeutralMode(NeutralMode.Coast);
    flywheelMotorLeft.enableVoltageCompensation(true);
    TalonUtil.checkError(
        flywheelMotorLeft.configVoltageCompSaturation(
            ShooterConfig.Flywheel.MAX_COMPENSATED_VOLTAGE, Config.CAN_TIMEOUT_MS),
        "Flywheel Left: Can't config comp voltage!");
    TalonUtil.checkError(
        flywheelMotorLeft.configVelocityMeasurementPeriod(
            SensorVelocityMeasPeriod.Period_10Ms, Config.CAN_TIMEOUT_MS),
        "Flywheel Left: Can't config velocity measurement period!");
    TalonUtil.checkError(
        flywheelMotorLeft.configVelocityMeasurementWindow(2, Config.CAN_TIMEOUT_MS),
        "Flywheel Left: Can't config velocity measurement period!");
    TalonUtil.checkError(
        flywheelMotorLeft.configSupplyCurrentLimit(
            new SupplyCurrentLimitConfiguration(true, 39.0, 39.0, 0.0), Config.CAN_TIMEOUT_MS),
        "Flywheel Left: Can't config supply currant limit!");
    TalonUtil.checkError(
        flywheelMotorLeft.configAllowableClosedloopError(0, 0.0, Config.CAN_TIMEOUT_MS),
        "Flywheel Left: Can't config allowable closed loop error!");
    // PID 0 is for velocity
    TalonUtil.checkError(
        flywheelMotorLeft.config_kP(
            0, ShooterConfig.Flywheel.MASTER_PID0_KP, Config.CAN_TIMEOUT_MS),
        "Flywheel Left: Can't set kp!");
    TalonUtil.checkError(
        flywheelMotorLeft.config_kI(
            0, ShooterConfig.Flywheel.MASTER_PID0_KI, Config.CAN_TIMEOUT_MS),
        "Flywheel Left: Can't set ki!");
    TalonUtil.checkError(
        flywheelMotorLeft.config_kD(
            0, ShooterConfig.Flywheel.MASTER_PID0_KD, Config.CAN_TIMEOUT_MS),
        "Flywheel Left: Can't set kd!");
    TalonUtil.checkError(
        flywheelMotorLeft.config_kF(
            0, ShooterConfig.Flywheel.MASTER_PID0_KF, Config.CAN_TIMEOUT_MS),
        "Flywheel Left: Can't set kf!");

    // TalonFX follow has to do
    flywheelMotorRight.configStatusFramePeriod(
        ShooterConfig.Flywheel.STATUS_FRAME_PERIOD, true, false, Config.CAN_TIMEOUT_MS);
    flywheelMotorRight.setInverted(!ShooterConfig.Flywheel.IS_INVERT);
    flywheelMotorRight.setNeutralMode(NeutralMode.Coast);
    flywheelMotorRight.enableVoltageCompensation(true);
    TalonUtil.checkError(
        flywheelMotorRight.configVoltageCompSaturation(
            ShooterConfig.Flywheel.MAX_COMPENSATED_VOLTAGE, Config.CAN_TIMEOUT_MS),
        "Flywheel Right: Can't config comp voltage!");
    TalonUtil.checkError(
        flywheelMotorRight.configVelocityMeasurementPeriod(
            SensorVelocityMeasPeriod.Period_10Ms, Config.CAN_TIMEOUT_MS),
        "Flywheel Right: Can't config velocity measurement period!");
    TalonUtil.checkError(
        flywheelMotorRight.configVelocityMeasurementWindow(2, Config.CAN_TIMEOUT_MS),
        "Flywheel Right: Can't config velocity measurement period!");
    TalonUtil.checkError(
        flywheelMotorRight.configSupplyCurrentLimit(
            new SupplyCurrentLimitConfiguration(true, 39.0, 39.0, 0.0), Config.CAN_TIMEOUT_MS),
        "Flywheel Right: Can't config supply currant limit!");
    TalonUtil.checkError(
        flywheelMotorRight.configAllowableClosedloopError(0, 0.0, Config.CAN_TIMEOUT_MS),
        "Flywheel Right: Can't config allowable closed loop error!");
    // PID 0 is for velocity
    TalonUtil.checkError(
        flywheelMotorRight.config_kP(
            0, ShooterConfig.Flywheel.SLAVE_PID0_KP, Config.CAN_TIMEOUT_MS),
        "Flywheel Right: Can't set kp!");
    TalonUtil.checkError(
        flywheelMotorRight.config_kI(
            0, ShooterConfig.Flywheel.SLAVE_PID0_KI, Config.CAN_TIMEOUT_MS),
        "Flywheel Right: Can't set ki!");
    TalonUtil.checkError(
        flywheelMotorRight.config_kD(
            0, ShooterConfig.Flywheel.SLAVE_PID0_KD, Config.CAN_TIMEOUT_MS),
        "Flywheel Right: Can't set kd!");
    TalonUtil.checkError(
        flywheelMotorRight.config_kF(
            0, ShooterConfig.Flywheel.SLAVE_PID0_KF, Config.CAN_TIMEOUT_MS),
        "Flywheel Right: Can't set kf!");
  }

  /************************************************************************************************
   * Function Enabler *
   ************************************************************************************************/
  public synchronized void enableOdometryAim() {
    isOdometryAimEnable = true;
  }

  public synchronized void disableOdometryAim() {
    isOdometryAimEnable = false;
  }

  /************************************************************************************************
   * Getter & Setter *
   ************************************************************************************************/
  public synchronized void setFlywheelOpenLoop(double output) {
    periodicOutput.flywheelMotorControlMode = ControlMode.PercentOutput;
    periodicOutput.flywheelMotorOutput = output;
    periodicOutput.flywheelMasterFFOutput = 0.0;
    periodicOutput.flywheelSlaveFFOutput = 0.0;
  }

  public synchronized void setFlywheelEncoderVelocity(double encoder_units_per_100_ms) {
    periodicOutput.flywheelMotorControlMode = ControlMode.Velocity;
    periodicOutput.flywheelMotorOutput =
        Util.limit(encoder_units_per_100_ms, 0.0, ShooterConfig.Flywheel.MAX_SPEED);
    periodicOutput.flywheelMasterFFOutput =
        flywheelMotorLeftFFModel.calculate(
                flywheelEncoderUnitsPer100msToRps(encoder_units_per_100_ms))
            / ShooterConfig.Flywheel.MAX_COMPENSATED_VOLTAGE;
    periodicOutput.flywheelSlaveFFOutput =
        flywheelMotorRightFFModel.calculate(
                flywheelEncoderUnitsPer100msToRps(encoder_units_per_100_ms))
            / ShooterConfig.Flywheel.MAX_COMPENSATED_VOLTAGE;
  }

  public synchronized void setFlywheelRpm(double rpm) {
    setFlywheelEncoderVelocity(Math.abs(flywheelRpmToEncoderUnitsPer100ms(rpm)));
  }

  public synchronized void setFlywheelCoast() {
    setFlywheelOpenLoop(ShooterConfig.Flywheel.COAST_SPEED_OPEN_LOOP);
  }

  public boolean isFlyWheelVelocityOnTarget() {
    var masterFlyWheelVelRatio =
        periodicInput.flywheelMasterEncoderVelocity / periodicOutput.flywheelMotorOutput;
    var slaveFlyWheelVelRatio =
        periodicInput.flywheelSlaveEncoderVelocity / periodicOutput.flywheelMotorOutput;

    return !(Util.epsilonEquals(periodicOutput.flywheelMotorOutput, 0.0, 100.0))
        && masterFlyWheelVelRatio > ShooterConfig.Flywheel.MIN_ALLOWABLE_FLYWHEEL_VEL_RATIO
        && slaveFlyWheelVelRatio > ShooterConfig.Flywheel.MIN_ALLOWABLE_FLYWHEEL_VEL_RATIO
        && periodicOutput.flywheelMotorControlMode == ControlMode.Velocity;
  }

  public boolean isReadyToShoot(boolean is_strict) {
    return isFlyWheelVelocityOnTarget() && turret.isTurretOnTargets(is_strict);
  }

  /************************************************************************************************
   * Util *
   ************************************************************************************************/
  private double flywheelEncoderUnitsPer100msToRpm(double encoder_units_per_100ms) {
    return encoder_units_per_100ms
        / ShooterConfig.Flywheel.ENCODER_UNITS_PER_DEGREE
        * 600.0
        / 360.0;
  }

  private double flywheelRpmToEncoderUnitsPer100ms(double rpm) {
    return rpm / 600.0 * 360.0 * ShooterConfig.Flywheel.ENCODER_UNITS_PER_DEGREE;
  }

  private double flywheelEncoderUnitsPer100msToRps(double encoder_units_per_100ms) {
    return encoder_units_per_100ms / ShooterConfig.Flywheel.ENCODER_UNITS_PER_RADIANS * 10.0;
  }

  /************************************************************************************************
   * Stop & Disable Actions *
   ************************************************************************************************/
  @Override
  public void disable() {
    setState(ShooterState.DISABLE);
  }

  /************************************************************************************************
   * Log & self-test *
   ************************************************************************************************/
  private NetworkTableEntry ShooterStateEntry;

  private NetworkTableEntry targetSpeedEntry;
  private NetworkTableEntry masterSpeedEntry;
  private NetworkTableEntry slaveSpeedEntry;

  public void configSmartDashboard() {
    var tab = Shuffleboard.getTab("Shooter");

    ShooterStateEntry = tab.add("Shooter State", "None").getEntry();
    masterSpeedEntry = tab.add("Shooter Master Speed RPM", 999.99).getEntry();
    slaveSpeedEntry = tab.add("Shooter Slave Speed RPM", 999.99).getEntry();
    targetSpeedEntry = tab.add("Shooter target Speed RPM", 999.99).getEntry();
  }

  @Override
  public void logToSmartDashboard() {
    if (Config.ENABLE_DEBUG_OUTPUT) {
      turret.logToSmartDashboard();
      ShooterStateEntry.setString(shooterState.value);
      masterSpeedEntry.setNumber(
          flywheelEncoderUnitsPer100msToRpm(periodicInput.flywheelMasterEncoderVelocity));
      slaveSpeedEntry.setNumber(
          flywheelEncoderUnitsPer100msToRpm(periodicInput.flywheelSlaveEncoderVelocity));
      targetSpeedEntry.setNumber(
          flywheelEncoderUnitsPer100msToRpm(periodicOutput.flywheelMotorOutput));
    }
  }
}
