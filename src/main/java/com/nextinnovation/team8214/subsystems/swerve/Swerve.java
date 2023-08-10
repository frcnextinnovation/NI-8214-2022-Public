package com.nextinnovation.team8214.subsystems.swerve;

import com.nextinnovation.lib.controllers.HeadingController;
import com.nextinnovation.lib.geometry.Pose2dWithCurvature;
import com.nextinnovation.lib.geometry.Rotation2d;
import com.nextinnovation.lib.geometry.Translation2d;
import com.nextinnovation.lib.kinematics.SwerveInverseKinematics;
import com.nextinnovation.lib.loops.ILoop;
import com.nextinnovation.lib.loops.ILooper;
import com.nextinnovation.team8214.Odometry;
import com.nextinnovation.team8214.managers.CancoderManager;
import com.nextinnovation.team8214.planners.DriveMotionPlanner;
import com.nextinnovation.lib.subsystems.BaseSubsystem;
import com.nextinnovation.lib.trajectory.TimedView;
import com.nextinnovation.lib.trajectory.Trajectory;
import com.nextinnovation.lib.trajectory.TrajectoryIterator;
import com.nextinnovation.lib.trajectory.timing.TimedState;
import com.nextinnovation.lib.utils.Util;
import com.nextinnovation.team8214.Config;
import com.nextinnovation.team8214.Ports;
import com.nextinnovation.team8214.devices.ahrs.AhrsPigeon2;
import com.nextinnovation.team8214.devices.ahrs.BaseAhrs;
import com.nextinnovation.team8214.managers.ControlSignalManager;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;

import java.util.Arrays;
import java.util.List;

public class Swerve extends BaseSubsystem {
  /***********************************************************************************************
   * Control Loop *
   ***********************************************************************************************/
  @Override
  public void registerEnabledLoops(ILooper enabled_looper) {
    ILoop loop =
        new ILoop() {
          @Override
          public void onStart(double timestamp) {
            setState(SwerveState.MANUAL);
          }

          @Override
          public void onLoop(double timestamp) {
            var controlSignalManager = ControlSignalManager.getInstance();

            switch (swerveState) {
              case MANUAL:
                var translationalInput = controlSignalManager.getSwerveManualTranslation();
                var rotationalInput = controlSignalManager.getSwerveManualRotationMagnitude();

                if (Util.epsilonEquals(rotationalInput, 0.0)) {
                  if (!isHeadingControllerEnabled()
                      && Math.abs(getAngularVelocity().getUnboundedDegrees()) <= 5.625) {
                    setTargetHeadingToCurrentHeading();
                    enableHeadingController();
                  }
                } else {
                  disableHeadingController();
                }
                updateNormalizedVectorialVelocityControl(
                    translationalInput, rotationalInput, false, timestamp);
                break;

              case TRAJECTORY:
                if (!driveMotionPlanner.isDone() || !headingController.onTarget()) {
                  var translationalTrajectoryInput =
                      driveMotionPlanner.update(odometry.getLatestFieldCentricRobotPose());
                  enableHeadingController();
                  updateNormalizedVectorialVelocityControl(
                      translationalTrajectoryInput, 0.0, true, timestamp);
                } else {
                  disableModules();
                  return;
                }
                break;

              case DISABLE:
                disable();
                break;
            }

            updateOdometry();
          }

          @Override
          public void onStop(double timestamp) {
            disable();
          }
        };
    enabled_looper.register(loop);
  }

  /***********************************************************************************************
   * Periodic IO *
   ***********************************************************************************************/
  private static class PeriodicInput {
    public Rotation2d ahrsHeading = new Rotation2d();
    public Rotation2d ahrsAngularVelocity = new Rotation2d();
  }

  @Override
  public void readPeriodicInputs() {
    periodicInput.ahrsHeading = ahrs.getRobotHeading();
    periodicInput.ahrsAngularVelocity = ahrs.getRobotAngularVelocity();
    modules.forEach(SwerveDriveModule::readPeriodicInputs);
  }

  @Override
  public void writePeriodicOutputs() {
    modules.forEach(SwerveDriveModule::writePeriodicOutputs);
  }

  /***********************************************************************************************
   * Subsystem States *
   ***********************************************************************************************/
  private SwerveState swerveState;

  public synchronized void setState(SwerveState new_state) {
    swerveState = new_state;

    switch (swerveState) {
      case MANUAL:
      case TRAJECTORY:
        enableHeadingController();
        break;
      case DISABLE:
        disableHeadingController();
        disableModules();
        break;
    }
  }

  /***********************************************************************************************
   * Singleton *
   ***********************************************************************************************/
  private static Swerve instance = null;

  public static synchronized Swerve getInstance() {
    if (instance == null) {
      instance = new Swerve();
    }
    return instance;
  }

  /***********************************************************************************************
   * Init & Config *
   ***********************************************************************************************/
  private final PeriodicInput periodicInput = new PeriodicInput();

  private final SwerveInverseKinematics inverseKinematics =
      new SwerveInverseKinematics(
          SwerveConfig.MODULE_COUNT, SwerveConfig.POSITIONS_RELATIVE_TO_DRIVE_CENTER);

  private final HeadingController headingController = new HeadingController();
  private final SwerveDriveModule frontRightModule;
  private final SwerveDriveModule frontLeftModule;
  private final SwerveDriveModule rearLeftModule;
  private final SwerveDriveModule rearRightModule;
  private final List<SwerveDriveModule> modules;
  private final Odometry odometry;
  private DriveMotionPlanner driveMotionPlanner =
      DriveMotionPlanner.createDefault(SwerveConfig.MAX_SPEED_METERS_PER_SECOND);

  private final BaseAhrs ahrs = AhrsPigeon2.getInstance();

  private Swerve() {
    headingController.setTargetHeading(Rotation2d.fromDegrees(Config.INIT_HEADING));
    headingController.setOutputRange(-0.31, 0.31);

    configHeadingController(0.375, 0.0, 0.01, 0.01);

    var cancoderManager = CancoderManager.getInstance();

    frontLeftModule =
        new SwerveDriveModule(
            0,
            Ports.Can.FRONT_LEFT_DRIVE_MOTOR,
            Ports.Can.FRONT_LEFT_ROTATION_MOTOR,
            cancoderManager.getFrontLeft(),
            SwerveConfig.FRONT_LEFT_CALIBRATION_OFFSET);
    rearLeftModule =
        new SwerveDriveModule(
            1,
            Ports.Can.REAR_LEFT_DRIVE_MOTOR,
            Ports.Can.REAR_LEFT_ROTATION_MOTOR,
            cancoderManager.getRearLeft(),
            SwerveConfig.REAR_LEFT_CALIBRATION_OFFSET);
    rearRightModule =
        new SwerveDriveModule(
            2,
            Ports.Can.REAR_RIGHT_DRIVE_MOTOR,
            Ports.Can.REAR_RIGHT_ROTATION_MOTOR,
            cancoderManager.getRearRight(),
            SwerveConfig.REAR_RIGHT_CALIBRATION_OFFSET);
    frontRightModule =
        new SwerveDriveModule(
            3,
            Ports.Can.FRONT_RIGHT_DRIVE_MOTOR,
            Ports.Can.FRONT_RIGHT_ROTATION_MOTOR,
            cancoderManager.getFrontRight(),
            SwerveConfig.FRONT_RIGHT_CALIBRATION_OFFSET);

    modules = Arrays.asList(frontLeftModule, rearLeftModule, rearRightModule, frontRightModule);
    odometry = Odometry.getInstance();

    configSmartDashboard();
    configModules();
    setState(SwerveState.DISABLE);
  }

  private synchronized void configModules() {
    rearLeftModule.enableRotationMotorInverted(false);
    rearRightModule.enableRotationMotorInverted(false);
  }

  public synchronized void configHeadingController(
      double kp, double ki, double kd, double error_tolerance) {
    headingController.configParmas(kp, ki, kd, error_tolerance);
  }

  /************************************************************************************************
   * Function Enabler *
   ************************************************************************************************/
  public void enableHeadingController() {
    headingController.enable();
  }

  public void disableHeadingController() {
    headingController.disable();
  }

  /************************************************************************************************
   * Getter & Setter *
   ************************************************************************************************/
  public Rotation2d getTargetHeading() {
    return headingController.getTargetHeading();
  }

  public Rotation2d getFieldCentricHeading() {
    return periodicInput.ahrsHeading;
  }

  public SwerveModuleState[] getModuleStates() {
    return new SwerveModuleState[] {
      modules.get(0).getWpilibModuleState(),
      modules.get(1).getWpilibModuleState(),
      modules.get(2).getWpilibModuleState(),
      modules.get(3).getWpilibModuleState()
    };
  }

  public Rotation2d getAngularVelocity() {
    return periodicInput.ahrsAngularVelocity;
  }

  public double getRawAngularVelocity() {
    return ahrs.getRobotRawAngularVel();
  }

  public synchronized void setHeading(Rotation2d target_absolute_heading_degrees) {
    headingController.setTargetHeading(target_absolute_heading_degrees);
  }

  public synchronized void setHeading(double target_absolute_heading_degrees) {
    headingController.setTargetHeading(new Rotation2d(target_absolute_heading_degrees));
  }

  public synchronized void setTargetHeadingToCurrentHeading() {
    setHeading(periodicInput.ahrsHeading);
  }

  private synchronized void setNormalizedModuleVelocityTargets(
      List<Translation2d> module_velocities, boolean enable_closed_loop_control) {
    for (int i = 0; i < modules.size(); i++) {
      var module = modules.get(i);
      var moduleVelocity = module_velocities.get(i);
      if (Util.shouldReverseRotation(
          module_velocities.get(i).direction().getDegrees(),
          modules.get(i).getRobotCentricRotationHeading().getDegrees())) {
        if (!Util.epsilonEquals(moduleVelocity.norm(), 0.0)) {
          module.setRotationHeadingTarget(
              moduleVelocity.direction().rotateBy(Rotation2d.fromDegrees(180.0)));
        }
        if (enable_closed_loop_control) {
          module.setNormalizedTranslationVelocityTarget(-moduleVelocity.norm());
        } else {
          module.setTranslationOpenLoop(-moduleVelocity.norm());
        }
      } else {
        if (!Util.epsilonEquals(moduleVelocity.norm(), 0.0)) {
          module.setRotationHeadingTarget(moduleVelocity.direction());
        }
        if (enable_closed_loop_control) {
          module.setNormalizedTranslationVelocityTarget(moduleVelocity.norm());
        } else {
          module.setTranslationOpenLoop(moduleVelocity.norm());
        }
      }
    }
  }

  public void setTrajectory(
      Trajectory<TimedState<Pose2dWithCurvature>> trajectory, Translation2d following_center) {
    driveMotionPlanner = DriveMotionPlanner.createDefault(SwerveConfig.MAX_SPEED_METERS_PER_SECOND);

    setState(SwerveState.TRAJECTORY);
    driveMotionPlanner.reset();
    driveMotionPlanner.setFollowingCenter(following_center);
    driveMotionPlanner.setTrajectory(new TrajectoryIterator<>(new TimedView<>(trajectory)));
  }

  public synchronized void setTrajectory(Trajectory<TimedState<Pose2dWithCurvature>> trajectory) {
    setTrajectory(trajectory, Translation2d.identity());
  }

  public boolean isDoneWithTrajectory() {
    if (swerveState != SwerveState.TRAJECTORY) {
      return false;
    } else {
      return driveMotionPlanner.isDone();
    }
  }

  public boolean isHeadingControllerEnabled() {
    return headingController.isEnabled();
  }

  /************************************************************************************************
   * Update *
   ************************************************************************************************/
  public synchronized void updateOdometry() {
    odometry.updateWo(getFieldCentricHeading(), getModuleStates());
  }

  /**
   * Update set normalized translation and rotation velocity of swerve, usually used in teleop mode
   *
   * @param translation_vector Normalized translation vector in [-1.0, 1.0]
   * @param rotation_magnitude Normalized magnitude vector in [-1.0, 1.0]
   * @param enable_closed_loop_Control Is module translation motor Velocity or Percent Output mode
   * @param timestamp Current timestamp in FPGA timer
   */
  public synchronized void updateNormalizedVectorialVelocityControl(
      Translation2d translation_vector,
      double rotation_magnitude,
      boolean enable_closed_loop_Control,
      double timestamp) {
    setNormalizedModuleVelocityTargets(
        inverseKinematics.calculateNormalizedModuleVelocities(
            translation_vector,
            rotation_magnitude + headingController.calculate(getFieldCentricHeading(), timestamp),
            getFieldCentricHeading()),
        enable_closed_loop_Control);
  }
  /************************************************************************************************
   * Stop & Disable Actions *
   ************************************************************************************************/
  public void disableModules() {
    modules.forEach(SwerveDriveModule::disable);
  }

  @Override
  public void disable() {
    setState(SwerveState.DISABLE);
  }

  /************************************************************************************************
   * Log & self-test *
   ************************************************************************************************/
  private NetworkTableEntry swerveStateEntry;

  private NetworkTableEntry isHeadingControllerEnabledEntry;
  private NetworkTableEntry angularVelEntry;

  public void configSmartDashboard() {
    var tab = Shuffleboard.getTab("Swerve");

    swerveStateEntry = tab.add("Swerve State", "None").getEntry();
    isHeadingControllerEnabledEntry = tab.add("Is HeadingController Enabled", false).getEntry();
    angularVelEntry = tab.add("Angular Vel", 0.0).getEntry();
  }

  @Override
  public void logToSmartDashboard() {
    if (Config.ENABLE_DEBUG_OUTPUT) {
      modules.forEach(SwerveDriveModule::logToSmartDashboard);
      swerveStateEntry.setString(swerveState.value);
      isHeadingControllerEnabledEntry.setBoolean(isHeadingControllerEnabled());
      angularVelEntry.setNumber(ahrs.getRobotAngularVelocity().getDegrees());
    }
  }
}
