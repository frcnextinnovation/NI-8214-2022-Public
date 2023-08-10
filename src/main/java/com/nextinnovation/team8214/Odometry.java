package com.nextinnovation.team8214;

import com.nextinnovation.lib.geometry.Pose2d;
import com.nextinnovation.lib.geometry.Rotation2d;
import com.nextinnovation.team8214.dashboard.FieldViewer;
import com.nextinnovation.team8214.subsystems.swerve.SwerveConfig;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class Odometry {
  /***********************************************************************************************
   * Singleton *
   ***********************************************************************************************/
  private static Odometry instance = null;

  public static synchronized Odometry getInstance() {
    if (instance == null) {
      instance = new Odometry();
    }
    return instance;
  }

  /***********************************************************************************************
   * Init & Config *
   ***********************************************************************************************/
  private final SwerveDrivePoseEstimator estimator;

  private final FieldViewer fieldViewer;
  private boolean isAllowVoWork = false;
  private int odometryResetForVoCount = 0;

  private Odometry() {
    fieldViewer = new FieldViewer();
    estimator =
        new SwerveDrivePoseEstimator(
            Rotation2d.identity().toWpilibRotation2d(),
            Pose2d.identity().toWpilibPose2d(),
            SwerveConfig.WPILIB_SWERVE_KINEMATICS,
            VecBuilder.fill(0.02, 0.02, 0.01),
            VecBuilder.fill(0.01),
            VecBuilder.fill(0.5, 0.5, 0.1));
    isAllowVoWork = false;
  }

  /************************************************************************************************
   * Function Enabler *
   ************************************************************************************************/
  public synchronized void enableVo() {
    isAllowVoWork = true;
  }

  public synchronized void disableVo() {
    isAllowVoWork = false;
  }

  public synchronized boolean getIsVoEnable() {
    return isAllowVoWork;
  }

  /************************************************************************************************
   * Getter & Setter *
   ************************************************************************************************/
  public synchronized void setPose(Pose2d pose) {
    estimator.resetPosition(pose.toWpilibPose2d(), pose.toWpilibPose2d().getRotation());
  }

  public synchronized Pose2d getLatestFieldCentricRobotPose() {
    return Pose2d.fromWpilibPose2d(estimator.getEstimatedPosition());
  }

  public boolean isInSelfZone() {
    var fieldToRobot = getLatestFieldCentricRobotPose();
    return fieldToRobot.getTranslation().x() < Field.X_MID
        || fieldToRobot.getTranslation().y() > Field.Y_MID;
  }

  /************************************************************************************************
   * Update *
   ************************************************************************************************/
  public synchronized void updateWo(Rotation2d heading, SwerveModuleState... moduleStates) {
    estimator.update(heading.toWpilibRotation2d(), moduleStates);
  }

  public synchronized void updateVo(Pose2d visionEstimatedPose, double timestamp) {
    fieldViewer.setVoPose(visionEstimatedPose);

    if (isAllowVoWork) {
      var voPosition = visionEstimatedPose.getTranslation();
      var currantPosition = getLatestFieldCentricRobotPose().getTranslation();

      var voPositionToCurrantPositionDistance = voPosition.distance(currantPosition);

      if (odometryResetForVoCount > 20) {
        setPose(visionEstimatedPose);
        odometryResetForVoCount = 0;
      } else if (voPositionToCurrantPositionDistance > 1.0) {
        odometryResetForVoCount++;
      } else {
        estimator.addVisionMeasurement(visionEstimatedPose.toWpilibPose2d(), timestamp);
        odometryResetForVoCount = 0;
      }
    } else {
      odometryResetForVoCount = 0;
    }
  }

  /************************************************************************************************
   * Log *
   ************************************************************************************************/
  public void logToSmartDashBoard() {
    fieldViewer.setRobotPose(getLatestFieldCentricRobotPose());
  }
}
