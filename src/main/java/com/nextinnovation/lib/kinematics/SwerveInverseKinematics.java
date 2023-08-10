package com.nextinnovation.lib.kinematics;

import com.nextinnovation.lib.geometry.Rotation2d;
import com.nextinnovation.lib.geometry.Translation2d;
import com.nextinnovation.lib.utils.Util;

import java.util.ArrayList;
import java.util.List;

public class SwerveInverseKinematics {
  private final int moduleCount;
  private final List<Translation2d> modulePositionsRelativeToDriveCenter;
  private List<Translation2d> modulePositionsRelativeToRotationCenter;
  private List<Rotation2d> moduleRotationVectors;

  /**
   * Constructor
   *
   * @param module_count Num of modules
   * @param module_positions_relative_to_drive_center Module position related to robot centric
   */
  public SwerveInverseKinematics(
      int module_count, List<Translation2d> module_positions_relative_to_drive_center) {
    moduleCount = module_count;
    modulePositionsRelativeToDriveCenter = module_positions_relative_to_drive_center;
    modulePositionsRelativeToRotationCenter = modulePositionsRelativeToDriveCenter;
    resetCenterOfRotation();
  }

  /**
   * Set the following center
   *
   * @param center_of_rotation Following center position related to robot centric
   */
  public void setCenterOfRotation(Translation2d center_of_rotation) {
    modulePositionsRelativeToRotationCenter = new ArrayList<>(moduleCount);
    for (Translation2d position : modulePositionsRelativeToDriveCenter) {
      modulePositionsRelativeToRotationCenter.add(
          position.translateBy(center_of_rotation.inverse()));
    }
    moduleRotationVectors = new ArrayList<>(moduleCount);
    for (Translation2d position : modulePositionsRelativeToRotationCenter) {
      moduleRotationVectors.add(position.direction().normal());
    }
  }

  /** Reset the following center */
  public void resetCenterOfRotation() {
    setCenterOfRotation(Translation2d.identity());
  }

  /**
   * Basic method to solve swerve inverse kinematics
   *
   * @param translation_vector Normalized translation vector, norm is [-1.0, 1.0]
   * @param rotation_magnitude Normalized rotation magnitude in [-1.0, 1.0]
   * @param field_centric_robot_heading Robot heading in degrees
   * @return Solved drive vector of modules
   */
  public List<Translation2d> calculateNormalizedModuleVelocities(
      final Translation2d translation_vector,
      double rotation_magnitude,
      Rotation2d field_centric_robot_heading) {
    Translation2d robot_centric_translation_vector =
        translation_vector.rotateBy(field_centric_robot_heading.inverse());
    List<Translation2d> moduleDriveVectors = new ArrayList<>(moduleCount);
    double maxCalculatedModuleVelocity = 1.0;
    for (Rotation2d rotationVector : moduleRotationVectors) {
      Translation2d driveVector =
          robot_centric_translation_vector.translateBy(
              Translation2d.fromPolar(rotationVector, rotation_magnitude));
      double translationVelocity = driveVector.norm();

      if (translationVelocity > maxCalculatedModuleVelocity) {
        maxCalculatedModuleVelocity = translationVelocity;
      }

      moduleDriveVectors.add(driveVector);
    }

    if (maxCalculatedModuleVelocity - 1.0 > Util.EPSILON_VALUE) {
      double velocityConstraintScalar = 1.0 / maxCalculatedModuleVelocity;
      moduleDriveVectors.replaceAll(vector -> vector.scale(velocityConstraintScalar));
    }

    return moduleDriveVectors;
  }
}
