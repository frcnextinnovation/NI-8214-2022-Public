package com.nextinnovation.lib.trajectory.timing;

import com.nextinnovation.lib.geometry.Pose2dWithCurvature;

public class CurvatureVelocityConstraint implements ITimingConstraint<Pose2dWithCurvature> {
  // ! This is added by @Rocky
  // ? I don't understand why here is a strong coupling? So I changed it.
  private final double CHASSIS_MAX_DRIVE_SPEED_INCHES_PER_SECOND;

  public CurvatureVelocityConstraint(double chassisMaxDriveSpeedInchesPerSecond) {
    CHASSIS_MAX_DRIVE_SPEED_INCHES_PER_SECOND = chassisMaxDriveSpeedInchesPerSecond;
  }

  @Override
  public double getMaxVelocity(final Pose2dWithCurvature state) {
    return CHASSIS_MAX_DRIVE_SPEED_INCHES_PER_SECOND
        / (1 + Math.abs(4.0 * state.getCurvature())); // 6.0
  }

  @Override
  public MinMaxAcceleration getMinMaxAcceleration(
      final Pose2dWithCurvature state, final double velocity) {
    return MinMaxAcceleration.kNoLimits;
  }
}
