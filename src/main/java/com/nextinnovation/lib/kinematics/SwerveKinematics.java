package com.nextinnovation.lib.kinematics;

import com.nextinnovation.lib.geometry.Pose2d;
import com.nextinnovation.lib.geometry.Rotation2d;
import com.nextinnovation.lib.geometry.Translation2d;

import java.util.ArrayList;
import java.util.List;

public class SwerveKinematics {
  private double previousUpdateTimestamp = Double.NaN;
  private Pose2d pose = new Pose2d();
  private Translation2d velocity = new Translation2d();
  private Translation2d deltaPosition = new Translation2d();
  private double totalDistance = 0.0;
  private final double distanceDevianceThreshold;

  public SwerveKinematics(double distanceDevianceThreshold) {
    this.distanceDevianceThreshold = distanceDevianceThreshold;
  }

  public Pose2d getPose() {
    return pose;
  }

  public void setPose(Pose2d pose) {
    this.pose = pose;
  }

  public Translation2d getVelocity() {
    return velocity;
  }

  public Translation2d getDeltaPosition() {
    return deltaPosition;
  }

  public double getTotalDistance() {
    return totalDistance;
  }

  public void resetTotalDistance() {
    totalDistance = 0.0;
  }

  public synchronized void update(
      List<Pose2d> moduleEstimatedRobotPoses, Rotation2d robotHeading, double timestamp) {
    updateKinematicState(estimateRobotPosition(moduleEstimatedRobotPoses), robotHeading, timestamp);
    previousUpdateTimestamp = timestamp;
  }

  private Translation2d estimateRobotPosition(List<Pose2d> moduleEstimatedRobotPoses) {
    List<Double> distances = new ArrayList<>(moduleEstimatedRobotPoses.size());
    double averageDistance = 0.0;
    for (Pose2d p : moduleEstimatedRobotPoses) {
      double distance = p.getTranslation().translateBy(getPose().getTranslation().inverse()).norm();
      distances.add(distance);
      averageDistance += distance;
    }
    averageDistance /= moduleEstimatedRobotPoses.size();
    double minDistanceDeviance = Double.POSITIVE_INFINITY;
    Pose2d estimatedPoseWithMinDistanceDeviance = null;
    List<Pose2d> acceptedEstimatedPoses = new ArrayList<>();
    for (int i = 0; i < moduleEstimatedRobotPoses.size(); i++) {
      double distanceDeviance = Math.abs(distances.get(i) - averageDistance);
      if (distanceDeviance <= distanceDevianceThreshold) {
        acceptedEstimatedPoses.add(moduleEstimatedRobotPoses.get(i));
      }
      if (distanceDeviance < minDistanceDeviance) {
        minDistanceDeviance = distanceDeviance;
        estimatedPoseWithMinDistanceDeviance = moduleEstimatedRobotPoses.get(i);
      }
    }
    if (acceptedEstimatedPoses.isEmpty()) {
      acceptedEstimatedPoses.add(estimatedPoseWithMinDistanceDeviance);
    }
    Translation2d estimatedRobotPosition = new Translation2d();
    for (Pose2d pose : acceptedEstimatedPoses) {
      estimatedRobotPosition = estimatedRobotPosition.translateBy(pose.getTranslation());
    }
    return estimatedRobotPosition.scale(1.0 / acceptedEstimatedPoses.size());
  }

  private void updateKinematicState(
      Translation2d robotPosition, Rotation2d robotHeading, double timestamp) {
    deltaPosition = robotPosition.translateBy(getPose().getTranslation().inverse());
    pose = new Pose2d(robotPosition, robotHeading);
    velocity = deltaPosition.scale(1.0 / (timestamp - previousUpdateTimestamp));
    totalDistance += deltaPosition.norm();
  }
}
