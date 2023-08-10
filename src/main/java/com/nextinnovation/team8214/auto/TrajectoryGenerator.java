package com.nextinnovation.team8214.auto;

import com.nextinnovation.lib.geometry.Pose2d;
import com.nextinnovation.lib.geometry.Pose2dWithCurvature;
import com.nextinnovation.lib.geometry.Rotation2d;
import com.nextinnovation.lib.trajectory.DistanceView;
import com.nextinnovation.lib.trajectory.Trajectory;
import com.nextinnovation.lib.trajectory.TrajectoryUtil;
import com.nextinnovation.lib.trajectory.timing.ITimingConstraint;
import com.nextinnovation.lib.trajectory.timing.TimedState;
import com.nextinnovation.lib.trajectory.timing.TimingUtil;

import java.util.ArrayList;
import java.util.List;

public class TrajectoryGenerator {
  public static final class TrajectoryGeneratorConfig {
    public static final double MAX_DELTA_X = 0.05; // meter
    public static final double MAX_DELTA_Y = 0.005; // meter
    public static final double MAX_DELTA_THETA = Math.toRadians(2.5); // rad
    public static final double MAX_ABS_ACCEL = 3.8; // meter/s^2 org3.5
  }

  /**
   * Basic method to create a completed trajectory with given constraints and parameters.
   *
   * @param need_reversed Is trajectory need to reversed
   * @param waypoints Key waypoints to go through
   * @param constraints Velocity or curvature constraints of chassis
   * @param start_vel Start velocity in meters/s, usually zero
   * @param end_vel End velocity in meters/s, usually zero
   * @param max_vel Max velocity for trajectory
   * @param max_abs_accel Max abstract velocity in meters/s^2 for trajectory
   * @return Trajectory
   */
  public static Trajectory<TimedState<Pose2dWithCurvature>> generateTrajectory(
      boolean need_reversed,
      final List<Pose2d> waypoints,
      final List<ITimingConstraint<Pose2dWithCurvature>> constraints,
      double start_vel,
      double end_vel,
      double max_vel, // meters/s
      double max_abs_accel // meters/s^2
      ) {
    var waypointsMaybeFlipped = waypoints;
    final var flip = Pose2d.fromRotation(new Rotation2d(-1, 0, false));
    if (need_reversed) {
      waypointsMaybeFlipped = new ArrayList<>(waypoints.size());
      for (var waypoint : waypoints) {
        waypointsMaybeFlipped.add(waypoint.transformBy(flip));
      }
    }

    // Create a trajectory from splines.
    var trajectory =
        TrajectoryUtil.trajectoryFromSplineWaypoints(
            waypointsMaybeFlipped,
            TrajectoryGeneratorConfig.MAX_DELTA_X,
            TrajectoryGeneratorConfig.MAX_DELTA_Y,
            TrajectoryGeneratorConfig.MAX_DELTA_THETA);

    if (need_reversed) {
      List<Pose2dWithCurvature> flipped = new ArrayList<>(trajectory.length());
      for (int i = 0; i < trajectory.length(); ++i) {
        flipped.add(
            new Pose2dWithCurvature(
                trajectory.getState(i).getPose().transformBy(flip),
                -trajectory.getState(i).getCurvature(),
                trajectory.getState(i).getDCurvatureDs()));
      }
      trajectory = new Trajectory<>(flipped);
    }

    List<ITimingConstraint<Pose2dWithCurvature>> all_constraints = new ArrayList<>();
    if (constraints != null) {
      all_constraints.addAll(constraints);
    }

    // Generate the timed trajectory.
    return TimingUtil.timeParameterizeTrajectory(
        need_reversed,
        new DistanceView<>(trajectory),
        TrajectoryGeneratorConfig.MAX_DELTA_X,
        all_constraints,
        start_vel,
        end_vel,
        max_vel,
        max_abs_accel);
  }

  /**
   * Create a completed swerve translation trajectory, since the trajectory generator can't set
   * swerve kinematics constraint, so the max translation velocity should be adjusted manually.
   *
   * @param need_reversed Is trajectory need to reversed
   * @param waypoints Key waypoints to go through
   * @param max_translation_vel Max velocity for swerve translation
   * @return Trajectory
   */
  public static Trajectory<TimedState<Pose2dWithCurvature>> generateSwerveTrajectory(
      boolean need_reversed, final List<Pose2d> waypoints, double max_translation_vel) {
    return generateTrajectory(
        need_reversed,
        waypoints,
        List.of(),
        0.0,
        0.0,
        max_translation_vel,
        TrajectoryGeneratorConfig.MAX_ABS_ACCEL);
  }
}
