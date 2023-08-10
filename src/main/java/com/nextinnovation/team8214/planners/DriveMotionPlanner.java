package com.nextinnovation.team8214.planners;

import com.nextinnovation.lib.controllers.Lookahead;
import com.nextinnovation.lib.geometry.Pose2d;
import com.nextinnovation.lib.geometry.Pose2dWithCurvature;
import com.nextinnovation.lib.geometry.Translation2d;
import com.nextinnovation.lib.trajectory.TrajectoryIterator;
import com.nextinnovation.lib.trajectory.timing.TimedState;
import com.nextinnovation.lib.utils.Util;
import edu.wpi.first.wpilibj.Timer;

public class DriveMotionPlanner {
  private final double minLookaheadTimeSecs;
  private final double minAdapativeLookaheadDistance;
  private final double maxAdaptiveLookaheadDistance;
  private final double adaptiveErrLookaheadDistanceKp;
  private final Lookahead lookahead;
  private final double minLookaheadDistanceMeters;
  private final double chassisMaxVelocityMetersPerSecond;
  private boolean isReversed = false;
  private boolean useDefaultCook = true;
  private final double defaultCook;
  private TimedState<Pose2dWithCurvature> setpoint =
      new TimedState<>(Pose2dWithCurvature.identity());
  private Translation2d output = Translation2d.identity();
  private double currentTrajectoryLength = 0.0;
  private Translation2d followingCenter = Translation2d.identity();
  private Translation2d err = Translation2d.identity();
  private TrajectoryIterator<TimedState<Pose2dWithCurvature>> currentTrajectory;
  private double lastTimestamp = Double.NEGATIVE_INFINITY;

  public static DriveMotionPlanner createDefault(double chassis_max_velocity_meters_per_second) {
    return new DriveMotionPlanner(
        0.25, 0.305, 0.15, 0.61, 0.1, chassis_max_velocity_meters_per_second, 0.11);
  }

  /**
   * Constructor of DriveMotionPlanner
   *
   * @param min_lookahead_time_secs Min lookahead time in seconds
   * @param min_lookahead_distance_meters Min chassis lookahead distance in meters
   * @param chassis_max_velocity_meters_per_second chassis max velocity in meter/s
   * @param min_normalized_drive_speed Min normalized follow speed
   */
  public DriveMotionPlanner(
      double min_lookahead_time_secs,
      double min_lookahead_distance_meters,
      double min_adaptive_lookahead_distance_meters,
      double max_adaptive_lookahead_distance_meters,
      double adaptive_err_lookahead_distance_kp,
      double chassis_max_velocity_meters_per_second,
      double min_normalized_drive_speed) {
    minLookaheadTimeSecs = min_lookahead_time_secs;
    minLookaheadDistanceMeters = min_lookahead_distance_meters;
    minAdapativeLookaheadDistance = min_adaptive_lookahead_distance_meters;
    maxAdaptiveLookaheadDistance = max_adaptive_lookahead_distance_meters;
    adaptiveErrLookaheadDistanceKp = adaptive_err_lookahead_distance_kp;
    chassisMaxVelocityMetersPerSecond = chassis_max_velocity_meters_per_second;
    defaultCook = min_normalized_drive_speed;

    lookahead =
        new Lookahead(
            minAdapativeLookaheadDistance,
            maxAdaptiveLookaheadDistance,
            0.0,
            chassisMaxVelocityMetersPerSecond);
  }

  /** Clear output. */
  public void reset() {
    output = Translation2d.identity();
    err = Translation2d.identity();
  }

  /**
   * Set the robot centric following center
   *
   * @param following_center Robot centric following center
   */
  public void setFollowingCenter(Translation2d following_center) {
    this.followingCenter = following_center;
  }

  /**
   * Set the trajectory to follow
   *
   * @param new_trajectory The trajectory to follow, usually created by TrajectoryGenerator
   */
  public void setTrajectory(
      final TrajectoryIterator<TimedState<Pose2dWithCurvature>> new_trajectory) {
    currentTrajectory = new_trajectory;
    setpoint = new_trajectory.getState();
    currentTrajectoryLength = new_trajectory.trajectory().getLastState().t();

    for (int i = 0; i < new_trajectory.trajectory().length(); i++) {
      if (new_trajectory.trajectory().getState(i).velocity() > Util.EPSILON_VALUE) {
        isReversed = false;
        break;
      } else if (new_trajectory.trajectory().getState(i).velocity() < -Util.EPSILON_VALUE) {
        isReversed = true;
        break;
      }
    }
  }

  /**
   * Get remain time for trajectory to follow
   *
   * @return remain time in seconds
   */
  public double getTrajectoryRemainingProgress() {
    if (currentTrajectory != null) {
      return currentTrajectory.getRemainingProgress();
    } else {
      return 0.0;
    }
  }

  /**
   * Get normalized trajectory progress
   *
   * @return Normalized progress of trajectory
   */
  public double getNormalizedProgress() {
    return currentTrajectory.getProgress() / currentTrajectoryLength;
  }

  /**
   * Get normalized output of motion planner, which is a normalized drive vector according to max
   * allowable velocity of chassis
   *
   * @return A normalized drive vector for chassis. For holonomic chassis this can be used directly
   */
  public Translation2d getNormalizedOutput() {
    return output;
  }

  /**
   * Get normalized constrain of rotation speed, which is a normalized drive vector according to max
   * allowable velocity of chassis
   *
   * @return A normalized constrain of rotation speed
   */
  public double getNormalizedMaxRotationSpeed() {
    return 1.0 - output.norm();
  }

  /**
   * Get is trajectory done
   *
   * @return Is trajectory done
   */
  public boolean isDone() {
    return currentTrajectory != null && currentTrajectory.isDone();
  }

  /**
   * Update create a normalized drive vector for chassis by adaptive pure pursuit
   *
   * @param current_state Current Pose2d of robot
   * @return A normalized drive vector output. Can be also get by getNormalizedOutput()
   */
  private Translation2d updateAdaptivePurePursuit(Pose2d current_state) {
    var lookaheadTime = minLookaheadTimeSecs;
    final var lookaheadAdaptiveDeltaTime = 0.01;

    var lookaheadState = currentTrajectory.preview(lookaheadTime).state();
    var actualLookaheadDistance = setpoint.state().distance(lookaheadState.state());
    var adaptive_lookahead_distance =
        lookahead.getLookaheadForSpeed(setpoint.velocity())
            + adaptiveErrLookaheadDistanceKp * err.norm();

    // ! Check is near the endpoint.
    while (actualLookaheadDistance < adaptive_lookahead_distance
        && currentTrajectory.getRemainingProgress() > lookaheadTime) {
      lookaheadTime += lookaheadAdaptiveDeltaTime;
      lookaheadState = currentTrajectory.preview(lookaheadTime).state();
      actualLookaheadDistance = setpoint.state().distance(lookaheadState.state());
    }

    if (actualLookaheadDistance < adaptive_lookahead_distance) {
      lookaheadState =
          new TimedState<>(
              new Pose2dWithCurvature(
                  lookaheadState
                      .state()
                      .getPose()
                      .transformBy(
                          Pose2d.fromTranslation(
                              new Translation2d(
                                  (isReversed ? -1.0 : 1.0)
                                      * (minLookaheadDistanceMeters - actualLookaheadDistance),
                                  0.0))),
                  0.0),
              lookaheadState.t(),
              lookaheadState.velocity(),
              lookaheadState.acceleration());
    }

    final var lookaheadTranslation =
        new Translation2d(current_state.getTranslation(), lookaheadState.state().getTranslation());

    var steeringDirection = lookaheadTranslation.direction();
    var normalizedSpeed = Math.abs(setpoint.velocity()) / chassisMaxVelocityMetersPerSecond;

    if (normalizedSpeed > defaultCook || setpoint.t() > (currentTrajectoryLength / 2.0)) {
      useDefaultCook = false;
    }
    if (useDefaultCook) {
      normalizedSpeed = defaultCook;
    }

    return Translation2d.fromPolar(steeringDirection, normalizedSpeed);
  }

  /** Update find a proper setpoint to reach */
  private void updateProperSetpoint(Pose2d current_state) {
    // These code try to find a proper setpoint to reach.
    var searchStepSize = 1.0;
    var previewQuantity = 0.0;
    var forwardDistance = distance(current_state, previewQuantity + searchStepSize);
    var reverseDistance = distance(current_state, previewQuantity - searchStepSize);
    var searchDirection = Math.signum(reverseDistance - forwardDistance);
    while (searchStepSize > 0.001) {
      if (Util.epsilonEquals(distance(current_state, previewQuantity), 0.0, 0.01)) break;
      /* next point is closer than current point */
      while (distance(current_state, previewQuantity + searchStepSize * searchDirection)
          < distance(current_state, previewQuantity)) {
        /* move to next point */
        previewQuantity += searchStepSize * searchDirection;
      }
      searchStepSize /= 10.0;
      searchDirection *= -1;
    }

    var samplePoint = currentTrajectory.advance(previewQuantity);
    setpoint = samplePoint.state();
  }

  /**
   * Update create a normalized drive vector for chassis by selected follower type
   *
   * @param current_state Current pose of robot
   * @return A normalized drive vector output. Can be also get by getNormalizedOutput()
   */
  public Translation2d update(Pose2d current_state) {
    var currentTimestamp = Timer.getFPGATimestamp();

    if (Double.isFinite(lastTimestamp)) {
      lastTimestamp = currentTimestamp;
    }

    if (currentTrajectory == null) {
      return Translation2d.identity();
    }
    current_state = current_state.transformBy(Pose2d.fromTranslation(followingCenter));
    currentTrajectory.advance(currentTimestamp - lastTimestamp);
    lastTimestamp = currentTimestamp;
    // These code try to find a comfortable point as the next point to be reached.
    updateProperSetpoint(current_state);
    if (!isDone()) {
      err = current_state.getTranslation().inverse().translateBy(setpoint.state().getTranslation());
      output = updateAdaptivePurePursuit(current_state);
    } else {
      output = Translation2d.identity();
    }
    return output;
  }

  /**
   * Update create a normalized drive vector for chassis by selected follower type
   *
   * @param current_state Current pose of robot
   * @param additional_progress lookahead progress
   * @return The distance in inches.
   */
  private double distance(Pose2d current_state, double additional_progress) {
    return currentTrajectory
        .preview(additional_progress)
        .state()
        .state()
        .getPose()
        .distance(current_state);
  }
}
