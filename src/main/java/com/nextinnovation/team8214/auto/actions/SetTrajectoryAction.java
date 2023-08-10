package com.nextinnovation.team8214.auto.actions;

import com.nextinnovation.lib.auto.actions.BaseAction;
import com.nextinnovation.lib.geometry.Pose2dWithCurvature;
import com.nextinnovation.lib.trajectory.Trajectory;
import com.nextinnovation.lib.trajectory.timing.TimedState;
import com.nextinnovation.team8214.subsystems.swerve.Swerve;
import edu.wpi.first.wpilibj.Timer;

public class SetTrajectoryAction extends BaseAction {
  private final Trajectory<TimedState<Pose2dWithCurvature>> trajectory;
  private final double goalHeading;
  private final double rotationDelay;
  private boolean isRotationStart = false;
  private double startTimestamp = 0.0;
  private final Swerve swerve;

  public SetTrajectoryAction(
      Trajectory<TimedState<Pose2dWithCurvature>> trajectory, double goal_heading) {
    this(trajectory, goal_heading, 0.0);
  }

  public SetTrajectoryAction(
      Trajectory<TimedState<Pose2dWithCurvature>> trajectory,
      double goal_heading,
      double rotation_delay) {
    this.trajectory = trajectory;
    goalHeading = goal_heading;
    rotationDelay = rotation_delay;
    swerve = Swerve.getInstance();
  }

  @Override
  public void start() {
    swerve.setTrajectory(trajectory);
    startTimestamp = Timer.getFPGATimestamp();
    System.out.println("Trajectory start!");
  }

  @Override
  public void update() {
    if (!isRotationStart && (Timer.getFPGATimestamp() - startTimestamp) > rotationDelay) {
      swerve.setHeading(goalHeading);
      isRotationStart = true;
    }
    swerve.setHeading(goalHeading);
  }

  @Override
  public void done() {}

  @Override
  public boolean isFinished() {
    if (swerve.isDoneWithTrajectory()) {
      System.out.println("Trajectory finished or timeout!");
      swerve.disableModules();
      return true;
    } else {
      return false;
    }
  }
}
