package com.nextinnovation.team8214.auto;

import com.nextinnovation.lib.geometry.Pose2d;
import com.nextinnovation.lib.geometry.Pose2dWithCurvature;
import com.nextinnovation.lib.trajectory.Trajectory;
import com.nextinnovation.lib.trajectory.timing.TimedState;
import com.nextinnovation.team8214.Field;
import edu.wpi.first.wpilibj.Timer;

import java.util.ArrayList;
import java.util.List;

public class TrajectorySet {
  /***********************************************************************************************
   * Singleton *
   ***********************************************************************************************/
  private static TrajectorySet instance = null;

  public static synchronized TrajectorySet getInstance() {
    if (instance == null) {
      instance = new TrajectorySet();
    }
    return instance;
  }

  /***********************************************************************************************
   * Init & Config *
   ***********************************************************************************************/
  public final Trajectory<TimedState<Pose2dWithCurvature>> leftStartToLeftBall;

  public final Trajectory<TimedState<Pose2dWithCurvature>> leftBallToLeftEnd;

  public final Trajectory<TimedState<Pose2dWithCurvature>> midStartToMidEnd;

  public final Trajectory<TimedState<Pose2dWithCurvature>> rightStartToRightBall;
  public final Trajectory<TimedState<Pose2dWithCurvature>> rightBallToMidBall;
  public final Trajectory<TimedState<Pose2dWithCurvature>> midBallToHumanStationBall;
  public final Trajectory<TimedState<Pose2dWithCurvature>> humanStationBallToHumanStationWait;
  public final Trajectory<TimedState<Pose2dWithCurvature>> humanStationWaitToMidShoot;

  private TrajectorySet() {
    // Left
    leftStartToLeftBall = getLeftStartToLeftBall();
    leftBallToLeftEnd = getLeftBallToLeftEnd();

    // Mid
    midStartToMidEnd = getMidStartToMidEnd();

    // Right
    rightStartToRightBall = getRightStartToRightBall();
    rightBallToMidBall = getRightBallToMidBall();
    midBallToHumanStationBall = getMidBallToHumanStationBall();
    humanStationBallToHumanStationWait = getHumanStationBallToHumanStationWait();
    humanStationWaitToMidShoot = getHumanStationWaitToMidShoot();
  }

  /************************************************************************************************
   * Getter & Setter *
   ************************************************************************************************/
  private Trajectory<TimedState<Pose2dWithCurvature>> getLeftStartToLeftBall() {
    List<Pose2d> waypoints = new ArrayList<>();
    waypoints.add(Field.Waypoints.LEFT_START_POSITION);
    waypoints.add(Field.Waypoints.LEFT_BALL_IN_POSITION);

    return TrajectoryGenerator.generateSwerveTrajectory(false, waypoints, 3.5);
  }

  private Trajectory<TimedState<Pose2dWithCurvature>> getLeftBallToLeftEnd() {
    List<Pose2d> waypoints = new ArrayList<>();
    waypoints.add(Field.Waypoints.LEFT_BALL_OUT_POSITION);
    waypoints.add(Field.Waypoints.LEFT_END_POSITION);

    return TrajectoryGenerator.generateSwerveTrajectory(false, waypoints, 3.5);
  }

  private Trajectory<TimedState<Pose2dWithCurvature>> getMidStartToMidEnd() {
    List<Pose2d> waypoints = new ArrayList<>();
    waypoints.add(Field.Waypoints.MID_START_OUT_POSITION);
    waypoints.add(Field.Waypoints.MID_END_POSITION);

    return TrajectoryGenerator.generateSwerveTrajectory(false, waypoints, 3.5);
  }

  private Trajectory<TimedState<Pose2dWithCurvature>> getRightStartToRightBall() {
    List<Pose2d> waypoints = new ArrayList<>();
    Timer.delay(0.1);
    waypoints.add(Field.Waypoints.RIGHT_START_POSITION);
    waypoints.add(Field.Waypoints.RIGHT_BALL_IN_POSITION);

    return TrajectoryGenerator.generateSwerveTrajectory(false, waypoints, 4.572);
  }

  private Trajectory<TimedState<Pose2dWithCurvature>> getRightBallToMidBall() {
    List<Pose2d> waypoints = new ArrayList<>();
    waypoints.add(Field.Waypoints.RIGHT_BALL_OUT_POSITION);
    waypoints.add(Field.Waypoints.MID_BALL_POSITION);

    return TrajectoryGenerator.generateSwerveTrajectory(false, waypoints, 4.572);
  }

  private Trajectory<TimedState<Pose2dWithCurvature>> getMidBallToHumanStationBall() {
    List<Pose2d> waypoints = new ArrayList<>();
    waypoints.add(Field.Waypoints.MID_BALL_POSITION);
    waypoints.add(Field.Waypoints.HUMAN_STATION_BALL_IN_POSITION);

    return TrajectoryGenerator.generateSwerveTrajectory(false, waypoints, 4.572);
  }

  private Trajectory<TimedState<Pose2dWithCurvature>> getHumanStationBallToHumanStationWait() {
    List<Pose2d> waypoints = new ArrayList<>();
    waypoints.add(Field.Waypoints.HUMAN_STATION_BALL_OUT_POSITION);
    waypoints.add(Field.Waypoints.HUMAN_STATION_WAIT_IN_POSITION);

    return TrajectoryGenerator.generateSwerveTrajectory(false, waypoints, 4.572);
  }

  private Trajectory<TimedState<Pose2dWithCurvature>> getHumanStationWaitToMidShoot() {
    List<Pose2d> waypoints = new ArrayList<>();
    waypoints.add(Field.Waypoints.HUMAN_STATION_WAIT_OUT_POSITION);
    waypoints.add(Field.Waypoints.RIGHT_END_SHOOT_POSITION);

    return TrajectoryGenerator.generateSwerveTrajectory(false, waypoints, 4.572);
  }
}
