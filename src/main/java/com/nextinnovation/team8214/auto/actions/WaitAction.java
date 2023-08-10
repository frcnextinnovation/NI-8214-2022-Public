package com.nextinnovation.team8214.auto.actions;

import com.nextinnovation.lib.auto.actions.BaseAction;
import edu.wpi.first.wpilibj.Timer;

/**
 * Action to wait for a given amount of time To use this Action, call runAction(new
 * WaitAction(timeToWait)
 */
public class WaitAction extends BaseAction {
  private final double timeToWait;
  private double startTime;

  public WaitAction(double time_to_wait) {
    timeToWait = time_to_wait;
  }

  @Override
  public void start() {
    startTime = Timer.getFPGATimestamp();
    System.out.println("Start to wait for " + timeToWait + " seconds!");
  }

  @Override
  public void update() {}

  @Override
  public boolean isFinished() {
    return Timer.getFPGATimestamp() - startTime >= timeToWait;
  }

  @Override
  public void done() {
    System.out.println("Wait finished!");
  }
}
