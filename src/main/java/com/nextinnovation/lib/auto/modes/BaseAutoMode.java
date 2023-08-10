package com.nextinnovation.lib.auto.modes;

import com.nextinnovation.lib.auto.AutoModeEndedException;
import com.nextinnovation.lib.auto.actions.BaseAction;

/**
 * An abstract class that is the basis of the robot's autonomous routines. This is implemented in
 * auto modes (which are routines that do actions).
 */
public abstract class BaseAutoMode {
  protected double update_rate = 1.0 / 50.0;
  protected boolean isActive = false;

  protected abstract void routine() throws AutoModeEndedException;

  public void run() {
    isActive = true;
    try {
      routine();
    } catch (AutoModeEndedException e) {
      System.out.println("Auto mode done, ended early");
      return;
    }

    done();
  }

  public void done() {
    System.out.println("Auto mode done");
  }

  public void stop() {
    isActive = false;
  }

  public boolean isActive() {
    return isActive;
  }

  public boolean isActiveWithThrow() throws AutoModeEndedException {
    if (!isActive()) {
      throw new AutoModeEndedException();
    }

    return isActive();
  }

  public void runAction(final BaseAction action) throws AutoModeEndedException {
    isActiveWithThrow();
    long waitTime = (long) (update_rate * 1000.0);

    action.start();

    while (isActiveWithThrow() && !action.isFinished()) {
      action.update();

      try {
        Thread.sleep(waitTime);
      } catch (InterruptedException e) {
        e.printStackTrace();
      }
    }

    action.done();
  }
}
