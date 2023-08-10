package com.nextinnovation.lib.auto.actions;

/**
 * Action base class, an interface that describes an iterative action. It is run by an autonomous
 * action, called by the method runAction in AutoModeBase (or more commonly in autonomous modes that
 * extend AutoModeBase)
 */
public abstract class BaseAction {
  /**
   * Returns whether the code has finished execution. When implementing this interface, this method
   * is used by the runAction method every cycle to know when to stop running the action
   *
   * @return boolean
   */
  public abstract boolean isFinished();

  /** Run code once when the action is started, for set up */
  public abstract void start();

  /**
   * Called by runAction in AutoModeBase iteratively until isFinished returns true. Iterative logic
   * lives in this method
   */
  public abstract void update();

  /** Run code once when the action finishes, usually for clean up */
  public abstract void done();
}
