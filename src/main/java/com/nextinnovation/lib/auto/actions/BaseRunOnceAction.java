package com.nextinnovation.lib.auto.actions;

public abstract class BaseRunOnceAction extends BaseAction {
  protected boolean isRunOnceActionFinished = false;

  @Override
  public boolean isFinished() {
    return isRunOnceActionFinished;
  }

  @Override
  public void update() {
    if (!isRunOnceActionFinished) {
      runOnce();
      isRunOnceActionFinished = true;
    }
  }

  @Override
  public void done() {}

  @Override
  public void start() {}

  public abstract void runOnce();
}
