package com.nextinnovation.team8214.auto.actions;

import com.nextinnovation.lib.auto.actions.BaseRunOnceAction;
import com.nextinnovation.team8214.managers.ControlSignalManager;

public class SetFlywheelUnlockAction extends BaseRunOnceAction {
  private final boolean isFlywheelUnlock;

  private final ControlSignalManager controlSignalManager;

  public SetFlywheelUnlockAction(boolean is_flywheel_unlock) {
    controlSignalManager = ControlSignalManager.getInstance();
    isFlywheelUnlock = is_flywheel_unlock;
  }

  @Override
  public void runOnce() {
    controlSignalManager.isTurretSetToSolidAngle = false;
    controlSignalManager.isFlyWheelUnlock = isFlywheelUnlock;
  }
}
