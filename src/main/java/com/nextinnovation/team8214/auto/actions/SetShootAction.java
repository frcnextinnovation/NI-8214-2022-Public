package com.nextinnovation.team8214.auto.actions;

import com.nextinnovation.lib.auto.actions.BaseRunOnceAction;
import com.nextinnovation.team8214.managers.ControlSignalManager;

public class SetShootAction extends BaseRunOnceAction {
  private final boolean isRobotShoot;

  private final ControlSignalManager controlSignalManager;

  public SetShootAction(boolean is_robot_shoot) {
    controlSignalManager = ControlSignalManager.getInstance();
    isRobotShoot = is_robot_shoot;
  }

  @Override
  public void runOnce() {
    controlSignalManager.isTurretSetToSolidAngle = false;
    controlSignalManager.isRobotShoot = isRobotShoot;
  }
}
