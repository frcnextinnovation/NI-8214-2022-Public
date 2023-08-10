package com.nextinnovation.team8214.auto.actions;

import com.nextinnovation.lib.auto.actions.BaseRunOnceAction;
import com.nextinnovation.team8214.managers.ControlSignalManager;

public class SetInjectAction extends BaseRunOnceAction {
  private final boolean isRobotInject;

  private final ControlSignalManager controlSignalManager;

  public SetInjectAction(boolean is_robot_inject) {
    controlSignalManager = ControlSignalManager.getInstance();
    isRobotInject = is_robot_inject;
  }

  @Override
  public void runOnce() {
    controlSignalManager.isRobotEject = false;
    controlSignalManager.isRobotInject = isRobotInject;
  }
}
