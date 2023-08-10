package com.nextinnovation.team8214.auto.actions;

import com.nextinnovation.lib.auto.actions.BaseRunOnceAction;
import com.nextinnovation.team8214.Odometry;

public class SetVoEnabledAction extends BaseRunOnceAction {
  private final Odometry odometry;
  private final boolean isVoEnabled;

  public SetVoEnabledAction(boolean is_vo_enabled) {
    odometry = Odometry.getInstance();
    isVoEnabled = is_vo_enabled;
  }

  @Override
  public void runOnce() {
    if (isVoEnabled) {
      odometry.enableVo();
    } else {
      odometry.disableVo();
    }
  }
}
