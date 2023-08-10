package com.nextinnovation.team8214.auto.actions;

import com.nextinnovation.lib.auto.actions.BaseRunOnceAction;
import com.nextinnovation.team8214.subsystems.tunnel.Tunnel;

public class SetEnableColorSortingAction extends BaseRunOnceAction {
  private final Tunnel tunnel;
  private final boolean isColorSortingEnabled;

  public SetEnableColorSortingAction(boolean is_color_sorting_enabled) {
    tunnel = Tunnel.getInstance();
    isColorSortingEnabled = is_color_sorting_enabled;
  }

  @Override
  public void runOnce() {
    if (isColorSortingEnabled) {
      tunnel.enableColorReject();
    } else {
      tunnel.disableColorReject();
    }
  }
}
