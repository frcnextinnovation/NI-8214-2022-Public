package com.nextinnovation.team8214.auto.actions;

import com.nextinnovation.lib.auto.actions.BaseRunOnceAction;
import com.nextinnovation.team8214.devices.DoubleColorSensor;
import com.nextinnovation.team8214.subsystems.tunnel.Tunnel;

public class UpdateSensorRbOffsetAction extends BaseRunOnceAction {
  private final DoubleColorSensor doubleColorSensor;
  private final Tunnel tunnel;

  public UpdateSensorRbOffsetAction() {
    doubleColorSensor = DoubleColorSensor.getInstance();
    tunnel = Tunnel.getInstance();
  }

  @Override
  public void runOnce() {
    if (doubleColorSensor.isSensor0Connected()) {
      tunnel.updateRedBlueLinearOffset();
    }
  }
}
