package com.nextinnovation.team8214.auto.modes;

import com.nextinnovation.lib.auto.AutoModeEndedException;
import com.nextinnovation.lib.auto.modes.BaseAutoMode;
import com.nextinnovation.lib.geometry.Rotation2d;
import com.nextinnovation.team8214.Field;
import com.nextinnovation.team8214.auto.actions.ResetHeadingAction;
import com.nextinnovation.team8214.auto.actions.ResetPoseAction;
import com.nextinnovation.team8214.auto.actions.SetVoEnabledAction;

public class Silence extends BaseAutoMode {
  @Override
  protected void routine() throws AutoModeEndedException {
    System.out.println("Silence auto mode");
    runAction(new ResetPoseAction(Field.Waypoints.MID_START_POSITION));
    runAction(new ResetHeadingAction(new Rotation2d(180.0)));
    runAction(new SetVoEnabledAction(true));
  }
}
