package com.nextinnovation.team8214.subsystems.intake;

public class IntakeConfig {
  // Control Inversion
  public static final boolean IS_INVERT = false;

  // Init Solenoid State
  public static final boolean IS_ELEVATE = true;

  // Speed Config
  public static final double INJECT_SPEED = 0.39;
  public static final double EJECT_SPEED = -0.3;

  // Communication Parameters
  public static final int STATUS_FRAME_PERIOD = 20;
}
