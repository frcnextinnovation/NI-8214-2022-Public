package com.nextinnovation.team8214.subsystems.tunnel;

public class TunnelConfig {
  // Control Inversion
  public static final boolean IS_INVERT = false;

  // Speed Config
  public static final double TUNNEL_FEED_SPEED = 0.29;
  public static final double TUNNEL_PRELOAD_SPEED = 0.43;
  public static final double TUNNEL_INJECT_SPEED = 0.43;
  public static final double TUNNEL_EJECT_SPEED = -0.43;
  public static final double TUNNEL_REJECT_SPEED = 0.35;

  // Speed Config
  public static final double FEEDER_FEED_SPEED = 0.16;
  public static final double FEEDER_PRELOAD_SPEED = -0.3;
  public static final double FEEDER_INJECT_SPEED = -0.3;
  public static final double FEEDER_EJECT_SPEED = -0.3;
  public static final double FEEDER_REJECT_SPEED = 0.3;

  public static final double FEEDER_MAX_SPEED = 14400.0; // in encoder units / 0.1s

  // PID Parameters
  // Slot 0 is for velocity
  public static final double PID0_KP = 0.095;
  public static final double PID0_KI = 0.0;
  public static final double PID0_KD = 0.0;
  public static final double PID0_KF = 0.0;

  // Communication Parameters
  public static final int STATUS_FRAME_PERIOD = 20;
}
