package com.nextinnovation.team8214.subsystems.tunnel;

public enum TunnelState {
  MANUAL("Manual"),
  DISABLE("Disable"),
  AUTO("Auto");

  public final String value;

  TunnelState(String name) {
    this.value = name;
  }
}
