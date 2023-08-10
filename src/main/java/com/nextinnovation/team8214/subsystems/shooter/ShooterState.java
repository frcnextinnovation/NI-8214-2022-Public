package com.nextinnovation.team8214.subsystems.shooter;

public enum ShooterState {
  MANUAL("Manual"),
  DISABLE("Disable"),
  AUTO("Auto");

  public final String value;

  ShooterState(String name) {
    this.value = name;
  }
}
