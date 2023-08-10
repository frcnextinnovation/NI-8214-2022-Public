package com.nextinnovation.team8214.subsystems.intake;

public enum IntakeState {
  MANUAL("Manual"),
  DISABLE("Disable"),
  AUTO("Auto");

  public final String value;

  IntakeState(String name) {
    this.value = name;
  }
}
