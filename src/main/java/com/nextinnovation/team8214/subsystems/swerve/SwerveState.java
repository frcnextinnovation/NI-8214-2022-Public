package com.nextinnovation.team8214.subsystems.swerve;

public enum SwerveState {
  MANUAL("Manual"),
  DISABLE("Disable"),
  TRAJECTORY("Trajectory");

  public final String value;

  SwerveState(String name) {
    this.value = name;
  }
}
