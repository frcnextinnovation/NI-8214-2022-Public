package com.nextinnovation.team8214.subsystems.vision;

public enum VisionState {
  ENABLE("Enable"),
  OFF("Off"),
  BLINK("Blink");

  public final String value;

  VisionState(String name) {
    this.value = name;
  }
}
