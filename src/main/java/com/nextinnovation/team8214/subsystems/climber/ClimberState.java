package com.nextinnovation.team8214.subsystems.climber;

public enum ClimberState {
  INACTIVE("Inactive"),
  INIT("Init"),
  MID_EXTEND_ARM("Mid_Extend"),
  MID_LIFT("Mid_Lift"),
  MID_2_HIGH_REACH("Mid_2_High_Reach"),
  MID_2_HIGH_EXTEND_ARM("Mid_2_High_Extend_Arm"),
  MID_2_HIGH_HOOK("Mid_2_High_Hook"),
  MID_2_HIGH_LIFT_WAIT("Mid_2_High_Lift_Wait"),
  MID_2_HIGH_LIFT("Mid_2_High_Lift"),
  HIGH_2_TRAVERSE_REACH("High_2_Traverse_Reach"),
  HIGH_2_TRAVERSE_EXTEND_ARM("High_2_Traverse_Extend_Arm"),
  HIGH_2_TRAVERSE_HOOK("High_2_Traverse_Hook"),
  HIGH_2_TRAVERSE_LIFT("High_2_Traverse_Lift"),
  IDLE("Idle"),
  TEST("Test"),
  DISABLE("Disable");

  public final String value;

  ClimberState(String name) {
    this.value = name;
  }
}
