package com.nextinnovation.lib.drivers;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

public class LazyDoubleSolenoid extends DoubleSolenoid {
  private Value lastValue = Value.kOff;

  public LazyDoubleSolenoid(
      final PneumaticsModuleType moduleType, final int forwardChannel, final int reverseChannel) {
    super(moduleType, forwardChannel, reverseChannel);
  }

  @Override
  public void set(Value value) {
    if (value != lastValue) {
      lastValue = value;
      super.set(value);
    }
  }
}
