package com.nextinnovation.lib.utils;

public class SimpleEdgeScanner {
  private boolean prevValue;
  private boolean currentValue;

  public SimpleEdgeScanner(boolean init_value) {
    currentValue = init_value;
    prevValue = currentValue;
  }

  public void update(boolean new_value) {
    prevValue = currentValue;
    currentValue = new_value;
  }

  public boolean isRisingEgdeScanned() {
    return !prevValue && currentValue;
  }

  public boolean isFallingEdgeScanned() {
    return prevValue && currentValue;
  }
}
