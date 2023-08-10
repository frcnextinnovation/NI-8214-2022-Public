package com.nextinnovation.lib.drivers;

import edu.wpi.first.wpilibj.DutyCycleEncoder;

public class AS5048A extends DutyCycleEncoder {
  public AS5048A(int channel, boolean is_counter_clockwise) {
    super(channel);
    setDistancePerRotation(360.0 * (is_counter_clockwise ? 1.0 : -1.0));
  }

  public double getPosition() {
    var angle = getDistance() % 360.0;

    if (angle < 0.0) {
      angle += 360.0;
    }

    return angle;
  }
}
