package com.nextinnovation.lib.utils;

public class SimpleCounter {
  private final double loopPeriod;
  private double targetCountTime = 0;
  private double currantTime = 0.0;

  public SimpleCounter(double loop_period) {
    loopPeriod = loop_period;
  }

  public void setTargetCount(double target_count_time) {
    targetCountTime = target_count_time;
    reset();
  }

  public void update() {
    if (currantTime <= targetCountTime) {
      currantTime += loopPeriod;
    }
  }

  public void reset() {
    currantTime = 0.0;
  }

  public boolean isTimeReached() {
    return currantTime >= targetCountTime;
  }
}
