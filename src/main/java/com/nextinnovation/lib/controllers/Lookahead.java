package com.nextinnovation.lib.controllers;

public class Lookahead {
  public final double minDistance;
  public final double maxDistance;
  public final double minSpeed;
  public final double maxSpeed;
  protected final double deltaDistance;
  protected final double deltaSpeed;

  public Lookahead(double min_distance, double max_distance, double min_speed, double max_speed) {
    this.minDistance = min_distance;
    this.maxDistance = max_distance;
    this.minSpeed = min_speed;
    this.maxSpeed = max_speed;
    deltaDistance = max_distance - min_distance;
    deltaSpeed = max_speed - min_speed;
  }

  public double getLookaheadForSpeed(double speed) {
    double lookahead = deltaDistance * (speed - minSpeed) / deltaSpeed + minDistance;
    return Double.isNaN(lookahead)
        ? minDistance
        : Math.max(minDistance, Math.min(maxDistance, lookahead));
  }
}
