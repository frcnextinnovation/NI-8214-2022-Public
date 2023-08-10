package com.nextinnovation.team8214.devices.ahrs;

import com.nextinnovation.lib.geometry.Rotation2d;

public abstract class BaseAhrs {

  public boolean isReady() {
    return true;
  }

  public abstract Rotation2d getRobotHeading();

  public abstract Rotation2d getRobotAngularVelocity();

  public abstract double getRobotRawAngularVel();

  public abstract void setRobotHeading(Rotation2d heading);

  public abstract void resetRobotHeading();
}
