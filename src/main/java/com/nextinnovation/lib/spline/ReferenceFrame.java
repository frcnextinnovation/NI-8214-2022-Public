package com.nextinnovation.lib.spline;

import com.nextinnovation.lib.geometry.Pose2d;

public enum ReferenceFrame {
  NONE(new Pose2d()), // TODO: figure out these values
  START(new Pose2d()), // TODO: figure out these values
  SCALE(new Pose2d()); // TODO: figure out these values

  private Pose2d mReferenceFrame;

  ReferenceFrame(Pose2d mReferenceFrame) {
    this.mReferenceFrame = mReferenceFrame;
  }

  public Pose2d getReferenceFrame() {
    return this.mReferenceFrame;
  }
}
