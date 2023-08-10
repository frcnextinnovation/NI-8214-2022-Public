package com.nextinnovation.lib.trajectory;

import com.nextinnovation.lib.geometry.Pose2d;
import com.nextinnovation.lib.geometry.Twist2d;

public interface IPathFollower {
  Twist2d steer(Pose2d current_pose);

  boolean isDone();
}
