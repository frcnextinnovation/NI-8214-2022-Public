package com.nextinnovation.lib.loops;

/**
 * Interface for looper runs all the robot's loops. Loop objects are stored in a List object. They
 * are started when the robot powers up and stopped after the match.
 */
public interface ILooper {
  /**
   * Method to add loop for looper.
   *
   * @param loop The control loop from subsystems, drivers, etc.
   */
  void register(ILoop loop);
}
