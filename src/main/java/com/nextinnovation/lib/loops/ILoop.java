package com.nextinnovation.lib.loops;

/**
 * Interface for loops, which are routine that run periodically in the robot code (such as periodic
 * gyroscope calibration, etc.)
 */
public interface ILoop {
  /**
   * Method runs once when the Looper start.
   *
   * @param timestamp Currant timestamp get from FPGA timer.
   */
  void onStart(double timestamp);

  /**
   * Method runs periodically when the Looper Loop.
   *
   * @param timestamp Currant timestamp get from FPGA timer.
   */
  void onLoop(double timestamp);

  /**
   * Method runs once when the Looper stop.
   *
   * @param timestamp Currant timestamp get from FPGA timer.
   */
  void onStop(double timestamp);
}
