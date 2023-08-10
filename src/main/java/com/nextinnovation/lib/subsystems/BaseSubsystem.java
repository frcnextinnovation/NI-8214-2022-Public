package com.nextinnovation.lib.subsystems;

import com.nextinnovation.lib.loops.ILooper;
import edu.wpi.first.wpilibj.Timer;

/**
 * The Subsystem abstract class, which serves as a basic framework for all robot subsystems. Each
 * subsystem outputs commands to SmartDashboard, has a stop routine (for after each match), and a
 * routine to zero all sensors, which helps with calibration.
 *
 * <p>All Subsystems only have one instance (after all, one robot does not have two drivetrains),
 * and functions get the instance of the drivetrain and act accordingly. Subsystems are also a state
 * machine with a desired state and actual state; the robot code will try to match the two states
 * with actions. Each Subsystem also is responsible for instantiating all member components at the
 * start of the match.
 */
public abstract class BaseSubsystem {
  private double timeCostPerLoopSecs = 255.0;
  private double startTimestamp = 0.0;

  /** Start to count time cost */
  public void startCountTimeCost() {
    startTimestamp = Timer.getFPGATimestamp();
  }

  /** End to count time cost */
  public void endCountTimeCost() {
    timeCostPerLoopSecs = Timer.getFPGATimestamp() - startTimestamp;
  }

  /**
   * Get last time cost per loop.
   *
   * @return Get last time cost per loop, which is always used to apply a delay compensation.
   */
  public double getLastTimeCostPerLoop() {
    return timeCostPerLoopSecs;
  }

  /** Only method for subsystem read value from sensor circularly. */
  public void readPeriodicInputs() {}

  /** Only method for subsystem to set output to actuators(Motor, Solenoid...). */
  public void writePeriodicOutputs() {}

  /**
   * Where subsystem zero all sensors.
   *
   * @apiNote Always called when the subsystem init.
   */
  public void resetSensors() {}

  /**
   * Where subsystem make whole subsystem disabled.
   *
   * @apiNote Disable() should be safer than stop().
   */
  public abstract void disable();

  /**
   * Where subsystem register the control loop to enabled looper.
   *
   * @param enabledLooper The looper runs during the enabled mode.
   */
  public void registerEnabledLoops(ILooper enabledLooper) {}

  /**
   * Where subsystem push all message to Smart Dashboard.
   *
   * @apiNote At least push a running state message to make the user know your exist.
   * @apiNote Don't add synchronized in this method.
   */
  public abstract void logToSmartDashboard();

  /** Where subsystem do a self test in test mode. */
  public boolean selfTest() {
    return true;
  }
}
