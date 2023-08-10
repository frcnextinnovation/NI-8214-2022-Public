package com.nextinnovation.lib.subsystems;

import com.nextinnovation.lib.loops.ILoop;
import com.nextinnovation.lib.loops.ILooper;
import com.nextinnovation.lib.loops.Looper;

import java.util.List;

/**
 * Group Used to reset, start, stop, and update all subsystems at once, which can decrease duplicate
 * code.
 */
public class SubsystemGroup extends BaseSubsystem {
  private final List<BaseSubsystem> subsystems;

  /**
   * Only constructor for SubsystemGroup.
   *
   * <p>Please ensure all the subsystems for register has init correctly.
   *
   * @param subsystems The subsystems will be registered for looper.
   */
  public SubsystemGroup(List<BaseSubsystem> subsystems) {
    this.subsystems = subsystems;
  }

  /** Only method for subsystems read value from sensor circularly. */
  @Override
  public void readPeriodicInputs() {
    subsystems.forEach(BaseSubsystem::readPeriodicInputs);
  }

  /** Only method for subsystems to set output to actuators(Motor, Solenoid...). */
  @Override
  public void writePeriodicOutputs() {
    subsystems.forEach(BaseSubsystem::writePeriodicOutputs);
  }

  /**
   * Where subsystems zero all sensors.
   *
   * <p>Always called when the subsystem init.
   */
  @Override
  public void resetSensors() {
    subsystems.forEach(BaseSubsystem::resetSensors);
  }

  /**
   * Where subsystem group make all subsystems disabled.
   *
   * <p>Disable() should be safer than stop().
   */
  @Override
  public void disable() {
    subsystems.forEach(BaseSubsystem::disable);
  }

  /**
   * Where register the control loop of all subsystem to enabled looper.
   *
   * @param enabledLooper The looper runs during the enabled mode.
   * @apiNote This method uses a new looper & loop to package the control loop of every subsystem.
   *     You can notice that we already write the readPeriodicInputs() and writePeriodicOutputs() in
   *     the onLoop() of thr new loop, which aims to decrease the use of duplicate code in every
   *     subsystem class.
   */
  @Override
  public void registerEnabledLoops(ILooper enabledLooper) {
    // ! This will ensure all control loops run with same timestamp everytime.
    Looper subsystemEnabledLooper = new Looper("Temp", 255.0);
    subsystems.forEach(subsystem -> subsystem.registerEnabledLoops(subsystemEnabledLooper));
    ILoop loop =
        new ILoop() {
          @Override
          public void onStart(double timestamp) {
            subsystemEnabledLooper.allOnStart(timestamp);
          }

          @Override
          public void onLoop(double timestamp) {
            for (BaseSubsystem subsystem : subsystems) {
//              subsystem.startCountTimeCost();
              subsystem.readPeriodicInputs();
            }
            subsystemEnabledLooper.allOnLoop(timestamp);
            for (BaseSubsystem subsystem : subsystems) {
              subsystem.writePeriodicOutputs();
//              subsystem.endCountTimeCost();
            }
          }

          @Override
          public void onStop(double timestamp) {
            subsystemEnabledLooper.allOnStop(timestamp);
          }
        };
    enabledLooper.register(loop);
  }

  /** Where subsystems push all message to Smart Dashboard. */
  @Override
  public void logToSmartDashboard() {
    subsystems.forEach(BaseSubsystem::logToSmartDashboard);
  }

  /** Where subsystems do a self test in test mode. */
  @Override
  public boolean selfTest() {
    boolean passesTest = true;
    for (BaseSubsystem subsystem : subsystems) {
      passesTest &= subsystem.selfTest();
    }
    return passesTest;
  }
}
