package com.nextinnovation.lib.loops;

import com.nextinnovation.lib.utils.BaseCrashTrackingRunnable;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.ArrayList;
import java.util.List;

/**
 * This code runs all the robot's loops. Loop objects are stored in a List object. They are started
 * when the robot powers up and stopped after the match.
 */
public class Looper implements ILooper {
  private final String looperName; // The name of looper.
  private final double periodSec; // The loop delta time in seconds.
  private final Notifier notifier; // The only object will start a new thread to run loops.
  private final List<ILoop> loops; // List for all control loops.
  private final Object taskRunningLock = new Object();
  private boolean isRunning; // The flag that control the running state.
  private double timestamp = 0;
  private double deltaTime = 0; // The variable that store the real loop delta time.

  /** Constructor for Looper. Running state is init as false. */
  public Looper(String looper_name, double period_sec) {
    looperName = looper_name;
    periodSec = period_sec;
    // The runnable object runs all loops and catch & throw all exceptions. All
    // loops run with same timestamp everytime.
    BaseCrashTrackingRunnable crashTrackingRunnable =
        new BaseCrashTrackingRunnable() {
          @Override
          public void runCrashTracked() {
            synchronized (taskRunningLock) {
              if (isRunning) {
                double currentTimestamp = Timer.getFPGATimestamp();
                allOnLoop(currentTimestamp);
                deltaTime = currentTimestamp - timestamp;
                timestamp = currentTimestamp;
              }
            }
          }
        };
    notifier = new Notifier(crashTrackingRunnable);
    isRunning = false;
    loops = new ArrayList<>();
  }

  /**
   * Get real delta time of looper.
   *
   * @return real delta time.
   */
  public double getDeltaTime() {
    return deltaTime;
  }

  /**
   * Method to add loop for looper.
   *
   * @param loop The control loop from subsystems, drivers, etc.
   *     <p>! Avoid fancy use. Don't add loops during the enabled mode.
   */
  @Override
  public synchronized void register(ILoop loop) {
    synchronized (taskRunningLock) {
      loops.add(loop);
    }
  }

  /** Start the looper, always called for once during init mode. */
  public synchronized void start() {
    if (!isRunning) {
      synchronized (taskRunningLock) {
        allOnStart(Timer.getFPGATimestamp());
        isRunning = true;
      }
      notifier.startPeriodic(periodSec);
    }
  }

  /** Restart the looper, always called for once when mode switch. */
  public synchronized void restart() {
    if (isRunning) {
      stop();
    }
    start();
  }

  /** Stop the looper, always called for once during disabled mode. */
  public synchronized void stop() {
    if (isRunning) {
      notifier.stop();
      synchronized (taskRunningLock) {
        isRunning = false;
        allOnStop(Timer.getFPGATimestamp());
      }
    }
  }

  /**
   * Run onStart method for all loops registered.
   *
   * @param timestamp Currant timestamp from FPGA timer.
   */
  public void allOnStart(double timestamp) {
    for (ILoop loop : loops) {
      loop.onStart(timestamp);
    }
  }

  /**
   * Run onLoop method for all loops registered.
   *
   * @param timestamp Currant timestamp from FPGA timer.
   */
  public void allOnLoop(double timestamp) {
    for (ILoop loop : loops) {
      loop.onLoop(timestamp);
    }
  }

  /**
   * Run onStop method for all loops registered.
   *
   * @param timestamp Currant timestamp from FPGA timer.
   */
  public void allOnStop(double timestamp) {
    for (ILoop loop : loops) {
      loop.onStop(timestamp);
    }
  }

  /**
   * Where looper push all message to Smart Dashboard.
   *
   * <p>At least push a running state message to make the user know your exist.
   */
  public void logToSmartDashboard() {
    SmartDashboard.putNumber(looperName + " Looper Delta Time", deltaTime);
  }
}
