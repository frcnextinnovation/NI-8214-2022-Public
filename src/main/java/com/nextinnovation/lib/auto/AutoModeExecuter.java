package com.nextinnovation.lib.auto;

import com.nextinnovation.lib.utils.BaseCrashTrackingRunnable;
import com.nextinnovation.lib.auto.modes.BaseAutoMode;

/** This class selects, runs, and stops (if necessary) a specified autonomous mode. */
public class AutoModeExecuter {
  private static AutoModeExecuter instance = null;

  public static synchronized AutoModeExecuter getInstance() {
    if (instance == null) {
      instance = new AutoModeExecuter();
    }
    return instance;
  }

  private BaseAutoMode autoMode;
  private Thread thread = null;

  public void setAutoMode(final BaseAutoMode new_auto_mode) {
    autoMode = new_auto_mode;
    thread =
        new Thread(
            new BaseCrashTrackingRunnable() {
              @Override
              public void runCrashTracked() {
                if (autoMode != null) {
                  autoMode.run();
                }
              }
            });
  }

  public void start() {
    if (thread != null) {
      thread.start();
    }
  }

  public void stop() {
    if (autoMode != null) {
      autoMode.stop();
    }

    thread = null;
  }

  public void reset() {
    if (isStarted()) {
      stop();
    }

    autoMode = null;
  }

  public boolean isStarted() {
    return autoMode != null && autoMode.isActive() && thread != null && thread.isAlive();
  }
}
