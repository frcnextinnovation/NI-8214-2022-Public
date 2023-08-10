package com.nextinnovation.lib.utils;

/** Runnable class with reports all uncaught throws to CrashTracker */
public abstract class BaseCrashTrackingRunnable implements Runnable {

  @Override
  public final void run() {
    try {
      runCrashTracked();
    } catch (Throwable t) {
      CrashTracker.logThrowableCrash(t);
      throw t;
    }
  }

  public abstract void runCrashTracked();
}
