package com.nextinnovation.team8214.devices;

import com.nextinnovation.lib.drivers.PicoColorSensor;
import edu.wpi.first.wpilibj.Timer;

public class DoubleColorSensor {
  private static DoubleColorSensor instance = null;

  public static synchronized DoubleColorSensor getInstance() {
    if (instance == null) {
      instance = new DoubleColorSensor();
    }
    return instance;
  }

  private final PicoColorSensor picoColorSensor;

  private DoubleColorSensor() {
    picoColorSensor = new PicoColorSensor();

    var initTime = Timer.getFPGATimestamp();
    while (!(isSensor0Connected())) {
      if (Timer.getFPGATimestamp() - initTime > 4.0) {
        System.out.print("Warning: Pico initialization timed out with ");
        System.out.print(Timer.getFPGATimestamp() - initTime);
        System.out.println(" seconds");
        break;
      }
      Timer.delay(0.01);
    }
  }

  public boolean isSensor0Connected() {
    return picoColorSensor.isSensor0Connected();
  }

  public boolean isSensor1Connected() {
    return picoColorSensor.isSensor1Connected();
  }

  public PicoColorSensor.RawColor getRawColor0() {
    return picoColorSensor.getRawColor0();
  }

  public PicoColorSensor.RawColor getRawColor1() {
    return picoColorSensor.getRawColor1();
  }

  public double getProximity0() {
    return picoColorSensor.getProximity0();
  }

  public double getProximity1() {
    return picoColorSensor.getProximity1();
  }

  public double getLastReadTimestampSeconds() {
    return picoColorSensor.getLastReadTimestampSeconds();
  }
}
