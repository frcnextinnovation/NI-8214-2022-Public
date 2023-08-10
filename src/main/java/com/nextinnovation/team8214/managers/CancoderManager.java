package com.nextinnovation.team8214.managers;

import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderStatusFrame;
import com.nextinnovation.lib.drivers.CanDeviceId;
import com.nextinnovation.team8214.Ports;
import edu.wpi.first.wpilibj.Timer;

public class CancoderManager {
  /***********************************************************************************************
   * Singleton *
   ***********************************************************************************************/
  private static CancoderManager instance = null;

  public static synchronized CancoderManager getInstance() {
    if (instance == null) {
      instance = new CancoderManager();
    }
    return instance;
  }

  /***********************************************************************************************
   * Init & Config *
   ***********************************************************************************************/
  private final CANCoder frontLeft;

  private final CANCoder rearLeft;
  private final CANCoder rearRight;
  private final CANCoder frontRight;

  private CancoderManager() {
    frontLeft = build(Ports.Can.FRONT_LEFT_ROTATION_SENSOR);
    rearLeft = build(Ports.Can.REAR_LEFT_ROTATION_SENSOR);
    rearRight = build(Ports.Can.REAR_RIGHT_ROTATION_SENSOR);
    frontRight = build(Ports.Can.FRONT_RIGHT_ROTATION_SENSOR);
  }

  private CANCoder build(CanDeviceId id) {
    var cancoder = new CANCoder(id.getId(), id.getBus());
    cancoder.setStatusFramePeriod(CANCoderStatusFrame.SensorData, 50);
    Timer.delay(0.2);
    return cancoder;
  }

  /************************************************************************************************
   * Getter & Setter *
   ************************************************************************************************/
  public CANCoder getFrontLeft() {
    return frontLeft;
  }

  public CANCoder getRearLeft() {
    return rearLeft;
  }

  public CANCoder getRearRight() {
    return rearRight;
  }

  public CANCoder getFrontRight() {
    return frontRight;
  }
}
