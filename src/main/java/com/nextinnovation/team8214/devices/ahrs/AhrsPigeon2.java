package com.nextinnovation.team8214.devices.ahrs;

import com.nextinnovation.lib.drivers.Pigeon2;
import com.nextinnovation.lib.geometry.Rotation2d;
import com.nextinnovation.team8214.Config;
import com.nextinnovation.team8214.Ports;
import edu.wpi.first.wpilibj.Timer;

public class AhrsPigeon2 extends BaseAhrs {
  private static AhrsPigeon2 instance = null;

  public static synchronized BaseAhrs getInstance() {
    if (instance == null) {
      instance = new AhrsPigeon2();
    }
    return instance;
  }

  private static final Pigeon2 pigeon2 =
      new Pigeon2(Ports.Can.PIGEON_CHASSIS, Config.CAN_TIMEOUT_MS);
  private static Rotation2d referenceHeading = Rotation2d.identity();

  private AhrsPigeon2() {
    init();
  }

  private void init() {
    pigeon2.configStatusFramePeriod(
        Config.LOOPER_CONTROL_DELTA_TIME_MS, false, Config.CAN_TIMEOUT_MS);
    pigeon2.configTemperatureCompensation(true);
    setRobotHeading(Rotation2d.fromDegrees(Config.INIT_HEADING));
    Timer.delay(0.5);
  }

  @Override
  public boolean isReady() {
    return pigeon2.isReady();
  }

  @Override
  public Rotation2d getRobotHeading() {
    return Rotation2d.fromDegrees(pigeon2.getFusedYaw()).rotateBy(referenceHeading.inverse());
  }

  public Rotation2d getOrgRobotHeading() {
    return Rotation2d.fromDegrees(pigeon2.getFusedYaw());
  }

  @Override
  public Rotation2d getRobotAngularVelocity() {
    return Rotation2d.fromDegrees(pigeon2.getGyroZ());
  }

  public double getRobotRawAngularVel() {
    return pigeon2.getGyroZ();
  }

  @Override
  public synchronized void setRobotHeading(Rotation2d heading) {
    referenceHeading = getOrgRobotHeading().rotateBy(heading.inverse());
  }

  @Override
  public synchronized void resetRobotHeading() {
    setRobotHeading(Rotation2d.identity());
  }
}
