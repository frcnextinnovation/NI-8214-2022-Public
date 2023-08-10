package com.nextinnovation.lib.drivers;

import com.ctre.phoenix.sensors.PigeonIMU_ControlFrame;
import com.ctre.phoenix.sensors.PigeonIMU_StatusFrame;

public class Pigeon2 {
  private static final int MAX_STATUS_FRAME_PERIOD = 255;
  private final int canTimeoutMs;
  private final com.ctre.phoenix.sensors.Pigeon2 pigeon2;

  public Pigeon2(CanDeviceId deviceId, int canTimeoutMs) {
    pigeon2 = new com.ctre.phoenix.sensors.Pigeon2(deviceId.getId(), deviceId.getBus());
    this.canTimeoutMs = canTimeoutMs;
  }

  public boolean isReady() {
    return true;
  }

  public double getRawYaw() {
    double[] ypr = new double[3];
    pigeon2.getYawPitchRoll(ypr);
    return -ypr[0];
  }

  public double getFusedYaw() {
    return pigeon2.getYaw();
  }

  public double getPitch() {
    return pigeon2.getPitch();
  }

  public double getRoll() {
    return pigeon2.getRoll();
  }

  public double[] getRawYpr() {
    double[] ypr = new double[3];
    pigeon2.getYawPitchRoll(ypr);
    return ypr;
  }

  public double getGyroX() {
    double[] xyz = new double[3];
    pigeon2.getRawGyro(xyz);
    return xyz[0];
  }

  public double getGyroY() {
    double[] xyz = new double[3];
    pigeon2.getRawGyro(xyz);
    return xyz[1];
  }

  public double getGyroZ() {
    double[] xyz = new double[3];
    pigeon2.getRawGyro(xyz);
    return xyz[2];
  }

  public double getTempC() {
    return pigeon2.getTemp();
  }

  public synchronized void setYaw(double yawDegrees) {
    pigeon2.setYaw(-yawDegrees, this.canTimeoutMs);
  }

  /**
   * Basic method to config status frame period
   *
   * @param period Delta period in ms
   * @param enableControl Is control need.
   * @param canTimeoutMs CAN time out in ms
   */
  public synchronized void configStatusFramePeriod(
      int period, boolean enableControl, int canTimeoutMs) {
    period = Math.min(period, MAX_STATUS_FRAME_PERIOD);
    setRequiredStatusFramePeriod(period, canTimeoutMs);
    setNonRequiredStatusFramePeriod(MAX_STATUS_FRAME_PERIOD, canTimeoutMs);
    setControlFramePeriod(enableControl ? period : MAX_STATUS_FRAME_PERIOD);
  }

  private synchronized void setRequiredStatusFramePeriod(int period, int canTimeoutMs) {
    pigeon2.setStatusFramePeriod(PigeonIMU_StatusFrame.BiasedStatus_2_Gyro, period, canTimeoutMs);
    pigeon2.setStatusFramePeriod(
        PigeonIMU_StatusFrame.CondStatus_6_SensorFusion, period, canTimeoutMs);
    pigeon2.setStatusFramePeriod(
        PigeonIMU_StatusFrame.CondStatus_9_SixDeg_YPR, period, canTimeoutMs);
  }

  private synchronized void setNonRequiredStatusFramePeriod(int period, int canTimeoutMs) {
    pigeon2.setStatusFramePeriod(PigeonIMU_StatusFrame.CondStatus_1_General, period, canTimeoutMs);
    pigeon2.setStatusFramePeriod(
        PigeonIMU_StatusFrame.CondStatus_2_GeneralCompass, period, canTimeoutMs);
    pigeon2.setStatusFramePeriod(
        PigeonIMU_StatusFrame.CondStatus_3_GeneralAccel, period, canTimeoutMs);
    pigeon2.setStatusFramePeriod(PigeonIMU_StatusFrame.RawStatus_4_Mag, period, canTimeoutMs);
    pigeon2.setStatusFramePeriod(PigeonIMU_StatusFrame.BiasedStatus_4_Mag, period, canTimeoutMs);
    pigeon2.setStatusFramePeriod(PigeonIMU_StatusFrame.BiasedStatus_6_Accel, period, canTimeoutMs);
  }

  private synchronized void setControlFramePeriod(int period) {
    pigeon2.setControlFramePeriod(PigeonIMU_ControlFrame.Control_1, period);
  }

  public synchronized void configTemperatureCompensation(boolean enable) {
    pigeon2.configDisableTemperatureCompensation(!enable, this.canTimeoutMs);
  }
}
