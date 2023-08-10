package com.nextinnovation.lib.drivers;

import com.ctre.phoenix.sensors.PigeonIMU;
import com.ctre.phoenix.sensors.PigeonIMU_ControlFrame;
import com.ctre.phoenix.sensors.PigeonIMU_StatusFrame;
import com.ctre.phoenix.sensors.PigeonIMU.PigeonState;

public class Pigeon {
  private static final int MAX_STATUS_FRAME_PERIOD = 255;
  private final int canTimeoutMs;
  private final PigeonIMU pigeon;

  public Pigeon(int deviceId, int canTimeoutMs) {
    pigeon = new PigeonIMU(deviceId);
    this.canTimeoutMs = canTimeoutMs;
  }

  public boolean isReady() {
    return pigeon.getState() == PigeonState.Ready;
  }

  public double getRawYaw() {
    double[] ypr = new double[3];
    pigeon.getYawPitchRoll(ypr);
    return -ypr[0];
  }

  public double getFusedYaw() {
    return pigeon.getFusedHeading();
  }

  public double getPitch() {
    double[] ypr = new double[3];
    pigeon.getYawPitchRoll(ypr);
    return ypr[1];
  }

  public double getRoll() {
    double[] ypr = new double[3];
    pigeon.getYawPitchRoll(ypr);
    return ypr[2];
  }

  public double[] getRawYpr() {
    double[] ypr = new double[3];
    pigeon.getYawPitchRoll(ypr);
    return ypr;
  }

  public double getGyroX() {
    double[] xyz = new double[3];
    pigeon.getRawGyro(xyz);
    return xyz[0];
  }

  public double getGyroY() {
    double[] xyz = new double[3];
    pigeon.getRawGyro(xyz);
    return xyz[1];
  }

  public double getGyroZ() {
    double[] xyz = new double[3];
    pigeon.getRawGyro(xyz);
    return xyz[2];
  }

  public double getTempC() {
    return pigeon.getTemp();
  }

  public synchronized void setYaw(double yawDegrees) {
    pigeon.setFusedHeading(-yawDegrees, this.canTimeoutMs);
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
    pigeon.setStatusFramePeriod(PigeonIMU_StatusFrame.BiasedStatus_2_Gyro, period, canTimeoutMs);
    pigeon.setStatusFramePeriod(
        PigeonIMU_StatusFrame.CondStatus_6_SensorFusion, period, canTimeoutMs);
    pigeon.setStatusFramePeriod(
        PigeonIMU_StatusFrame.CondStatus_9_SixDeg_YPR, period, canTimeoutMs);
  }

  private synchronized void setNonRequiredStatusFramePeriod(int period, int canTimeoutMs) {
    pigeon.setStatusFramePeriod(PigeonIMU_StatusFrame.CondStatus_1_General, period, canTimeoutMs);
    pigeon.setStatusFramePeriod(
        PigeonIMU_StatusFrame.CondStatus_2_GeneralCompass, period, canTimeoutMs);
    pigeon.setStatusFramePeriod(
        PigeonIMU_StatusFrame.CondStatus_3_GeneralAccel, period, canTimeoutMs);
    pigeon.setStatusFramePeriod(PigeonIMU_StatusFrame.RawStatus_4_Mag, period, canTimeoutMs);
    pigeon.setStatusFramePeriod(PigeonIMU_StatusFrame.BiasedStatus_4_Mag, period, canTimeoutMs);
    pigeon.setStatusFramePeriod(PigeonIMU_StatusFrame.BiasedStatus_6_Accel, period, canTimeoutMs);
  }

  private synchronized void setControlFramePeriod(int period) {
    pigeon.setControlFramePeriod(PigeonIMU_ControlFrame.Control_1, period);
  }

  public synchronized void configTemperatureCompensation(boolean enable) {
    pigeon.setTemperatureCompensationDisable(!enable, this.canTimeoutMs);
  }
}
