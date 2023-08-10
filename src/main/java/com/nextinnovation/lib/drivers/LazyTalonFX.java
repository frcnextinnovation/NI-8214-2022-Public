package com.nextinnovation.lib.drivers;

import com.ctre.phoenix.motorcontrol.ControlFrame;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

public class LazyTalonFX extends TalonFX {
  protected ControlMode previousControlMode = null;
  protected double previousSetpoint = Double.NaN;

  private static final int MAX_STATUS_FRAME_PERIOD = 1000;
  private static final int MAX_CONTROL_FRAME_PERIOD = 100;

  public LazyTalonFX(CanDeviceId deviceId) {
    super(deviceId.getId(), deviceId.getBus());
    configFactoryDefault();
  }

  public LazyTalonFX(int deviceId) {
    super(deviceId);
    configFactoryDefault();
  }

  public LazyTalonFX(int deviceId, String canBus) {
    super(deviceId, canBus);
    configFactoryDefault();
  }

  /**
   * Basic method to config status frame period
   *
   * @param period Delta period in ms
   * @param enableFeedback Is feedback control need
   * @param enableCurrentDebug Is current debug need
   * @param canTimeoutMs CAN time out in ms
   */
  public void configStatusFramePeriod(
      int period, boolean enableFeedback, boolean enableCurrentDebug, int canTimeoutMs) {
    period = Math.min(period, MAX_STATUS_FRAME_PERIOD);
    setCacheClear(canTimeoutMs);

    setCriticalStatusFramePeriod(period, canTimeoutMs);
    setFeedbackStatusFramePeriod(enableFeedback ? period : MAX_STATUS_FRAME_PERIOD, canTimeoutMs);
    setUselessStatusFramePeriod(canTimeoutMs);

    if (!enableCurrentDebug) {
      setStatusFramePeriod(
          StatusFrameEnhanced.Status_Brushless_Current, MAX_STATUS_FRAME_PERIOD, canTimeoutMs);
    }
  }

  /**
   * Basic method to config status frame period for slave
   *
   * @param canTimeoutMs CAN time out in ms
   */
  public void configSlaveStatusFramePeriod(int canTimeoutMs) {
    setStatusFramePeriod(StatusFrame.Status_1_General, MAX_STATUS_FRAME_PERIOD, canTimeoutMs);
    setStatusFramePeriod(StatusFrame.Status_2_Feedback0, MAX_STATUS_FRAME_PERIOD, canTimeoutMs);
    setControlFramePeriod(ControlFrame.Control_3_General, MAX_CONTROL_FRAME_PERIOD);
  }

  private void setCacheClear(int canTimeoutMs) {
    clearMotionProfileHasUnderrun(canTimeoutMs);
    clearMotionProfileTrajectories();
    clearStickyFaults();
  }

  private void setCriticalStatusFramePeriod(int period, int canTimeoutMs) {
    setStatusFramePeriod(StatusFrame.Status_1_General, period, canTimeoutMs);
    setControlFramePeriod(ControlFrame.Control_3_General, period);
  }

  private void setFeedbackStatusFramePeriod(int period, int canTimeoutMs) {
    setStatusFramePeriod(StatusFrame.Status_2_Feedback0, period, canTimeoutMs);
  }

  private void setUselessStatusFramePeriod(int canTimeoutMs) {
    changeMotionControlFramePeriod(MAX_STATUS_FRAME_PERIOD);
    setStatusFramePeriod(
        StatusFrameEnhanced.Status_3_Quadrature, MAX_STATUS_FRAME_PERIOD, canTimeoutMs);
    setStatusFramePeriod(StatusFrame.Status_4_AinTempVbat, MAX_STATUS_FRAME_PERIOD, canTimeoutMs);
    setStatusFramePeriod(StatusFrame.Status_6_Misc, MAX_STATUS_FRAME_PERIOD, canTimeoutMs);
    setStatusFramePeriod(StatusFrame.Status_7_CommStatus, MAX_STATUS_FRAME_PERIOD, canTimeoutMs);
    setStatusFramePeriod(
        StatusFrameEnhanced.Status_8_PulseWidth, MAX_STATUS_FRAME_PERIOD, canTimeoutMs);
    setStatusFramePeriod(StatusFrame.Status_9_MotProfBuffer, MAX_STATUS_FRAME_PERIOD, canTimeoutMs);
    setStatusFramePeriod(StatusFrame.Status_10_Targets, MAX_STATUS_FRAME_PERIOD, canTimeoutMs);
    setStatusFramePeriod(StatusFrame.Status_12_Feedback1, MAX_STATUS_FRAME_PERIOD, canTimeoutMs);
    setStatusFramePeriod(StatusFrame.Status_13_Base_PIDF0, MAX_STATUS_FRAME_PERIOD, canTimeoutMs);
    setStatusFramePeriod(StatusFrame.Status_14_Turn_PIDF1, MAX_STATUS_FRAME_PERIOD, canTimeoutMs);
    setStatusFramePeriod(
        StatusFrame.Status_15_FirmwareApiStatus, MAX_STATUS_FRAME_PERIOD, canTimeoutMs);
    setStatusFramePeriod(StatusFrame.Status_17_Targets1, MAX_STATUS_FRAME_PERIOD, canTimeoutMs);
  }

  @Override
  public void set(ControlMode controlMode, double setpoint) {
    if (setpoint != previousSetpoint || controlMode != previousControlMode) {
      previousControlMode = controlMode;
      previousSetpoint = setpoint;
      super.set(controlMode, setpoint);
    }
  }
}
