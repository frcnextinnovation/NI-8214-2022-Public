package com.nextinnovation.team8214.subsystems.climber;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.nextinnovation.lib.drivers.LazyDoubleSolenoid;
import com.nextinnovation.lib.drivers.LazyTalonFX;
import com.nextinnovation.lib.drivers.TalonUtil;
import com.nextinnovation.lib.loops.ILoop;
import com.nextinnovation.lib.loops.ILooper;
import com.nextinnovation.lib.subsystems.BaseSubsystem;
import com.nextinnovation.lib.utils.Util;
import com.nextinnovation.team8214.Config;
import com.nextinnovation.team8214.Ports;
import com.nextinnovation.team8214.managers.ControlSignalManager;
import com.nextinnovation.team8214.subsystems.shooter.Turret;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;

public class Climber extends BaseSubsystem {
  /***********************************************************************************************
   * Control Loop *
   ***********************************************************************************************/
  @Override
  public void registerEnabledLoops(ILooper enabledLooper) {
    ILoop loop =
        new ILoop() {
          @Override
          public void onStart(double timestamp) {
            setState(ClimberState.INACTIVE);
          }

          @Override
          public void onLoop(double timestamp) {
            synchronized (stateLock) {
              var controlSignalManager = ControlSignalManager.getInstance();
              var turret = Turret.getInstance();

              switch (climberState) {
                case INACTIVE:
                  setSolenoidReach();
                  setMotorPositionKeepToZero();
                  if (controlSignalManager.isInClimbMode && turret.isYawOnTargets(true)) {
                    setState(ClimberState.INIT);
                  }
                  break;

                case INIT:
                  if (!controlSignalManager.isInClimbMode) {
                    setState(ClimberState.INACTIVE);
                    return;
                  } else if (controlSignalManager.isClimberMove) {
                    setState(ClimberState.MID_EXTEND_ARM);
                    return;
                  }
                  break;

                case MID_EXTEND_ARM:
                  setSolenoidHook();
                  if (setMotorExtend()) {
                    switchStateToIdle();
                  }
                  break;

                case MID_LIFT:
                  if (setMotorContract()) {
                    setState(ClimberState.MID_2_HIGH_REACH);
                  }
                  break;

                case MID_2_HIGH_REACH:
                  setSolenoidReach();
                  switchStateToIdle();
                  break;

                case MID_2_HIGH_EXTEND_ARM:
                  if (setMotorExtend()) {
                    switchStateToIdle();
                  }
                  break;

                case MID_2_HIGH_HOOK:
                  setSolenoidHook();
                  switchStateToIdle();
                  break;

                case MID_2_HIGH_LIFT_WAIT:
                  if (setMotorContractWait()) {
                    switchStateToIdle();
                  }
                  break;

                case MID_2_HIGH_LIFT:
                  if (setMotorContractLower()) {
                    setState(ClimberState.HIGH_2_TRAVERSE_REACH);
                  }
                  break;

                case HIGH_2_TRAVERSE_REACH:
                  setSolenoidReach();
                  switchStateToIdle();
                  break;

                case HIGH_2_TRAVERSE_EXTEND_ARM:
                  if (setMotorExtend()) {
                    switchStateToIdle();
                  }
                  break;

                case HIGH_2_TRAVERSE_HOOK:
                  setSolenoidHook();
                  switchStateToIdle();
                  break;

                case HIGH_2_TRAVERSE_LIFT:
                  if (setMotorLand()) {
                    switchStateToIdle();
                  }
                  break;

                case TEST:
                  if (controlSignalManager.isManualClimberArmToggle) {
                    setSolenoidToggle();
                  }

                  if (controlSignalManager.isManualClimberArmStretch) {
                    setMotorOpenLoop(0.6);
                  } else if (controlSignalManager.isManualClimberArmContract) {
                    setMotorOpenLoop(-0.6);
                  } else {
                    setMotorOpenLoop(0.0);
                  }
                  break;

                case IDLE:
                  if (controlSignalManager.isClimberMove) {
                    switchStateToNextState();
                  }

                  if (controlSignalManager.isManualClimberArmStretch) {
                    setMotorOpenLoop(0.2);
                  } else if (controlSignalManager.isManualClimberArmContract) {
                    setMotorOpenLoop(-0.2);
                  } else {
                    setMotorOpenLoop(0.0);
                  }
                  break;

                case DISABLE:
                  setMotorOpenLoop(0.0);
                  break;
              }
            }
          }

          @Override
          public void onStop(double timestamp) {
            setState(ClimberState.DISABLE);
          }
        };
    enabledLooper.register(loop);
  }

  /***********************************************************************************************
   * Periodic IO *
   ***********************************************************************************************/
  private static class PeriodicInput {
    public double motorLeftPosition = 0.0;
    public double motorRightPosition = 0.0;
  }

  @Override
  public void readPeriodicInputs() {
    periodicInput.motorLeftPosition = motorLeft.getSelectedSensorPosition();
    periodicInput.motorRightPosition = motorRight.getSelectedSensorPosition();
  }

  private static class PeriodicOutput {
    public ControlMode motorControlMode = ControlMode.PercentOutput;
    public double motorOutput = 0.0;

    public boolean isSolenoidReach = true;
  }

  @Override
  public void writePeriodicOutputs() {
    motorLeft.set(periodicOutput.motorControlMode, periodicOutput.motorOutput);
    motorRight.set(periodicOutput.motorControlMode, periodicOutput.motorOutput);

    hook.set(
        periodicOutput.isSolenoidReach
            ? DoubleSolenoid.Value.kReverse
            : DoubleSolenoid.Value.kForward);
  }

  /***********************************************************************************************
   * Subsystem States *
   ***********************************************************************************************/
  private ClimberState climberState;

  private ClimberState lastClimberState = ClimberState.IDLE;
  private final Object stateLock = new Object();

  public synchronized void setState(ClimberState new_state) {
    synchronized (stateLock) {
      climberState = new_state;
    }
  }

  public synchronized void switchStateToIdle() {
    lastClimberState = climberState;
    setState(ClimberState.IDLE);
  }

  public synchronized void switchStateToNextState() {
    switch (lastClimberState) {
      case MID_EXTEND_ARM:
        setState(ClimberState.MID_LIFT);
        break;

      case MID_LIFT:
        setState(ClimberState.MID_2_HIGH_REACH);
        break;

      case MID_2_HIGH_REACH:
        setState(ClimberState.MID_2_HIGH_EXTEND_ARM);
        break;

      case MID_2_HIGH_EXTEND_ARM:
        setState(ClimberState.MID_2_HIGH_HOOK);
        break;

      case MID_2_HIGH_HOOK:
        setState(ClimberState.MID_2_HIGH_LIFT_WAIT);
        break;

      case MID_2_HIGH_LIFT_WAIT:
        setState(ClimberState.MID_2_HIGH_LIFT);
        break;

      case MID_2_HIGH_LIFT:
        setState(ClimberState.HIGH_2_TRAVERSE_REACH);
        break;

      case HIGH_2_TRAVERSE_REACH:
        setState(ClimberState.HIGH_2_TRAVERSE_EXTEND_ARM);
        break;

      case HIGH_2_TRAVERSE_EXTEND_ARM:
        setState(ClimberState.HIGH_2_TRAVERSE_HOOK);
        break;

      case HIGH_2_TRAVERSE_HOOK:
        setState(ClimberState.HIGH_2_TRAVERSE_LIFT);
        break;
    }
  }

  /***********************************************************************************************
   * Singleton *
   ***********************************************************************************************/
  private static Climber instance = null;

  public static Climber getInstance() {
    if (instance == null) {
      instance = new Climber();
    }
    return instance;
  }

  /***********************************************************************************************
   * Init & Config *
   ***********************************************************************************************/
  private final LazyTalonFX motorLeft = new LazyTalonFX(Ports.Can.CLIMBER_MOTOR_LEFT);

  private final LazyTalonFX motorRight = new LazyTalonFX(Ports.Can.CLIMBER_MOTOR_RIGHT);

  private final PeriodicInput periodicInput = new PeriodicInput();
  private final PeriodicOutput periodicOutput = new PeriodicOutput();

  private final PIDController pid =
      new PIDController(ClimberConfig.PID0_KP, ClimberConfig.PID0_KI, ClimberConfig.PID0_KD, 0.01);

  private final LazyDoubleSolenoid hook =
      new LazyDoubleSolenoid(
          Ports.Pcm.PCM_TYPE, Ports.Pcm.CLIMBER_ELEVATE_PULL, Ports.Pcm.CLIMBER_ELEVATE_PUSH);

  private Climber() {
    pid.setTolerance(ClimberConfig.PID_TOLERANCE);

    configSmartDashboard();
    configMotors();
    setState(ClimberState.INACTIVE);
  }

  private synchronized void configMotors() {
    // Left
    motorLeft.configStatusFramePeriod(
        ClimberConfig.STATUS_FRAME_PERIOD, true, false, Config.CAN_TIMEOUT_MS);
    motorLeft.setInverted(ClimberConfig.IS_INVERT);
    motorLeft.setNeutralMode(NeutralMode.Brake);
    motorLeft.enableVoltageCompensation(true);
    motorLeft.configVoltageCompSaturation(11.0);
    TalonUtil.checkError(
        motorLeft.configSupplyCurrentLimit(
            new SupplyCurrentLimitConfiguration(true, 38.0, 38.0, 0.0), Config.CAN_TIMEOUT_MS),
        "Climber Left: Can't set supply current limit!");
    TalonUtil.checkError(
        motorLeft.setSelectedSensorPosition(0.0, 0, Config.CAN_INSTANT_TIMEOUT_MS),
        "Climber Left: Can't homing!");

    // Right
    motorRight.configStatusFramePeriod(
        ClimberConfig.STATUS_FRAME_PERIOD, true, false, Config.CAN_TIMEOUT_MS);
    motorRight.setInverted(!ClimberConfig.IS_INVERT);
    motorRight.setNeutralMode(NeutralMode.Brake);
    motorRight.enableVoltageCompensation(true);
    motorRight.configVoltageCompSaturation(11.0);
    TalonUtil.checkError(
        motorRight.configSupplyCurrentLimit(
            new SupplyCurrentLimitConfiguration(true, 38.0, 38.0, 0.0), Config.CAN_TIMEOUT_MS),
        "Climber Right: Can't set supply current limit!");
    TalonUtil.checkError(
        motorRight.setSelectedSensorPosition(0.0, 0, Config.CAN_INSTANT_TIMEOUT_MS),
        "Climber Right: Can't homing!");
  }

  /************************************************************************************************
   * Getter & Setter *
   ************************************************************************************************/
  public synchronized void setMotorOpenLoop(double output) {
    periodicOutput.motorControlMode = ControlMode.PercentOutput;
    periodicOutput.motorOutput = output;
  }

  public synchronized void setSolenoidReach() {
    periodicOutput.isSolenoidReach = true;
  }

  public synchronized boolean setMotorPositionDownByMeter(double position_meter) {
    if (getHeightMeter() > position_meter) {
      setMotorOpenLoop(ClimberConfig.NORMALIZED_DOWN_VEL);
      return false;
    } else {
      setMotorOpenLoop(0.085);
      return true;
    }
  }

  public synchronized boolean setMotorPositionUpByMeter(double position_meter) {
    if (getHeightMeter() < position_meter) {
      setMotorOpenLoop(ClimberConfig.NORMALIZED_UP_VEL);
      return false;
    } else {
      setMotorOpenLoop(0.085);
      return true;
    }
  }

  public synchronized void setMotorPositionKeepToZero() {
    var output = pid.calculate(getHeightMeter(), 0.0);
    setMotorOpenLoop(output);
  }

  public synchronized boolean setMotorExtend() {
    return setMotorPositionUpByMeter(ClimberConfig.EXTEND_HEIGHT_METER);
  }

  public synchronized void setSolenoidHook() {
    periodicOutput.isSolenoidReach = false;
  }

  public synchronized boolean setMotorContract() {
    return setMotorPositionDownByMeter(ClimberConfig.CONTRACT_HEIGHT_METER);
  }

  public synchronized boolean setMotorContractLower() {
    return setMotorPositionDownByMeter(ClimberConfig.CONTRACT_HEIGHT_LOWER_METER);
  }

  public synchronized boolean setMotorContractWait() {
    return setMotorPositionDownByMeter(ClimberConfig.CONTRACT_WAIT_HEIGHT_METER);
  }

  public synchronized boolean setMotorLand() {
    return setMotorPositionDownByMeter(ClimberConfig.LAND_HEIGHT_METER);
  }

  public synchronized void setSolenoidToggle() {
    periodicOutput.isSolenoidReach = !periodicOutput.isSolenoidReach;
  }

  public synchronized double getHeightMeter() {
    return getClimberEncoderUnitsToMeters(periodicInput.motorLeftPosition);
  }

  public synchronized boolean isHeightOnTarget() {
    return (climberState != ClimberState.DISABLE)
        && (Util.epsilonEquals(
                periodicInput.motorLeftPosition,
                periodicOutput.motorOutput,
                getClimberMeterToEncoderUnits(0.05))
            && Util.epsilonEquals(
                periodicInput.motorRightPosition,
                periodicOutput.motorOutput,
                getClimberMeterToEncoderUnits(0.05)));
  }

  /************************************************************************************************
   * Util *
   ************************************************************************************************/
  private double getClimberMeterToEncoderUnits(double meters) {
    return meters * ClimberConfig.ENCODER_UNITS_PER_METER;
  }

  private double getClimberEncoderUnitsToMeters(double encoder_units) {
    return encoder_units / ClimberConfig.ENCODER_UNITS_PER_METER;
  }

  /************************************************************************************************
   * Stop & Disable Actions *
   ************************************************************************************************/
  @Override
  public void disable() {
    setMotorOpenLoop(0.0);
  }

  /************************************************************************************************
   * Log & self-test *
   ************************************************************************************************/
  private NetworkTableEntry climberStateEntry;

  private NetworkTableEntry isOnTargetEntry;
  private NetworkTableEntry positionEntry;

  public void configSmartDashboard() {
    var tab = Shuffleboard.getTab("Climber");

    climberStateEntry = tab.add("Climber State", "None").getEntry();
    isOnTargetEntry = tab.add("Is On Target", false).getEntry();
    positionEntry = tab.add("Position", 0.0).getEntry();
  }

  @Override
  public void logToSmartDashboard() {
    if (Config.ENABLE_DEBUG_OUTPUT) {
      climberStateEntry.setString(climberState.value);
      isOnTargetEntry.setBoolean(isHeightOnTarget());
      positionEntry.setNumber(getHeightMeter());
    }
  }
}
