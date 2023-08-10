package com.nextinnovation.team8214.subsystems.intake;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.nextinnovation.lib.drivers.LazyDoubleSolenoid;
import com.nextinnovation.lib.drivers.LazyTalonFX;
import com.nextinnovation.lib.drivers.TalonUtil;
import com.nextinnovation.lib.loops.ILoop;
import com.nextinnovation.lib.loops.ILooper;
import com.nextinnovation.lib.subsystems.BaseSubsystem;
import com.nextinnovation.team8214.Config;
import com.nextinnovation.team8214.Ports;
import com.nextinnovation.team8214.managers.ControlSignalManager;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Timer;

public class Intake extends BaseSubsystem {
  /***********************************************************************************************
   * Control Loop *
   ***********************************************************************************************/
  @Override
  public void registerEnabledLoops(ILooper enabledLooper) {
    ILoop loop =
        new ILoop() {
          @Override
          public void onStart(double timestamp) {
            setState(IntakeState.MANUAL);
          }

          @Override
          public void onLoop(double timestamp) {
            var controlSignalManager = ControlSignalManager.getInstance();

            switch (intakeState) {
              case AUTO:
              case MANUAL:
                if (controlSignalManager.isRobotEject) {
                  setEject();
                  setDegrade();
                } else if (controlSignalManager.isRobotInject) {
                  setInject();
                  setDegrade();
                } else {
                  setElevate();
                  setOpenLoop(0.0);
                }
                break;

              case DISABLE:
                disable();
                break;
            }
          }

          @Override
          public void onStop(double timestamp) {
            disable();
          }
        };
    enabledLooper.register(loop);
  }

  /***********************************************************************************************
   * Periodic IO *
   ***********************************************************************************************/
  private static class PeriodicOutput {
    public ControlMode motorControlMode = ControlMode.PercentOutput;
    public double motorOutput = 0.0;
    public boolean isSolenoidElevate = true;
  }

  @Override
  public synchronized void writePeriodicOutputs() {
    motor.set(periodicOutput.motorControlMode, periodicOutput.motorOutput);
    doubleSolenoid.set(
        periodicOutput.isSolenoidElevate
            ? DoubleSolenoid.Value.kForward
            : DoubleSolenoid.Value.kReverse);
  }

  /***********************************************************************************************
   * Subsystem States *
   ***********************************************************************************************/
  private IntakeState intakeState;

  public synchronized void setState(IntakeState new_state) {
    intakeState = new_state;
    switch (intakeState) {
      case MANUAL:
      case AUTO:
        break;

      case DISABLE:
        setOpenLoop(0.0);
        break;
    }
  }

  /***********************************************************************************************
   * Singleton *
   ***********************************************************************************************/
  private static Intake instance = null;

  public static synchronized Intake getInstance() {
    if (instance == null) {
      instance = new Intake();
    }

    return instance;
  }

  /***********************************************************************************************
   * Init & Config *
   ***********************************************************************************************/
  private final LazyTalonFX motor = new LazyTalonFX(Ports.Can.INTAKE_SPIN_MOTOR);

  private final LazyDoubleSolenoid doubleSolenoid =
      new LazyDoubleSolenoid(
          Ports.Pcm.PCM_TYPE, Ports.Pcm.INTAKE_ELEVATE_PULL, Ports.Pcm.INTAKE_ELEVATE_PUSH);

  private final PeriodicOutput periodicOutput = new PeriodicOutput();

  private Intake() {
    configMotor();
    setState(IntakeState.DISABLE);

    if (IntakeConfig.IS_ELEVATE) {
      setElevate();
    } else {
      setDegrade();
    }
  }

  private synchronized void configMotor() {
    motor.configStatusFramePeriod(
        IntakeConfig.STATUS_FRAME_PERIOD, false, false, Config.CAN_TIMEOUT_MS);
    motor.setInverted(IntakeConfig.IS_INVERT);
    motor.setNeutralMode(NeutralMode.Coast);
    TalonUtil.checkError(
        motor.configSupplyCurrentLimit(
            new SupplyCurrentLimitConfiguration(true, 28.0, 28.0, 0.0), Config.CAN_TIMEOUT_MS),
        "Intake: Can't set supply current limit!");

    Timer.delay(0.05);
  }

  /************************************************************************************************
   * Getter & Setter *
   ************************************************************************************************/
  public synchronized void setOpenLoop(double output) {
    periodicOutput.motorControlMode = ControlMode.PercentOutput;
    periodicOutput.motorOutput = output;
  }

  public synchronized void setInject() {
    setOpenLoop(IntakeConfig.INJECT_SPEED);
  }

  public synchronized void setEject() {
    setOpenLoop(IntakeConfig.EJECT_SPEED);
  }

  public synchronized void setElevate() {
    periodicOutput.isSolenoidElevate = true;
  }

  public synchronized void setDegrade() {
    periodicOutput.isSolenoidElevate = false;
  }

  /************************************************************************************************
   * Stop & Disable Actions *
   ************************************************************************************************/
  @Override
  public void disable() {
    setState(IntakeState.DISABLE);
  }

  /************************************************************************************************
   * Log & self-test *
   ************************************************************************************************/
  @Override
  public void logToSmartDashboard() {}
}
