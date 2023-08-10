package com.nextinnovation.team8214.subsystems.tunnel;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.nextinnovation.lib.drivers.LazyTalonFX;
import com.nextinnovation.lib.drivers.PicoColorSensor;
import com.nextinnovation.lib.drivers.TalonUtil;
import com.nextinnovation.lib.loops.ILoop;
import com.nextinnovation.lib.loops.ILooper;
import com.nextinnovation.lib.subsystems.BaseSubsystem;
import com.nextinnovation.lib.utils.Util;
import com.nextinnovation.team8214.Config;
import com.nextinnovation.team8214.Ports;
import com.nextinnovation.team8214.auto.AutoModeChooser;
import com.nextinnovation.team8214.devices.DoubleColorSensor;
import com.nextinnovation.team8214.managers.ControlSignalManager;
import com.nextinnovation.team8214.subsystems.shooter.Shooter;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;

public class Tunnel extends BaseSubsystem {
  /***********************************************************************************************
   * Control Loop *
   ***********************************************************************************************/
  @Override
  public void registerEnabledLoops(ILooper enabledLooper) {
    ILoop loop =
        new ILoop() {
          @Override
          public void onStart(double timestamp) {
            setState(TunnelState.MANUAL);
          }

          @Override
          public void onLoop(double timestamp) {
            var controlSignalManager = ControlSignalManager.getInstance();
            var shooter = Shooter.getInstance();

            switch (tunnelState) {
              case AUTO:
              case MANUAL:
                if (controlSignalManager.isForceResetSensorOffset) {
                  updateRedBlueLinearOffset();
                }

                if (controlSignalManager.isRobotEject) {
                  setLowerEject();
                } else if (controlSignalManager.isRobotShoot) {
                  if (!isExitHasBall()) {
                    setPreload();
                  } else if (shooter.isReadyToShoot(true)) {
                    setFeed();
                  } else {
                    setTunnelOpenLoop(0.0);
                    setFeederOpenLoop(0.0);
                  }
                } else if (isExitHasBall() && isEnableColorSorting) {
                  if (!periodicInput.isExitColorFriend && shooter.isReadyToShoot(false)) {
                    setUpperReject();
                  } else {
                    setTunnelOpenLoop(0.0);
                    setFeederOpenLoop(0.0);
                  }
                } else if (controlSignalManager.isRobotInject) {
                  setInject();
                } else {
                  setTunnelOpenLoop(0.0);
                  setFeederOpenLoop(0.0);
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
  private static class PeriodicInput {
    // Exit
    public PicoColorSensor.RawColor rawColor = null;
    public double redBlueColorRatio = 1.0;
    public boolean isExitColorHasBall = false;
    public boolean isExitColorFriend = false;
  }

  @Override
  public void readPeriodicInputs() {
    // Exit
    periodicInput.rawColor = doubleColorSensor.getRawColor0();
    periodicInput.redBlueColorRatio =
        (periodicInput.rawColor.red + redBlueLinearOffset) / (double) periodicInput.rawColor.blue;

    periodicInput.isExitColorHasBall =
        isBallHasDebouncer.calculate(
            !Util.epsilonEquals(periodicInput.redBlueColorRatio, 1.0, 0.2));

    if (AutoModeChooser.getInstance().isAllianceRed()) {
      if (periodicInput.redBlueColorRatio > 1.35) {
        periodicInput.isExitColorFriend = isColorFriendDebouncer.calculate(true);
      } else if (periodicInput.redBlueColorRatio < 0.65) {
        periodicInput.isExitColorFriend = isColorFriendDebouncer.calculate(false);
      }
    } else {
      if (periodicInput.redBlueColorRatio > 1.35) {
        periodicInput.isExitColorFriend = isColorFriendDebouncer.calculate(false);
      } else if (periodicInput.redBlueColorRatio < 0.65) {
        periodicInput.isExitColorFriend = isColorFriendDebouncer.calculate(true);
      }
    }
  }

  private static class PeriodicOutput {
    public ControlMode tunnelMotorControlMode = ControlMode.PercentOutput;
    public double tunnelMotorOutput = 0.0;

    public ControlMode feederMotorControlMode = ControlMode.PercentOutput;
    public double feederMotorOutput = 0.0;
  }

  @Override
  public void writePeriodicOutputs() {
    tunnelMotor.set(periodicOutput.tunnelMotorControlMode, periodicOutput.tunnelMotorOutput);
    feederMotor.set(periodicOutput.feederMotorControlMode, periodicOutput.feederMotorOutput);
  }

  /***********************************************************************************************
   * Subsystem States *
   ***********************************************************************************************/
  private TunnelState tunnelState;

  public synchronized void setState(TunnelState new_state) {
    tunnelState = new_state;
    switch (tunnelState) {
      case AUTO:
      case MANUAL:
        break;

      case DISABLE:
        setTunnelOpenLoop(0.0);
        setFeederOpenLoop(0.0);
        break;
    }
  }

  /***********************************************************************************************
   * Singleton *
   ***********************************************************************************************/
  private static Tunnel instance = null;

  public static Tunnel getInstance() {
    if (instance == null) {
      instance = new Tunnel();
    }
    return instance;
  }

  /***********************************************************************************************
   * Init & Config *
   ***********************************************************************************************/
  private final LazyTalonFX tunnelMotor = new LazyTalonFX(Ports.Can.TUNNEL_SPIN_MOTOR);

  private final LazyTalonFX feederMotor = new LazyTalonFX(Ports.Can.FEEDER_WHEEL_MOTOR);

  private final PeriodicInput periodicInput = new PeriodicInput();
  private final PeriodicOutput periodicOutput = new PeriodicOutput();

  private final DoubleColorSensor doubleColorSensor = DoubleColorSensor.getInstance();

  private final Debouncer isBallHasDebouncer = new Debouncer(0.1, Debouncer.DebounceType.kFalling);

  private final Debouncer isColorFriendDebouncer =
      new Debouncer(0.1, Debouncer.DebounceType.kFalling);

  private double redBlueLinearOffset = 132.0;
  private boolean isEnableColorSorting = true;

  private Tunnel() {
    configSmartDashboard();
    configMotor();
  }

  private synchronized void configMotor() {
    tunnelMotor.configStatusFramePeriod(
        TunnelConfig.STATUS_FRAME_PERIOD, false, false, Config.CAN_TIMEOUT_MS);
    tunnelMotor.setInverted(TunnelConfig.IS_INVERT);
    tunnelMotor.setNeutralMode(NeutralMode.Brake);
    TalonUtil.checkError(
        tunnelMotor.configSupplyCurrentLimit(
            new SupplyCurrentLimitConfiguration(true, 28.0, 28.0, 0.0), Config.CAN_TIMEOUT_MS),
        "Tunnel: Can't set supply current limit!");

    feederMotor.configStatusFramePeriod(
        TunnelConfig.STATUS_FRAME_PERIOD, true, false, Config.CAN_TIMEOUT_MS);
    feederMotor.setInverted(!TunnelConfig.IS_INVERT);
    feederMotor.setNeutralMode(NeutralMode.Brake);
    feederMotor.enableVoltageCompensation(true);
    TalonUtil.checkError(
        feederMotor.configVoltageCompSaturation(9.0, Config.CAN_TIMEOUT_MS),
        "Feeder: Can't config comp voltage!");
    TalonUtil.checkError(
        tunnelMotor.configSupplyCurrentLimit(
            new SupplyCurrentLimitConfiguration(true, 28.0, 28.0, 0.0), Config.CAN_TIMEOUT_MS),
        "Feeder: Can't set supply current limit!");
    TalonUtil.checkError(
        feederMotor.config_kP(0, TunnelConfig.PID0_KP, Config.CAN_TIMEOUT_MS),
        "Feeder: Can't set kp!");
    TalonUtil.checkError(
        feederMotor.config_kI(0, TunnelConfig.PID0_KI, Config.CAN_TIMEOUT_MS),
        "Feeder: Can't set ki!");
    TalonUtil.checkError(
        feederMotor.config_kD(0, TunnelConfig.PID0_KD, Config.CAN_TIMEOUT_MS),
        "Feeder: Can't set kd!");
    TalonUtil.checkError(
        feederMotor.config_kF(0, TunnelConfig.PID0_KF, Config.CAN_TIMEOUT_MS),
        "Feeder: Can't set kf!");
  }

  /************************************************************************************************
   * Function Enabler *
   ************************************************************************************************/
  public void enableColorReject() {
    isEnableColorSorting = true;
  }

  public void disableColorReject() {
    isEnableColorSorting = false;
  }

  /************************************************************************************************
   * Getter & Setter *
   ************************************************************************************************/
  public synchronized void setTunnelOpenLoop(double output) {
    periodicOutput.tunnelMotorControlMode = ControlMode.PercentOutput;
    periodicOutput.tunnelMotorOutput = output;
  }

  public synchronized void setFeederOpenLoop(double output) {
    periodicOutput.feederMotorControlMode = ControlMode.PercentOutput;
    periodicOutput.feederMotorOutput = output;
  }

  public synchronized void setFeederNormalizedVelocity(double normalized_velocity) {
    periodicOutput.feederMotorControlMode = ControlMode.Velocity;
    periodicOutput.feederMotorOutput = normalized_velocity * TunnelConfig.FEEDER_MAX_SPEED;
  }

  public synchronized void setPreload() {
    setTunnelOpenLoop(TunnelConfig.TUNNEL_PRELOAD_SPEED);
    setFeederOpenLoop(TunnelConfig.FEEDER_PRELOAD_SPEED);
  }

  public synchronized void setFeed() {
    setTunnelOpenLoop(TunnelConfig.TUNNEL_FEED_SPEED);
    setFeederNormalizedVelocity(TunnelConfig.FEEDER_FEED_SPEED);
  }

  public synchronized void setInject() {
    setTunnelOpenLoop(TunnelConfig.TUNNEL_INJECT_SPEED);
    setFeederOpenLoop(TunnelConfig.FEEDER_INJECT_SPEED);
  }

  public synchronized void setLowerEject() {
    setTunnelOpenLoop(TunnelConfig.TUNNEL_EJECT_SPEED);
    setFeederOpenLoop(TunnelConfig.FEEDER_EJECT_SPEED);
  }

  public synchronized void setUpperReject() {
    setTunnelOpenLoop(TunnelConfig.TUNNEL_REJECT_SPEED);
    setFeederOpenLoop(TunnelConfig.FEEDER_REJECT_SPEED);
  }

  public boolean isExitHasBall() {
    return periodicInput.isExitColorHasBall;
  }

  public boolean isShooterNeedReject() {
    return isExitHasBall() && !periodicInput.isExitColorFriend && isEnableColorSorting;
  }

  /************************************************************************************************
   * Update *
   ************************************************************************************************/
  public void updateRedBlueLinearOffset() {
    if (periodicInput.rawColor != null) {
      redBlueLinearOffset = periodicInput.rawColor.blue - periodicInput.rawColor.red;
    }
  }

  /************************************************************************************************
   * Stop & Disable Actions *
   ************************************************************************************************/
  @Override
  public void disable() {
    setState(TunnelState.DISABLE);
  }

  /************************************************************************************************
   * Compressor Limit *
   ************************************************************************************************/
  public boolean canCompressorEnable() {
    return !Util.epsilonEquals(
        periodicOutput.feederMotorOutput, TunnelConfig.FEEDER_FEED_SPEED, 0.01);
  }

  /************************************************************************************************
   * Log & self-test *
   ************************************************************************************************/
  private NetworkTableEntry tunnelStateEntry;

  private NetworkTableEntry rbRatioEntry;
  private NetworkTableEntry proximityEntry;
  private NetworkTableEntry isFriendEntry;
  private NetworkTableEntry isBallHasEntry;
  private NetworkTableEntry isSensor0ConnectedEntry;
  private NetworkTableEntry rbOffsetEntry;

  public void configSmartDashboard() {
    var tab = Shuffleboard.getTab("Tunnel");

    tunnelStateEntry = tab.add("Tunnel State", "None").getEntry();
    rbRatioEntry = tab.add("RB Ratio", 1.0).getEntry();
    proximityEntry = tab.add("Proximity", 99.99).getEntry();
    isFriendEntry = tab.add("Is Friend", false).getEntry();
    isBallHasEntry = tab.add("Is Ball Has", false).getEntry();
    isSensor0ConnectedEntry = tab.add("Is Sensor0 Connected", false).getEntry();
    rbOffsetEntry = tab.add("RB Offset", -99.99).getEntry();
  }

  @Override
  public void logToSmartDashboard() {
    if (Config.ENABLE_DEBUG_OUTPUT) {
      tunnelStateEntry.setString(tunnelState.value);
      rbRatioEntry.setNumber(periodicInput.redBlueColorRatio);
      proximityEntry.setNumber(doubleColorSensor.getProximity0());
      isFriendEntry.setBoolean(periodicInput.isExitColorFriend);
      isBallHasEntry.setBoolean(periodicInput.isExitColorHasBall);
      isSensor0ConnectedEntry.setBoolean(doubleColorSensor.isSensor0Connected());
      rbOffsetEntry.setNumber(redBlueLinearOffset);
    }
  }
}
