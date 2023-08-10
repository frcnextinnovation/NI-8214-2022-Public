package com.nextinnovation.team8214;

import com.nextinnovation.lib.auto.AutoModeExecuter;
import com.nextinnovation.lib.loops.Looper;
import com.nextinnovation.lib.subsystems.SubsystemGroup;
import com.nextinnovation.lib.utils.CrashTracker;
import com.nextinnovation.lib.utils.Logger;
import com.nextinnovation.lib.wpilib.TimedRobot;
import com.nextinnovation.team8214.auto.AutoModeChooser;
import com.nextinnovation.team8214.auto.TrajectorySet;
import com.nextinnovation.team8214.devices.PneumaticCompressor;
import com.nextinnovation.team8214.devices.ahrs.AhrsPigeon2;
import com.nextinnovation.team8214.managers.CancoderManager;
import com.nextinnovation.team8214.managers.ControlSignalManager;
import com.nextinnovation.team8214.subsystems.climber.Climber;
import com.nextinnovation.team8214.subsystems.climber.ClimberState;
import com.nextinnovation.team8214.subsystems.intake.Intake;
import com.nextinnovation.team8214.subsystems.shooter.Shooter;
import com.nextinnovation.team8214.subsystems.shooter.ShooterState;
import com.nextinnovation.team8214.subsystems.swerve.Swerve;
import com.nextinnovation.team8214.subsystems.tunnel.Tunnel;
import com.nextinnovation.team8214.subsystems.vision.Vision;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;

import java.util.Arrays;

public class Robot extends TimedRobot {
  private final Looper controlLooper = new Looper("Control", Config.LOOPER_CONTROL_PERIOD_SEC);
  private AutoModeExecuter autoModeExecuter;
  private AutoModeChooser autoModeChooser;

  private ControlSignalManager controlSignalManager;
  private Odometry odometry;
  private PneumaticCompressor pneumaticCompressor;

  private Climber climber;
  private Vision vision;
  private Shooter shooter;
  private Swerve swerve;
  private Intake intake;
  private Tunnel tunnel;
  private SubsystemGroup subsystems;

  private void initManagers() {
    controlSignalManager = ControlSignalManager.getInstance();
    odometry = Odometry.getInstance();
    CancoderManager.getInstance();

    controlSignalManager.registerEnabledLoops(controlLooper);
    System.out.println("Managers Init");
  }

  private void initDevices() {
    AhrsPigeon2.getInstance();
    pneumaticCompressor = PneumaticCompressor.getInstance();
    pneumaticCompressor.enable();
    //    VideoFeeder.getInstance().enable();
    System.out.println("Devices Init");
  }

  private void initSubsystems() {
    vision = Vision.getInstance();
    swerve = Swerve.getInstance();
    shooter = Shooter.getInstance();
    tunnel = Tunnel.getInstance();
    intake = Intake.getInstance();
    climber = Climber.getInstance();

    subsystems =
        new SubsystemGroup(Arrays.asList(vision, swerve, intake, climber, shooter, tunnel));
    subsystems.registerEnabledLoops(controlLooper);
    System.out.println("Subsystems Init");
  }

  private void initAutoTools() {
    TrajectorySet.getInstance();
    autoModeChooser = AutoModeChooser.getInstance();
    autoModeExecuter = AutoModeExecuter.getInstance();
    System.out.println("Auto Tools Init");
  }

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    Logger.clearLog();
    LiveWindow.disableAllTelemetry();

    try {
      initManagers();
      initDevices();
      initSubsystems();
      initAutoTools();
      System.out.println("Robot Init");
    } catch (Throwable t) {
      CrashTracker.logThrowableCrash(t);
      throw t;
    }
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {}

  /** This function is called once when autonomous is enabled. */
  @Override
  public void autonomousInit() {
    try {
      controlLooper.start();
      if (autoModeChooser.getSelectedAutoMode() != null) {
        autoModeExecuter.setAutoMode(autoModeChooser.getSelectedAutoMode());
      }
      logToSmartDashboard();
      autoModeExecuter.start();
    } catch (Throwable t) {
      CrashTracker.logThrowableCrash(t);
      throw t;
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    teleopPeriodic();
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    try {
      if (autoModeExecuter != null) {
        autoModeExecuter.stop();
      }
      controlLooper.restart();
      autoModeChooser.logToSmartDashboard();
      odometry.enableVo();
      tunnel.enableColorReject();
    } catch (Throwable t) {
      CrashTracker.logThrowableCrash(t);
      throw t;
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    try {
      pneumaticCompressor.update();
      logToSmartDashboard();
    } catch (Throwable t) {
      CrashTracker.logThrowableCrash(t);
      throw t;
    }
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {
    try {
      controlLooper.stop();
    } catch (Throwable t) {
      throw t;
    }
  }

  @Override
  public void disabledPeriodic() {
    try {
      autoModeChooser.update();
      swerve.logToSmartDashboard();
      autoModeChooser.logToSmartDashboard();
      tunnel.updateRedBlueLinearOffset();
    } catch (Throwable t) {
      throw t;
    }
  }

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {
    try {
      if (autoModeExecuter != null) {
        autoModeExecuter.stop();
      }
      controlLooper.restart();
      shooter.setState(ShooterState.DISABLE);
      climber.setState(ClimberState.TEST);
    } catch (Throwable t) {
      CrashTracker.logThrowableCrash(t);
      throw t;
    }
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
    teleopPeriodic();
  }

  private void logToSmartDashboard() {
    autoModeChooser.logToSmartDashboard();
    subsystems.logToSmartDashboard();
    controlSignalManager.logToSmartDashBoard();
    odometry.logToSmartDashBoard();
  }
}
