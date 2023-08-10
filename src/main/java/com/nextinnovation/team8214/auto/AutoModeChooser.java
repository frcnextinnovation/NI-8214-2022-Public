package com.nextinnovation.team8214.auto;

import com.nextinnovation.lib.auto.modes.BaseAutoMode;
import com.nextinnovation.team8214.auto.modes.Mid0Ball;
import com.nextinnovation.team8214.auto.modes.Right5Balls;
import com.nextinnovation.team8214.auto.modes.Silence;
import com.nextinnovation.team8214.auto.modes.Left2Balls;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class AutoModeChooser {
  private static AutoModeChooser instance = null;

  public static synchronized AutoModeChooser getInstance() {
    if (instance == null) {
      instance = new AutoModeChooser();
    }
    return instance;
  }

  public enum AutoOption {
    SILENCE("Silence"),
    RIGHT_5_BALLS("Right5Balls"),
    LEFT_2_BALLS("Left2Balls"),
    MID_0_BALL("Mid0Ball");

    public final String name;

    AutoOption(String name) {
      this.name = name;
    }
  }

  private static final AutoOption DEFAULT_MODE = AutoOption.LEFT_2_BALLS;
  private final SendableChooser<AutoOption> modeChooser;
  private AutoOption selectedOption;
  private AutoOption lastSelectedOption;
  private BaseAutoMode autoMode;
  private boolean isRedAlliance = false;

  private AutoModeChooser() {
    // Mode Chooser
    modeChooser = new SendableChooser<>();
    modeChooser.setDefaultOption(DEFAULT_MODE.name, DEFAULT_MODE);
    modeChooser.addOption(AutoOption.SILENCE.name, AutoOption.SILENCE);
    modeChooser.addOption(AutoOption.LEFT_2_BALLS.name, AutoOption.LEFT_2_BALLS);
    modeChooser.addOption(AutoOption.MID_0_BALL.name, AutoOption.MID_0_BALL);

    selectedOption = DEFAULT_MODE;
    lastSelectedOption = null;
    autoMode = null;
  }

  private void updateSelectedAutoMode() {
    selectedOption = modeChooser.getSelected();

    if (selectedOption == null) {
      selectedOption = AutoOption.LEFT_2_BALLS;
    }

    if (lastSelectedOption != selectedOption) {
      System.out.println("Auto option selected -> " + selectedOption.name);
      autoMode = createAutoMode(selectedOption);
    }

    lastSelectedOption = selectedOption;
  }

  public synchronized void update() {
    updateSelectedAutoMode();
    isRedAlliance = DriverStation.getAlliance() == DriverStation.Alliance.Red;
  }

  public BaseAutoMode getSelectedAutoMode() {
    return autoMode;
  }

  public synchronized boolean isAllianceRed() {
    return isRedAlliance;
  }

  private BaseAutoMode createAutoMode(AutoOption option) {
    switch (option) {
      case SILENCE:
        return new Silence();
      case RIGHT_5_BALLS:
        return new Right5Balls();
      case LEFT_2_BALLS:
        return new Left2Balls();
      case MID_0_BALL:
        return new Mid0Ball();
      default:
        System.out.println("ERROR: unexpected auto mode: " + option);
        return null;
    }
  }

  public void logToSmartDashboard() {
    SmartDashboard.putData("Mode Chooser", modeChooser);
    SmartDashboard.putString("Selected Auto Mode", selectedOption.name);
  }
}
