package com.nextinnovation.team8214.managers;

import com.nextinnovation.lib.geometry.Translation2d;
import com.nextinnovation.lib.io.StatefulXboxController;
import com.nextinnovation.lib.loops.ILoop;
import com.nextinnovation.lib.loops.ILooper;
import com.nextinnovation.lib.utils.Util;
import com.nextinnovation.team8214.Ports;
import edu.wpi.first.wpilibj.DriverStation;

public class ControlSignalManager {
  /***********************************************************************************************
   * Config *
   ***********************************************************************************************/
  private static final boolean IS_SINGLE_MODE = false;

  /***********************************************************************************************
   * Control Loop *
   ***********************************************************************************************/
  public void registerEnabledLoops(ILooper enabledLooper) {
    ILoop loop =
        new ILoop() {
          @Override
          public void onStart(double timestamp) {
            resetControlFlags();
          }

          @Override
          public void onLoop(double timestamp) {
            if (DriverStation.isTeleopEnabled() || DriverStation.isTest()) {
              update();
            }
          }

          @Override
          public void onStop(double timestamp) {
            resetControlFlags();
          }
        };
    enabledLooper.register(loop);
  }

  /***********************************************************************************************
   * Singleton *
   ***********************************************************************************************/
  private static ControlSignalManager instance = null;

  public static synchronized ControlSignalManager getInstance() {
    if (instance == null) {
      instance = new ControlSignalManager();
    }
    return instance;
  }

  /***********************************************************************************************
   * Init & Config *
   ***********************************************************************************************/
  private final StatefulXboxController driverController;

  private final StatefulXboxController codriverController;

  // Control flags

  // Let robot inject.
  // true: Intake will always put down & inject, tunnel will inject until top sensor get friend
  // ball.
  // false: Intake will stand up & stop, tunnel will stop inject but still reject enemy ball.
  public boolean isRobotInject = false;

  // Let robot eject.
  // true: Intake will put down & eject, tunnel will eject from entrance.
  // false: Intake & tunnel stop.
  public boolean isRobotEject = false;

  // Let turret set to robot centric 0.0 degree.
  // true: Set to robot centric 0.0 degree.
  // false: Auto aim.
  public boolean isTurretSetToSolidAngle = false;

  // Let flywheel reach target shoot velocity.
  // true: Flywheel can reach target shoot velocity. Tunnel can only feed when flywheel reaches
  // target speed.
  // false: Flywheel keep coast speed. Tunnel can feed when flywheel velocity more than a specific
  // value.
  public boolean isFlyWheelUnlock = false;

  // Let robot shoot.
  // true: Tunnel can feed when shooter allows to feed.
  // false: Tunnel can't feed.
  public boolean isRobotShoot = false;

  // Check is robot in climb mode.
  // true: Only chassis & climber can work. Turret will set to solid angle.
  // false: Only climber can't work.
  public boolean isInClimbMode = false;

  // Climber switches to next state.
  // true: Climber will move to next state.
  // false: Do nothing.
  public boolean isClimberMove = false;

  public boolean isForceResetSensorOffset = false;

  public boolean isManualClimberArmToggle = false;
  public boolean isManualClimberArmStretch = false;
  public boolean isManualClimberArmContract = false;

  private ControlSignalManager() {
    driverController =
        new StatefulXboxController(Ports.DriverJoysticks.DRIVER_CONTROLLER_PORT, 0.5);
    codriverController =
        new StatefulXboxController(Ports.DriverJoysticks.CODRIVER_CONTROLLER_PORT, 0.5);

    resetControlFlags();
  }

  /************************************************************************************************
   * Reset *
   ************************************************************************************************/
  public synchronized void resetControlFlags() {
    isRobotInject = false;
    isRobotEject = false;

    isTurretSetToSolidAngle = false;
    isFlyWheelUnlock = false;
    isRobotShoot = false;

    isInClimbMode = false;
    isClimberMove = false;

    isForceResetSensorOffset = false;
  }

  /************************************************************************************************
   * Getter & Setter *
   ************************************************************************************************/
  public Translation2d getSwerveManualTranslation() {
    var input =
        Util.applyRemappedCircularDeadband(
                new Translation2d(-driverController.getRightY(), -driverController.getRightX()),
                0.09375)
            .scale(0.91);

    input = Translation2d.fromPolar(input.direction(), Math.pow(input.norm(), 2));

    return input;
  }

  public double getSwerveManualRotationMagnitude() {
    var input = Util.applyRemappedDeadband(-driverController.getLeftX(), 0.06375) * 0.42;

    return input;
  }

  /************************************************************************************************
   * Update *
   ************************************************************************************************/
  private synchronized void update() {
    // Use when in manual mode
    driverController.updateButtons();
    codriverController.updateButtons();

    if (IS_SINGLE_MODE
        ? driverController.getButton(StatefulXboxController.ButtonId.BUTTON_X).wasPressed()
        : codriverController.getButton(StatefulXboxController.ButtonId.BUTTON_X).wasPressed()) {
      isInClimbMode = !isInClimbMode;
    }

    isRobotInject =
        (IS_SINGLE_MODE
                ? driverController
                    .getButton(StatefulXboxController.ButtonId.BUMPER_LEFT)
                    .isBeingPressed()
                : codriverController
                    .getButton(StatefulXboxController.ButtonId.BUMPER_LEFT)
                    .isBeingPressed())
            && !isInClimbMode;

    isRobotEject =
        (IS_SINGLE_MODE
                ? driverController
                    .getButton(StatefulXboxController.ButtonId.BUMPER_RIGHT)
                    .isBeingPressed()
                : codriverController
                    .getButton(StatefulXboxController.ButtonId.BUMPER_RIGHT)
                    .isBeingPressed())
            && !isInClimbMode;

    isTurretSetToSolidAngle =
        (IS_SINGLE_MODE
                ? driverController
                    .getButton(StatefulXboxController.ButtonId.TRIGGER_LEFT)
                    .isBeingPressed()
                : codriverController
                    .getButton(StatefulXboxController.ButtonId.TRIGGER_LEFT)
                    .isBeingPressed())
            || isInClimbMode;

    if (IS_SINGLE_MODE
        ? driverController.getButton(StatefulXboxController.ButtonId.BUTTON_Y).wasPressed()
        : codriverController.getButton(StatefulXboxController.ButtonId.BUTTON_Y).wasPressed()) {
      isFlyWheelUnlock = !isFlyWheelUnlock && !isInClimbMode;
    }

    isRobotShoot =
        (IS_SINGLE_MODE
                ? driverController
                    .getButton(StatefulXboxController.ButtonId.TRIGGER_RIGHT)
                    .isBeingPressed()
                : codriverController
                    .getButton(StatefulXboxController.ButtonId.TRIGGER_RIGHT)
                    .isBeingPressed())
            && !isInClimbMode;

    if (isRobotShoot) {
      isFlyWheelUnlock = !isInClimbMode;
    }

    isForceResetSensorOffset =
        driverController.getButton(StatefulXboxController.ButtonId.BUMPER_RIGHT).wasPressed();

    isClimberMove =
        (IS_SINGLE_MODE
                ? driverController
                    .getButton(StatefulXboxController.ButtonId.BUMPER_RIGHT)
                    .wasPressed()
                : codriverController
                    .getButton(StatefulXboxController.ButtonId.BUMPER_RIGHT)
                    .wasPressed())
            && isInClimbMode;

    isManualClimberArmToggle =
        IS_SINGLE_MODE
            ? driverController.getButton(StatefulXboxController.ButtonId.BUTTON_B).wasPressed()
            : codriverController.getButton(StatefulXboxController.ButtonId.BUTTON_B).wasPressed();

    isManualClimberArmStretch =
        IS_SINGLE_MODE
            ? driverController.getButton(StatefulXboxController.ButtonId.BUTTON_Y).isBeingPressed()
            : codriverController
                .getButton(StatefulXboxController.ButtonId.BUTTON_Y)
                .isBeingPressed();

    isManualClimberArmContract =
        IS_SINGLE_MODE
            ? driverController.getButton(StatefulXboxController.ButtonId.BUTTON_A).isBeingPressed()
            : codriverController
                .getButton(StatefulXboxController.ButtonId.BUTTON_A)
                .isBeingPressed();
  }

  /************************************************************************************************
   * Log *
   ************************************************************************************************/
  public void logToSmartDashBoard() {}
}
