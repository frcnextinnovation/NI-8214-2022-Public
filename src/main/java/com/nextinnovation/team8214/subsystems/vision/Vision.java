package com.nextinnovation.team8214.subsystems.vision;

import com.nextinnovation.lib.drivers.Limelight;
import com.nextinnovation.lib.geometry.Rotation2d;
import com.nextinnovation.lib.geometry.Translation2d;
import com.nextinnovation.lib.loops.ILoop;
import com.nextinnovation.lib.loops.ILooper;
import com.nextinnovation.lib.subsystems.BaseSubsystem;
import com.nextinnovation.lib.utils.Units;
import com.nextinnovation.team8214.Config;
import com.nextinnovation.team8214.Field;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;

public class Vision extends BaseSubsystem {
  /***********************************************************************************************
   * Control Loop *
   ***********************************************************************************************/
  @Override
  public void registerEnabledLoops(ILooper enabledLooper) {
    ILoop loop =
        new ILoop() {
          @Override
          public void onStart(double timestamp) {
            setState(VisionState.ENABLE);
          }

          @Override
          public void onLoop(double timestamp) {}

          @Override
          public void onStop(double timestamp) {}
        };
    enabledLooper.register(loop);
  }

  /***********************************************************************************************
   * Periodic IO *
   ***********************************************************************************************/
  public static class VisionTargetInfo {
    public Rotation2d targetHeading;
    public Rotation2d targetElevation;
    public double targetDistance;
    public double timestamp;
    public boolean isGoodTarget;

    public VisionTargetInfo(
        Rotation2d target_heading,
        Rotation2d target_elevation,
        double target_distance,
        double timestamp,
        boolean is_good_target) {
      targetHeading = target_heading;
      targetElevation = target_elevation;
      targetDistance = target_distance;
      this.timestamp = timestamp;
      isGoodTarget = is_good_target;
    }
  }

  private static class PeriodicInput {
    double latency = 255.0;
    double timestamp = 0.0;
    boolean isUpdated = false;
    boolean hasTarget = false;
    Rotation2d targetX = Rotation2d.identity();
    Rotation2d targetY = Rotation2d.identity();
    boolean isGoodTarget = false;
  }

  @Override
  public void readPeriodicInputs() {
    periodicInput.isUpdated = true;
    periodicInput.latency = limelight.getLatency();
    periodicInput.timestamp = Timer.getFPGATimestamp() - periodicInput.latency;
    periodicInput.hasTarget = limelight.hasTarget();
    if (periodicInput.hasTarget) {
      periodicInput.targetX = limelight.getTargetX();
      periodicInput.targetY = limelight.getTargetY();

      var cornersLen = limelight.getCorners().length;
      periodicInput.isGoodTarget = (cornersLen >= 4 && cornersLen % 4 == 0);
    }
  }

  /***********************************************************************************************
   * Subsystem States *
   ***********************************************************************************************/
  private VisionState visionState = VisionState.OFF;

  public synchronized void setState(VisionState new_state) {
    visionState = new_state;
    switch (visionState) {
      case OFF:
        off();
        break;
      case ENABLE:
        enable();
        break;
      case BLINK:
        blink();
        break;
    }
  }

  /***********************************************************************************************
   * Singleton *
   ***********************************************************************************************/
  private static Vision instance = null;

  public static synchronized Vision getInstance() {
    if (instance == null) {
      instance = new Vision();
    }
    return instance;
  }

  /***********************************************************************************************
   * Init & Config *
   ***********************************************************************************************/
  private final PeriodicInput periodicInput = new PeriodicInput();

  private final Limelight limelight = new Limelight();
  private boolean isEnabled = false;

  public Vision() {
    configSmartDashboard();
    setState(VisionState.OFF);
  }

  /************************************************************************************************
   * Function Enabler *
   ************************************************************************************************/
  private void enable() {
    limelight.setLedMode(0);
    isEnabled = true;
  }

  private void off() {
    limelight.setLedMode(1);
    isEnabled = false;
  }

  private void blink() {
    limelight.setLedMode(2);
    isEnabled = false;
  }

  /************************************************************************************************
   * Getter & Setter *
   ************************************************************************************************/
  public boolean isEnabled() {
    return isEnabled;
  }

  public boolean isUpdated() {
    if (periodicInput.isUpdated) {
      periodicInput.isUpdated = false;
      return true;
    } else {
      return false;
    }
  }

  public boolean hasTarget() {
    return periodicInput.hasTarget && isEnabled;
  }

  private Rotation2d getTargetElevation() {
    return periodicInput.targetY;
  }

  private Rotation2d getTargetHeading() {
    return periodicInput.targetX.inverse();
  }

  /**
   * Get camera latency in ms
   *
   * @return camera latency in ms
   */
  private double getLatency() {
    return periodicInput.latency / 1000.0;
  }

  /**
   * Get target timestamp in seconds
   *
   * @return target timestamp in seconds
   */
  private double getTargetTimestamp() {
    return periodicInput.timestamp;
  }

  /**
   * Get camera centric distance from vision target in inch
   *
   * @return target distance in inch
   */
  private double getTargetDistance() {
    return (Field.VISUAL_TARGET_VISUAL_CENTER_HEIGHT - VisionConfig.CAMERA_HEIGHT_METER)
        / Math.tan(getTargetElevation().getRadians() + VisionConfig.CAMERA_ELEVATION.getRadians());
  }

  /**
   * Get camera centric orientation from vision target with distance in inch
   *
   * @return target distance in inch by a Translation2d
   */
  public Translation2d getTargetOrientation() {
    return Translation2d.fromPolar(getTargetHeading(), getTargetDistance());
  }

  public boolean isGoodTarget() {
    return periodicInput.isGoodTarget;
  }

  /**
   * Get camera centric vision target info
   *
   * @return vision target info
   */
  public VisionTargetInfo getVisionTargetInfo() {
    return new VisionTargetInfo(
        getTargetHeading(),
        getTargetElevation(),
        getTargetDistance(),
        getTargetTimestamp(),
        isGoodTarget());
  }

  /************************************************************************************************
   * Stop & Disable Actions *
   ************************************************************************************************/
  @Override
  public void disable() {
    setState(VisionState.OFF);
  }

  /************************************************************************************************
   * Log & self-test *
   ************************************************************************************************/
  private NetworkTableEntry visionStateEntry;

  private NetworkTableEntry hasVisionTargetEntry;
  private NetworkTableEntry targetOrientationEntry;
  private NetworkTableEntry targetElevationEntry;
  private NetworkTableEntry fixedDistanceEntry;
  private NetworkTableEntry cameraLatencyEntry;

  public void configSmartDashboard() {
    var tab = Shuffleboard.getTab("Vision");

    visionStateEntry = tab.add("Vision State", "None").getEntry();
    hasVisionTargetEntry = tab.add("Has Vision Target", false).getEntry();
    targetOrientationEntry = tab.add("Target Orientation", 0.0).getEntry();
    targetElevationEntry = tab.add("Target Elevation", 0.0).getEntry();
    fixedDistanceEntry = tab.add("Fixed Distance", 99.99).getEntry();
    cameraLatencyEntry = tab.add("Camera Latency", 99.99).getEntry();
  }

  @Override
  public void logToSmartDashboard() {
    if (Config.ENABLE_DEBUG_OUTPUT) {
      visionStateEntry.setString(visionState.value);
      hasVisionTargetEntry.setBoolean(hasTarget());
      targetOrientationEntry.setNumber(getTargetOrientation().direction().getDegrees());
      targetElevationEntry.setNumber(getTargetElevation().getDegrees());
      fixedDistanceEntry.setNumber(Units.inches_to_meters(getTargetDistance()));
      cameraLatencyEntry.setNumber(getLatency());
    }
  }
}
