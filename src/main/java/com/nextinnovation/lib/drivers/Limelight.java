package com.nextinnovation.lib.drivers;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import com.nextinnovation.lib.geometry.Rotation2d;
import com.nextinnovation.lib.utils.Util;

public class Limelight {
  private NetworkTableEntry getEntry(String key) {
    return NetworkTableInstance.getDefault().getTable("limelight").getEntry(key);
  }

  private double getEntryDoubleVal(String key) {
    return getEntry(key).getDouble(0.0);
  }

  private void setEntryNumberVal(String key, Number val) {
    getEntry(key).setNumber(val);
  }

  public boolean hasTarget() {
    return Util.epsilonEquals(getEntryDoubleVal("tv"), 1.0);
  }

  public Rotation2d getTargetX() {
    return Rotation2d.fromDegrees(getEntryDoubleVal("tx"));
  }

  public Rotation2d getTargetY() {
    return Rotation2d.fromDegrees(getEntryDoubleVal("ty"));
  }

  public double getTargetAreaPercent() {
    return getEntryDoubleVal("ta");
  }

  public Rotation2d getTargetSkew() {
    return Rotation2d.fromDegrees(getEntryDoubleVal("ts"));
  }

  public double[] getCorners() {
    return getEntry("tcornxy").getDoubleArray(new double[] {});
  }

  public double getLatency() {
    return getEntryDoubleVal("tl");
  }

  public double getTargetShortSide() {
    return getEntryDoubleVal("tshort");
  }

  public double getTargetLongSide() {
    return getEntryDoubleVal("tlong");
  }

  public double getTargetHorizontalSide() {
    return getEntryDoubleVal("thor");
  }

  public double getTargetVerticalSide() {
    return getEntryDoubleVal("tvert");
  }

  public int getPipelineIndex() {
    return Util.roundToInt(getEntryDoubleVal("getpipe"));
  }

  public void setLedMode(Number val) {
    setEntryNumberVal("ledMode", val);
  }

  public void setCamMode(Number val) {
    setEntryNumberVal("camMode", val);
  }

  public void setStreamingMode(Number val) {
    setEntryNumberVal("stream", val);
  }

  public void setPipelineIndex(Number val) {
    setEntryNumberVal("pipeline", val);
  }
}
