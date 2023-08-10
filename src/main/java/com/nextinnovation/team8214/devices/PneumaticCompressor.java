package com.nextinnovation.team8214.devices;

import com.nextinnovation.team8214.subsystems.tunnel.Tunnel;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

public class PneumaticCompressor {
  private static PneumaticCompressor instance = null;
  private boolean isEnabled = false;
  private boolean lastIsEnabled = true;

  public static synchronized PneumaticCompressor getInstance() {
    if (instance == null) {
      instance = new PneumaticCompressor();
    }
    return instance;
  }

  private final Compressor compressor = new Compressor(PneumaticsModuleType.REVPH);

  public void enable() {
    compressor.enableDigital();
  }

  public void disable() {
    compressor.disable();
  }

  public boolean isEnabled() {
    return compressor.enabled();
  }

  public boolean pressureOnTarget() {
    return compressor.getPressureSwitchValue();
  }

  public void update() {
    isEnabled = Tunnel.getInstance().canCompressorEnable();

    if (isEnabled != lastIsEnabled) {
      if (isEnabled) {
        enable();
      } else {
        disable();
      }
    }

    lastIsEnabled = isEnabled;
  }
}
