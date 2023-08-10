package com.nextinnovation.team8214.devices;

import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoMode.PixelFormat;
import edu.wpi.first.cameraserver.CameraServer;

public class VideoFeeder {
  public static final class VideoFeederConfig {
    public static final int DRIVER_CAMERA_RESOLUTION_WIDTH = 480;
    public static final int DRIVER_CAMERA_RESOLUTION_HEIGHT = 360;
    public static final int DRIVER_CAMERA_FPS = 18;
  }

  private static VideoFeeder instance = null;

  public static synchronized VideoFeeder getInstance() {
    if (instance == null) {
      instance = new VideoFeeder();
    }
    return instance;
  }

  private UsbCamera mainCamera;

  public void enable() {
    mainCamera = CameraServer.startAutomaticCapture();
    configCamera();
  }

  private synchronized void configCamera() {
    mainCamera.setPixelFormat(PixelFormat.kMJPEG);
    mainCamera.setResolution(
        VideoFeederConfig.DRIVER_CAMERA_RESOLUTION_WIDTH,
        VideoFeederConfig.DRIVER_CAMERA_RESOLUTION_HEIGHT);
    mainCamera.setFPS(VideoFeederConfig.DRIVER_CAMERA_FPS);
  }
}
