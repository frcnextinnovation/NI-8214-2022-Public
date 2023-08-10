package com.nextinnovation.team8214;

import com.nextinnovation.lib.drivers.CanDeviceId;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

/** Final class to store all ID of CAN devices or the solenoids connected to PCM. */
public final class Ports {
  /** Inner class to store IDs of CAN devices. */
  public static final class Can {
    // Can Bus
    public static final String CANIVORE_BUS_NAME = "canivore";

    // IMU
    public static final CanDeviceId PIGEON_CHASSIS = new CanDeviceId(0, CANIVORE_BUS_NAME);

    // Swerve
    public static final CanDeviceId FRONT_LEFT_DRIVE_MOTOR = new CanDeviceId(0);
    public static final CanDeviceId FRONT_LEFT_ROTATION_MOTOR = new CanDeviceId(1);
    public static final CanDeviceId FRONT_LEFT_ROTATION_SENSOR = new CanDeviceId(8);

    public static final CanDeviceId REAR_LEFT_DRIVE_MOTOR = new CanDeviceId(2);
    public static final CanDeviceId REAR_LEFT_ROTATION_MOTOR = new CanDeviceId(3);
    public static final CanDeviceId REAR_LEFT_ROTATION_SENSOR = new CanDeviceId(9);

    public static final CanDeviceId REAR_RIGHT_DRIVE_MOTOR = new CanDeviceId(4, CANIVORE_BUS_NAME);
    public static final CanDeviceId REAR_RIGHT_ROTATION_MOTOR = new CanDeviceId(5, CANIVORE_BUS_NAME);
    public static final CanDeviceId REAR_RIGHT_ROTATION_SENSOR = new CanDeviceId(10, CANIVORE_BUS_NAME);

    public static final CanDeviceId FRONT_RIGHT_DRIVE_MOTOR = new CanDeviceId(6, CANIVORE_BUS_NAME);
    public static final CanDeviceId FRONT_RIGHT_ROTATION_MOTOR = new CanDeviceId(7, CANIVORE_BUS_NAME);
    public static final CanDeviceId FRONT_RIGHT_ROTATION_SENSOR = new CanDeviceId(11, CANIVORE_BUS_NAME);

    // Intake
    public static final CanDeviceId INTAKE_SPIN_MOTOR = new CanDeviceId(12);

    // Tunnel
    public static final CanDeviceId TUNNEL_SPIN_MOTOR = new CanDeviceId(13, CANIVORE_BUS_NAME);
    public static final CanDeviceId FEEDER_WHEEL_MOTOR = new CanDeviceId(14, CANIVORE_BUS_NAME);

    // Shooter
    public static final CanDeviceId TURRET_YAW_MOTOR = new CanDeviceId(15, CANIVORE_BUS_NAME);
    public static final CanDeviceId FLY_WHEEL_MOTOR_RIGHT = new CanDeviceId(16, CANIVORE_BUS_NAME);
    public static final CanDeviceId FLY_WHEEL_MOTOR_LEFT = new CanDeviceId(17, CANIVORE_BUS_NAME);
    public static final CanDeviceId TURRET_PITCH_MOTOR = new CanDeviceId(18, CANIVORE_BUS_NAME);

    // Climber
    public static final CanDeviceId CLIMBER_MOTOR_LEFT = new CanDeviceId(19);
    public static final CanDeviceId CLIMBER_MOTOR_RIGHT = new CanDeviceId(20, CANIVORE_BUS_NAME);
  }

  public static final class DriverJoysticks {
    public static final int DRIVER_CONTROLLER_PORT = 0;
    public static final int CODRIVER_CONTROLLER_PORT = 1;
  }

  /** Inner class to store IDs of the solenoids connected to PCM. */
  public static final class Pcm {
    public static final PneumaticsModuleType PCM_TYPE = PneumaticsModuleType.REVPH;

    // Intake
    public static final int INTAKE_ELEVATE_PULL = 3;
    public static final int INTAKE_ELEVATE_PUSH = 2;

    // Climber
    public static final int CLIMBER_ELEVATE_PULL = 1;
    public static final int CLIMBER_ELEVATE_PUSH = 0;
  }
}
