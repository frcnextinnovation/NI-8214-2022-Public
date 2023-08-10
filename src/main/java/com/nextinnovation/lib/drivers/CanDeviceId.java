package com.nextinnovation.lib.drivers;

public class CanDeviceId {
  private final String bus;
  private final int id;

  public CanDeviceId(int id, String bus) {
    this.id = id;
    this.bus = bus;
  }

  public CanDeviceId(int can_id) {
    this(can_id, "");
  }

  public int getId() {
    return id;
  }

  public String getBus() {
    return bus;
  }

  public boolean equals(CanDeviceId other) {
    return other.id == id && other.bus == bus;
  }
}
