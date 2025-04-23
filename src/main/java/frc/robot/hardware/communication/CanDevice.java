package frc.robot.hardware.communication;

public record CanDevice(int id, String bus) {
  @Override
  public boolean equals(Object o) {
    if (this == o) {
      return true;
    }
    if (!(o instanceof CanDevice device)) {
      return false;
    }
    return id == device.id && bus.equals(device.bus);
  }
}
