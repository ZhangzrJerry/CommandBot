package frc.robot.interfaces.hardwares.sensors.digital;

import org.littletonrobotics.junction.AutoLog;

public interface BiDigitalIO {
  @AutoLog
  public class BiDigitalIOInputs {
    public boolean connected = false;
    public boolean value1 = false;
    public boolean value2 = false;
  }

  default void updateInputs(BiDigitalIOInputs inputs) {}

  default boolean getValue1() {
    return false;
  }

  default boolean getValue2() {
    return false;
  }
}
