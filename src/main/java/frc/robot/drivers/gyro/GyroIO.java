package frc.robot.drivers.gyro;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface GyroIO {
  @AutoLog
  class GyroIOInputs {
    public boolean connected = false;

    public double yawRad = 0.0;
    public double omegaRadPerSec = 0.0;

    public double temperature = 35.0;
  }

  default void updateInputs(GyroIOInputs inputs) {
  }
}
