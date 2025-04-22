package frc.robot.drivers.gyro;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface GyroIO {
  @AutoLog
  class GyroIOInputs {
    public boolean connected = false;

    public Rotation2d yaw = new Rotation2d();
    public double omegaRadPerSec = 0.0;

    public double temperature = 35.0;
  }

  default void updateInputs(GyroIOInputs inputs) {}

  default Rotation2d getYaw() {
    return null;
  }
}
