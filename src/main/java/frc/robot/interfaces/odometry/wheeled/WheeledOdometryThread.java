package frc.robot.interfaces.odometry.wheeled;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import java.util.concurrent.ArrayBlockingQueue;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public interface WheeledOdometryThread {
  public record WheeledObservation(
      double timestamp, SwerveModulePosition[] wheelPositions, Rotation2d yaw) {}

  static class WheeledObservationInputsLogged implements LoggableInputs {
    double[] observationTimestamps;
    SwerveModulePosition[] observationFLPositions;
    SwerveModulePosition[] observationBLPositions;
    SwerveModulePosition[] observationBRPositions;
    SwerveModulePosition[] observationFRPositions;
    Rotation2d[] observationYaws;

    @Override
    public void toLog(LogTable table) {
      table.put("ObservationTimestamps", observationTimestamps);
      table.put("ObservationFLPositions", observationFLPositions);
      table.put("ObservationBLPositions", observationBLPositions);
      table.put("ObservationBRPositions", observationBRPositions);
      table.put("ObservationFRPositions", observationFRPositions);

      if (observationYaws.length != 0 && observationYaws[0] != null) {
        table.put("ObservationYaws", observationYaws);
      }
    }

    @Override
    public void fromLog(LogTable table) {
      observationTimestamps = table.get("ObservationTimestamps", observationTimestamps);
      observationFLPositions = table.get("ObservationFLPositions", observationFLPositions);
      observationBLPositions = table.get("ObservationBLPositions", observationBLPositions);
      observationBRPositions = table.get("ObservationBRPositions", observationBRPositions);
      observationFRPositions = table.get("ObservationFRPositions", observationFRPositions);
      observationYaws = table.get("ObservationYaws", observationYaws);
    }
  }

  default ArrayBlockingQueue<WheeledObservation> start() {
    return new ArrayBlockingQueue<WheeledObservation>(0);
  }
}
