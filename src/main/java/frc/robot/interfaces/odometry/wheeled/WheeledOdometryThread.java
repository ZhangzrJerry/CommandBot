package frc.robot.interfaces.odometry.wheeled;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import java.util.concurrent.ArrayBlockingQueue;

/**
 * Wheeled odometry thread interface Defines the basic behavior for wheeled odometry data collection
 */
public interface WheeledOdometryThread {

  public record WheeledObservation(
      double timestamp, SwerveModulePosition[] wheelPositions, Rotation2d yaw) {}

  /**
   * Starts the odometry data collection thread
   *
   * @return A blocking queue used to store observation data
   */
  default ArrayBlockingQueue<WheeledObservation> start() {
    return new ArrayBlockingQueue<WheeledObservation>(1);
  }

  /** Stops the odometry data collection thread */
  default void stop() {}
}
