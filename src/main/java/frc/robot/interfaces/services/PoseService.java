package frc.robot.interfaces.services;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

/**
 * Robot pose service interface that provides robot position and orientation information. This
 * service is responsible for tracking and estimating the robot's pose in 2D space.
 */
public interface PoseService extends Service {
  /**
   * Represents a relative motion observation with associated uncertainty.
   *
   * @param startTimestamp Start time of the observation
   * @param endTimestamp End time of the observation
   * @param transform The relative transformation between start and end
   * @param stdDevs Standard deviations of the transformation components
   */
  public record TransformObservation(
      double startTimestamp, double endTimestamp, Transform2d transform, Matrix<N3, N1> stdDevs) {}

  /**
   * Represents an absolute position observation with associated uncertainty.
   *
   * @param timestamp Time of the observation
   * @param pose The absolute pose of the robot
   * @param stdDevs Standard deviations of the pose components
   */
  public record PoseObservation(double timestamp, Pose2d pose, Matrix<N3, N1> stdDevs) {}

  /**
   * Gets the current estimated pose of the robot.
   *
   * @return The current robot pose in field coordinates
   */
  default Pose2d getCurrentPose() {
    return new Pose2d();
  }

  /**
   * Gets the current heading (rotation) of the robot.
   *
   * @return The current robot heading as a Rotation2d
   */
  default Rotation2d getCurrentHeading() {
    return new Rotation2d();
  }

  /**
   * Gets the current velocity of the robot.
   *
   * @return The current robot velocity as a Translation2d
   */
  default Translation2d getCurrentVelocity() {
    return new Translation2d();
  }

  /**
   * Resets the robot pose to a specific position and orientation. This is typically used when the
   * robot's absolute position is known.
   *
   * @param pose The new pose to set
   */
  default void resetPose(Pose2d pose) {}

  /**
   * Resets the robot heading to a specific angle. This is typically used when the robot's heading
   * is known.
   *
   * @param heading The new heading to set
   */
  default void resetHeading(Rotation2d heading) {}

  /**
   * Gets the estimated position accuracy of the pose estimation system. This represents the
   * standard deviation of position estimates.
   *
   * @return Position accuracy in meters
   */
  default double getPositionAccuracy() {
    return 0.05; // Default 5cm accuracy
  }

  /**
   * Gets the estimated heading accuracy of the pose estimation system. This represents the standard
   * deviation of heading estimates.
   *
   * @return Heading accuracy in radians
   */
  default double getHeadingAccuracy() {
    return Math.toRadians(1.0); // Default 1 degree accuracy
  }

  /**
   * Adds a relative motion observation to the pose estimation system. This is typically used for
   * dead reckoning or sensor fusion.
   *
   * @param observation The transform observation to add
   */
  default void addTransformObservation(TransformObservation observation) {}

  /**
   * Adds an absolute position observation to the pose estimation system. This is typically used for
   * sensor fusion or position correction.
   *
   * @param observation The pose observation to add
   */
  default void addPoseObservation(PoseObservation observation) {}
}
