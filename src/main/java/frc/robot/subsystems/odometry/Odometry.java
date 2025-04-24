package frc.robot.subsystems.odometry;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.utils.VirtualSubsystem;
import frc.robot.utils.logging.LoggedUtil;
import frc.robot.utils.math.PoseUtil.UncertainPose2d;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import java.util.function.Supplier;
import lombok.Getter;
import org.littletonrobotics.junction.Logger;

/** A subsystem that fuses multiple pose observations with covariance tracking. */
public class Odometry extends VirtualSubsystem {
  private final List<PoseObservation> observations = new ArrayList<>();

  @Getter private UncertainPose2d estimatedPose = new UncertainPose2d(new Pose2d());
  private UncertainPose2d bestEstimatedPose =
      new UncertainPose2d(
          new Pose2d(),
          Double.POSITIVE_INFINITY,
          Double.POSITIVE_INFINITY,
          Double.POSITIVE_INFINITY);
  private double lastUpdateTime = Timer.getFPGATimestamp();

  // Time decay parameters
  private static final double HALF_LIFE_SECONDS =
      1.0; // Observations decay to half weight after this time
  private static final double MIN_WEIGHT = 0.1; // Minimum weight for any observation
  private static final double STALE_THRESHOLD_SECONDS = 0.3; // Consider stale after 2 seconds
  private static final double COVARIANCE_GROWTH_RATE = 1.2; // Covariance growth factor per second

  /** Represents a pose observation source with covariance. */
  public record PoseObservation(
      String name,
      Supplier<UncertainPose2d> poseSupplier,
      Supplier<Double> timestampSupplier,
      double baseWeight) {
    public PoseObservation {
      if (name == null || name.isEmpty()) {
        throw new IllegalArgumentException("Observation name cannot be null or empty");
      }
      if (poseSupplier == null) {
        throw new IllegalArgumentException("Pose supplier cannot be null");
      }
      if (timestampSupplier == null) {
        throw new IllegalArgumentException("Timestamp supplier cannot be null");
      }
      baseWeight = Math.max(0.0, Math.min(1.0, baseWeight));
    }

    public PoseObservation(
        String name, Supplier<UncertainPose2d> poseSupplier, Supplier<Double> timestampSupplier) {
      this(name, poseSupplier, timestampSupplier, 1.0);
    }

    public double getTimeWeight(double currentTime) {
      double age = currentTime - timestampSupplier.get();
      // Exponential decay based on age
      double timeWeight = Math.pow(0.5, age / HALF_LIFE_SECONDS);
      return Math.max(MIN_WEIGHT, timeWeight * baseWeight);
    }
  }

  /** Registers a new pose observation source */
  public Command registerObservation(PoseObservation observation) {
    return Commands.runOnce(
            () -> {
              if (observations.stream().anyMatch(obs -> obs.name().equals(observation.name()))) {
                throw new IllegalArgumentException(
                    "Observation with name '" + observation.name() + "' already exists");
              }
              observations.add(observation);
              bestEstimatedPose =
                  bestEstimatedPose.isBetterThan(observation.poseSupplier.get())
                      ? observation.poseSupplier.get()
                      : bestEstimatedPose;
            })
        .withName("[Odometry] Register Observation: " + observation.name());
  }

  @Override
  public void periodic() {
    if (observations.isEmpty()) {
      return;
    }

    double currentTime = Timer.getFPGATimestamp();
    double timeSinceLastUpdate = currentTime - lastUpdateTime;
    lastUpdateTime = currentTime;

    estimatedPose =
        observations.stream()
            .max(Comparator.comparingDouble(observation -> observation.getTimeWeight(currentTime)))
            .map(observation -> observation.poseSupplier.get())
            .orElseGet(
                () -> {
                  return new UncertainPose2d(
                      estimatedPose.getPose(),
                      estimatedPose.getCovariance().times(COVARIANCE_GROWTH_RATE));
                });

    for (PoseObservation observation : observations) {
      UncertainPose2d currentPose = observation.poseSupplier.get();
      String baseKey = "Odometry/Observation/" + observation.name();
      Logger.recordOutput(baseKey + "/Pose", currentPose.getPose());
      LoggedUtil.logMatrix(baseKey + "/Covariance", currentPose.getCovariance());
      Logger.recordOutput(baseKey + "/Weight", observation.getTimeWeight(currentTime));
      Logger.recordOutput(baseKey + "/Age", currentTime - observation.timestampSupplier.get());

      double weight = observation.getTimeWeight(currentTime);
      double age = currentTime - observation.timestampSupplier.get();
      if (weight < MIN_WEIGHT * 0.5 || age > STALE_THRESHOLD_SECONDS) {
        continue; // Skip stale observations
      }
      double ageFactor = 1.0 + (age / HALF_LIFE_SECONDS);

      estimatedPose =
          estimatedPose.fusedWith(
              new UncertainPose2d(
                  currentPose.getPose(), currentPose.getCovariance().times(ageFactor / weight)));
    }

    Logger.recordOutput("Odometry/EstimatedPose", estimatedPose.getPose());
    LoggedUtil.logMatrix("Odometry/EstimatedCovariance", estimatedPose.getCovariance());
  }
}
