package frc.robot.subsystems.odometry;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N3;
import frc.robot.virtuals.VirtualSubsystem;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;
import lombok.Getter;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

/** A subsystem that fuses multiple pose observations with covariance tracking. */
public class Odometry extends VirtualSubsystem {
  private static final double CONVERGENCE_THRESHOLD = 0.01; // meters
  private static final int MAX_ITERATIONS = 10;
  private static final double DEFAULT_COVARIANCE = 0.1;

  private final List<PoseObservation> observations = new ArrayList<>();
  private final Map<String, Matrix<N3, N3>> observationCovariances = new HashMap<>();

  @AutoLogOutput(key = "Odometry/EstimatedPose")
  @Getter
  private Pose2d estimatedPose = new Pose2d();

  @Getter private Matrix<N3, N3> poseCovariance = Matrix.eye(Nat.N3()).times(DEFAULT_COVARIANCE);

  /** Represents a pose observation source with covariance. */
  public record PoseObservation(
      String name,
      Supplier<Pose2d> poseSupplier,
      Supplier<Matrix<N3, N3>> covarianceSupplier,
      Supplier<Double> timestampSupplier) {

    public PoseObservation {
      if (name == null || name.isEmpty()) {
        throw new IllegalArgumentException("Observation name cannot be null or empty");
      }
      if (poseSupplier == null) {
        throw new IllegalArgumentException("Pose supplier cannot be null");
      }
      if (covarianceSupplier == null) {
        throw new IllegalArgumentException("Covariance supplier cannot be null");
      }
      if (timestampSupplier == null) {
        throw new IllegalArgumentException("Timestamp supplier cannot be null");
      }
    }
  }

  /**
   * Registers a new pose observation source and returns a supplier that indicates if the fusion is
   * better than this source.
   */
  public BooleanSupplier registerObservation(PoseObservation observation) {
    if (observations.stream().anyMatch(obs -> obs.name().equals(observation.name()))) {
      throw new IllegalArgumentException(
          "Observation with name '" + observation.name() + "' already exists");
    }
    observations.add(observation);

    // 返回一个BooleanSupplier，用于判断融合位姿是否比当前源的位姿估计更好
    return () -> {
      Matrix<N3, N3> sourceCovariance = observation.covarianceSupplier().get();
      Matrix<N3, N3> fusedCovariance = poseCovariance;

      // 比较协方差矩阵的迹（trace），迹越小表示估计越准确
      double sourceTrace = sourceCovariance.trace();
      double fusedTrace = fusedCovariance.trace();

      return fusedTrace < sourceTrace;
    };
  }

  @Override
  public void periodic() {
    if (!observations.isEmpty()) {
      optimizePoseEstimate();
      logObservations();
    }
  }

  private void optimizePoseEstimate() {
    // 实现位姿优化算法
    // 这里可以使用卡尔曼滤波或其他融合算法
  }

  private void logObservations() {
    for (PoseObservation observation : observations) {
      String baseKey = "Odometry/Observation/" + observation.name();
      Logger.recordOutput(baseKey + "/Pose", observation.poseSupplier().get());
      Logger.recordOutput(baseKey + "/Timestamp", observation.timestampSupplier.get());
      Logger.recordOutput(baseKey + "/cov X", observation.covarianceSupplier.get().get(0, 0));
      Logger.recordOutput(baseKey + "/cov Y", observation.covarianceSupplier.get().get(1, 1));
      Logger.recordOutput(baseKey + "/cov W", observation.covarianceSupplier.get().get(2, 2));
    }
  }
}
