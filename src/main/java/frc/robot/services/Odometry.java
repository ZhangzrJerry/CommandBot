package frc.robot.services;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.interfaces.services.OdometryService;
import frc.robot.utils.logging.LoggedTunableNumber;
import java.util.HashMap;
import java.util.Optional;
import lombok.Getter;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Odometry implements OdometryService {

  private static final double POSE_BUFFER_SIZE_SEC = 2.0;

  private static Odometry instance = null;

  public static Odometry getInstance() {
    if (instance == null) {
      instance = new Odometry();
    }

    return instance;
  }

  @AutoLogOutput(key = "Odometry/IncrementalPose")
  @Getter
  private Pose2d incrementalPose;

  @AutoLogOutput(key = "Odometry/EstimatedPose")
  @Getter
  private Pose2d estimatedPose;

  private ServiceState state = ServiceState.STOPPED;
  private final TimeInterpolatableBuffer<Pose2d> poseBuffer =
      TimeInterpolatableBuffer.createBuffer(POSE_BUFFER_SIZE_SEC);
  private final HashMap<Double, TransformObservation> transformObservations = new HashMap<>();
  private final HashMap<Double, PoseObservation> poseObservations = new HashMap<>();
  private final LoggedTunableNumber poseBufferSize =
      new LoggedTunableNumber("Odometry/PoseBufferSize", POSE_BUFFER_SIZE_SEC);

  @Override
  public void init() {
    state = ServiceState.INITIALIZED;
    incrementalPose = new Pose2d();
    estimatedPose = new Pose2d();
  }

  @Override
  public void update() {
    if (state != ServiceState.RUNNING && state != ServiceState.INITIALIZED) {
      return;
    }

    state = ServiceState.RUNNING;
    double currentTime = Timer.getFPGATimestamp();

    // 更新位姿缓冲区
    if (poseBufferSize.hasChanged(hashCode())) { // Pass an appropriate integer argument
      poseBuffer.clear();
    }

    // 处理变换观测
    for (var entry : transformObservations.entrySet()) {
      TransformObservation observation = entry.getValue();
      if (observation.endTimestamp() <= currentTime - POSE_BUFFER_SIZE_SEC) {
        transformObservations.remove(entry.getKey());
        continue;
      }

      // 更新增量位姿
      if (observation.endTimestamp() == currentTime) {
        incrementalPose = incrementalPose.plus(observation.transform());
      }
    }

    // 处理位姿观测
    for (var entry : poseObservations.entrySet()) {
      PoseObservation observation = entry.getValue();
      if (observation.timestamp() <= currentTime - POSE_BUFFER_SIZE_SEC) {
        poseObservations.remove(entry.getKey());
        continue;
      }

      // 更新估计位姿
      if (observation.timestamp() == currentTime) {
        estimatedPose = observation.pose();
      }
    }

    // 记录到日志
    Logger.recordOutput("Services/Odometry/CurrentTime", currentTime);
    Logger.recordOutput("Services/Odometry/State", state);
  }

  @Override
  public void stop() {
    state = ServiceState.STOPPED;
    poseBuffer.clear();
    transformObservations.clear();
    poseObservations.clear();
  }

  @Override
  public String getName() {
    return "Odometry";
  }

  @Override
  public ServiceState getState() {
    return state;
  }

  @Override
  public int getPriority() {
    return 0;
  }

  @Override
  public Pose2d getCurrentPose() {
    return estimatedPose;
  }

  @Override
  public Rotation2d getCurrentHeading() {
    return estimatedPose.getRotation();
  }

  @Override
  public Translation2d getCurrentVelocity() {
    double currentTime = Timer.getFPGATimestamp();
    Optional<Pose2d> currentPose = poseBuffer.getSample(currentTime);
    Optional<Pose2d> previousPose = poseBuffer.getSample(currentTime - 0.02);

    if (currentPose.isPresent() && previousPose.isPresent()) {
      Transform2d transform = new Transform2d(previousPose.get(), currentPose.get());
      return transform.getTranslation().div(0.02);
    }

    return new Translation2d();
  }

  @Override
  public void resetPose(Pose2d pose) {
    estimatedPose = pose;
    incrementalPose = pose;
    poseBuffer.clear();
    transformObservations.clear();
    poseObservations.clear();
  }

  @Override
  public void resetHeading(Rotation2d heading) {
    estimatedPose = new Pose2d(estimatedPose.getTranslation(), heading);
    incrementalPose = new Pose2d(incrementalPose.getTranslation(), heading);
  }

  @Override
  public void addTransformObservation(TransformObservation observation) {
    transformObservations.put(observation.endTimestamp(), observation);
  }

  @Override
  public void addPoseObservation(PoseObservation observation) {
    poseObservations.put(observation.timestamp(), observation);
    poseBuffer.addSample(observation.timestamp(), observation.pose());
  }
}
