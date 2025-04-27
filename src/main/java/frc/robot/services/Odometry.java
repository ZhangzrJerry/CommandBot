package frc.robot.services;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.interfaces.services.PoseService;
import frc.robot.utils.dashboard.TunableNumber;
import frc.robot.utils.math.PoseUtil.UncertainPose2d;
import java.util.HashMap;
import lombok.Getter;
import org.littletonrobotics.junction.Logger;

// TODO
public class Odometry implements PoseService {
  @Getter private ServiceState state = ServiceState.STOPPED;
  @Getter private String errorMessage = "";

  private final HashMap<Double, UncertainPose2d> poseHistory = new HashMap<>();
  private final TunableNumber historyLength = new TunableNumber("Odometry/HistoryLength", 1.5);

  private Pose2d currentPose = new Pose2d();
  private Translation2d currentVelocity = new Translation2d();
  private Rotation2d currentHeading = new Rotation2d();

  public Odometry() {}

  @Override
  public String getName() {
    return "Odometry";
  }

  @Override
  public void setState(ServiceState state) {
    this.state = state;
  }

  @Override
  public void setError(String errorMessage) {
    this.errorMessage = errorMessage;
    setState(ServiceState.ERROR);
  }

  @Override
  public void init() {
    setState(ServiceState.RUNNING);
  }

  @Override
  public void update() {
    try {
      // 更新当前位姿
      updateCurrentPose();

      // 记录到 SmartDashboard
      SmartDashboard.putNumber("Odometry/X", currentPose.getX());
      SmartDashboard.putNumber("Odometry/Y", currentPose.getY());
      SmartDashboard.putNumber("Odometry/Heading", currentPose.getRotation().getDegrees());

      // 记录到 Logger
      Logger.recordOutput("Services/Odometry/Pose", currentPose);
      Logger.recordOutput("Services/Odometry/Velocity", currentVelocity);
    } catch (Exception e) {
      setError("Update error: " + e.getMessage());
    }
  }

  @Override
  public Pose2d getCurrentPose() {
    return currentPose;
  }

  @Override
  public Rotation2d getCurrentHeading() {
    return currentHeading;
  }

  @Override
  public Translation2d getCurrentVelocity() {
    return currentVelocity;
  }

  @Override
  public void resetPose(Pose2d pose) {
    currentPose = pose;
    currentHeading = pose.getRotation();
    poseHistory.clear();
  }

  @Override
  public void resetHeading(Rotation2d heading) {
    currentHeading = heading;
    currentPose = new Pose2d(currentPose.getTranslation(), heading);
  }

  @Override
  public void addTransformObservation(TransformObservation observation) {
    try {
      double startTime = observation.startTimestamp();
      double endTime = observation.endTimestamp();
      Transform2d transform = observation.transform();
      Matrix<N3, N1> stdDevs = observation.stdDevs();

      // 更新位姿缓冲区
      if (poseHistory.isEmpty()) {
        poseHistory.put(startTime, new UncertainPose2d(currentPose, Matrix.eye(Nat.N3())));
      }
      Pose2d endPose = currentPose.plus(transform);
      poseHistory.put(endTime, new UncertainPose2d(endPose, Matrix.eye(Nat.N3())));

      // 清理旧数据
      cleanupOldData();
    } catch (Exception e) {
      setError("Transform observation error: " + e.getMessage());
    }
  }

  @Override
  public void addPoseObservation(PoseObservation observation) {
    try {
      double timestamp = observation.timestamp();
      Pose2d pose = observation.pose();
      Matrix<N3, N1> stdDevs = observation.stdDevs();

      // 更新历史记录
      poseHistory.put(timestamp, new UncertainPose2d(pose, Matrix.eye(Nat.N3())));

      // 清理旧数据
      cleanupOldData();
    } catch (Exception e) {
      setError("Pose observation error: " + e.getMessage());
    }
  }

  private void updateCurrentPose() {
    try {
      // 使用最新的位姿观测值
      if (!poseHistory.isEmpty()) {
        double latestTime = poseHistory.keySet().stream().max(Double::compareTo).get();
        UncertainPose2d latestPose = poseHistory.get(latestTime);
        currentPose = latestPose.getPose();
        currentHeading = currentPose.getRotation();
      }

      // 计算速度
      if (poseHistory.size() >= 2) {
        double currentTime = Timer.getFPGATimestamp();
        double previousTime = currentTime - 0.02; // 20ms
        UncertainPose2d currentPose = poseHistory.get(currentTime);
        UncertainPose2d previousPose = poseHistory.get(previousTime);

        if (currentPose != null && previousPose != null) {
          Transform2d transform = new Transform2d(previousPose.getPose(), currentPose.getPose());
          currentVelocity = new Translation2d(transform.getX() / 0.02, transform.getY() / 0.02);
        }
      }
    } catch (Exception e) {
      setError("Pose update error: " + e.getMessage());
    }
  }

  private void cleanupOldData() {
    double currentTime = Timer.getFPGATimestamp();
    double cutoffTime = currentTime - historyLength.get();

    // 清理历史记录
    poseHistory.entrySet().removeIf(entry -> entry.getKey() < cutoffTime);
  }
}
