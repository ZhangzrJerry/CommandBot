package frc.robot.interfaces.services;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

/** 里程计服务接口，提供机器人位置和姿态信息 */
public interface OdometryService extends Service {
  /** 变换观测记录，表示相对运动 */
  public record TransformObservation(
      double startTimestamp, double endTimestamp, Transform2d transform, Matrix<N3, N1> stdDevs) {}

  /** 位姿观测记录，表示绝对位置 */
  public record PoseObservation(double timestamp, Pose2d pose, Matrix<N3, N1> stdDevs) {}

  /**
   * 获取当前机器人位置
   *
   * @return 机器人当前位置
   */
  Pose2d getCurrentPose();

  /**
   * 获取当前机器人朝向
   *
   * @return 机器人当前朝向
   */
  Rotation2d getCurrentHeading();

  /**
   * 获取当前机器人速度
   *
   * @return 机器人当前速度
   */
  Translation2d getCurrentVelocity();

  /**
   * 重置里程计位置
   *
   * @param pose 新的位置
   */
  void resetPose(Pose2d pose);

  /**
   * 重置里程计朝向
   *
   * @param heading 新的朝向
   */
  void resetHeading(Rotation2d heading);

  /**
   * 获取里程计精度
   *
   * @return 位置精度（米）
   */
  default double getPositionAccuracy() {
    return 0.05; // 默认5厘米
  }

  /**
   * 获取朝向精度
   *
   * @return 朝向精度（弧度）
   */
  default double getHeadingAccuracy() {
    return Math.toRadians(1.0); // 默认1度
  }

  /**
   * 添加变换观测
   *
   * @param observation 变换观测
   */
  void addTransformObservation(TransformObservation observation);

  /**
   * 添加位姿观测
   *
   * @param observation 位姿观测
   */
  void addPoseObservation(PoseObservation observation);
}
