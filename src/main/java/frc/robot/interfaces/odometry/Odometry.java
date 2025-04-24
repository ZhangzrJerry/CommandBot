package frc.robot.interfaces.odometry;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;

public interface Odometry {
  /** 获取当前机器人位姿 */
  Pose2d getPose();

  /** 重置里程计到指定位姿 */
  void resetPose(Pose2d pose);

  /** 更新里程计状态 */
  void update();

  /** 获取当前速度 */
  Twist2d getVelocity();

  /** 获取当前偏航角 */
  Rotation2d getHeading();

  /** 获取里程计置信度 */
  double getConfidence();
}
