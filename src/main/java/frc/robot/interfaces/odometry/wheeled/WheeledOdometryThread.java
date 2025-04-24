package frc.robot.interfaces.odometry.wheeled;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import java.util.concurrent.ArrayBlockingQueue;

/** 轮式里程计线程接口 定义了轮式里程计数据采集的基本行为 */
public interface WheeledOdometryThread {
  public record WheeledObservation(
      double timestamp, SwerveModulePosition[] wheelPositions, Rotation2d yaw) {}

  /**
   * 启动里程计数据采集线程
   *
   * @return 用于存储观测数据的阻塞队列
   */
  default ArrayBlockingQueue<WheeledObservation> start() {
    return new ArrayBlockingQueue<WheeledObservation>(1);
  }

  /** 停止里程计数据采集线程 */
  default void stop() {}
}
