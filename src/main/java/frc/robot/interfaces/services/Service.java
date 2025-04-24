package frc.robot.interfaces.services;

/** 服务接口，定义所有服务的基本行为 */
public interface Service {
  /** 服务状态枚举 */
  enum ServiceState {
    STOPPED, // 未初始化
    INITIALIZED,
    RUNNING, // 运行中
    PAUSED, // 已暂停
    ERROR // 错误状态
  }

  /** 初始化服务 */
  void init();

  /** 更新服务状态 */
  void update();

  void stop();

  /**
   * 获取服务名称
   *
   * @return 服务名称
   */
  String getName();

  /**
   * 获取服务当前状态
   *
   * @return 服务状态
   */
  ServiceState getState();

  /**
   * 获取服务优先级（数字越小优先级越地）
   *
   * @return 服务优先级
   */
  default int getPriority() {
    return 0;
  }

  /** 暂停服务 */
  default void pause() {
    // 默认实现为空
  }

  /** 恢复服务 */
  default void resume() {
    // 默认实现为空
  }
}
