package frc.robot.interfaces.services;

/** 服务接口，定义所有服务的基本行为 */
public interface Service {
  /** 服务状态枚举 */
  enum ServiceState {
    STOPPED, // 已停止
    RUNNING, // 运行中
    PAUSED, // 已暂停
    ERROR // 错误状态
  }

  /**
   * 获取服务当前状态
   *
   * @return 服务状态
   */
  ServiceState getState();

  void setState(ServiceState state);

  /**
   * 获取服务优先级（数字越小优先级越地）
   *
   * @return 服务优先级
   */
  default int getPriority() {
    return 0;
  }

  /**
   * 获取服务名称
   *
   * @return 服务名称
   */
  default String getName() {
    return "";
  }

  /** 初始化服务 */
  default void init() {

    setState(ServiceState.RUNNING);
  }

  /** 更新服务状态 */
  default void update() {}

  default void stop() {

    setState(ServiceState.STOPPED);
  }

  /** 暂停服务 */
  default void pause() {
    setState(ServiceState.PAUSED);
  }

  /** 恢复服务 */
  default void resume() {
    setState(ServiceState.RUNNING);
  }
}
