package frc.robot.interfaces.services;

/** 服务接口，定义服务的基本行为 */
public interface Service {
  /** 服务状态枚举 */
  enum ServiceState {
    UNINITIALIZED, // 未初始化
    INITIALIZING, // 初始化中
    RUNNING, // 运行中
    PAUSED, // 已暂停
    STOPPED, // 已停止
    ERROR // 错误状态
  }

  /** 初始化服务 */
  void initialize();

  /** 更新服务 */
  void update();

  /** 停止服务 */
  void stop();

  /**
   * 获取服务名称
   *
   * @return 服务名称
   */
  String getName();

  /**
   * 获取服务状态
   *
   * @return 服务状态
   */
  ServiceState getState();

  /**
   * 获取服务优先级
   *
   * @return 优先级，数值越小优先级越高
   */
  default int getPriority() {
    return 0;
  }

  /**
   * 获取服务依赖的其他服务
   *
   * @return 依赖的服务类数组
   */
  default Class<? extends Service>[] getDependencies() {
    return new Class[0];
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
