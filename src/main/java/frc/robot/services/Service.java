package frc.robot.services;

/**
 * 服务接口，所有服务类都应该实现这个接口
 */
public interface Service {
    /**
     * 初始化服务
     */
    void initialize();

    /**
     * 更新服务状态
     */
    void update();

    /**
     * 停止服务
     */
    void stop();
} 