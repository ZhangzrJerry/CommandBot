package frc.robot.services;

import java.util.ArrayList;
import java.util.List;

/**
 * 服务管理器，负责管理所有服务的生命周期
 */
public class ServiceManager {
    private static ServiceManager instance;
    private final List<Service> services = new ArrayList<>();

    private ServiceManager() {}

    /**
     * 获取服务管理器实例
     */
    public static ServiceManager getInstance() {
        if (instance == null) {
            instance = new ServiceManager();
        }
        return instance;
    }

    /**
     * 注册服务
     * @param service 要注册的服务
     */
    public void registerService(Service service) {
        services.add(service);
        service.initialize();
    }

    /**
     * 更新所有服务
     */
    public void updateAll() {
        for (Service service : services) {
            service.update();
        }
    }

    /**
     * 停止所有服务
     */
    public void stopAll() {
        for (Service service : services) {
            service.stop();
        }
    }

    /**
     * 获取指定类型的服务
     * @param serviceClass 服务类型
     * @return 服务实例，如果未找到返回null
     */
    @SuppressWarnings("unchecked")
    public <T extends Service> T getService(Class<T> serviceClass) {
        for (Service service : services) {
            if (serviceClass.isInstance(service)) {
                return (T) service;
            }
        }
        return null;
    }
} 