package frc.robot.services;

import frc.robot.interfaces.services.Service;
import frc.robot.utils.dashboard.AlertManager;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.concurrent.ConcurrentHashMap;
import java.util.stream.Collectors;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

/** 服务管理器，负责管理所有服务的生命周期 */
public class ServiceManager {
  private static ServiceManager instance;
  private final Map<String, Service> services = new ConcurrentHashMap<>();
  private final List<Service> serviceList = new ArrayList<>();
  private final List<AlertManager> alertList = new ArrayList<>();

  @AutoLogOutput(key = "Services/Initialized")
  private boolean isInitialized = false;

  private ServiceManager() {}

  /**
   * 获取 ServiceManager 的单例实例
   *
   * @return ServiceManager 实例
   */
  public static ServiceManager getInstance() {
    if (instance == null) {
      instance = new ServiceManager();
    }
    return instance;
  }

  /**
   * 注册服务
   *
   * @param service 要注册的服务
   * @throws IllegalArgumentException 如果服务名称为空或已存在同名服务
   */
  public void registerService(Service service) {
    if (service == null) {
      throw new IllegalArgumentException("Service cannot be null");
    }

    String name = service.getName();
    if (name == null || name.trim().isEmpty()) {
      throw new IllegalArgumentException("Service name cannot be null or empty");
    }

    if (services.containsKey(name)) {
      throw new IllegalArgumentException("Service with name '" + name + "' already exists");
    }

    services.put(name, service);
    serviceList.add(service);
    serviceList.sort((s1, s2) -> Integer.compare(s2.getPriority(), s1.getPriority()));
    alertList.add(new AlertManager(name + " ERRORED", AlertManager.AlertType.ERROR));
  }

  /**
   * 获取指定名称的服务
   *
   * @param name 服务名称
   * @return 服务实例，如果不存在则返回 null
   */
  public Service getService(String name) {
    return services.get(name);
  }

  /**
   * 获取指定类型的服务
   *
   * @param <T> 服务类型
   * @param serviceClass 服务类
   * @return 服务实例，如果不存在则返回 null
   */
  @SuppressWarnings("unchecked")
  public <T extends Service> T getService(Class<T> serviceClass) {
    for (Service service : serviceList) {
      if (serviceClass.isInstance(service)) {
        return (T) service;
      }
    }
    return null;
  }

  /**
   * 获取指定类型和名称的服务
   *
   * @param <T> 服务类型
   * @param serviceClass 服务类
   * @param name 服务名称
   * @return 服务实例，如果不存在则返回 null
   */
  @SuppressWarnings("unchecked")
  public <T extends Service> T getService(Class<T> serviceClass, String name) {
    Service service = services.get(name);
    if (service != null && serviceClass.isInstance(service)) {
      return (T) service;
    }
    return null;
  }

  /** 初始化所有服务 */
  public void initAll() {
    if (isInitialized) {
      return;
    }

    for (Service service : serviceList) {
      try {
        service.init();
      } catch (Exception e) {
        System.err.println(
            "Error initializing service " + service.getName() + ": " + e.getMessage());
      }
    }
    isInitialized = true;
  }

  /** 更新所有服务 */
  public void updateAll() {
    if (!isInitialized) {
      return;
    }

    for (Service service : serviceList) {
      try {
        if (service.getState() == Service.ServiceState.RUNNING) {
          service.update();
        }
      } catch (Exception e) {
        System.err.println("Error updating service " + service.getName() + ": " + e.getMessage());
      }
    }
    logServicesStatus();
  }

  public void stopAll() {
    if (!isInitialized) {
      return;
    }

    for (Service service : serviceList) {
      try {
        if (service.getState() == Service.ServiceState.RUNNING) {
          service.update();
        }
      } catch (Exception e) {
        System.err.println("Error updating service " + service.getName() + ": " + e.getMessage());
      }
    }
  }

  /**
   * 获取所有已注册的服务
   *
   * @return 服务列表
   */
  public List<Service> getAllServices() {
    return new ArrayList<>(serviceList);
  }

  /**
   * 获取服务状态统计信息
   *
   * @return 状态统计信息字符串
   */
  public String getServiceStatus() {
    StringBuilder status = new StringBuilder();
    status.append("----------------------------------------\n");
    status.append(String.format("%-20s %-15s %-10s\n", "服务名称", "状态", "优先级"));
    status.append("----------------------------------------\n");

    for (Service service : serviceList) {
      status.append(
          String.format(
              "%-20s %-15s %-10d\n", service.getName(), service.getState(), service.getPriority()));
    }

    return status.toString();
  }

  public void logServicesStatus() {
    for (int i = 0; i < serviceList.size(); ++i) {
      alertList.get(i).set(serviceList.get(i).getState() == Service.ServiceState.ERROR);
      Logger.recordOutput(
          "Services/" + serviceList.get(i).getName() + "/Status", serviceList.get(i).getState());
      Logger.recordOutput(
          "Services/" + serviceList.get(i).getName() + "/Priority",
          serviceList.get(i).getPriority());
    }
  }

  /**
   * 暂停指定服务
   *
   * @param serviceName 服务名称
   */
  public void pauseService(String serviceName) {
    Service service = services.get(serviceName);
    if (service != null) {
      service.pause();
    }
  }

  /**
   * 恢复指定服务
   *
   * @param serviceName 服务名称
   */
  public void resumeService(String serviceName) {
    Service service = services.get(serviceName);
    if (service != null) {
      service.resume();
    }
  }

  /**
   * 获取指定状态的服务列表
   *
   * @param state 要查询的状态
   * @return 处于指定状态的服务列表
   */
  public List<Service> getServicesByState(Service.ServiceState state) {
    return serviceList.stream()
        .filter(service -> service.getState() == state)
        .collect(Collectors.toList());
  }
}
