package frc.robot.services;

import frc.robot.interfaces.services.Service;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;
import java.util.Map;
import java.util.concurrent.ConcurrentHashMap;

/** Service manager for managing service lifecycles */
public class ServiceManager {
  private static ServiceManager instance;
  private final List<Service> services = new ArrayList<>();
  private final Map<Class<? extends Service>, Service> serviceMap = new ConcurrentHashMap<>();
  private boolean isInitialized = false;

  /** Custom exception for service-related errors */
  public static class ServiceException extends RuntimeException {
    public ServiceException(String message) {
      super(message);
    }

    public ServiceException(String message, Throwable cause) {
      super(message, cause);
    }
  }

  private ServiceManager() {}

  /** Get the singleton instance */
  public static ServiceManager getInstance() {
    if (instance == null) {
      instance = new ServiceManager();
    }
    return instance;
  }

  /**
   * Register a service
   *
   * @param service Service to register
   * @throws ServiceException If service registration fails
   */
  public void registerService(Service service) {
    if (isInitialized) {
      throw new ServiceException("Cannot register service after initialization");
    }

    if (service == null) {
      throw new ServiceException("Service cannot be null");
    }

    if (serviceMap.containsKey(service.getClass())) {
      throw new ServiceException("Service " + service.getClass().getName() + " already registered");
    }

    for (Class<? extends Service> dependency : service.getDependencies()) {
      if (!serviceMap.containsKey(dependency)) {
        throw new ServiceException(
            "Missing dependency: "
                + dependency.getName()
                + " for service: "
                + service.getClass().getName());
      }
    }

    services.add(service);
    serviceMap.put(service.getClass(), service);
  }

  /** Initialize all services */
  public void initializeAll() {
    if (isInitialized) {
      return;
    }

    Collections.sort(services, Comparator.comparingInt(Service::getPriority));

    for (Service service : services) {
      executeServiceOperation(service, "initialize", Service::initialize);
    }

    isInitialized = true;
  }

  /** Update all running services */
  public void updateAll() {
    if (!isInitialized) {
      return;
    }

    for (Service service : services) {
      if (service.getState() == Service.ServiceState.RUNNING) {
        executeServiceOperation(service, "update", Service::update);
      }
    }
  }

  /** Stop all services in reverse priority order */
  public void stopAll() {
    if (!isInitialized) {
      return;
    }

    List<Service> reversedServices = new ArrayList<>(services);
    Collections.reverse(reversedServices);

    for (Service service : reversedServices) {
      executeServiceOperation(service, "stop", Service::stop);
    }
  }

  /**
   * Get service by type
   *
   * @param serviceClass Service type
   * @return Service instance or null if not found
   * @throws ServiceException If service class is null
   */
  @SuppressWarnings("unchecked")
  public <T extends Service> T getService(Class<T> serviceClass) {
    if (serviceClass == null) {
      throw new ServiceException("Service class cannot be null");
    }
    return (T) serviceMap.get(serviceClass);
  }

  /** Get status string for all services */
  public String getServiceStatus() {
    StringBuilder status = new StringBuilder("Service Status:\n");
    for (Service service : services) {
      status.append(
          String.format(
              "%-30s %-15s Priority: %d\n",
              service.getName(), service.getState(), service.getPriority()));
    }
    return status.toString();
  }

  /** Set service state (pause/resume) */
  public void setServiceState(Class<? extends Service> serviceClass, boolean pause) {
    Service service = getService(serviceClass);
    if (service != null) {
      if (pause) {
        service.pause();
      } else {
        service.resume();
      }
    }
  }

  private void executeServiceOperation(
      Service service, String operationName, ServiceOperation operation) {
    try {
      operation.execute(service);
    } catch (ServiceException e) {
      // Re-throw ServiceException as is
      throw e;
    } catch (Exception e) {
      throw new ServiceException("Error " + operationName + "ing service: " + service.getName(), e);
    }
  }

  @FunctionalInterface
  private interface ServiceOperation {
    void execute(Service service);
  }
}
