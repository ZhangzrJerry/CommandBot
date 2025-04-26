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

/**
 * Manages the lifecycle of all services in the robot. This singleton class
 * handles service
 * registration, initialization, updates, and state management. Services are
 * prioritized and can be
 * paused/resumed individually.
 */
public class ServiceManager {
  private static ServiceManager instance;
  private final Map<String, Service> services = new ConcurrentHashMap<>();
  private final List<Service> serviceList = new ArrayList<>();
  private final List<Integer> servicePriorities = new ArrayList<>();
  private final List<AlertManager> alertList = new ArrayList<>();
  private final Map<String, String> lastErrorMessages = new ConcurrentHashMap<>();

  @AutoLogOutput(key = "Services/Initialized")
  private boolean isInitialized = false;

  private ServiceManager() {
  }

  /**
   * Gets the singleton instance of the ServiceManager.
   *
   * @return The ServiceManager instance
   */
  public static ServiceManager getInstance() {
    if (instance == null) {
      instance = new ServiceManager();
    }
    return instance;
  }

  /**
   * Registers a new service with the manager. Services are automatically sorted
   * by priority after
   * registration.
   *
   * @param service The service to register
   * @throws IllegalArgumentException if the service is null, has an empty name,
   *                                  or a service with
   *                                  the same name already exists
   */
  public void registerService(Service service) {
    registerService(service, 99);
  }

  public void registerService(Service service, int priority) {
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
    servicePriorities.add(priority);
    // Sort both lists based on priority
    for (int i = 0; i < serviceList.size() - 1; i++) {
      for (int j = i + 1; j < serviceList.size(); j++) {
        if (servicePriorities.get(i) > servicePriorities.get(j)) {
          // Swap services
          Service tempService = serviceList.get(i);
          serviceList.set(i, serviceList.get(j));
          serviceList.set(j, tempService);
          // Swap priorities
          int tempPriority = servicePriorities.get(i);
          servicePriorities.set(i, servicePriorities.get(j));
          servicePriorities.set(j, tempPriority);
        }
      }
    }
    alertList.add(new AlertManager(name + " ERRORED", AlertManager.AlertType.ERROR));
  }

  /**
   * Retrieves a service by its name.
   *
   * @param name The name of the service to retrieve
   * @return The service instance, or null if not found
   */
  public Service getService(String name) {
    return services.get(name);
  }

  /**
   * Retrieves a service by its class type.
   *
   * @param <T>          The type of service to retrieve
   * @param serviceClass The class of the service to retrieve
   * @return The service instance, or null if not found
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
   * Retrieves a service by both its class type and name.
   *
   * @param <T>          The type of service to retrieve
   * @param serviceClass The class of the service to retrieve
   * @param name         The name of the service to retrieve
   * @return The service instance, or null if not found
   */
  @SuppressWarnings("unchecked")
  public <T extends Service> T getService(Class<T> serviceClass, String name) {
    Service service = services.get(name);
    if (service != null && serviceClass.isInstance(service)) {
      return (T) service;
    }
    return null;
  }

  /**
   * Initializes all registered services. This method should be called once during
   * robot
   * initialization.
   */
  public void initAll() {
    if (isInitialized) {
      return;
    }

    for (Service service : serviceList) {
      try {
        service.init();
      } catch (Exception e) {
        service.setError("Init error: " + e.getMessage());
      }
    }
    isInitialized = true;
  }

  /**
   * Updates all running services. This method should be called periodically
   * (e.g., in the robot's
   * periodic method).
   */
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
        service.setError("Update error: " + e.getMessage());
      }
    }
    logServicesStatus();
  }

  /**
   * Stops all running services. This method should be called when the robot is
   * shutting down.
   */
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
        service.setError("Stop error: " + e.getMessage());
      }
    }
  }

  /**
   * Gets a list of all registered services.
   *
   * @return A new list containing all registered services
   */
  public List<Service> getAllServices() {
    return new ArrayList<>(serviceList);
  }

  /**
   * Gets a formatted string containing the status of all services.
   *
   * @return A formatted string showing service names, states, and priorities
   */
  public String getServiceStatus() {
    StringBuilder status = new StringBuilder();
    status.append("-----------------------------------------------\n");
    status.append(
        String.format(
            "%-20s %-15s %-10s\n", "Service Name", "State", "Priority"));
    status.append("-----------------------------------------------\n");

    for (int i = 0; i < serviceList.size(); i++) {
      Service service = serviceList.get(i);
      status.append(
          String.format(
              "%-20s %-15s %-10d\n",
              service.getName(),
              service.getState(),
              servicePriorities.get(i)));
    }
    status.append("-----------------------------------------------\n");

    return status.toString();
  }

  /**
   * Logs the current status of all services. This includes state and priority
   * information for each
   * service.
   */
  public void logServicesStatus() {
    for (int i = 0; i < serviceList.size(); ++i) {
      Service service = serviceList.get(i);
      boolean isErrored = service.getState() == Service.ServiceState.ERROR;
      String currentErrorMessage = service.getErrorMessage();
      String serviceName = service.getName();

      if (isErrored) {
        String lastErrorMessage = lastErrorMessages.get(serviceName);
        if (!currentErrorMessage.equals(lastErrorMessage)) {
          System.out.println("Services/" + serviceName + "/Error: " + currentErrorMessage);
          lastErrorMessages.put(serviceName, currentErrorMessage);
        }
      } else {
        lastErrorMessages.remove(serviceName);
      }

      alertList.get(i).set(isErrored);

      Logger.recordOutput("Services/" + serviceName + "/Status", service.getState());
      Logger.recordOutput("Services/" + serviceName + "/Priority", servicePriorities.get(i));
      Logger.recordOutput("Services/" + serviceName + "/ErrorMessage", currentErrorMessage);
    }
  }

  /**
   * Pauses a specific service by name.
   *
   * @param serviceName The name of the service to pause
   */
  public void pauseService(String serviceName) {
    Service service = services.get(serviceName);
    if (service != null) {
      service.pause();
    }
  }

  /**
   * Resumes a specific service by name.
   *
   * @param serviceName The name of the service to resume
   */
  public void resumeService(String serviceName) {
    Service service = services.get(serviceName);
    if (service != null) {
      service.resume();
    }
  }

  /**
   * Gets all services in a specific state.
   *
   * @param state The state to filter services by
   * @return A list of services in the specified state
   */
  public List<Service> getServicesByState(Service.ServiceState state) {
    return serviceList.stream()
        .filter(service -> service.getState() == state)
        .collect(Collectors.toList());
  }

  /**
   * Gets all services that are in ERROR state.
   *
   * @return A list of services in ERROR state
   */
  public List<Service> getErroredServices() {
    return getServicesByState(Service.ServiceState.ERROR);
  }

  /**
   * Gets all error messages from services in ERROR state.
   *
   * @return A list of error messages
   */
  public List<String> getAllErrorMessages() {
    return serviceList.stream()
        .filter(service -> service.getState() == Service.ServiceState.ERROR)
        .map(Service::getErrorMessage)
        .collect(Collectors.toList());
  }
}
