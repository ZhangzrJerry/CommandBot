package frc.robot.interfaces.services;

/**
 * Service interface that defines the basic behavior of all services. This interface provides
 * service lifecycle management, state control, and priority management.
 */
public interface Service {
  /** Service state enum representing different possible states of a service. */
  enum ServiceState {
    /** Service is stopped */
    STOPPED,
    /** Service is running normally */
    RUNNING,
    /** Service is paused */
    PAUSED,
    /** Service is in error state */
    ERROR
  }

  /**
   * Gets the current state of the service.
   *
   * @return Current service state
   */
  ServiceState getState();

  /**
   * Sets the service state.
   *
   * @param state New state to set
   */
  void setState(ServiceState state);

  /**
   * Gets the service priority. Lower numerical values indicate lower priority.
   *
   * @return Service priority value
   */
  default int getPriority() {
    return 0;
  }

  /**
   * Gets the service name.
   *
   * @return Service name
   */
  default String getName() {
    return "";
  }

  /** Initializes the service. Default implementation sets the service state to RUNNING. */
  default void init() {
    setState(ServiceState.RUNNING);
  }

  /**
   * Updates the service state. Subclasses can override this method to implement specific update
   * logic.
   */
  default void update() {}

  /** Stops the service. Default implementation sets the service state to STOPPED. */
  default void stop() {
    setState(ServiceState.STOPPED);
  }

  /** Pauses the service. Default implementation sets the service state to PAUSED. */
  default void pause() {
    setState(ServiceState.PAUSED);
  }

  /** Resumes the service. Default implementation sets the service state to RUNNING. */
  default void resume() {
    setState(ServiceState.RUNNING);
  }
}
