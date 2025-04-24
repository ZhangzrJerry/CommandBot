package frc.robot.services;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.interfaces.services.Service;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/**
 * Visualization service for robot components in Advantage Scope. Maintains a transform tree and
 * updates component poses periodically.
 */
public class Visualization implements Service {
  private final List<VisualizationComponent> components = new ArrayList<>();
  private static final String LOG_PREFIX = "Services/Visualization/";
  private Service.ServiceState state = Service.ServiceState.STOPPED;
  public static String errorMsg = "";

  /**
   * Component record for visualization system.
   *
   * @param componentId Component identifier (0 to N)
   * @param parentId Parent component identifier (-1 for robot frame)
   * @param transformSupplier Transform supplier from parent to this component
   */
  private record VisualizationComponent(
      int componentId, int parentId, Supplier<Transform3d> transformSupplier) {}

  @Override
  public void init() {
    state = Service.ServiceState.INITIALIZED;
  }

  @Override
  public void update() {
    state = Service.ServiceState.RUNNING;

    if (components.isEmpty()) return;

    Pose3d[] poses = new Pose3d[components.size()];
    Transform3d[] transforms = new Transform3d[components.size()];

    for (int i = 0; i < components.size(); i++) {
      transforms[i] = components.get(i).transformSupplier().get();
      if (components.get(i).parentId() != -1) {
        transforms[i] = transforms[components.get(i).parentId()].plus(transforms[i]);
      }
      poses[i] = new Pose3d(transforms[i].getTranslation(), transforms[i].getRotation());
    }

    Logger.recordOutput(LOG_PREFIX + "Components", poses);
  }

  @Override
  public void stop() {
    state = Service.ServiceState.STOPPED;
  }

  @Override
  public String getName() {
    return "Visualization";
  }

  @Override
  public Service.ServiceState getState() {
    return state;
  }

  @Override
  public int getPriority() {
    return 0;
  }

  /**
   * Register a component for visualization
   *
   * @param component Component to register
   * @throws IllegalArgumentException If component ID is invalid or already exists
   */
  public void registerVisualizationComponent(
      int componentId, int parentId, Supplier<Transform3d> transformSupplier) {
    if (components.stream().anyMatch(c -> c.componentId() == componentId)) {
      state = Service.ServiceState.ERROR;
      errorMsg += "Component ID already exists: " + componentId + "\n";
      logErrorMsg();
      return;
    }
    if (componentId < 0) {
      state = Service.ServiceState.ERROR;
      errorMsg += "Invalid componentId: " + componentId + "\n";
      logErrorMsg();
      return;
    }
    if (parentId < -1) {
      state = Service.ServiceState.ERROR;
      errorMsg += "Invalid parentId: " + parentId + "\n";
      logErrorMsg();
      return;
    }
    if (parentId >= componentId) {
      state = Service.ServiceState.ERROR;
      errorMsg += "Parent ID must be less than component ID\n";
      logErrorMsg();
      return;
    }
    if (transformSupplier == null) {
      state = Service.ServiceState.ERROR;
      errorMsg += "Transform supplier cannot be null\n";
      logErrorMsg();
      return;
    }

    components.add(new VisualizationComponent(componentId, parentId, transformSupplier));
    components.sort(Comparator.comparingInt(VisualizationComponent::componentId));

    for (int i = 0; i < components.size(); i++) {
      if (components.get(i).componentId() != i) {
        state = Service.ServiceState.ERROR;
        errorMsg += "Component IDs must be continuous, missing ID: " + i + "\n";
        logErrorMsg();
        return;
      }
    }
    Logger.recordOutput(LOG_PREFIX + "ComponentCount", components.size());
  }

  @Override
  public void pause() {
    state = Service.ServiceState.PAUSED;
  }

  @Override
  public void resume() {
    state = Service.ServiceState.RUNNING;
  }

  private void logErrorMsg() {
    Logger.recordOutput(LOG_PREFIX + "ErrorMsg", errorMsg);
    System.err.println(errorMsg);
  }
}
