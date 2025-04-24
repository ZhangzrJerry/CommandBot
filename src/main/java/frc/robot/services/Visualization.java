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
  private ServiceState state = ServiceState.STOPPED;

  /**
   * Component record for visualization system.
   *
   * @param componentId Component identifier (0 to N)
   * @param parentId Parent component identifier (-1 for robot frame)
   * @param transformSupplier Transform supplier from parent to this component
   */
  public record VisualizationComponent(
      int componentId, int parentId, Supplier<Transform3d> transformSupplier) {
    public VisualizationComponent {
      validateComponent(componentId, parentId, transformSupplier);
    }

    private void validateComponent(
        int componentId, int parentId, Supplier<Transform3d> transformSupplier) {
      if (componentId < 0)
        throw new IllegalArgumentException("Invalid componentId: " + componentId);
      if (parentId < -1) throw new IllegalArgumentException("Invalid parentId: " + parentId);
      if (parentId >= componentId)
        throw new IllegalArgumentException("Parent ID must be less than component ID");
      if (transformSupplier == null)
        throw new IllegalArgumentException("Transform supplier cannot be null");
    }
  }

  @Override
  public void initialize() {
    state = ServiceState.RUNNING;
    Logger.recordOutput(LOG_PREFIX + "Running", true);
  }

  @Override
  public void update() {
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
    state = ServiceState.STOPPED;
    Logger.recordOutput(LOG_PREFIX + "Running", false);
  }

  @Override
  public String getName() {
    return "Visualization";
  }

  @Override
  public ServiceState getState() {
    return state;
  }

  /**
   * Register a component for visualization
   *
   * @param component Component to register
   * @throws IllegalArgumentException If component ID is invalid or already exists
   */
  public void registerVisualizationComponent(VisualizationComponent component) {
    if (components.stream().anyMatch(c -> c.componentId() == component.componentId())) {
      throw new IllegalArgumentException("Component ID already exists: " + component.componentId());
    }

    components.add(component);
    components.sort(Comparator.comparingInt(VisualizationComponent::componentId));

    validateComponentIds();
    Logger.recordOutput(LOG_PREFIX + "ComponentCount", components.size());
  }

  private void validateComponentIds() {
    for (int i = 0; i < components.size(); i++) {
      if (components.get(i).componentId() != i) {
        throw new IllegalArgumentException("Component IDs must be continuous, missing ID: " + i);
      }
    }
  }
}
