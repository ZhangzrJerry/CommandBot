package frc.robot.services;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.interfaces.services.Service;
import frc.robot.utils.math.GeomUtil;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import java.util.function.Supplier;
import lombok.Getter;
import lombok.Setter;
import lombok.experimental.ExtensionMethod;
import org.littletonrobotics.junction.Logger;

/**
 * Service for visualizing robot components in 3D space. This service manages a hierarchical
 * structure of robot components and their transformations, allowing for real-time visualization of
 * the robot's state.
 */
@ExtensionMethod({GeomUtil.class})
public class TransformTree implements Service {
  @Getter @Setter ServiceState state = ServiceState.STOPPED;

  @Override
  public int getPriority() {
    return 0;
  }

  @Override
  public String getName() {
    return "TransformTree";
  }

  @Override
  public void init() {
    setState(ServiceState.RUNNING);
    components.sort(Comparator.comparingInt(TransformComponent::componentId));
    // Verify component IDs are continuous
    for (int i = 0; i < components.size(); ++i) {
      if (components.get(i).componentId() != i) {
        throw new IllegalArgumentException("componentId should be continuous, missing id: " + i);
      }
    }
  }

  @Override
  public void update() {
    componentPoses = new Pose3d[components.size()];
    componentTransforms = new Transform3d[components.size()];

    if (STRICT_BIG_ENDIAN) {
      for (int i = 0; i < components.size(); ++i) {
        componentTransforms[i] = components.get(i).transformSupplier().get();

        if (components.get(i).parentId() != -1) {
          // Propagate transform matrix through the component hierarchy
          componentTransforms[i] =
              componentTransforms[components.get(i).parentId()].plus(componentTransforms[i]);
        }
        componentPoses[i] = componentTransforms[i].toPose3d();
      }
    }

    Logger.recordOutput("Services/" + getName() + "/Components", componentPoses);
  }

  public Pose3d getComponentPose(int componentId) {
    if (componentId < 0 || componentId >= componentPoses.length) {
      throw new IllegalArgumentException("componentId out of index");
    }
    return componentPoses[componentId];
  }

  public Transform3d getComponentTransform(int componentId) {
    if (componentId < 0 || componentId >= componentTransforms.length) {
      throw new IllegalArgumentException("componentId out of index");
    }
    return componentTransforms[componentId];
  }

  private static final List<TransformComponent> components = new ArrayList<>();
  private static final Boolean STRICT_BIG_ENDIAN = true;
  private static Pose3d[] componentPoses = new Pose3d[0];
  private static Transform3d[] componentTransforms = new Transform3d[0];

  /**
   * Represents a component in the transform hierarchy. Each component has a unique ID, a parent
   * component, and a transform relative to its parent.
   *
   * @param componentId Unique identifier for the component, must be in range [0,N]
   * @param parentId Identifier of the parent component, must be in range [-1,componentId). -1
   *     indicates the robot frame as parent
   * @param transformSupplier Supplier providing the transform matrix from parent to this component
   */
  public record TransformComponent(
      int componentId, int parentId, Supplier<Transform3d> transformSupplier) {
    public TransformComponent {
      if (componentId < 0) {
        throw new IllegalArgumentException("componentId out of index");
      }
      if (parentId < -1) {
        throw new IllegalArgumentException("parentId out of index");
      }
      if (STRICT_BIG_ENDIAN && parentId >= componentId) {
        throw new IllegalArgumentException(
            "componentId should not greater than or equal to parentId");
      }
      if (transformSupplier == null) {
        throw new IllegalArgumentException("transformSupplier should not be null");
      }
    }
  }

  /**
   * Registers a component for transform tracking. Components should typically be registered during
   * subsystem initialization.
   *
   * @param component The component to be registered
   * @throws IllegalArgumentException if a component with the same ID already exists
   */
  public void registerTransformComponent(TransformComponent component) {
    for (TransformComponent existing : components) {
      if (existing.componentId() == component.componentId()) {
        throw new IllegalArgumentException("componentId already exists");
      }
    }
    components.add(component);
    Logger.recordOutput("Services/" + getName() + "/RegisteredComponents", components.size());
  }

  /**
   * Convenience method to register a component with individual parameters.
   *
   * @param componentId Unique identifier for the component
   * @param parentId Identifier of the parent component
   * @param transformSupplier Supplier providing the transform matrix
   */
  public void registerTransformComponent(
      int componentId, int parentId, Supplier<Transform3d> transformSupplier) {
    registerTransformComponent(new TransformComponent(componentId, parentId, transformSupplier));
  }
}
