package frc.robot.virtuals.visualize;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/**
 * Visualization subsystem that helps visualize robot components in Advantage
 * Scope.
 * Maintains a transform tree and updates component poses periodically.
 */
public class Visualize extends SubsystemBase {
  private static final List<VisualizeComponent> components = new ArrayList<>();
  private static final Boolean isStrictBigEndian = true;

  private Visualize() {
  }

  /**
   * Component record for visualization system.
   * 
   * @param componentId       the id of the component (0 to N)
   * @param parentId          the id of the parent component (-1 for robot frame)
   * @param transformSupplier supplier of transform from parent to this component
   */
  public record VisualizeComponent(
      int componentId,
      int parentId,
      Supplier<Transform3d> transformSupplier) {
    public VisualizeComponent {
      if (componentId < 0) {
        throw new IllegalArgumentException("componentId out of index");
      }
      if (parentId < -1) {
        throw new IllegalArgumentException("parentId out of index");
      }
      if (isStrictBigEndian && parentId >= componentId) {
        throw new IllegalArgumentException(
            "componentId should not be greater than or equal to parentId");
      }
      if (transformSupplier == null) {
        throw new IllegalArgumentException("transformSupplier should not be null");
      }
    }
  }

  /**
   * Register a component for visualization.
   * 
   * @param visualizeComponent the component to register
   */
  public void register(VisualizeComponent visualizeComponent) {
    for (VisualizeComponent component : components) {
      if (component.componentId() == visualizeComponent.componentId()) {
        throw new IllegalArgumentException("componentId already exists");
      }
    }
    components.add(visualizeComponent);
    components.sort(Comparator.comparingInt(VisualizeComponent::componentId));

    // Validate component IDs are continuous
    for (int i = 0; i < components.size(); ++i) {
      if (components.get(i).componentId() != i) {
        throw new IllegalArgumentException("componentId should be continuous");
      }
    }
  }

  @Override
  public void periodic() {
    Pose3d[] poses = new Pose3d[components.size()];
    Transform3d[] tfs = new Transform3d[components.size()];

    if (isStrictBigEndian) {
      for (int i = 0; i < components.size(); ++i) {
        tfs[i] = components.get(i).transformSupplier().get();

        if (components.get(i).parentId() != -1) {
          // Propagate the transform matrix
          tfs[i] = tfs[components.get(i).parentId()].plus(tfs[i]);
        }
        poses[i] = new Pose3d(tfs[i].getTranslation(), tfs[i].getRotation());
      }
    }

    Logger.recordOutput("Visualize/Component", poses);
  }

  public static Visualize getInstance() {
    if (instance == null) {
      instance = new Visualize();
    }
    return instance;
  }

  private static Visualize instance;
}