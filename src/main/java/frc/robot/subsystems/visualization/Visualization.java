package frc.robot.subsystems.visualization;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.utils.VirtualSubsystem;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/**
 * Visualization subsystem that helps visualize robot components in Advantage
 * Scope. Maintains a
 * transform tree and updates component poses periodically.
 */
public class Visualization extends VirtualSubsystem {
  private static final List<VisualizationComponent> components = new ArrayList<>();
  private static final Boolean isStrictBigEndian = true;

  /**
   * Component record for visualization system.
   *
   * @param componentId       the id of the component (0 to N)
   * @param parentId          the id of the parent component (-1 for robot frame)
   * @param transformSupplier supplier of transform from parent to this component
   */
  public record VisualizationComponent(
      int componentId, int parentId, Supplier<Transform3d> transformSupplier) {
    public VisualizationComponent {
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
  public void registerVisualizationComponent(
      VisualizationComponent visualizationComponent) {
    for (VisualizationComponent component : components) {
      if (component.componentId() == visualizationComponent.componentId()) {
        throw new IllegalArgumentException("componentId already exists");
      }
    }
    components.add(visualizationComponent);
    components.sort(Comparator.comparingInt(VisualizationComponent::componentId));

    // Validate component IDs are continuous
    for (int i = 0; i < components.size(); ++i) {
      if (components.get(i).componentId() != i) {
        throw new IllegalArgumentException("componentId should be continuous");
      }
    }
    System.out.println("[Visualization] Register Visualization Component: "
        + visualizationComponent.componentId());
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

    Logger.recordOutput("Visualization/Component", poses);
  }

  private Visualization() {
  }

  private Visualization instance;

  public Visualization getInstance() {
    if (instance == null) {
      instance = new Visualization();
    }
    return instance;
  }
}
