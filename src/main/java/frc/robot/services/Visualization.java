package frc.robot.services;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.services.Service;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/**
 * Visualization service that helps visualize robot components in Advantage
 * Scope.
 * Maintains a transform tree and updates component poses periodically.
 * 
 * This service should be initialized in RobotContainer and injected into
 * subsystems
 * that need visualization capabilities.
 */
public class Visualization implements Service {
  private final List<VisualizationComponent> components = new ArrayList<>();
  private final static Boolean IS_STRICT_BIG_ENDIAN = true;

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
      if (IS_STRICT_BIG_ENDIAN && parentId >= componentId) {
        throw new IllegalArgumentException(
            "componentId should not be greater than or equal to parentId");
      }
      if (transformSupplier == null) {
        throw new IllegalArgumentException("transformSupplier should not be null");
      }
    }
  }

  @Override
  public void initialize() {
    // 初始化可视化服务
    Logger.recordOutput("Visualization/Running", true);
  }

  @Override
  public void update() {
    Pose3d[] poses = new Pose3d[components.size()];
    Transform3d[] tfs = new Transform3d[components.size()];

    if (IS_STRICT_BIG_ENDIAN) {
      for (int i = 0; i < components.size(); ++i) {
        tfs[i] = components.get(i).transformSupplier().get();

        if (components.get(i).parentId() != -1) {
          // Propagate the transform matrix
          tfs[i] = tfs[components.get(i).parentId()].plus(tfs[i]);
        }
        poses[i] = new Pose3d(tfs[i].getTranslation(), tfs[i].getRotation());
      }
    }

    Logger.recordOutput("Visualization/Components", poses);
  }

  @Override
  public void stop() {
    // 清理资源
    components.clear();
    Logger.recordOutput("Visualization/Running", false);
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
}
