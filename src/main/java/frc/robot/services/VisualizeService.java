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

@ExtensionMethod({GeomUtil.class})
public class VisualizeService implements Service {
  @Getter @Setter ServiceState state = ServiceState.STOPPED;

  @Override
  public int getPriority() {
    return 0;
  }

  @Override
  public String getName() {
    return "VisualizeService";
  }

  @Override
  public void init() {
    setState(ServiceState.RUNNING);
    components.sort(Comparator.comparingInt(VisualizeComponent::componentId));
    // check if the componentId is continuous
    for (int i = 0; i < components.size(); ++i) {
      if (components.get(i).componentId() != i) {
        throw new IllegalArgumentException("componentId should be continuous, missing id: " + i);
      }
    }
  }

  @Override
  public void update() {
    Pose3d[] poses = new Pose3d[components.size()];
    Transform3d[] tfs = new Transform3d[components.size()];

    if (STRICT_BIG_ENDIAN) {
      for (int i = 0; i < components.size(); ++i) {
        tfs[i] = components.get(i).transformSupplier().get();

        if (components.get(i).parentId() != -1) {
          // propagate the transform matrix
          tfs[i] = tfs[components.get(i).parentId()].plus(tfs[i]);
        }
        poses[i] = tfs[i].toPose3d();
      }
    }

    Logger.recordOutput(getName() + "/Components", poses);
  }

  private static final List<VisualizeComponent> components = new ArrayList<>();
  private static final Boolean STRICT_BIG_ENDIAN = true;

  /**
   * @param componentId the id of the component, ∈[0,N]
   * @param parentId the id of the parent component, ∈[-1,componentId), -1 means robot frame
   * @param transformSupplier the supplier of the transform matrix from the parent component to the
   *     current component
   */
  public record VisualizeComponent(
      int componentId, int parentId, Supplier<Transform3d> transformSupplier) {
    public VisualizeComponent {
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
   * Register the component to the AkitViz
   *
   * @param akitVizComponent the component to be registered, should be declared in the constructor
   *     of the subsystem
   * @usage see {@link VisualizeService}
   */
  public void registerVisualizeComponent(VisualizeComponent akitVizComponent) {
    for (VisualizeComponent component : components) {
      if (component.componentId() == akitVizComponent.componentId()) {
        throw new IllegalArgumentException("componentId already exists");
      }
    }
    components.add(akitVizComponent);
  }

  public void registerVisualizeComponent(
      int componentId, int parentId, Supplier<Transform3d> transformSupplier) {
    registerVisualizeComponent(new VisualizeComponent(componentId, parentId, transformSupplier));
  }
}
