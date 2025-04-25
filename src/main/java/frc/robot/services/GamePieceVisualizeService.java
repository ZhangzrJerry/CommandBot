package frc.robot.services;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.Transform3d;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Comparator;
import java.util.List;
import java.util.Optional;
import java.util.function.Function;
import java.util.function.Supplier;
import lombok.Getter;
import lombok.Setter;
import lombok.experimental.ExtensionMethod;
import org.littletonrobotics.junction.Logger;
import frc.robot.interfaces.services.Service;

public class GamePieceVisualizeService implements Service {
  @Getter
  @Setter
  ServiceState state = ServiceState.STOPPED;

  private static final Pose3d[] unpickedCoral = new Pose3d[0];
  private static final Pose3d[] scoredCoral = new Pose3d[0];
  private static final Pose3d[] unpickedAlgae = new Pose3d[0];
  private static final Pose3d[] scoredAlgae = new Pose3d[0];

  public GamePieceVisualizeService() {

  }

  @Override
  public int getPriority() {
    return 0;
  }

  @Override
  public String getName() {
    return "GamePieceVisualize";
  }
}
