package frc.robot.services;

import edu.wpi.first.math.geometry.Pose3d;
import frc.robot.interfaces.services.Service;
import lombok.Getter;
import lombok.Setter;

public class GamePieceVisualizeService implements Service {
  @Getter @Setter ServiceState state = ServiceState.STOPPED;

  private static final Pose3d[] unpickedCoral = new Pose3d[0];
  private static final Pose3d[] scoredCoral = new Pose3d[0];
  private static final Pose3d[] unpickedAlgae = new Pose3d[0];
  private static final Pose3d[] scoredAlgae = new Pose3d[0];

  public GamePieceVisualizeService() {}

  @Override
  public int getPriority() {
    return 0;
  }

  @Override
  public String getName() {
    return "GamePieceVisualize";
  }
}
