package frc.robot.services;

import edu.wpi.first.math.geometry.Pose3d;
import frc.robot.interfaces.services.GamePieceVisualizeService;
import frc.robot.interfaces.services.Service;
import lombok.Getter;
import lombok.Setter;

public class GamePieceVisualize implements GamePieceVisualizeService {
  @Getter
  @Setter
  ServiceState state = ServiceState.STOPPED;
  private final String name;

  private static final Pose3d[] unpickedGamePiece = new Pose3d[0];
  private static boolean hasGamePiece = false;

  public GamePieceVisualize(String name) {
    this.name = name;
  }

  @Override
  public int getPriority() {
    return 0;
  }

  @Override
  public String getName() {
    return "GamePieceVisualize/" + name;
  }
}
