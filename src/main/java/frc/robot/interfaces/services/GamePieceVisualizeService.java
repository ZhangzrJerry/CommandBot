package frc.robot.interfaces.services;

import edu.wpi.first.math.geometry.Pose3d;

public interface GamePieceVisualizeService extends Service {
  default boolean tryPick(Pose3d currentPose) {
    return false;
  }

  default boolean tryScore(Pose3d currentPose, Pose3d targetPose) {
    return false;
  }
}
