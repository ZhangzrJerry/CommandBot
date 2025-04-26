package frc.robot.services;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import frc.robot.interfaces.services.Service;
import frc.robot.utils.math.GeomUtil;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;
import lombok.Getter;
import lombok.Setter;
import lombok.experimental.ExtensionMethod;
import org.littletonrobotics.junction.Logger;

/** rely on visualize service */
@ExtensionMethod({ GeomUtil.class })
public class GamePieceVisualize implements Service {
  @Getter
  @Setter
  ServiceState state = ServiceState.STOPPED;
  @Getter
  private String errorMessage = "";
  private final String name;

  private final Pose3d[] scorableGamePiecePose;
  private final Pose3d[] pickableGamePiecePose;
  private final Pose3d[] scoredGamePiecePose;
  @Getter
  private boolean hasGamePiece = false;
  private int scoredGamePieceNums = 0;

  private static final double PICKABLE_MAX_DISTANCE = 0.45;
  private static final double SCORABLE_MAX_DISTANCE = 0.7;

  @Setter
  private Supplier<Pose3d> pickMechanismPoseSupplier = () -> new Pose3d();
  @Setter
  private Supplier<Pose3d> scoreMechanismPoseSupplier = () -> new Pose3d();
  @Setter
  private BooleanSupplier tryPickSupplier = () -> false;
  @Setter
  private BooleanSupplier tryEjectSupplier = () -> false;
  @Setter
  private BooleanSupplier tryScoreSupplier = () -> false;

  @Override
  public void update() {
    try {
      Pose3d pickMechanismPose = pickMechanismPoseSupplier.get();
      Pose3d scoreMechanismPose = scoreMechanismPoseSupplier.get();
      if (tryPickSupplier.getAsBoolean()) {
        if (!hasGamePiece && pickMechanismPose != null) {
          double maxDistance = Double.POSITIVE_INFINITY;
          int minDistanceIndex = 0;
          for (int i = 0; i < pickableGamePiecePose.length; i++) {
            double distance = pickableGamePiecePose[i].getDistance(pickMechanismPose);
            if (distance < maxDistance) {
              maxDistance = distance;
              minDistanceIndex = i;
            }
          }
          if (maxDistance < PICKABLE_MAX_DISTANCE) {
            pickableGamePiecePose[minDistanceIndex] = new Pose3d(0x3f3f3f3f, 0x3f3f3f3f, 0x3f3f3f3f, new Rotation3d());
            hasGamePiece = true;
          }
        }
      }
      if (tryEjectSupplier.getAsBoolean()) {
        if (hasGamePiece) {
          hasGamePiece = false;
          // TODO: make it more elegant
        }
      }
      if (tryScoreSupplier.getAsBoolean()) {
        if (hasGamePiece && scoreMechanismPose != null) {
          double minDistance = Double.POSITIVE_INFINITY;
          int minDistanceIndex = -1;
          double minScoredDistance = Double.POSITIVE_INFINITY;
          int minScoredDistanceIndex = -1;

          // 遍历所有位置
          for (int i = 0; i < scorableGamePiecePose.length; i++) {
            // 检查该位置是否已经得分
            boolean isScored = false;
            for (int j = 0; j < scoredGamePieceNums; j++) {
              if (scoredGamePiecePose[j].equals(scorableGamePiecePose[i])) {
                isScored = true;
                break;
              }
            }

            double distance = scorableGamePiecePose[i].getDistance(scoreMechanismPose);

            // 记录未得分位置中的最近距离
            if (!isScored && distance < minDistance) {
              minDistance = distance;
              minDistanceIndex = i;
            }

            // 记录已得分位置中的最近距离
            if (isScored && distance < minScoredDistance) {
              minScoredDistance = distance;
              minScoredDistanceIndex = i;
            }
          }

          // 如果找到未得分位置且在阈值内，则在该位置得分
          if (minDistanceIndex != -1 && minDistance < SCORABLE_MAX_DISTANCE) {
            scoredGamePiecePose[scoredGamePieceNums] = scorableGamePiecePose[minDistanceIndex];
            scoredGamePieceNums++;
            hasGamePiece = false;
          }
          // 如果所有位置都已得分，则在最近的已得分位置得分
          else if (minScoredDistanceIndex != -1 && minScoredDistance < SCORABLE_MAX_DISTANCE) {
            scoredGamePiecePose[scoredGamePieceNums] = scorableGamePiecePose[minScoredDistanceIndex];
            scoredGamePieceNums++;
            hasGamePiece = false;
          }
        }
      }

      Logger.recordOutput("Services/" + name + "/hasGamePiece", hasGamePiece);
      Logger.recordOutput("Services/" + name + "/scoredGamePieceNums", scoredGamePieceNums);
      Logger.recordOutput("Services/" + name + "/pickMechanismPose", pickMechanismPose);
      Logger.recordOutput("Services/" + name + "/scoreMechanismPose", scoreMechanismPose);

      Logger.recordOutput("Services/" + name + "/pickableGamePiecePose", pickableGamePiecePose);
      Logger.recordOutput("Services/" + name + "/scoredGamePiecePose", scoredGamePiecePose);
      Logger.recordOutput("Services/" + name + "/scorableGamePiecePose", scorableGamePiecePose);

    } catch (Exception e) {
      setError(e.getMessage());
    }
  }

  public void setError(String errorMessage) {
    this.errorMessage = errorMessage;
    setState(ServiceState.ERROR);
  }

  public GamePieceVisualize(
      String name, Pose3d[] scorableGamePiecePose, Pose3d[] pickableGamePiecePose) {
    this.name = name;
    this.scorableGamePiecePose = scorableGamePiecePose;
    this.pickableGamePiecePose = pickableGamePiecePose;
    this.scoredGamePiecePose = new Pose3d[pickableGamePiecePose.length];
    for (int i = 0; i < pickableGamePiecePose.length; i++) {
      scoredGamePiecePose[i] = new Pose3d(0x3f3f3f3f, 0x3f3f3f3f, 0x3f3f3f3f, new Rotation3d());
    }
  }

  @Override
  public String getName() {
    return name;
  }
}
