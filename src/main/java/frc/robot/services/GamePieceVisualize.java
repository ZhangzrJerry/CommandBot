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
  private final String name;

  private final Pose3d[] scorableGamePiecePose;
  private final Pose3d[] pickableGamePiecePose;
  private final Pose3d[] scoredGamePiecePose;
  @Getter
  private boolean hasGamePiece = false;
  private int scoredGamePieceNums = 0;

  private static final double PICKABLE_MAX_DISTANCE = 0.5;
  private static final double SCORABLE_MAX_DISTANCE = 0.5;

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
        double maxDistance = Double.POSITIVE_INFINITY;
        int minDistanceIndex = 0;
        for (int i = 0; i < scorableGamePiecePose.length; i++) {
          double distance = scorableGamePiecePose[i].getDistance(scoreMechanismPose);
          if (distance < maxDistance) {
            maxDistance = distance;
            minDistanceIndex = i;
          }
        }
        if (maxDistance < SCORABLE_MAX_DISTANCE) {
          scoredGamePiecePose[scoredGamePieceNums] = scorableGamePiecePose[minDistanceIndex];
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
  public int getPriority() {
    return 0;
  }

  @Override
  public String getName() {
    return name;
  }
}
