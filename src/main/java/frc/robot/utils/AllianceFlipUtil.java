package frc.robot.utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.ReefScape;

public class AllianceFlipUtil {
  public static final double FIELD_LENGTH = ReefScape.Field.LENGTH;
  public static final double FIELD_WIDTH = ReefScape.Field.WIDTH;

  public static boolean isRedAlliance() {
    return DriverStation.getAlliance()
        .map(alliance -> alliance == DriverStation.Alliance.Red)
        .orElse(false);
  }

  public static boolean isRobotInBlueSide(Pose2d robotPose) {
    return robotPose.getX() < FIELD_LENGTH / 2;
  }

  public static double flipX(double x) {
    return FIELD_LENGTH - x;
  }

  public static double flipY(double y) {
    return FIELD_WIDTH - y;
  }

  public static double flipAngle(double angle) {
    return angle + 180;
  }

  public static double applyX(double x) {
    return isRedAlliance() ? flipX(x) : x;
  }

  public static double applyY(double y) {
    return isRedAlliance() ? flipY(y) : y;
  }

  public static double applyAngle(double angle) {
    return isRedAlliance() ? flipAngle(angle) : angle;
  }

  public static Translation2d flipTranslation(Translation2d translation) {
    return new Translation2d(flipX(translation.getX()), flipY(translation.getY()));
  }

  public static Translation2d applyTranslation(Translation2d translation) {
    return isRedAlliance() ? flipTranslation(translation) : translation;
  }

  public static Rotation2d flipRotation(Rotation2d rotation) {
    return rotation.plus(Rotation2d.kPi);
  }

  public static Rotation2d applyRotation(Rotation2d rotation) {
    return isRedAlliance() ? flipRotation(rotation) : rotation;
  }

  public static Pose2d flipPose(Pose2d pose) {
    return new Pose2d(flipTranslation(pose.getTranslation()), flipRotation(pose.getRotation()));
  }

  public static Pose2d applyPose(Pose2d pose) {
    return isRedAlliance() ? flipPose(pose) : pose;
  }

  public static Pose3d flipPose(Pose3d pose) {
    return new Pose3d(
        flipX(pose.getX()),
        flipY(pose.getY()),
        pose.getZ(),
        new Rotation3d(
            pose.getRotation().getMeasureX().magnitude(),
            pose.getRotation().getMeasureY().magnitude(),
            flipRotation(pose.getRotation().toRotation2d()).getRadians()));
  }

  public static Pose3d applyPose(Pose3d pose) {
    return isRedAlliance() ? flipPose(pose) : pose;
  }
}
