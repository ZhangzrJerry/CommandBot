package frc.robot.subsystems.swerve.controller;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.ReefScape.Field;
import frc.robot.subsystems.swerve.SwerveConfig;
import frc.robot.subsystems.swerve.SwerveController;
import frc.robot.utils.dashboard.TunableGains.TunablePidGains;
import frc.robot.utils.dashboard.TunableNumber;
import frc.robot.utils.math.GeomUtil;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class AutoAlignController implements SwerveController {
  public enum AlignType {
    PROCESSOR,
    NET,
    REEF_CORAL,
    REEF_ALGAE,
    CORAL_STATION_LEFT,
    CORAL_STATION_RIGHT
  }

  private static class AlignConfig {
    public final TunableNumber maxLineupShiftingYMeter;
    public final TunableNumber maxLineupShiftingXMeter;
    public final TunablePidGains translationGains;
    public final TunableNumber translationToleranceMeter;
    public final TunablePidGains rotationGains;
    public final TunableNumber rotationToleranceDegree;
    public final TunableNumber preTranslationToleranceMeter;
    public final TunableNumber alignmentAngleDegree;
    public final String name;

    public AlignConfig(
        String prefix,
        double maxY,
        double maxX,
        double transKP,
        double transKI,
        double transKD,
        double transTol,
        double preTransTol,
        double rotKP,
        double rotKI,
        double rotKD,
        double rotTol,
        double alignmentAngle) {
      this.name = prefix.split("/")[prefix.split("/").length - 1];
      this.maxLineupShiftingYMeter = new TunableNumber(prefix + "/MaxLineupShiftingYMeter", maxY);
      this.maxLineupShiftingXMeter = new TunableNumber(prefix + "/MaxLineupShiftingXMeter", maxX);
      this.translationGains =
          new TunablePidGains(prefix + "/TranslationGains", transKP, transKI, transKD);
      this.translationToleranceMeter =
          new TunableNumber(prefix + "/TranslationToleranceMeter", transTol);
      this.rotationGains = new TunablePidGains(prefix + "/RotationGains", rotKP, rotKI, rotKD);
      this.rotationToleranceDegree = new TunableNumber(prefix + "/RotationToleranceDegree", rotTol);
      this.preTranslationToleranceMeter =
          new TunableNumber(prefix + "/PreTranslationToleranceMeter", preTransTol);
      this.alignmentAngleDegree =
          new TunableNumber(prefix + "/AlignmentAngleDegree", alignmentAngle);
    }
  }

  private static final AlignConfig PROCESSOR_CONFIG =
      new AlignConfig(
          "Swerve/AlignController/Processor",
          3.0,
          1.2, // maxY, maxX
          2.1,
          0.0,
          0.1,
          0.08,
          0.15, // transKP, transKI, transKD, transTol, preTransTol
          4.0,
          0.0,
          0.0,
          5.0, // rotKP, rotKI, rotKD, rotTol
          180.0 // alignmentAngle
          );

  private static final AlignConfig NET_CONFIG =
      new AlignConfig(
          "Swerve/AlignController/Net",
          1.2,
          0.8, // maxY, maxX
          2.5,
          0.0,
          0.15,
          0.05,
          0.15, // transKP, transKI, transKD, transTol, preTransTol
          5.0,
          0.0,
          0.1,
          3.0, // rotKP, rotKI, rotKD, rotTol
          180.0 // alignmentAngle
          );

  private static final AlignConfig REEF_CORAL_CONFIG =
      new AlignConfig(
          "Swerve/AlignController/ReefCoral",
          1.0,
          0.6, // maxY, maxX
          2.0,
          0.0,
          0.08,
          0.06,
          0.15, // transKP, transKI, transKD, transTol, preTransTol
          3.5,
          0.0,
          0.0,
          4.0, // rotKP, rotKI, rotKD, rotTol
          180.0 // alignmentAngle
          );

  private static final AlignConfig REEF_ALGAE_CONFIG =
      new AlignConfig(
          "Swerve/AlignController/ReefAlgae",
          1.0,
          0.6, // maxY, maxX
          2.0,
          0.0,
          0.08,
          0.06,
          0.15, // transKP, transKI, transKD, transTol, preTransTol
          3.5,
          0.0,
          0.0,
          4.0, // rotKP, rotKI, rotKD, rotTol
          180.0 // alignmentAngle
          );

  private static final AlignConfig LEFT_CORAL_STATION_CONFIG =
      new AlignConfig(
          "Swerve/AlignController/LeftCoralStation",
          1.0,
          0.6, // maxY, maxX
          2.0,
          0.0,
          0.08,
          0.06,
          0.15, // transKP, transKI, transKD, transTol, preTransTol
          3.5,
          0.0,
          0.0,
          4.0, // rotKP, rotKI, rotKD, rotTol
          300.0 // alignmentAngle
          );

  private static final AlignConfig RIGHT_CORAL_STATION_CONFIG =
      new AlignConfig(
          "Swerve/AlignController/RightCoralStation",
          1.0,
          0.6, // maxY, maxX
          2.0,
          0.0,
          0.08,
          0.06,
          0.15, // transKP, transKI, transKD, transTol, preTransTol
          3.5,
          0.0,
          0.0,
          4.0, // rotKP, rotKI, rotKD, rotTol
          60.0 // alignmentAngle
          );
  private static final double maxTranslationVelMeterPerSec =
      SwerveConfig.MAX_TRANSLATION_VEL_METER_PER_SEC;
  private static final double maxRotationVelRadPerSec = SwerveConfig.MAX_ANGULAR_VEL_RAD_PER_SEC;

  @AutoLogOutput(key = "Swerve/AlignController/GoalPose")
  private final Supplier<Pose2d> goalPoseSupplier;

  private final Supplier<Pose2d> currentPoseSupplier;
  private final PIDController translationController = new PIDController(0.0, 0.0, 0.0);
  private final PIDController rotationController = new PIDController(0.0, 0.0, 0.0);
  private final AlignConfig config;

  private boolean hasDone = false;
  private boolean hasHeadingAtGoal = false;
  private boolean hasTranslationAtGoal = false;

  public AutoAlignController(
      AlignType type, Supplier<Pose2d> goalPoseSupplier, Supplier<Pose2d> currentPoseSupplier) {
    this.goalPoseSupplier = goalPoseSupplier;
    this.currentPoseSupplier = currentPoseSupplier;
    this.config = getConfigForType(type);
    rotationController.enableContinuousInput(-Math.PI, Math.PI);
  }

  private AlignConfig getConfigForType(AlignType type) {
    switch (type) {
      case PROCESSOR:
        return PROCESSOR_CONFIG;
      case NET:
        return NET_CONFIG;
      case REEF_CORAL:
        return REEF_CORAL_CONFIG;
      case REEF_ALGAE:
        return REEF_ALGAE_CONFIG;
      case CORAL_STATION_LEFT:
        return LEFT_CORAL_STATION_CONFIG;
      case CORAL_STATION_RIGHT:
        return RIGHT_CORAL_STATION_CONFIG;
      default:
        return PROCESSOR_CONFIG; // 默认使用processor配置
    }
  }

  @Override
  public ChassisSpeeds getChassisSpeeds() {
    translationController.setP(config.translationGains.getKP());
    translationController.setD(config.translationGains.getKD());
    translationController.setTolerance(config.translationToleranceMeter.get());

    rotationController.setP(config.rotationGains.getKP());
    rotationController.setD(config.rotationGains.getKD());
    rotationController.setTolerance(config.rotationToleranceDegree.get());

    var shiftedGoalPose = getShiftedGoalPose();
    var currentPose = currentPoseSupplier.get();
    Logger.recordOutput("Swerve/AlignController/GoalPose", shiftedGoalPose);

    var currentDistance =
        currentPose.getTranslation().getDistance(shiftedGoalPose.getTranslation());
    Logger.recordOutput("Swerve/AlignController/TranslationErrorMeter", currentDistance);

    var translationDir =
        currentPose.getTranslation().minus(shiftedGoalPose.getTranslation()).getAngle();
    var translationFeedback = translationController.calculate(currentDistance, 0.0);

    var rotationErrorDegree =
        currentPose.getRotation().minus(shiftedGoalPose.getRotation()).getDegrees();
    Logger.recordOutput("Swerve/AlignController/RotationErrorDegree", rotationErrorDegree);

    hasHeadingAtGoal = Math.abs(rotationErrorDegree) <= config.rotationToleranceDegree.get();
    hasTranslationAtGoal = Math.abs(currentDistance) <= config.translationToleranceMeter.get();
    hasDone = hasHeadingAtGoal && hasTranslationAtGoal;

    if (hasDone) {
      return new ChassisSpeeds();
    }

    var translationOutputScalar =
        hasHeadingAtGoal ? 1.0 : 1.0 - Math.abs(rotationErrorDegree) / 180.0;
    Logger.recordOutput("Swerve/AlignController/TranslationOutputScalar", translationOutputScalar);

    var translationVel =
        new Translation2d(
            MathUtil.clamp(
                translationFeedback * translationOutputScalar,
                -maxTranslationVelMeterPerSec,
                maxTranslationVelMeterPerSec),
            translationDir);

    var rotationVel =
        MathUtil.clamp(
            rotationController.calculate(
                MathUtil.angleModulus(currentPose.getRotation().getRadians()),
                MathUtil.angleModulus(shiftedGoalPose.getRotation().getRadians())),
            -maxRotationVelRadPerSec,
            maxRotationVelRadPerSec);

    return ChassisSpeeds.fromFieldRelativeSpeeds(
        translationVel.getX(), translationVel.getY(), rotationVel, currentPose.getRotation());
  }

  private Pose2d getShiftedGoalPose() {
    var goalPose = goalPoseSupplier.get();
    var currentPose = currentPoseSupplier.get();

    var offset = currentPose.relativeTo(goalPose);
    var yDistance = Math.abs(offset.getY());
    var xDistance = Math.abs(offset.getX());

    // 计算相对于目标点的角度
    var relativeAngle = Math.atan2(offset.getY(), offset.getX());
    var alignmentAngle = Math.toRadians(config.alignmentAngleDegree.get());

    // 计算偏移量，考虑当前相对位置和目标对齐角度的关系
    var angleDiff = MathUtil.angleModulus(relativeAngle - alignmentAngle);
    var distance = Math.sqrt(xDistance * xDistance + yDistance * yDistance);

    // 根据角度差计算偏移比例
    var shiftT =
        MathUtil.clamp(
            (distance / (Field.CoralStation.FACE_LENGTH * 2.0))
                * (1.0 - Math.abs(angleDiff) / Math.PI),
            0.0,
            1.0);

    // 计算偏移方向
    var shiftX = -shiftT * config.maxLineupShiftingXMeter.get() * Math.cos(alignmentAngle);
    var shiftY = shiftT * config.maxLineupShiftingYMeter.get() * Math.sin(alignmentAngle);

    var shiftedGoalPose = goalPose.transformBy(GeomUtil.toTransform2d(shiftX, shiftY));

    return new Pose2d(shiftedGoalPose.getTranslation(), goalPose.getRotation());
  }

  @Override
  public Boolean headingAtGoal() {
    return hasHeadingAtGoal;
  }

  @Override
  public Boolean translationAtGoal() {
    return hasTranslationAtGoal;
  }

  @Override
  public Boolean translationErrorWithin() {
    return Math.abs(
            currentPoseSupplier
                .get()
                .getTranslation()
                .getDistance(goalPoseSupplier.get().getTranslation()))
        <= config.preTranslationToleranceMeter.get();
  }

  @Override
  public String getName() {
    return "Auto Align Controller: " + config.name;
  }
}
