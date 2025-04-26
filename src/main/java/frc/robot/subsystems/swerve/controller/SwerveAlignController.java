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

public class SwerveAlignController implements SwerveController {
  public enum AlignType {
    PROCESSOR,
    NET,
    REEF_CORAL,
    REEF_ALGAE,
    CUSTOM
  }

  private static class AlignConfig {
    public final TunableNumber maxLineupShiftingYMeter;
    public final TunableNumber maxLineupShiftingXMeter;
    public final TunablePidGains translationGains;
    public final TunableNumber translationToleranceMeter;
    public final TunablePidGains rotationGains;
    public final TunableNumber rotationToleranceDegree;
    public final TunableNumber preTranslationToleranceMeter;

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
        double rotTol) {
      this.maxLineupShiftingYMeter = new TunableNumber(prefix + "/MaxLineupShiftingYMeter", maxY);
      this.maxLineupShiftingXMeter = new TunableNumber(prefix + "/MaxLineupShiftingXMeter", maxX);
      this.translationGains = new TunablePidGains(prefix + "/TranslationGains", transKP, transKI, transKD);
      this.translationToleranceMeter = new TunableNumber(prefix + "/TranslationToleranceMeter", transTol);
      this.rotationGains = new TunablePidGains(prefix + "/RotationGains", rotKP, rotKI, rotKD);
      this.rotationToleranceDegree = new TunableNumber(prefix + "/RotationToleranceDegree", rotTol);
      this.preTranslationToleranceMeter = new TunableNumber(prefix + "/PreTranslationToleranceMeter", preTransTol);
    }
  }

  private static final AlignConfig PROCESSOR_CONFIG = new AlignConfig(
      "Swerve/AlignController/Processor",
      1.5,
      1.0, // maxY, maxX
      2.1,
      0.0,
      0.1,
      0.08,
      0.15, // transKP, transKI, transKD, transTol, preTransTol
      4.0,
      0.0,
      0.0,
      5.0 // rotKP, rotKI, rotKD, rotTol
  );

  private static final AlignConfig NET_CONFIG = new AlignConfig(
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
      3.0 // rotKP, rotKI, rotKD, rotTol
  );

  private static final AlignConfig REEF_CORAL_CONFIG = new AlignConfig(
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
      4.0 // rotKP, rotKI, rotKD, rotTol
  );

  private static final AlignConfig REEF_ALGAE_CONFIG = new AlignConfig(
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
      4.0 // rotKP, rotKI, rotKD, rotTol
  );

  private static final double maxTranslationVelMeterPerSec = SwerveConfig.MAX_TRANSLATION_VEL_METER_PER_SEC;
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

  public SwerveAlignController(
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

    var currentDistance = currentPose.getTranslation().getDistance(shiftedGoalPose.getTranslation());
    Logger.recordOutput("Swerve/AlignController/TranslationErrorMeter", currentDistance);

    var translationDir = currentPose.getTranslation().minus(shiftedGoalPose.getTranslation()).getAngle();
    var translationFeedback = translationController.calculate(currentDistance, 0.0);

    var rotationErrorDegree = currentPose.getRotation().minus(shiftedGoalPose.getRotation()).getDegrees();
    Logger.recordOutput("Swerve/AlignController/RotationErrorDegree", rotationErrorDegree);

    hasHeadingAtGoal = Math.abs(rotationErrorDegree) <= config.rotationToleranceDegree.get();
    hasTranslationAtGoal = Math.abs(currentDistance) <= config.translationToleranceMeter.get();
    hasDone = hasHeadingAtGoal && hasTranslationAtGoal;

    if (hasDone) {
      return new ChassisSpeeds();
    }

    var translationOutputScalar = hasHeadingAtGoal ? 1.0 : 1.0 - Math.abs(rotationErrorDegree) / 180.0;
    Logger.recordOutput("Swerve/AlignController/TranslationOutputScalar", translationOutputScalar);

    var translationVel = new Translation2d(
        MathUtil.clamp(
            translationFeedback * translationOutputScalar,
            -maxTranslationVelMeterPerSec,
            maxTranslationVelMeterPerSec),
        translationDir);

    var rotationVel = MathUtil.clamp(
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

    var shiftXT = MathUtil.clamp(
        (yDistance / (Field.CoralStation.FACE_LENGTH * 2.0))
            + ((xDistance - 0.3) / (Field.CoralStation.FACE_LENGTH * 3.0)),
        0.0,
        1.0);

    var shiftYT = MathUtil.clamp(offset.getX() / Field.CoralStation.FACE_LENGTH, 0.0, 1.0);

    var flippedShiftedGoalPose = goalPose.transformBy(
        GeomUtil.toTransform2d(
            -shiftXT * config.maxLineupShiftingXMeter.get(),
            Math.copySign(
                shiftYT * config.maxLineupShiftingYMeter.get() * 0.8, offset.getY())));

    return new Pose2d(flippedShiftedGoalPose.getTranslation(), goalPose.getRotation());
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
            .getDistance(goalPoseSupplier.get().getTranslation())) <= config.preTranslationToleranceMeter.get();
  }

  @Override
  public String getName() {
    return "Align Controller";
  }
}
