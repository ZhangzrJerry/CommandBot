package frc.robot.subsystems.swerve.controller;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.ReefScape.Field;
import frc.robot.subsystems.swerve.SwerveConfig;
import frc.robot.subsystems.swerve.SwerveController;
import frc.robot.utils.dashboard.TunableGains.TunablePidGains;
import frc.robot.utils.dashboard.TunableNumber;
import frc.robot.utils.math.GeomUtil;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;

public class AutoAlignController implements SwerveController {
  public enum AlignType {
    PROCESSOR,
    NET,
    REEF_CORAL,
    REEF_ALGAE,
    CORAL_STATION_LEFT,
    CORAL_STATION_RIGHT,
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
          3.0, // maxY, maxX
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
          0.0 // alignmentAngle
          );

  private static final AlignConfig REEF_CORAL_CONFIG =
      new AlignConfig(
          "Swerve/AlignController/ReefCoral",
          5.0,
          3.0, // maxY, maxX
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
          3.0,
          1.5, // maxY, maxX
          2.0,
          0.0,
          0.08,
          0.06,
          0.45, // transKP, transKI, transKD, transTol, preTransTol
          3.5,
          0.0,
          0.0,
          4.0, // rotKP, rotKI, rotKD, rotTol
          180.0 // alignmentAngle
          );

  private static final AlignConfig LEFT_CORAL_STATION_CONFIG =
      new AlignConfig(
          "Swerve/AlignController/LeftCoralStation",
          2.0,
          2.0, // maxY, maxX
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
          2.0,
          2.0, // maxY, maxX
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
    translationController.setPID(
        config.translationGains.getKP(),
        config.translationGains.getKI(),
        config.translationGains.getKD());
    rotationController.setPID(
        config.rotationGains.getKP(), config.rotationGains.getKI(), config.rotationGains.getKD());
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
        return PROCESSOR_CONFIG; // Default to processor configuration
    }
  }

  @Override
  public ChassisSpeeds getChassisSpeeds() {
    Pose2d currentPose = currentPoseSupplier.get();
    Pose2d goalPose = getShiftedGoalPose();

    hasHeadingAtGoal =
        Math.abs(
                MathUtil.angleModulus(
                    currentPose.getRotation().getRadians()
                        - goalPoseSupplier.get().getRotation().getRadians()))
            <= config.rotationToleranceDegree.get();
    hasTranslationAtGoal =
        currentPose.getTranslation().getDistance(goalPoseSupplier.get().getTranslation())
            <= config.translationToleranceMeter.get();
    hasDone = hasHeadingAtGoal && hasTranslationAtGoal;
    if (hasDone) {
      return new ChassisSpeeds(0, 0, 0);
    }

    // Calculate angle relative to target point
    double angleToGoal =
        Math.atan2(goalPose.getY() - currentPose.getY(), goalPose.getX() - currentPose.getX());

    // Calculate offset considering current relative position and target alignment
    // angle
    double offsetX = goalPose.getX() - currentPose.getX();
    double offsetY = goalPose.getY() - currentPose.getY();

    // Calculate offset ratio based on angle difference
    double angleDiff = Math.abs(angleToGoal - goalPose.getRotation().getRadians());
    double offsetRatio = Math.cos(angleDiff);

    // Calculate offset direction
    double offsetDirection =
        Math.signum(offsetX * Math.cos(angleToGoal) + offsetY * Math.sin(angleToGoal));

    double translationOutput = translationController.calculate(0, offsetRatio * offsetDirection);
    double rotationOutput =
        rotationController.calculate(
            currentPose.getRotation().getRadians(), goalPose.getRotation().getRadians());

    return new ChassisSpeeds(
        translationOutput * maxTranslationVelMeterPerSec,
        0,
        rotationOutput * maxRotationVelRadPerSec);
  }

  private Pose2d getShiftedGoalPose() {
    var goalPose = goalPoseSupplier.get();
    var currentPose = currentPoseSupplier.get();

    var offset = currentPose.relativeTo(goalPose);
    var yDistance = Math.abs(offset.getY());
    var xDistance = Math.abs(offset.getX());

    // calculate the angle between the current pose and the goal pose
    var relativeAngle = Math.atan2(offset.getY(), offset.getX());
    var alignmentAngle = Math.toRadians(config.alignmentAngleDegree.get());

    var angleDiff = MathUtil.angleModulus(relativeAngle - alignmentAngle);
    var distance = Math.sqrt(xDistance * xDistance + yDistance * yDistance);

    var shiftT =
        MathUtil.clamp(
            (distance / (Field.CoralStation.FACE_LENGTH * 2.0))
                * (1.0 - Math.abs(angleDiff) / Math.PI),
            0.0,
            1.0);

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
  public Boolean translationErrorWithin(double tolerance) {
    return Math.abs(
            currentPoseSupplier
                .get()
                .getTranslation()
                .getDistance(goalPoseSupplier.get().getTranslation()))
        <= tolerance;
  }

  @Override
  public String getName() {
    return "Auto Align Controller: " + config.name;
  }
}
