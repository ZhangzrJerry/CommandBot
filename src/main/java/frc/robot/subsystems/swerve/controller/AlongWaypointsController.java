package frc.robot.subsystems.swerve.controller;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.swerve.SwerveConfig;
import frc.robot.subsystems.swerve.SwerveController;
import frc.robot.utils.dashboard.TunableGains.TunablePidGains;
import frc.robot.utils.dashboard.TunableNumber;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class AlongWaypointsController implements SwerveController {
  private final TunablePidGains translationGains =
      new TunablePidGains("AlongWaypointsController/TranslationGains", 0.5, 0.0, 0.0);
  private final TunablePidGains rotationGains =
      new TunablePidGains("AlongWaypointsController/RotationGains", 0.5, 0, 0);
  private final TunableNumber translationTolerance =
      new TunableNumber("AlongWaypointsController/TranslationToleranceMeters", 0.05);
  private final TunableNumber rotationTolerance =
      new TunableNumber("AlongWaypointsController/RotationToleranceDegrees", 2.0);

  private final ProfiledPIDController translationController =
      new ProfiledPIDController(
          translationGains.getKP(),
          translationGains.getKI(),
          translationGains.getKD(),
          new TrapezoidProfile.Constraints(
              SwerveConfig.MAX_TRANSLATION_VEL_METER_PER_SEC,
              SwerveConfig.MAX_TRANSLATION_ACC_METERS_PER_SEC));

  private final ProfiledPIDController rotationController =
      new ProfiledPIDController(
          rotationGains.getKP(),
          rotationGains.getKI(),
          rotationGains.getKD(),
          new TrapezoidProfile.Constraints(
              SwerveConfig.MAX_ANGULAR_VEL_RAD_PER_SEC, Double.POSITIVE_INFINITY));

  private final Pose2d[] waypoints;
  private final Supplier<Pose2d> currentPoseSupplier;
  private Pose2d currentGoalPose;
  private int currentIdx = -1;
  private double timestamp = Timer.getFPGATimestamp();

  private boolean hasDone = false;
  private boolean hasHeadingAtGoal = false;
  private boolean hasTranslationAtGoal = false;

  public AlongWaypointsController(Supplier<Pose2d> currentPoseSupplier, Pose2d... waypoints) {
    this.currentPoseSupplier = currentPoseSupplier;
    this.waypoints = waypoints;

    // Ensure all waypoints have the final rotation
    for (int i = 0; i < waypoints.length; i++) {
      this.waypoints[i] =
          new Pose2d(
              waypoints[i].getX(),
              waypoints[i].getY(),
              waypoints[waypoints.length - 1].getRotation());
    }

    // this.currentGoalPose = waypoints[0];
    this.currentGoalPose = currentPoseSupplier.get();
    Logger.recordOutput("Swerve/AlongWaypointsController/Waypoints", waypoints);
    this.rotationController.enableContinuousInput(-Math.PI, Math.PI);
  }

  @Override
  public ChassisSpeeds getChassisSpeeds() {
    translationController.setP(translationGains.getKP());
    translationController.setI(translationGains.getKI());
    translationController.setD(translationGains.getKD());

    rotationController.setP(rotationGains.getKP());
    rotationController.setI(rotationGains.getKI());
    rotationController.setD(rotationGains.getKD());

    // Update current goal pose based on waypoint progression
    if (currentIdx < waypoints.length - 1) {
      double currentTime = Timer.getFPGATimestamp();
      double deltaTime = currentTime - timestamp;
      timestamp = currentTime;

      Transform2d errorToNextWaypoint = waypoints[currentIdx + 1].minus(currentGoalPose);
      double maxDistanceThisStep = SwerveConfig.MAX_TRANSLATION_VEL_METER_PER_SEC * deltaTime * 0.5;

      if (errorToNextWaypoint.getTranslation().getNorm() <= maxDistanceThisStep) {
        // Move to next waypoint
        currentIdx++;
        currentGoalPose = waypoints[currentIdx];
      } else {
        // Move toward next waypoint
        Translation2d step =
            errorToNextWaypoint
                .getTranslation()
                .div(errorToNextWaypoint.getTranslation().getNorm())
                .times(maxDistanceThisStep);
        currentGoalPose =
            currentGoalPose.plus(new Transform2d(step, errorToNextWaypoint.getRotation()));
      }
    } else {
      // Stay at final waypoint
      currentGoalPose = waypoints[waypoints.length - 1];
    }

    Logger.recordOutput("Swerve/AlongWaypointsController/CurrentGoalPose", currentGoalPose);
    // 获取当前位姿
    Pose2d currentPose = currentPoseSupplier.get();

    // 计算联合平移误差（向量形式）
    Translation2d translationError =
        currentGoalPose.getTranslation().minus(currentPose.getTranslation());
    double translationErrorNorm = translationError.getNorm();
    double rotationError =
        Math.toDegrees(currentGoalPose.getRotation().minus(currentPose.getRotation()).getRadians());

    Logger.recordOutput(
        "Swerve/AlongWaypointsController/TranslationErrorMeters", translationErrorNorm);
    Logger.recordOutput("Swerve/AlongWaypointsController/RotationErrorDegrees", rotationError);

    // 检查是否到达目标
    hasTranslationAtGoal = translationErrorNorm <= translationTolerance.get();
    hasHeadingAtGoal = Math.abs(rotationError) <= rotationTolerance.get();
    hasDone = (currentIdx == waypoints.length - 1) && hasTranslationAtGoal && hasHeadingAtGoal;

    if (hasDone) {
      return new ChassisSpeeds();
    }

    // 计算联合平移输出
    double translationVelocity = translationController.calculate(0, translationErrorNorm);
    Translation2d velocityVector =
        translationError.div(translationErrorNorm).times(translationVelocity);

    // 计算旋转输出
    double omega =
        rotationController.calculate(
            currentPose.getRotation().getRadians(), currentGoalPose.getRotation().getRadians());

    // 转换为机器人相对速度
    return ChassisSpeeds.fromFieldRelativeSpeeds(
        velocityVector.getX(), velocityVector.getY(), omega, currentPose.getRotation());
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
    if (waypoints.length == 0) return true;
    return currentPoseSupplier
            .get()
            .getTranslation()
            .getDistance(waypoints[waypoints.length - 1].getTranslation())
        <= translationTolerance.get();
  }

  @Override
  public Boolean translationErrorWithin(double tolerance) {
    if (waypoints.length == 0) return true;
    return currentPoseSupplier
            .get()
            .getTranslation()
            .getDistance(waypoints[waypoints.length - 1].getTranslation())
        <= tolerance;
  }

  @Override
  public String getName() {
    return "Along Waypoints Controller";
  }

  @AutoLogOutput(key = "Swerve/AlongWaypointsController/AtFinalWaypoint")
  public boolean atFinalWaypoint() {
    return currentIdx == waypoints.length - 1;
  }
}
