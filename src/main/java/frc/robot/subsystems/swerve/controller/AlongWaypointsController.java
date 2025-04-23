package frc.robot.subsystems.swerve.controller;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.subsystems.swerve.SwerveConfig;
import frc.robot.subsystems.swerve.SwerveController;
import frc.robot.utils.GainsUtil.PidGains;
import frc.robot.utils.LoggedTunableGains;
import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class AlongWaypointsController implements SwerveController {
  private final LoggedTunableGains<PidGains> translationGains = new LoggedTunableGains<>(
      "AlongWaypointsController/translationGains", new PidGains(0.5, 0, 0));
  private final ProfiledPIDController translationController = new ProfiledPIDController(
      translationGains.get().kP(),
      translationGains.get().kI(),
      translationGains.get().kD(),
      new TrapezoidProfile.Constraints(
          SwerveConfig.MAX_TRANSLATION_VEL_METER_PER_SEC,
          SwerveConfig.MAX_TRANSLATION_ACC_METERS_PER_SEC));

  private final LoggedTunableGains<PidGains> rotationGains = new LoggedTunableGains<>(
      "AlongWaypointsController/rotationGains", new PidGains(0.5, 0, 0));
  private final ProfiledPIDController rotationController = new ProfiledPIDController(
      rotationGains.get().kP(),
      rotationGains.get().kI(),
      rotationGains.get().kD(),
      new TrapezoidProfile.Constraints(
          SwerveConfig.MAX_ANGULAR_VEL_RAD_PER_SEC, Double.POSITIVE_INFINITY));

  private final List<Pose2d> waypoints = new ArrayList<>();
  private final Supplier<Pose2d> currentPoseSupplier;

  public AlongWaypointsController(Supplier<Pose2d> currentPoseSupplier, Pose2d... waypoints) {
    this.currentPoseSupplier = currentPoseSupplier;
    for (Pose2d waypoint : waypoints) {
      this.waypoints.add(waypoint);
    }
    Logger.recordOutput("AlongWaypointsController/Waypoints", waypoints);
    this.rotationController.enableContinuousInput(-Math.PI, Math.PI);
  }

  public AlongWaypointsController(Supplier<Pose2d> currentPoseSupplier, List<Pose2d> waypoints) {
    this.currentPoseSupplier = currentPoseSupplier;
    this.waypoints.addAll(waypoints);
    this.rotationController.enableContinuousInput(-Math.PI, Math.PI);
  }

  private Pose2d getTargetPose2d() {
    if (waypoints.isEmpty()) {
      return currentPoseSupplier.get(); // No waypoints, return current pose
    }
    if (waypoints.size() == 1) {
      return waypoints.get(0);
    }
    return waypoints.get(1);
  }

  @Override
  public Translation2d getTranslation2d() {
    Pose2d currentPose = currentPoseSupplier.get();
    Translation2d eTranslation2d = getTargetPose2d().getTranslation().minus(currentPose.getTranslation());
    double magnitude = translationController.calculate(eTranslation2d.getNorm());
    return eTranslation2d.getNorm() > 0
        ? eTranslation2d.times(magnitude / eTranslation2d.getNorm())
        : new Translation2d(0, 0);
  }

  @Override
  public double getRotation() {
    return rotationController.calculate(
        getTargetPose2d()
            .getRotation()
            .minus(currentPoseSupplier.get().getRotation())
            .getRadians());
  }
}
