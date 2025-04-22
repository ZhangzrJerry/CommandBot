package frc.robot.subsystems.swerve.controller;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.subsystems.swerve.SwerveConfig;
import frc.robot.utils.GainsUtil.PidGains;
import java.util.function.Supplier;

public class HeadingController {
  private final Supplier<Rotation2d> yawSupplier, targetYawSupplier;
  private final ProfiledPIDController pid;

  public HeadingController(
      Supplier<Rotation2d> yawSupplier, Supplier<Rotation2d> targetYawSupplier, PidGains pidGains) {
    this.yawSupplier = yawSupplier;
    this.targetYawSupplier = targetYawSupplier;
    this.pid =
        new ProfiledPIDController(
            pidGains.kP(),
            pidGains.kI(),
            pidGains.kD(),
            new TrapezoidProfile.Constraints(
                SwerveConfig.MAX_ANGULAR_VEL_RAD_PER_SEC, Double.POSITIVE_INFINITY));
    this.pid.enableContinuousInput(-Math.PI, Math.PI);
  }

  public double calcRotation() {
    Rotation2d yaw = yawSupplier.get();
    Rotation2d targetYaw = targetYawSupplier.get();
    double rotation = pid.calculate(yaw.getRadians(), targetYaw.getRadians());
    return rotation;
  }
}
