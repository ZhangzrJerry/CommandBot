package frc.robot.subsystems.swerve.controller;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.subsystems.swerve.SwerveConfig;
import frc.robot.utils.GainsUtil.PidGains;
import frc.robot.utils.logging.LoggedTunableGains;
import java.util.function.Supplier;

public class HeadingController {
  private final Supplier<Rotation2d> yawSupplier, targetYawSupplier;
  private final ProfiledPIDController pid;
  private final LoggedTunableGains<PidGains> pidGains;

  @Deprecated
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
    this.pidGains = new LoggedTunableGains<>();
  }

  public HeadingController(
      Supplier<Rotation2d> yawSupplier,
      Supplier<Rotation2d> targetYawSupplier,
      LoggedTunableGains<PidGains> pidGains) {
    this.yawSupplier = yawSupplier;
    this.targetYawSupplier = targetYawSupplier;
    this.pid =
        new ProfiledPIDController(
            pidGains.get().kP(),
            pidGains.get().kI(),
            pidGains.get().kD(),
            new TrapezoidProfile.Constraints(
                SwerveConfig.MAX_ANGULAR_VEL_RAD_PER_SEC, Double.POSITIVE_INFINITY));
    this.pid.enableContinuousInput(-Math.PI, Math.PI);
    this.pidGains = pidGains;
  }

  @SuppressWarnings("static-access")
  public double calcRotation() {
    pidGains.ifChanged(
        hashCode(),
        () -> {
          PidGains gains = pidGains.get();
          pid.setPID(gains.kP(), gains.kI(), gains.kD());
        },
        pidGains);
    Rotation2d yaw = yawSupplier.get();
    Rotation2d targetYaw = targetYawSupplier.get();
    double rotation = pid.calculate(yaw.getRadians(), targetYaw.getRadians());
    return rotation;
  }
}
