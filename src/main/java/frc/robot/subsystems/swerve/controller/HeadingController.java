package frc.robot.subsystems.swerve.controller;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.subsystems.swerve.SwerveConfig;
import frc.robot.utils.dashboard.TunableGains.TunablePidGains;
import java.util.function.Supplier;

// TODO
public class HeadingController {
  private final Supplier<Rotation2d> yawSupplier, targetYawSupplier;
  private final ProfiledPIDController pid;
  private final TunablePidGains gains;

  public HeadingController(
      Supplier<Rotation2d> yawSupplier,
      Supplier<Rotation2d> targetYawSupplier,
      TunablePidGains gains) {
    this.yawSupplier = yawSupplier;
    this.targetYawSupplier = targetYawSupplier;
    this.pid =
        new ProfiledPIDController(
            gains.getKP(),
            gains.getKI(),
            gains.getKD(),
            new TrapezoidProfile.Constraints(
                SwerveConfig.MAX_ANGULAR_VEL_RAD_PER_SEC, Double.POSITIVE_INFINITY));
    this.pid.enableContinuousInput(-Math.PI, Math.PI);
    this.gains = gains;
  }

  public double calcRotation() {
    TunablePidGains.ifChanged(
        this.hashCode(),
        () -> {
          pid.setPID(gains.getKP(), gains.getKI(), gains.getKD());
        },
        gains);
    Rotation2d yaw = yawSupplier.get();
    Rotation2d targetYaw = targetYawSupplier.get();
    double rotation = pid.calculate(yaw.getRadians(), targetYaw.getRadians());
    return rotation;
  }
}
