package frc.robot.subsystems.swerve.controller;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.subsystems.swerve.SwerveController;
import java.util.function.Supplier;

public abstract class HeadingController implements SwerveController {
  Supplier<Rotation2d> yawSupplier, targetYawSupplier;

  public HeadingController(
      Supplier<Rotation2d> yawSupplier, Supplier<Rotation2d> targetYawSupplier) {
    this.yawSupplier = yawSupplier;
    this.targetYawSupplier = targetYawSupplier;
  }

  @Override
  public abstract Translation2d getTranslation2d();

  @Override
  public double getRotation() {
    // TODO
    return 0.0;
  }
}
