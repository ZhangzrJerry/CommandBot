package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.subsystems.swerve.SwerveController;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class SwerveTeleopControl implements SwerveController {
  private final DoubleSupplier xSupplier, ySupplier, wSupplier;
  private final Supplier<Rotation2d> yawSupplier;

  public SwerveTeleopControl(
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier wSupplier,
      Supplier<Rotation2d> yawSupplier) {
    this.xSupplier = xSupplier;
    this.ySupplier = ySupplier;
    this.wSupplier = wSupplier;
    this.yawSupplier = yawSupplier;
  }

  @Override
  public ChassisSpeeds getChassisSpeeds() {
    return ChassisSpeeds.fromFieldRelativeSpeeds(
        xSupplier.getAsDouble(),
        ySupplier.getAsDouble(),
        wSupplier.getAsDouble(),
        yawSupplier.get());
  }
}
