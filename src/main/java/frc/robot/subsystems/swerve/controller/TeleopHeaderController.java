package frc.robot.subsystems.swerve.controller;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.subsystems.swerve.SwerveController;
import frc.robot.utils.LoggedTunableNumber;
import java.util.function.DoubleSupplier;

public class TeleopHeaderController implements SwerveController {
  private static final LoggedTunableNumber translationDeadband = new LoggedTunableNumber(
      "Swerve/TeleopController/TranslationDeadband", 0.12);
  private static final LoggedTunableNumber translationScalar = new LoggedTunableNumber(
      "Swerve/TeleopController/TranslationScalar", 1.0);
  private static final LoggedTunableNumber rotationDeadband = new LoggedTunableNumber(
      "Swerve/TeleopController/RotationDeadband", 0.08);
  private static final LoggedTunableNumber rotationScalar = new LoggedTunableNumber(
      "Swerve/TeleopController/RotationScalar", 0.8);

  protected DoubleSupplier xSupplier, ySupplier, omegaSupplier, slowerSupplier;

  public TeleopHeaderController(DoubleSupplier x, DoubleSupplier y, DoubleSupplier omega, DoubleSupplier slower) {
    this.xSupplier = x;
    this.ySupplier = y;
    this.omegaSupplier = omega;
    this.slowerSupplier = slower;
  }

  @Override
  public ChassisSpeeds getChassisSpeeds() {
    Translation2d translation = calcTranslation();
    double rotation = calcRotation();

    return new ChassisSpeeds(translation.getX(), translation.getY(), rotation);
  }

  protected Translation2d calcTranslation() {
    double x = xSupplier.getAsDouble();
    double y = ySupplier.getAsDouble();
    double magnitude = MathUtil.applyDeadband(Math.hypot(x, y), translationDeadband.get());
    double direction = magnitude < 1e-16 ? 0.0 : Math.atan2(y, x);

    // Square the magnitude for better sensitivity
    return new Translation2d(magnitude * magnitude * slowerSupplier.getAsDouble(), new Rotation2d(direction));
  }

  protected double calcRotation() {
    double magnitude = MathUtil.applyDeadband(omegaSupplier.getAsDouble(), rotationDeadband.get());
    // Square the magnitude for better sensitivity while preserving sign
    return Math.copySign(magnitude * magnitude * slowerSupplier.getAsDouble(), magnitude);
  }
}
