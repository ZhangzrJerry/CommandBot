package frc.robot.subsystems.swerve.controller;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.subsystems.swerve.SwerveConfig;
import frc.robot.subsystems.swerve.SwerveController;
import frc.robot.utils.dashboard.TunableNumber;
import java.util.function.DoubleSupplier;

public class TeleopHeaderController implements SwerveController {
  private static final TunableNumber translationDeadband =
      new TunableNumber("Swerve/TeleopController/TranslationDeadband", 0.12);
  private static final TunableNumber translationScalar =
      new TunableNumber("Swerve/TeleopController/TranslationScalar", 0.8);
  private static final TunableNumber rotationDeadband =
      new TunableNumber("Swerve/TeleopController/RotationDeadband", 0.08);
  private static final TunableNumber rotationScalar =
      new TunableNumber("Swerve/TeleopController/RotationScalar", 0.8);

  protected DoubleSupplier xSupplier, ySupplier, omegaSupplier;

  public TeleopHeaderController(DoubleSupplier x, DoubleSupplier y, DoubleSupplier omega) {
    this.xSupplier = x;
    this.ySupplier = y;
    this.omegaSupplier = omega;
  }

  @Override
  public Translation2d getTranslation2d() {
    double x = xSupplier.getAsDouble();
    double y = ySupplier.getAsDouble();
    double magnitude =
        MathUtil.applyDeadband(Math.hypot(x, y), translationDeadband.getAsDouble())
            * translationScalar.getAsDouble();
    double direction = magnitude < 1e-16 ? 0.0 : Math.atan2(y, x);

    // Square the magnitude for better sensitivity
    return new Translation2d(
        magnitude * magnitude * SwerveConfig.MAX_TRANSLATION_VEL_METER_PER_SEC,
        new Rotation2d(direction));
  }

  @Override
  public double getRotation() {
    double magnitude =
        MathUtil.applyDeadband(omegaSupplier.getAsDouble(), rotationDeadband.getAsDouble())
            * rotationScalar.getAsDouble();
    // Square the magnitude for better sensitivity while preserving sign
    return Math.copySign(
        magnitude * magnitude * SwerveConfig.MAX_ANGULAR_VEL_RAD_PER_SEC, magnitude);
  }

  @Override
  public String getName() {
    return "Teleop Header Controller";
  }
}
