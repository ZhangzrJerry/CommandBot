package frc.robot.subsystems.swerve.controller;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class TeleopHeadingController extends HeadingController {
  private final TeleopHeadlessController teleopHeadlessController;

  public TeleopHeadingController(
      DoubleSupplier x,
      DoubleSupplier y,
      DoubleSupplier omega,
      Supplier<Rotation2d> yaw,
      Supplier<Rotation2d> targetHeading) {
    super(yaw, targetHeading);
    this.teleopHeadlessController = new TeleopHeadlessController(x, y, omega, yaw);
  }

  @Override
  public Translation2d getTranslation2d() {
    return this.teleopHeadlessController.getTranslation2d();
  }
}
