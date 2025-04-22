package frc.robot.subsystems.swerve.controller;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.utils.GainsUtil.PidGains;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class TeleopHeadingController extends TeleopHeadlessController {
  private final HeadingController headingController;

  public TeleopHeadingController(
      DoubleSupplier x,
      DoubleSupplier y,
      DoubleSupplier omega,
      Supplier<Rotation2d> yaw,
      Supplier<Rotation2d> targetHeading) {
    super(x, y, omega, yaw);
    this.headingController = new HeadingController(yaw, targetHeading, new PidGains(2, 0, 0));
  }

  @Override
  public double getRotation() {
    return headingController.calcRotation();
  }
}
