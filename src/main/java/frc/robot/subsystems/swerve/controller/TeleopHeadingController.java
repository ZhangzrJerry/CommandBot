package frc.robot.subsystems.swerve.controller;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.utils.dashboard.TunableGains.TunablePidGains;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class TeleopHeadingController extends TeleopHeadlessController {
  private final HeadingController headingController;
  private final TunablePidGains headingGains =
      new TunablePidGains("Swerve/HeadingController", 2.0, 0.0, 0.0);

  public TeleopHeadingController(
      DoubleSupplier x,
      DoubleSupplier y,
      DoubleSupplier omega,
      Supplier<Rotation2d> yaw,
      Supplier<Rotation2d> targetHeading) {
    super(x, y, omega, yaw);
    this.headingController = new HeadingController(yaw, targetHeading, headingGains);
  }

  @Override
  public double getRotation() {
    return headingController.calcRotation();
  }

  @Override
  public String getName() {
    return "Teleop Heading Controller";
  }
}
