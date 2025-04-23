package frc.robot.subsystems.swerve.controller;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.utils.GainsUtil.PidGains;
import frc.robot.utils.LoggedTunableGains;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class TeleopHeadingController extends TeleopHeadlessController {
  private final HeadingController headingController;
  private final LoggedTunableGains<PidGains> headingGains =
      new LoggedTunableGains<>("TeleopHeadingController/headingGains", new PidGains(2, 0, 0));

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
