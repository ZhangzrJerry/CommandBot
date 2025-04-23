package frc.robot.subsystems.swerve.controller;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.utils.FieldUtil;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class TeleopHeadlessController extends TeleopHeaderController {
  Supplier<Rotation2d> yawSupplier;

  public TeleopHeadlessController(
      DoubleSupplier x, DoubleSupplier y, DoubleSupplier omega, Supplier<Rotation2d> yaw) {
    super(x, y, omega);
    this.yawSupplier = yaw;
  }

  @Override
  public Translation2d getTranslation2d() {
    return super.getTranslation2d()
        .rotateBy(yawSupplier.get().unaryMinus())
        .rotateBy(FieldUtil.isRedAlliance() ? Rotation2d.kPi : Rotation2d.kZero);
  }

  @Override
  public String getName() {
    return "Headless Control";
  }
}
