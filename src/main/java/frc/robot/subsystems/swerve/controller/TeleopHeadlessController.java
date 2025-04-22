package frc.robot.subsystems.swerve.controller;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class TeleopHeadlessController extends TeleopHeaderController {
  Supplier<Rotation2d> yawSupplier;

  public TeleopHeadlessController(DoubleSupplier x, DoubleSupplier y, DoubleSupplier omega,
      Supplier<Rotation2d> yaw, DoubleSupplier slower) {
    super(x, y, omega, slower);
    this.yawSupplier = yaw;
  }

  @Override
  public ChassisSpeeds getChassisSpeeds() {
    Rotation2d yaw = yawSupplier.get();
    Translation2d translation = calcTranslation().rotateBy(yaw);
    double rotation = calcRotation();
    return new ChassisSpeeds(translation.getX(), translation.getY(), rotation);
  }

}
