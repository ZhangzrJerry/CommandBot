package frc.robot.subsystems.swerve;

import edu.wpi.first.math.kinematics.ChassisSpeeds;

public interface SwerveController {
  default ChassisSpeeds getChassisSpeeds() {
    return new ChassisSpeeds();
  }
}
