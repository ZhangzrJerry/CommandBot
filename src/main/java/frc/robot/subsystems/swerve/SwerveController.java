package frc.robot.subsystems.swerve;

import edu.wpi.first.math.kinematics.ChassisSpeeds;

public interface SwerveController {
  default ChassisSpeeds getChassisSpeeds() {
    return new ChassisSpeeds();
  }

  default Boolean headingAtGoal(double toleranceRadian) {
    return false;
  }

  default Boolean positionAtGoal(double toleranceMeter) {
    return false;
  }
}
