package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public interface SwerveController {
  default ChassisSpeeds getChassisSpeeds() {
    Translation2d translation = getTranslation2d();
    double rotation = getRotation();
    return new ChassisSpeeds(translation.getX(), translation.getY(), rotation);
  }

  default Translation2d getTranslation2d() {
    return new Translation2d();
  }

  default double getRotation() {
    return 0.0;
  }

  default Boolean headingAtGoal() {
    return false;
  }

  default Boolean positionAtGoal() {
    return false;
  }
}
