package frc.robot.virtuals.odometry;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import frc.robot.virtuals.VirtualSubsystem;
import java.util.function.Supplier;
import lombok.Getter;
import org.littletonrobotics.junction.AutoLogOutput;

public class Odometry extends VirtualSubsystem {
  @AutoLogOutput(key = "Odometry/LastModulePositions")
  SwerveModulePosition[] lastModulePositions = new SwerveModulePosition[4];

  @AutoLogOutput(key = "Odometry/LastGyroYaw")
  Rotation2d lastGyroYaw = new Rotation2d();

  Supplier<Rotation2d> gyroYawSupplier = () -> lastGyroYaw;
  Supplier<SwerveModulePosition[]> modulePositionsSupplier = () -> lastModulePositions;
  SwerveDriveKinematics kinematics;

  @AutoLogOutput(key = "Odometry/WheeledPose")
  @Getter
  private Pose2d wheeledPose = new Pose2d();

  @AutoLogOutput(key = "Odometry/EstimatedPose")
  @Getter
  private Pose2d estimatedPose = new Pose2d();

  public Odometry(
      Supplier<SwerveModulePosition[]> modulePositionsSupplier,
      Supplier<Rotation2d> gyroYawSupplier,
      SwerveDriveKinematics kinematics) {
    this.kinematics = kinematics;
    this.gyroYawSupplier = gyroYawSupplier;
    this.modulePositionsSupplier = modulePositionsSupplier;

    this.lastModulePositions = modulePositionsSupplier.get();
    this.lastGyroYaw = gyroYawSupplier.get();
  }

  @Override
  public void periodic() {
    // update wheel pose
    var modulePositions = modulePositionsSupplier.get();
    var twist = kinematics.toTwist2d(lastModulePositions, modulePositions);
    lastModulePositions = modulePositions.clone();
    if (lastGyroYaw != null) {
      var gyroYaw = gyroYawSupplier.get();
      twist = new Twist2d(twist.dx, twist.dy, gyroYaw.minus(lastGyroYaw).getRadians());
      lastGyroYaw = gyroYaw;
    }
    wheeledPose = wheeledPose.exp(twist);
    estimatedPose = estimatedPose.exp(twist);
  }

  Pose2d getPose() {
    return estimatedPose;
  }

  public Rotation2d getGyroYaw() {
    if (lastGyroYaw != null) {
      return lastGyroYaw;
    } else {
      return estimatedPose.getRotation();
    }
  }
}
