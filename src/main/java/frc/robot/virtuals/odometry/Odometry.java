package frc.robot.virtuals.odometry;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.virtuals.VirtualSubsystem;
import java.util.function.Supplier;

public class Odometry extends VirtualSubsystem {
  Supplier<Pose2d> visionOdomPoseSupplier;
  Supplier<Pose2d> wheelOdomPoseSupplier;

  @Override
  public void periodic() {}

  Pose2d getPose() {
    return new Pose2d();
  }
}
