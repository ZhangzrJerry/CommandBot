package frc.robot.subsystems.swerve;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Config;
import frc.robot.drivers.dcmotor.DCMotorIO;

public class Swerve extends SubsystemBase {
  private final SwerveModule[] modules = new SwerveModule[4];

  public Swerve() {
    DCMotorIO flDriveIO = new DCMotorIO() {
    };
  }
}
