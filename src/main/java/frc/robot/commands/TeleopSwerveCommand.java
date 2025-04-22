package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve.Swerve;

public class TeleopSwerveCommand extends Command {
  public TeleopSwerveCommand(Swerve swerve) {
    swerve.setState(null);
  }
}
