package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.swerve.Swerve;

public class RobotContainer {
  // real subsystems
  private final Swerve swerve;

  // virtual subsystems
  private final CommandXboxController joystick;

  public RobotContainer() {
    // real subsystems
    if (Robot.isReal()) {
      swerve = Swerve.createReal();
    } else if (Robot.isSimulation()) {
      swerve = Swerve.createSim();
    } else {
      swerve = Swerve.createIO();
    }

    joystick = new CommandXboxController(Ports.Joystick.DRIVER);
  }

  public Command getAutonomousCommand() {
    return Commands.none();
  }
}
