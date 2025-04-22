package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.swerve.Swerve;

public class RobotContainer {
  // commander
  private final CommandXboxController joystick = new CommandXboxController(Ports.Joystick.DRIVER);

  // subsystem
  private final Swerve swerve;

  public RobotContainer() {
    swerve = new Swerve();

    configureBindings();
  }

  private void configureBindings() {}

  public Command getAutonomousCommand() {
    return Commands.none();
  }

  private Command joystickRumbleCommand() {
    return Commands.startEnd(
        () -> joystick.getHID().setRumble(GenericHID.RumbleType.kBothRumble, 1.0),
        () -> joystick.getHID().setRumble(GenericHID.RumbleType.kBothRumble, 0.0));
  }
}
