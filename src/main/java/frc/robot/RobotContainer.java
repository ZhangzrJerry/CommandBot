package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.composite.CompositeCommands;
import frc.robot.commands.joystick.JoystickCommands;
import frc.robot.commands.swerve.SwerveCommands;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.virtuals.odometry.Odometry;

public class RobotContainer {
  // real subsystems
  private final Swerve swerve;

  // virtual subsystems
  private final Odometry odometry;
  private final CommandXboxController joystick;

  // commands
  private final SwerveCommands swerveCommands;
  private final JoystickCommands joystickCommands;
  private final CompositeCommands compositeCommands;

  public RobotContainer() {
    // real subsystems
    swerve = Swerve.create();

    // virtual subsystem
    odometry =
        new Odometry(
            () -> swerve.getPositions(), () -> swerve.getGyroYaw(), swerve.getKinematics());
    joystick = new CommandXboxController(Ports.Joystick.DRIVER);

    // commands
    swerveCommands = new SwerveCommands(swerve, odometry, joystick);
    joystickCommands = new JoystickCommands(joystick);
    compositeCommands = new CompositeCommands(swerveCommands, joystickCommands, swerve);

    handlePhysicalSubsystemSignals();
    handleVirtualSubsystemSignals();
  }

  private void handlePhysicalSubsystemSignals() {}

  private void handleVirtualSubsystemSignals() {
    joystick.a().whileTrue(compositeCommands.createAutoWaypointCommand());
    joystick.b().whileTrue(joystickCommands.createRumbleCommand(0.5));
    joystick.x().whileTrue(compositeCommands.createEmergencyStopCommand());
  }

  public Command getAutonomousCommand() {
    return Commands.none();
  }
}
