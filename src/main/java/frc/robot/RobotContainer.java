package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.controller.TeleopHeadingController;
import frc.robot.virtuals.odometry.Odometry;

public class RobotContainer {
  // commander
  private final CommandXboxController joystick = new CommandXboxController(Ports.Joystick.DRIVER);

  // subsystem
  private final Swerve swerve;

  // virtual subsystem
  private final Odometry odometry;

  public RobotContainer() {
    // subsystem
    swerve = new Swerve();

    // virtual subsystem
    odometry =
        new Odometry(
            () -> swerve.getPositions(), () -> swerve.getGyroYaw(), swerve.getKinematics());

    configureBindings();
  }

  private void configureBindings() {
    swerve.setController(
        new TeleopHeadingController(
            () -> -joystick.getLeftY(),
            () -> -joystick.getLeftX(),
            () -> -joystick.getRightX(),
            () -> odometry.getGyroYaw(),
            () -> odometry.getEstimatedPose().getTranslation().getAngle()));
  }

  public Command getAutonomousCommand() {
    return Commands.none();
  }

  private Command joystickRumbleCommand() {
    return Commands.startEnd(
        () -> joystick.getHID().setRumble(GenericHID.RumbleType.kBothRumble, 1.0),
        () -> joystick.getHID().setRumble(GenericHID.RumbleType.kBothRumble, 0.0));
  }
}
