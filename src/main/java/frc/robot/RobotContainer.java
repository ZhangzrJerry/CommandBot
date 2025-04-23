package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.controller.AlongWaypointsController;
import frc.robot.subsystems.swerve.controller.TeleopHeadlessController;
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
    odometry = new Odometry(
        () -> swerve.getPositions(), () -> swerve.getGyroYaw(), swerve.getKinematics());

    configureDefaultBindings();
    configureBindings();
  }

  private void configureBindings() {

    joystick
        .a()
        .whileTrue(
            Commands.run(
                () -> swerve.setController(
                    new AlongWaypointsController(
                        () -> odometry.getEstimatedPose(), new Pose2d(0, 0, new Rotation2d()))),
                swerve).withName("## Proceed to waypoint"));
  }

  private void configureDefaultBindings() {
    swerve.setDefaultCommand(
        Commands.run(
            () -> swerve.setController(
                new TeleopHeadlessController(
                    () -> -joystick.getLeftY(),
                    () -> -joystick.getLeftX(),
                    () -> -joystick.getRightX(),
                    () -> odometry.getGyroYaw())),
            swerve)
            .withName("[Swerve] Teleop"));
  }

  public Command getAutonomousCommand() {
    return Commands.none();
  }

  private Command joystickRumbleCommand() {
    return Commands.startEnd(
        () -> joystick.getHID().setRumble(GenericHID.RumbleType.kBothRumble, 1.0),
        () -> joystick.getHID().setRumble(GenericHID.RumbleType.kBothRumble, 0.0))
        .withName("[Joystick] Rumble");
  }
}
